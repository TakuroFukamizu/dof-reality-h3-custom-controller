#include <M5Unified.h>

#include <cstdlib>
#include <cstring>
#include <strings.h>

#include <driver/gpio.h>

#include "config.h"
#include "control_task.h"
#include "encoder_pcnt.h"
#include "motor_output.h"
#include "profile_store.h"

namespace {

int requested_speed = 0;
int applied_speed = 0;  // 制御タスクが実際に書いた値（スナップショットから）
bool relay_on = false;
bool brake_on = false;
uint32_t relay_on_ms = 0;
bool was_live = false;
bool ui_dirty = true;
bool sprite_ok = false;
ControlSnapshot snap;
uint32_t next_snap_ms = 0;
uint32_t next_enc_ms = 0;
uint32_t last_request_seq = 0;
AutoPhase last_phase = AutoPhase::Idle;
AutoKind last_kind = AutoKind::Idle;
CalStep last_cal_step = CalStep::None;
// HomeP 開始時の記録（原点ずれ警告用）
bool home_had_origin = false;
int32_t home_start_pos = 0;
int64_t home_start_raw = 0;

char line_buf[64];
size_t line_len = 0;
uint32_t next_repeat_ms = 0;

M5Canvas canvas(&M5.Display);

int relayOffLevel() { return RELAY_ON_LEVEL ? 0 : 1; }

bool outputLive() {
  return relay_on && (millis() - relay_on_ms >= RELAY_SETTLE_MS);
}

void driveOutput(gpio_num_t pin, int level) {
  gpio_set_level(pin, level);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  gpio_set_level(pin, level);
}

void publish(const char* prefix) {
  const ControlSnapshot now = control_snapshot();
  Serial.printf("%s R=%d B=%d S=%d OUT=%d DIR=%d PWR_PIN=%d BRK_PIN=%d LIVE=%d ENC=%lld A=%d B=%d\n",
                prefix, relay_on ? 1 : 0, brake_on ? 1 : 0, requested_speed,
                now.applied_speed, digitalRead(PIN_MD13S_DIR),
                digitalRead(PIN_POWER_RELAY), digitalRead(PIN_MOTOR_BRAKE),
                outputLive() ? 1 : 0, static_cast<long long>(now.raw_count),
                now.a_level, now.b_level);
}

void publishEnc(const ControlSnapshot& now) {
  Serial.printf("ENC motor=%u count=%lld pos=%ld cps=%d a=%d b=%d origin=%d cal=%d stroke=%ld neutral=%ld sign=%d auto=%s phase=%s reason=%s\n",
                static_cast<unsigned>(now.profile.motor), static_cast<long long>(now.raw_count),
                static_cast<long>(now.position), now.count_per_s, now.a_level, now.b_level,
                now.origin_valid ? 1 : 0, now.profile.calibrated ? 1 : 0,
                static_cast<long>(now.profile.stroke), static_cast<long>(now.profile.neutral),
                static_cast<int>(now.working_sign), auto_kind_name(now.kind), auto_phase_name(now.phase),
                now.reason);
}

// 自動移動の要求。制御タスクが可否を判定し、結果は snapshot.request_seq / request_error に出る。
void requestAuto(AutoKind kind) {
  control_request(kind);
}

bool autoBusy() {
  const ControlSnapshot now = control_snapshot();
  return now.phase == AutoPhase::Moving || now.phase == AutoPhase::Dwell;
}

// 系統の切り替え。電源 OFF かつ自動移動中でないときだけ。
bool selectMotor(uint8_t motor) {
  if (!profile_motor_ok(motor)) {
    Serial.println("ERR motor_arg");
    return false;
  }
  if (relay_on || autoBusy()) {
    Serial.println("ERR busy");
    return false;
  }
  profile_store_select_motor(motor);
  const MotorProfile p = profile_store_load(motor);
  control_set_profile(p);
  Serial.printf("OK motor=%u cal=%d stroke=%ld neutral=%ld sign=%d\n", static_cast<unsigned>(motor),
                p.calibrated ? 1 : 0, static_cast<long>(p.stroke), static_cast<long>(p.neutral),
                static_cast<int>(p.sign));
  ui_dirty = true;
  return true;
}

void setRelay(bool on) {
  if (on == relay_on) {
    publish("OK");
    return;
  }
  if (!on) {
    requested_speed = 0;
    control_set_manual_speed(0);
  }
  digitalWrite(PIN_POWER_RELAY, on ? RELAY_ON_LEVEL : relayOffLevel());
  relay_on = on;
  relay_on_ms = millis();
  was_live = false;
  ui_dirty = true;
  publish("OK");
}

void setBrake(bool on) {
  if (on == brake_on) {
    publish("OK");
    return;
  }
  digitalWrite(PIN_MOTOR_BRAKE, on ? RELAY_ON_LEVEL : relayOffLevel());
  brake_on = on;
  ui_dirty = true;
  publish("OK");
}

bool setSpeed(int percent) {
  if (percent > SPEED_MAX) {
    percent = SPEED_MAX;
  } else if (percent < SPEED_MIN) {
    percent = SPEED_MIN;
  }
  if (percent != 0 && !relay_on) {
    Serial.println("ERR relay_off");
    return false;
  }
  if (percent == requested_speed) {
    return true;
  }
  requested_speed = percent;
  control_set_manual_speed(requested_speed);
  publish("OK");
  ui_dirty = true;
  return true;
}

void estop() {
  requested_speed = 0;
  control_set_manual_speed(0);
  control_pulse_estop();
  digitalWrite(PIN_POWER_RELAY, relayOffLevel());
  digitalWrite(PIN_MOTOR_BRAKE, relayOffLevel());
  relay_on = false;
  brake_on = false;
  was_live = false;
  ui_dirty = true;
  publish("ESTOP");
}

void printHelp() {
  Serial.println("MD13S sign-magnitude / power relay / motor brake");
  Serial.println("  R 0|1  or  RELAY ON|OFF");
  Serial.println("  B 0|1  or  BRAKE ON|OFF");
  Serial.println("  S -100..100  or  SPEED n");
  Serial.println("  X or STOP     speed 0, relays stay");
  Serial.println("  E or ESTOP    speed 0, power off, brake off");
  Serial.println("  ? or STATUS  (STAT line + ENC line)");
  Serial.println("  M 1|2|3       select motor profile (relay off only)");
  Serial.println("  HOME          drive to P end (speed +) and set origin 0");
  Serial.println("  UPPER         drive to N end (speed -), needs origin");
  Serial.println("  NEUTRAL       move to neutral count, needs origin + calibration");
  Serial.println("  CAL           HOME -> UPPER -> save -> NEUTRAL");
  Serial.println("  UNCAL         clear calibration of selected motor (relay off only)");
  Serial.printf("PortB IO1/Yellow GPIO%d = PWM, IO2/White GPIO%d = DIR\n",
                PIN_MD13S_PWM, PIN_MD13S_DIR);
  Serial.printf("PortA Yellow/G2 GPIO%d = power, White/G1 GPIO%d = brake, ON level %d\n",
                PIN_POWER_RELAY, PIN_MOTOR_BRAKE, RELAY_ON_LEVEL);
}

const char* skipSpace(const char* s) {
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  return s;
}

void handleLine(char* line) {
  const char* cursor = skipSpace(line);
  if (*cursor == '\0') {
    return;
  }

  char cmd[16];
  size_t cmd_len = 0;
  while (*cursor && *cursor != ' ' && *cursor != '\t' &&
         cmd_len + 1 < sizeof(cmd)) {
    cmd[cmd_len++] = *cursor++;
  }
  cmd[cmd_len] = '\0';
  cursor = skipSpace(cursor);

  if (strcasecmp(cmd, "HELP") == 0) {
    printHelp();
    return;
  }
  if (strcasecmp(cmd, "?") == 0 || strcasecmp(cmd, "STATUS") == 0) {
    publish("STAT");
    publishEnc(control_snapshot());
    return;
  }
  if (strcasecmp(cmd, "M") == 0 || strcasecmp(cmd, "MOTOR") == 0) {
    char* end = nullptr;
    const long value = strtol(cursor, &end, 10);
    if (end == cursor || *skipSpace(end) != '\0') {
      Serial.println("ERR motor_arg");
      return;
    }
    selectMotor(static_cast<uint8_t>(value));
    return;
  }
  if (strcasecmp(cmd, "HOME") == 0) {
    requestAuto(AutoKind::HomeP);
    return;
  }
  if (strcasecmp(cmd, "UPPER") == 0) {
    requestAuto(AutoKind::HomeN);
    return;
  }
  if (strcasecmp(cmd, "NEUTRAL") == 0) {
    requestAuto(AutoKind::Neutral);
    return;
  }
  if (strcasecmp(cmd, "CAL") == 0) {
    requestAuto(AutoKind::Cal);
    return;
  }
  if (strcasecmp(cmd, "UNCAL") == 0) {
    // 選択中の系統の校正を消す（間違った系統で校正したときの取り消し）。電源 OFF かつ停止中だけ。
    if (relay_on || autoBusy()) {
      Serial.println("ERR busy");
      return;
    }
    MotorProfile p = control_snapshot().profile;
    p.stroke = 0;
    p.neutral = 0;
    p.calibrated = false;
    if (profile_store_save(p)) {
      control_set_profile(p);
      Serial.printf("OK uncal motor=%u\n", static_cast<unsigned>(p.motor));
      ui_dirty = true;
    } else {
      Serial.println("ERR nvs_save");
    }
    return;
  }
  if (strcasecmp(cmd, "X") == 0 || strcasecmp(cmd, "STOP") == 0) {
    control_pulse_stop();
    setSpeed(0);
    return;
  }
  if (strcasecmp(cmd, "E") == 0 || strcasecmp(cmd, "ESTOP") == 0) {
    estop();
    return;
  }
  if (strcasecmp(cmd, "R") == 0 || strcasecmp(cmd, "RELAY") == 0) {
    if (strcmp(cursor, "1") == 0 || strcasecmp(cursor, "ON") == 0) {
      setRelay(true);
    } else if (strcmp(cursor, "0") == 0 || strcasecmp(cursor, "OFF") == 0) {
      setRelay(false);
    } else {
      Serial.println("ERR relay_arg");
    }
    return;
  }
  if (strcasecmp(cmd, "B") == 0 || strcasecmp(cmd, "BRAKE") == 0) {
    if (strcmp(cursor, "1") == 0 || strcasecmp(cursor, "ON") == 0) {
      setBrake(true);
    } else if (strcmp(cursor, "0") == 0 || strcasecmp(cursor, "OFF") == 0) {
      setBrake(false);
    } else {
      Serial.println("ERR brake_arg");
    }
    return;
  }
  if (strcasecmp(cmd, "S") == 0 || strcasecmp(cmd, "SPEED") == 0) {
    if (*cursor == '\0') {
      Serial.println("ERR speed_arg");
      return;
    }
    char* end = nullptr;
    const long value = strtol(cursor, &end, 10);
    if (end == cursor || *skipSpace(end) != '\0') {
      Serial.println("ERR speed_arg");
      return;
    }
    long clamped = value;
    if (clamped > SPEED_MAX) {
      clamped = SPEED_MAX;
    } else if (clamped < SPEED_MIN) {
      clamped = SPEED_MIN;
    }
    setSpeed(static_cast<int>(clamped));
    return;
  }

  Serial.println("ERR unknown");
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line_buf[line_len] = '\0';
      if (line_len > 0) {
        handleLine(line_buf);
      }
      line_len = 0;
      continue;
    }
    if (line_len + 1 < sizeof(line_buf)) {
      line_buf[line_len++] = c;
    }
  }
}

// KeyStream src/display.cpp と同じ警告色（RGB565）。
constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
constexpr uint16_t kOrange = rgb565(255, 90, 0);
constexpr uint16_t kAmber = rgb565(230, 70, 0);
constexpr uint16_t kDarkRed = rgb565(50, 10, 10);
constexpr uint16_t kDarkOrange = rgb565(35, 12, 0);
constexpr uint16_t kGreen = rgb565(0, 255, 100);
constexpr uint16_t kGray = rgb565(60, 60, 60);
constexpr uint16_t kRed = rgb565(255, 0, 0);

enum class Action : uint8_t { Power, Brake, Speed, Stop, Estop };

struct Button {
  int x;
  int y;
  int w;
  int h;
  const char* label;
  bool enabled;
  bool selected;
  int speed_delta;
  Action action;
};

Button buttons[8];
char power_label[20];
char brake_label[24];
char notice[32];
uint32_t notice_until_ms = 0;

void showNotice(const char* text) {
  snprintf(notice, sizeof(notice), "%s", text);
  notice_until_ms = millis() + 1200;
  ui_dirty = true;
}

void layoutButtons(int width) {
  const int margin = 6;
  const int gap = 6;
  const int half_w = (width - margin * 2 - gap) / 2;
  const int speed_w = (width - margin * 2 - gap * 3) / 4;

  snprintf(power_label, sizeof(power_label), "電源 切");
  snprintf(brake_label, sizeof(brake_label), "ブレーキ 切");
  buttons[0] = {margin, 32, half_w, 40, power_label, true, false, 0, Action::Power};
  buttons[1] = {margin + half_w + gap, 32, half_w, 40, brake_label, true, false, 0,
                Action::Brake};

  const int deltas[4] = {-10, -1, 1, 10};
  const char* labels[4] = {"-10", "-1", "+1", "+10"};
  for (int i = 0; i < 4; ++i) {
    buttons[2 + i] = {margin + i * (speed_w + gap), 112, speed_w, 40, labels[i],
                      true, false, deltas[i], Action::Speed};
  }
  buttons[6] = {margin, 158, half_w, 42, "停止", true, false, 0, Action::Stop};
  buttons[7] = {margin + half_w + gap, 158, half_w, 42, "非常停止", true, false, 0,
                Action::Estop};
}

void drawButton(const Button& button) {
  uint16_t fill = kDarkOrange;
  uint16_t edge = kOrange;
  uint16_t text = kOrange;
  if (!button.enabled) {
    fill = TFT_BLACK;
    edge = kGray;
    text = kGray;
  } else if (button.action == Action::Estop) {
    fill = kDarkRed;
    edge = kRed;
    text = TFT_WHITE;
  } else if (button.selected) {
    edge = kGreen;
    text = kGreen;
  }
  canvas.fillRect(button.x, button.y, button.w, button.h, fill);
  canvas.drawRect(button.x, button.y, button.w, button.h, edge);
  canvas.setFont(&fonts::lgfxJapanGothic_16);
  canvas.setTextColor(text, fill);
  canvas.setTextDatum(middle_center);
  canvas.drawString(button.label, button.x + button.w / 2,
                    button.y + button.h / 2);
}

const Button* hitButton(int x, int y) {
  for (const Button& button : buttons) {
    if (x >= button.x && x < button.x + button.w && y >= button.y &&
        y < button.y + button.h) {
      return &button;
    }
  }
  return nullptr;
}

void onButton(const Button& button) {
  switch (button.action) {
    case Action::Power:
      setRelay(!relay_on);
      break;
    case Action::Brake:
      setBrake(!brake_on);
      break;
    case Action::Speed:
      if (!button.enabled) {
        showNotice("先に電源をON");
        return;
      }
      setSpeed(requested_speed + button.speed_delta);
      break;
    case Action::Stop:
      control_pulse_stop();
      setSpeed(0);
      break;
    case Action::Estop:
      estop();
      break;
  }
}

void pollTouch() {
  const auto touch = M5.Touch.getDetail();
  if (touch.wasPressed()) {
    if (const Button* button = hitButton(touch.x, touch.y)) {
      onButton(*button);
    }
    next_repeat_ms = millis() + 400;
    return;
  }
  if (touch.isHolding() && static_cast<int32_t>(millis() - next_repeat_ms) >= 0) {
    if (const Button* button = hitButton(touch.x, touch.y)) {
      if (button->enabled && button->action == Action::Speed) {
        onButton(*button);
      }
    }
    next_repeat_ms = millis() + 80;
  }
}

void render() {
  if (!sprite_ok) {
    return;
  }

  const bool live = outputLive();
  const int width = canvas.width();
  const bool power_high = digitalRead(PIN_POWER_RELAY) != 0;
  const bool brake_high = digitalRead(PIN_MOTOR_BRAKE) != 0;

  snprintf(power_label, sizeof(power_label), "電源 %s", relay_on ? "入" : "切");
  snprintf(brake_label, sizeof(brake_label), "ブレーキ %s", brake_on ? "入" : "切");
  buttons[0].selected = relay_on;
  buttons[1].selected = brake_on;
  for (int i = 2; i <= 5; ++i) {
    buttons[i].enabled = relay_on;
  }

  canvas.fillSprite(TFT_BLACK);
  canvas.fillRect(0, 0, width, 28, kDarkRed);
  canvas.drawLine(0, 28, width, 28, kOrange);
  canvas.setFont(&fonts::lgfxJapanGothic_16);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(kOrange, kDarkRed);
  canvas.drawString("MD13S", 8, 14);
  canvas.setTextDatum(middle_right);
  if (notice[0] != '\0') {
    canvas.setTextColor(kRed, kDarkRed);
    canvas.drawString(notice, width - 8, 14);
  } else {
    const char* state = "停止";
    uint16_t state_color = kGray;
    if (relay_on && !live) {
      state = "待機";
      state_color = kAmber;
    } else if (live && applied_speed != 0) {
      state = "運転";
      state_color = kGreen;
    }
    canvas.setTextColor(state_color, kDarkRed);
    canvas.drawString(state, width - 8, 14);
  }

  canvas.fillRect(6, 76, width - 12, 32, kDarkOrange);
  canvas.drawRect(6, 76, width - 12, 32, kOrange);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(kOrange, kDarkOrange);
  canvas.drawString("速度", 14, 92);
  uint16_t speed_color = TFT_WHITE;
  if (requested_speed != 0) {
    speed_color = (live && applied_speed != 0) ? kGreen : kOrange;
  }
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(speed_color, kDarkOrange);
  canvas.setTextDatum(middle_center);
  char speed_text[8];
  snprintf(speed_text, sizeof(speed_text), "%d", requested_speed);
  canvas.drawString(speed_text, width / 2 + 16, 92);

  for (const Button& button : buttons) {
    drawButton(button);
  }

  canvas.drawLine(6, 206, width - 6, 206, kAmber);
  canvas.setFont(&fonts::lgfxJapanGothic_16);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("状態", 8, 222);
  const char* direction = "停止";
  if (requested_speed > 0) {
    direction = "正転";
  } else if (requested_speed < 0) {
    direction = "逆転";
  }
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString(direction, 48, 222);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("電源", 96, 222);
  canvas.setTextColor(power_high ? kGreen : kGray, TFT_BLACK);
  canvas.drawString(power_high ? "H" : "L", 136, 222);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("ブレーキ", 156, 222);
  canvas.setTextColor(brake_high ? kGreen : kGray, TFT_BLACK);
  canvas.drawString(brake_high ? "H" : "L", 228, 222);

  canvas.pushSprite(0, 0);
}

void reportAuto(const ControlSnapshot& now) {
  const bool active = now.phase == AutoPhase::Moving || now.phase == AutoPhase::Dwell;
  if (now.request_seq != last_request_seq) {
    last_request_seq = now.request_seq;
    if (now.request_error[0] != '\0') {
      Serial.printf("ERR auto_%s\n", now.request_error);
      showNotice(now.request_error);
    }
  }
  const bool leg_changed = now.phase != last_phase || now.kind != last_kind || now.cal_step != last_cal_step;
  if (leg_changed) {
    const bool started_home_p = active && ((now.kind == AutoKind::HomeP && last_phase != AutoPhase::Moving) ||
                                           (now.kind == AutoKind::Cal && now.cal_step == CalStep::ToP &&
                                            last_cal_step != CalStep::ToP));
    if (started_home_p) {
      home_had_origin = now.origin_valid;
      home_start_pos = now.position;
      home_start_raw = now.raw_count;
    }
    Serial.printf("AUTO %s %s %s pos=%ld\n", auto_kind_name(now.kind), auto_phase_name(now.phase), now.reason,
                  static_cast<long>(now.position));
    if (now.phase == AutoPhase::Done) {
      const bool home_leg = now.kind == AutoKind::HomeP;
      if (home_leg && home_had_origin && now.profile.calibrated) {
        const int32_t shift = home_start_pos + static_cast<int32_t>((now.raw_count - home_start_raw) * now.working_sign);
        const int32_t shift_abs = shift < 0 ? -shift : shift;
        if (shift_abs > now.profile.stroke / 10) {
          Serial.printf("AUTO warn origin_shift=%ld\n", static_cast<long>(shift));
        }
      }
      if (now.kind == AutoKind::HomeN && now.profile.calibrated) {
        const int32_t diff = now.last_measured_stroke - now.profile.stroke;
        if ((diff < 0 ? -diff : diff) > now.profile.stroke / 20) {
          Serial.printf("AUTO warn stroke measured=%ld saved=%ld\n", static_cast<long>(now.last_measured_stroke),
                        static_cast<long>(now.profile.stroke));
        }
      }
    }
    last_phase = now.phase;
    last_kind = now.kind;
    last_cal_step = now.cal_step;
    ui_dirty = true;
  }
  if (active && static_cast<int32_t>(millis() - next_enc_ms) >= 0) {
    next_enc_ms = millis() + 500;
    publishEnc(now);
  }
  if (now.profile_dirty) {
    if (profile_store_save(now.profile)) {
      Serial.printf("AUTO saved motor=%u stroke=%ld neutral=%ld sign=%d\n", static_cast<unsigned>(now.profile.motor),
                    static_cast<long>(now.profile.stroke), static_cast<long>(now.profile.neutral),
                    static_cast<int>(now.profile.sign));
    } else {
      Serial.println("ERR nvs_save");
    }
    control_clear_profile_dirty();
  }
}

}  // namespace

void setup() {
  driveOutput(static_cast<gpio_num_t>(PIN_POWER_RELAY), relayOffLevel());
  driveOutput(static_cast<gpio_num_t>(PIN_MOTOR_BRAKE), relayOffLevel());
  driveOutput(static_cast<gpio_num_t>(PIN_MD13S_DIR), DIR_POSITIVE_LEVEL);
  driveOutput(static_cast<gpio_num_t>(PIN_MD13S_PWM), 0);

  auto cfg = M5.config();
  cfg.clear_display = true;
#if MC_USB_POWER
  // env:m5stack-cores3-usbpower。USB のみで動かし、CoreS3 が Grove 5V を供給する。12V は接続しない（Issue #1）。
  cfg.output_power = true;
#else
  cfg.output_power = false;  // DIN側からBUSの5Vを受ける設定
#endif
  cfg.internal_imu = false;  // CoreS3 SE に BMI270 は無い
  cfg.internal_mic = false;
  cfg.internal_spk = false;
  cfg.external_imu = false;
  cfg.external_rtc = false;
  // Port A を外部機器の探索に使わせない。GPIO1 と GPIO2 がリレーなので、I2C の High で入れたくない。
  cfg.external_display_value = 0;
  cfg.external_speaker_value = 0;
  M5.begin(cfg);
#if !MC_USB_POWER
  M5.Power.setUsbOutput(false);  // CoreS3からPC側へのUSB給電を無効化
#endif
  Serial.begin(115200);

  if (M5.Display.width() < M5.Display.height()) {
    M5.Display.setRotation(M5.Display.getRotation() ^ 1);
  }

  pinMode(PIN_POWER_RELAY, OUTPUT);
  digitalWrite(PIN_POWER_RELAY, relayOffLevel());
  pinMode(PIN_MOTOR_BRAKE, OUTPUT);
  digitalWrite(PIN_MOTOR_BRAKE, relayOffLevel());
  pinMode(PIN_MD13S_DIR, OUTPUT);
  digitalWrite(PIN_MD13S_DIR, DIR_POSITIVE_LEVEL);
  motor_output_begin();
  if (!encoder_begin()) {
    Serial.println("ERR pcnt");
  }
  if (!profile_store_begin()) {
    Serial.println("ERR nvs");
  }
  control_begin(profile_store_load(profile_store_selected_motor()));

  canvas.setColorDepth(16);
  canvas.setPsram(true);
  sprite_ok = canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr;
  if (!sprite_ok) {
    canvas.setPsram(false);
    sprite_ok = canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr;
  }
  if (!sprite_ok) {
    Serial.println("ERR sprite");
  }
  layoutButtons(M5.Display.width());

  printHelp();
  publish("BOOT");
  ui_dirty = true;
}

void loop() {
  M5.update();
  pollSerial();
  pollTouch();

  if (notice[0] != '\0' && static_cast<int32_t>(millis() - notice_until_ms) >= 0) {
    notice[0] = '\0';
    ui_dirty = true;
  }

  const bool live = outputLive();
  control_set_relay_live(live);
  control_set_brake_released(brake_on);
  if (live != was_live) {
    was_live = live;
    publish("STAT");
    ui_dirty = true;
  }
  if (static_cast<int32_t>(millis() - next_snap_ms) >= 0) {
    next_snap_ms = millis() + 50;
    snap = control_snapshot();
    if (snap.applied_speed != applied_speed) {
      applied_speed = snap.applied_speed;
      ui_dirty = true;
    }
    reportAuto(snap);
  }

  if (ui_dirty) {
    render();
    ui_dirty = false;
  }
}
