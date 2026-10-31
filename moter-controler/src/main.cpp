#include <M5Unified.h>

#include <cstdlib>
#include <cstring>
#include <strings.h>

#include <driver/gpio.h>

#include "config.h"
#include "control_task.h"
#include "encoder_pcnt.h"
#include "motor_output.h"
#include "pmic_log.h"
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
uint32_t last_manual_fault_seq = 0;
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
int brakeOffLevel() { return BRAKE_ON_LEVEL ? 0 : 1; }

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
  digitalWrite(PIN_MOTOR_BRAKE, on ? BRAKE_ON_LEVEL : brakeOffLevel());
  brake_on = on;
  ui_dirty = true;
  publish("OK");
}

void showNotice(const char* text);

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
  if (autoBusy() && percent != 0) {
    // 自動移動中の速度操作は「中止」であって手動への乗り換えではない（仕様: PWM 0）。
    control_pulse_stop();
    requested_speed = 0;
    control_set_manual_speed(0);
    showNotice("自動移動を中止");
    publish("OK");
    ui_dirty = true;
    return true;
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
  digitalWrite(PIN_MOTOR_BRAKE, brakeOffLevel());
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
  Serial.printf("PortB(Base15) IO1/Yellow GPIO%d = PWM, IO2/White GPIO%d = DIR\n",
                PIN_MD13S_PWM, PIN_MD13S_DIR);
  Serial.printf("XH2pin#2 GPIO%d = power (ON level %d), XH8pin#5 GPIO%d = brake (ON level %d)\n",
                PIN_POWER_RELAY, RELAY_ON_LEVEL, PIN_MOTOR_BRAKE, BRAKE_ON_LEVEL);
  Serial.printf("XH8pin#2/#3 GPIO%d/%d = encoder A/B, XH8pin#6 GPIO%d = VIOUT (not read)\n",
                PIN_ENC_A, PIN_ENC_B, PIN_VIOUT);
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

enum class Action : uint8_t { Power, Brake, Speed, Stop, Estop, Motor, HomeP, HomeN, Neutral, Cal };

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

constexpr int kButtonCount = 13;
Button buttons[kButtonCount];
char power_label[20];
char brake_label[24];
char motor_label[8];
char home_p_label[16];
char home_n_label[16];
char notice[32];
uint32_t notice_until_ms = 0;
uint32_t result_until_ms = 0;   // 到達 / 中止 / 失敗 の表示期限
AutoPhase result_phase = AutoPhase::Idle;
char result_reason[24] = "";

void showNotice(const char* text) {
  snprintf(notice, sizeof(notice), "%s", text);
  notice_until_ms = millis() + 1200;
  ui_dirty = true;
}

void layoutButtons(int width) {
  const int margin = 6;
  const int gap = 6;
  const int half_w = (width - margin * 2 - gap) / 2;
  const int quarter_w = (width - margin * 2 - gap * 3) / 4;
  // 速度行は −10 / −1 / [数値] / +1 / +10 の 5 枠。数値枠はボタンではない。
  const int fifth_w = (width - margin * 2 - gap * 4) / 5;

  snprintf(power_label, sizeof(power_label), "電源 切");
  snprintf(brake_label, sizeof(brake_label), "ブレーキ 作動");
  snprintf(motor_label, sizeof(motor_label), "#1");
  snprintf(home_p_label, sizeof(home_p_label), "%sへ", p_end_label(1));
  snprintf(home_n_label, sizeof(home_n_label), "%sへ", n_end_label(1));

  buttons[8] = {margin, 2, 48, 24, motor_label, true, false, 0, Action::Motor};
  buttons[0] = {margin, 30, half_w, 34, power_label, true, false, 0, Action::Power};
  buttons[1] = {margin + half_w + gap, 30, half_w, 34, brake_label, true, false, 0, Action::Brake};

  const int deltas[4] = {-10, -1, 1, 10};
  const char* labels[4] = {"-10", "-1", "+1", "+10"};
  const int slots[4] = {0, 1, 3, 4};
  for (int i = 0; i < 4; ++i) {
    buttons[2 + i] = {margin + slots[i] * (fifth_w + gap), 102, fifth_w, 34, labels[i], true, false, deltas[i],
                      Action::Speed};
  }
  buttons[9] = {margin, 140, quarter_w, 34, home_p_label, false, false, 0, Action::HomeP};
  buttons[10] = {margin + (quarter_w + gap), 140, quarter_w, 34, home_n_label, false, false, 0, Action::HomeN};
  buttons[11] = {margin + 2 * (quarter_w + gap), 140, quarter_w, 34, "中立へ", false, false, 0, Action::Neutral};
  buttons[12] = {margin + 3 * (quarter_w + gap), 140, quarter_w, 34, "校正", false, false, 0, Action::Cal};
  buttons[6] = {margin, 178, half_w, 38, "停止", true, false, 0, Action::Stop};
  buttons[7] = {margin + half_w + gap, 178, half_w, 38, "非常停止", true, false, 0, Action::Estop};
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
  canvas.drawString(button.label, button.x + button.w / 2, button.y + button.h / 2);
}

const Button* hitButton(int x, int y) {
  for (const Button& button : buttons) {
    if (x >= button.x && x < button.x + button.w && y >= button.y && y < button.y + button.h) {
      return &button;
    }
  }
  return nullptr;
}

bool snapActive() { return snap.phase == AutoPhase::Moving || snap.phase == AutoPhase::Dwell; }

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
    case Action::Motor:
      if (!button.enabled) {
        showNotice("電源OFFで切替");
        return;
      }
      selectMotor(static_cast<uint8_t>(snap.profile.motor % 3 + 1));
      break;
    case Action::HomeP:
    case Action::HomeN:
    case Action::Neutral:
    case Action::Cal:
      if (!button.enabled) {
        if (!outputLive()) {
          showNotice("電源ONが必要");
        } else if (!brake_on) {
          showNotice("ブレーキ解除が必要");
        } else if (requested_speed != 0) {
          showNotice("速度を0に");
        } else if (!snap.origin_valid) {
          showNotice("先に原点を取る");
        } else if (button.action == Action::Neutral && !snap.profile.calibrated) {
          showNotice("未校正");
        } else {
          showNotice("操作不可");
        }
        return;
      }
      requestAuto(button.action == Action::HomeP ? AutoKind::HomeP
                  : button.action == Action::HomeN ? AutoKind::HomeN
                  : button.action == Action::Neutral ? AutoKind::Neutral : AutoKind::Cal);
      break;
  }
}

uint32_t last_press_ms = 0;

void pollTouch() {
  const auto touch = M5.Touch.getDetail();
  if (touch.wasPressed()) {
    // 同じ押下で wasPressed が 2 回成立することがある（実機で確認）。250 ms 以内の再押下は無視する。
    const uint32_t now = millis();
    if (now - last_press_ms < 250) {
      return;
    }
    last_press_ms = now;
    if (const Button* button = hitButton(touch.x, touch.y)) {
      onButton(*button);
    }
    next_repeat_ms = now + 400;
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

const char* autoLabel(AutoKind kind, uint8_t motor) {
  static char text[16];
  switch (kind) {
    case AutoKind::HomeP:
      snprintf(text, sizeof(text), "%sへ", p_end_label(motor));
      return text;
    case AutoKind::HomeN:
      snprintf(text, sizeof(text), "%sへ", n_end_label(motor));
      return text;
    case AutoKind::Neutral:
      return "中立へ";
    case AutoKind::Cal:
      return "校正中";
    case AutoKind::Idle:
      break;
  }
  return "";
}

void render() {
  if (!sprite_ok) {
    return;
  }

  const bool live = outputLive();
  const int width = canvas.width();
  const bool power_high = digitalRead(PIN_POWER_RELAY) != 0;
  const bool brake_high = digitalRead(PIN_MOTOR_BRAKE) != 0;
  const bool active = snapActive();
  const uint8_t motor = snap.profile.motor;
  const bool auto_ok = live && brake_on && requested_speed == 0 && !active;

  snprintf(power_label, sizeof(power_label), "電源 %s", relay_on ? "入" : "切");
  snprintf(brake_label, sizeof(brake_label), "ブレーキ %s", brake_on ? "解除" : "作動");
  snprintf(motor_label, sizeof(motor_label), "#%u", static_cast<unsigned>(motor));
  snprintf(home_p_label, sizeof(home_p_label), "%sへ", p_end_label(motor));
  snprintf(home_n_label, sizeof(home_n_label), "%sへ", n_end_label(motor));
  buttons[0].selected = relay_on;
  buttons[1].selected = brake_on;
  for (int i = 2; i <= 5; ++i) {
    buttons[i].enabled = relay_on;
  }
  buttons[8].enabled = !relay_on && !active;
  buttons[9].enabled = auto_ok;
  buttons[10].enabled = auto_ok && snap.origin_valid;
  buttons[11].enabled = auto_ok && snap.origin_valid && snap.profile.calibrated;
  buttons[12].enabled = auto_ok;

  canvas.fillSprite(TFT_BLACK);
  canvas.fillRect(0, 0, width, 28, kDarkRed);
  canvas.drawLine(0, 28, width, 28, kOrange);
  canvas.setFont(&fonts::lgfxJapanGothic_16);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(kOrange, kDarkRed);
  canvas.drawString("MD13S", 62, 14);
  canvas.setTextDatum(middle_right);
  if (notice[0] != '\0') {
    canvas.setTextColor(kRed, kDarkRed);
    canvas.drawString(notice, width - 8, 14);
  } else {
    const char* state = "停止";
    uint16_t state_color = kGray;
    char result_text[32];
    if (active) {
      state = autoLabel(snap.kind, motor);
      state_color = kGreen;
    } else if (result_until_ms != 0 && static_cast<int32_t>(millis() - result_until_ms) < 0) {
      if (result_phase == AutoPhase::Done) {
        state = "到達";
        state_color = kGreen;
      } else if (result_phase == AutoPhase::Cancelled) {
        state = "中止";
        state_color = kAmber;
      } else {
        snprintf(result_text, sizeof(result_text), "失敗:%s", result_reason);
        state = result_text;
        state_color = kRed;
      }
    } else if (relay_on && !live) {
      state = "待機";
      state_color = kAmber;
    } else if (live && applied_speed != 0) {
      state = "運転";
      state_color = kGreen;
    }
    canvas.setTextColor(state_color, kDarkRed);
    canvas.drawString(state, width - 8, 14);
  }

  // センサー行
  canvas.fillRect(6, 68, width - 12, 30, kDarkOrange);
  canvas.drawRect(6, 68, width - 12, 30, kOrange);
  canvas.setFont(&fonts::Font4);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(snap.origin_valid ? TFT_WHITE : kAmber, kDarkOrange);
  char pos_text[16];
  snprintf(pos_text, sizeof(pos_text), "%ld", static_cast<long>(snap.position));
  canvas.drawString(pos_text, 12, 83);
  canvas.setFont(&fonts::lgfxJapanGothic_12);
  canvas.setTextDatum(middle_right);
  char info_text[48];
  snprintf(info_text, sizeof(info_text), "%d c/s  A%d B%d  %s %s", snap.count_per_s, snap.a_level, snap.b_level,
           snap.origin_valid ? "原点" : "原点なし", snap.profile.calibrated ? "校正済" : "未校正");
  canvas.setTextColor(snap.profile.calibrated ? kOrange : kRed, kDarkOrange);
  canvas.drawString(info_text, width - 12, 83);

  for (const Button& button : buttons) {
    drawButton(button);
  }

  // 速度の数値（速度行の中央枠）
  {
    const int margin = 6;
    const int gap = 6;
    const int fifth_w = (width - margin * 2 - gap * 4) / 5;
    const int x = margin + 2 * (fifth_w + gap);
    canvas.fillRect(x, 102, fifth_w, 34, kDarkOrange);
    canvas.drawRect(x, 102, fifth_w, 34, kOrange);
    uint16_t speed_color = TFT_WHITE;
    if (requested_speed != 0) {
      speed_color = (live && applied_speed != 0) ? kGreen : kOrange;
    }
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(speed_color, kDarkOrange);
    canvas.setTextDatum(middle_center);
    char speed_text[8];
    snprintf(speed_text, sizeof(speed_text), "%d", requested_speed);
    canvas.drawString(speed_text, x + fifth_w / 2, 119);
  }

  canvas.drawLine(6, 220, width - 6, 220, kAmber);
  canvas.setFont(&fonts::lgfxJapanGothic_12);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("DIR", 8, 231);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString(digitalRead(PIN_MD13S_DIR) ? "H" : "L", 34, 231);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("PWR", 50, 231);
  canvas.setTextColor(power_high ? kGreen : kGray, TFT_BLACK);
  canvas.drawString(power_high ? "H" : "L", 80, 231);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("BRK", 96, 231);
  canvas.setTextColor(brake_high ? kGreen : kGray, TFT_BLACK);
  canvas.drawString(brake_high ? "H" : "L", 124, 231);
  canvas.setTextColor(kOrange, TFT_BLACK);
  char stroke_text[40];
  snprintf(stroke_text, sizeof(stroke_text), "行程 %ld 中立 %ld %+d", static_cast<long>(snap.profile.stroke),
           static_cast<long>(snap.profile.neutral), static_cast<int>(snap.working_sign));
  canvas.setTextDatum(middle_right);
  canvas.drawString(stroke_text, width - 8, 231);

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
  if (now.manual_fault_seq != last_manual_fault_seq) {
    // 手動速度中にセンサーの計数が止まった。制御タスクは既に出力 0。速度指令も 0 に戻して通知する。
    last_manual_fault_seq = now.manual_fault_seq;
    requested_speed = 0;
    control_set_manual_speed(0);
    Serial.printf("WARN manual_%s speed->0\n", now.manual_fault_reason);
    Serial.flush();
    showNotice(strcmp(now.manual_fault_reason, "slow") == 0 ? "遅すぎ停止 速度0" : "無パルス停止 速度0");
    ui_dirty = true;
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
    Serial.flush();
    if (now.phase == AutoPhase::Done || now.phase == AutoPhase::Cancelled || now.phase == AutoPhase::Failed) {
      result_phase = now.phase;
      snprintf(result_reason, sizeof(result_reason), "%s", now.reason);
      result_until_ms = millis() + 4000;  // 失敗理由（stall / slow）を読める長さ
    }
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
    Serial.flush();
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
  driveOutput(static_cast<gpio_num_t>(PIN_MOTOR_BRAKE), brakeOffLevel());
  driveOutput(static_cast<gpio_num_t>(PIN_MD13S_DIR), DIR_POSITIVE_LEVEL);
  driveOutput(static_cast<gpio_num_t>(PIN_MD13S_PWM), 0);

  auto cfg = M5.config();
  cfg.clear_display = true;
#if MC_USB_POWER
  // env:m5stack-cores3-usbpower。USB のみで動かし、CoreS3 が Grove 5V を供給する。Bottom の DC 入力は接続しない。
  // （DIN Base 時代の Issue #1 回避用。Base15 の DC と USB の同時接続は未検証）
  cfg.output_power = true;
#else
  cfg.output_power = false;  // Bottom（Base15 の DC ジャック → MP1584 → BUS 5V）から 5V を受ける設定
#endif
  cfg.internal_imu = false;  // CoreS3 SE に BMI270 は無い
  cfg.internal_mic = false;
  cfg.internal_spk = false;
  cfg.external_imu = false;
  cfg.external_rtc = false;
  // 外部機器の探索をさせない。理由は 2 つ。
  //  - Port.A（G1 SCL / G2 SDA）の I2C 探索抑止。G2 は主電源リレー IN なので I2C の High で入れたくない
  //    （G1 はブレーキを G13 に移して空いたが、I2C は両線が要るので緩めない）。
  //  - Module Display / RCA の検出が走ると M5Unified が mbus_pin23 = G13 を I2S DOUT に取る。
  //    G13 はブレーキ解除リレー IN なので external_speaker_value = 0 で止める。internal_spk = false も同じ理由で保つ。
  cfg.external_display_value = 0;
  cfg.external_speaker_value = 0;
  M5.begin(cfg);
#if MC_PMIC_LOG
  pmic_log_boot();  // 前回の電源断要因（REG21）を、ほかの処理より先に読む
#endif
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
  digitalWrite(PIN_MOTOR_BRAKE, brakeOffLevel());
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
#if MC_PMIC_LOG
  pmic_log_tick();
#endif

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
    const ControlSnapshot previous = snap;
    snap = control_snapshot();
    if (snap.applied_speed != applied_speed) {
      applied_speed = snap.applied_speed;
      ui_dirty = true;
    }
    if (snap.position != previous.position || snap.origin_valid != previous.origin_valid ||
        snap.profile.calibrated != previous.profile.calibrated || snap.profile.motor != previous.profile.motor ||
        snap.profile.stroke != previous.profile.stroke || snap.profile.neutral != previous.profile.neutral ||
        snap.working_sign != previous.working_sign || snap.phase != previous.phase ||
        snap.a_level != previous.a_level || snap.b_level != previous.b_level ||
        snap.count_per_s != previous.count_per_s) {
      ui_dirty = true;
    }
    if (result_until_ms != 0 && static_cast<int32_t>(millis() - result_until_ms) >= 0) {
      result_until_ms = 0;
      ui_dirty = true;
    }
    reportAuto(snap);
  }

  if (ui_dirty) {
    render();
    ui_dirty = false;
  }
}
