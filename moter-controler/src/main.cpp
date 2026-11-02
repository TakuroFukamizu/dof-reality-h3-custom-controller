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
#include "usb_diag.h"

#if MC_PROTO_V1
#include <esp_log.h>

#include "v1_codec.h"
#include "v1_link.h"
#endif

namespace {

M5Canvas canvas(&M5.Display);
bool sprite_ok = false;

int relayOffLevel() { return RELAY_ON_LEVEL ? 0 : 1; }
int brakeOffLevel() { return BRAKE_ON_LEVEL ? 0 : 1; }

void driveOutput(gpio_num_t pin, int level) {
  gpio_set_level(pin, level);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  gpio_set_level(pin, level);
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

// USB 診断行の色（Issue #11）。緑: バスも IN token も来ている。橙: SOF はあるがホストがこのアドレスを読みに来ない
//（ポート未オープン、またはアドレス不一致）。赤: SOF が止まっている（バス停止 / suspend / PHY）。
uint16_t usbDiagColor(const UsbDiag& d) {
  if (!d.sof_alive) return kRed;
  if (!d.in_token || d.out_data_avail) return kAmber;
  return kGreen;
}

UsbDiag usb_diag{};
char usb_diag_text[96] = "usb -";
uint32_t next_usb_diag_ms = 0;

// 500 ms ごとに USB-Serial/JTAG の状態を読み、行を作り直す（両 env 共通）。行が変わったら true（呼び出し側が描き直す）。
// bus_reset_seen は、このサンプルで bus reset の回数が増えたとき true。
bool usbDiagTick(bool* bus_reset_seen) {
  *bus_reset_seen = false;
  if (static_cast<int32_t>(millis() - next_usb_diag_ms) < 0) return false;
  next_usb_diag_ms = millis() + 500;
  const uint32_t resets_before = usb_diag.bus_resets;
  usb_diag_sample(&usb_diag);
  *bus_reset_seen = usb_diag.bus_resets != resets_before;
  char text[sizeof(usb_diag_text)];
  usb_diag_format(usb_diag, text, sizeof(text));
  if (strcmp(text, usb_diag_text) == 0) return false;
  snprintf(usb_diag_text, sizeof(usb_diag_text), "%s", text);
  return true;
}

// ピンを安全側に固定してから M5 を初期化する。設定は従来 env と v1 env で同一。
void boardBegin() {
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
  // USB CDC をホストが読んでいないと write / flush は既定 100 ms ずつ待つ。自動移動中は AUTO 行と ENC 行の
  // flush が続くので、UI ループが 200 ms 止まると制御タスクの入力鮮度の守りが働いてしまう。20 ms に短縮する。
  Serial.setTxTimeoutMs(20);
  usb_diag_begin();  // Issue #11: USB-Serial/JTAG の状態を画面に出す（読むだけ）

  if (M5.Display.width() < M5.Display.height()) {
    M5.Display.setRotation(M5.Display.getRotation() ^ 1);
  }

  pinMode(PIN_POWER_RELAY, OUTPUT);
  digitalWrite(PIN_POWER_RELAY, relayOffLevel());
  pinMode(PIN_MOTOR_BRAKE, OUTPUT);
  digitalWrite(PIN_MOTOR_BRAKE, brakeOffLevel());
  pinMode(PIN_MD13S_DIR, OUTPUT);
  digitalWrite(PIN_MD13S_DIR, DIR_POSITIVE_LEVEL);
}

bool spriteBegin() {
  canvas.setColorDepth(16);
  canvas.setPsram(true);
  sprite_ok = canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr;
  if (!sprite_ok) {
    canvas.setPsram(false);
    sprite_ok = canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr;
  }
  return sprite_ok;
}

}  // namespace

#if !MC_PROTO_V1

namespace {

int requested_speed = 0;
int applied_speed = 0;  // 制御タスクが実際に書いた値（スナップショットから）
bool relay_on = false;
bool brake_on = false;
uint32_t relay_on_ms = 0;
bool was_live = false;
bool ui_dirty = true;
ControlSnapshot snap;
uint32_t next_snap_ms = 0;
uint32_t next_enc_ms = 0;
uint32_t next_profile_save_ms = 0;  // NVS 保存に失敗したら 2 s 後に再試行（Issue #15 M6）
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

bool outputLive() {
  return relay_on && (millis() - relay_on_ms >= RELAY_SETTLE_MS);
}

// 1 行を書き、Serial.write が途中で諦めた（tx timeout）回数を usb_diag に残す（Issue #11 の TX 側の切り分け）。
void txLine(const char* line) {
  const size_t len = strlen(line);
  const size_t written = Serial.write(reinterpret_cast<const uint8_t*>(line), len);
  usb_diag_note_tx(len, written);
}

// USBDIAG: 画面に出ている行をそのまま返す。ここで usb_diag_sample() を呼ぶと、直後の定期サンプルと 1 ms 以内に
// 2 回読むことになり、SOF フレーム番号が進んでいない（sof0 = 赤）と誤判定する。
void publishUsbDiag() {
  char line[sizeof(usb_diag_text) + 2];
  snprintf(line, sizeof(line), "%s\n", usb_diag_text);
  txLine(line);
}

void publish(const char* prefix) {
  const ControlSnapshot now = control_snapshot();
  char line[160];
  snprintf(line, sizeof(line), "%s R=%d B=%d S=%d OUT=%d DIR=%d PWR_PIN=%d BRK_PIN=%d LIVE=%d ENC=%lld A=%d B=%d\n",
           prefix, relay_on ? 1 : 0, brake_on ? 1 : 0, requested_speed, now.applied_speed, digitalRead(PIN_MD13S_DIR),
           digitalRead(PIN_POWER_RELAY), digitalRead(PIN_MOTOR_BRAKE), outputLive() ? 1 : 0,
           static_cast<long long>(now.raw_count), now.a_level, now.b_level);
  txLine(line);
}

void publishEnc(const ControlSnapshot& now) {
  char line[256];  // 全フィールドが最長でも約 210 字（reason は 23 字まで）。切れると改行が落ちて次の行とつながる
  snprintf(line, sizeof(line),
           "ENC motor=%u count=%lld pos=%ld cps=%d a=%d b=%d origin=%d cal=%d stroke=%ld neutral=%ld sign=%d auto=%s "
           "phase=%s reason=%s\n",
           static_cast<unsigned>(now.profile.motor), static_cast<long long>(now.raw_count),
           static_cast<long>(now.position), now.count_per_s, now.a_level, now.b_level, now.origin_valid ? 1 : 0,
           now.profile.calibrated ? 1 : 0, static_cast<long>(now.profile.stroke),
           static_cast<long>(now.profile.neutral), static_cast<int>(now.working_sign), auto_kind_name(now.kind),
           auto_phase_name(now.phase), now.reason);
  txLine(line);
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
  bool invalid = false;
  const MotorProfile p = profile_store_load(motor, &invalid);
  if (invalid) Serial.printf("WARN profile_invalid motor=%u（NVS の保存が壊れている。既定値。CAL で上書き）\n", static_cast<unsigned>(motor));
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
  Serial.println("  USBDIAG       USB-Serial/JTAG state line (same as the bottom line on screen, Issue #11)");
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
  if (strcasecmp(cmd, "USBDIAG") == 0) {
    publishUsbDiag();
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
    usb_diag_note_rx(1);
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
        // 系統が切り替わらなかった理由をシリアルにも残す（画面の通知は 1.2 s で消え、校正を別の系統に保存してしまう）
        Serial.printf("UI motor_disabled motor=%u relay=%d active=%d\n", static_cast<unsigned>(snap.profile.motor),
                      relay_on ? 1 : 0, snapActive() ? 1 : 0);
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
        // 無効なボタンが押された理由をシリアルにも残す（画面の通知は 1.5 s で消える）
        Serial.printf("UI auto_disabled live=%d brake=%d speed=%d origin=%d cal=%d active=%d\n", outputLive() ? 1 : 0,
                      brake_on ? 1 : 0, requested_speed, snap.origin_valid ? 1 : 0, snap.profile.calibrated ? 1 : 0,
                      snapActive() ? 1 : 0);
        Serial.flush();
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

  // USB 診断行（Issue #11）。6x8 の ASCII フォントで 1 行。SOF が止まる／IN token が来ない／OUT FIFO 残りを色で示す。
  canvas.drawLine(6, 217, width - 6, 217, kAmber);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(usbDiagColor(usb_diag), TFT_BLACK);
  canvas.drawString(usb_diag_text, 8, 222);

  canvas.setFont(&fonts::lgfxJapanGothic_12);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("DIR", 8, 233);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString(digitalRead(PIN_MD13S_DIR) ? "H" : "L", 34, 233);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("PWR", 50, 233);
  canvas.setTextColor(power_high ? kGreen : kGray, TFT_BLACK);
  canvas.drawString(power_high ? "H" : "L", 80, 233);
  canvas.setTextColor(kOrange, TFT_BLACK);
  canvas.drawString("BRK", 96, 233);
  canvas.setTextColor(brake_high ? kGreen : kGray, TFT_BLACK);
  canvas.drawString(brake_high ? "H" : "L", 124, 233);
  canvas.setTextColor(kOrange, TFT_BLACK);
  char stroke_text[40];
  snprintf(stroke_text, sizeof(stroke_text), "行程 %ld 中立 %ld %+d", static_cast<long>(snap.profile.stroke),
           static_cast<long>(snap.profile.neutral), static_cast<int>(snap.working_sign));
  canvas.setTextDatum(middle_right);
  canvas.drawString(stroke_text, width - 8, 233);

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
  // 校正結果の保存。失敗したら dirty を残して 2 s 後に再試行する（Issue #15 M6。以前は成否に関わらず下ろしていた）
  if (now.profile_dirty && static_cast<int32_t>(millis() - next_profile_save_ms) >= 0) {
    if (profile_store_save(now.profile)) {
      Serial.printf("AUTO saved motor=%u stroke=%ld neutral=%ld sign=%d\n", static_cast<unsigned>(now.profile.motor),
                    static_cast<long>(now.profile.stroke), static_cast<long>(now.profile.neutral),
                    static_cast<int>(now.profile.sign));
      control_clear_profile_dirty();
      next_profile_save_ms = millis() + 200;
    } else {
      Serial.println("ERR nvs_save retry_in=2000ms");
      next_profile_save_ms = millis() + 2000;
    }
  }
}

}  // namespace

void setup() {
  boardBegin();
  motor_output_begin();
  if (!encoder_begin()) {
    Serial.println("ERR pcnt");
  }
  if (!profile_store_begin()) {
    Serial.println("ERR nvs");
  }
  {
    bool invalid = false;
    const uint8_t motor = profile_store_selected_motor();
    const MotorProfile initial = profile_store_load(motor, &invalid);
    if (invalid) Serial.printf("WARN profile_invalid motor=%u（NVS の保存が壊れている。既定値で起動。CAL で上書き）\n", static_cast<unsigned>(motor));
    control_begin(initial);
  }

  if (!spriteBegin()) {
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
  {
    // Issue #11: USB-Serial/JTAG の状態（500 ms ごと）。bus reset はシリアルにも出すが、直後は HWCDC が
    // connected=false なので TX リングの置き換え経路に入り、ホストが戻る前に ENC/STAT 行に押し出されて届かないことが
    // ある（tx! にも出ない）。確実なのは画面の br。
    bool bus_reset_seen = false;
    if (usbDiagTick(&bus_reset_seen)) {
      ui_dirty = true;
    }
    if (bus_reset_seen) {
      char line[80];
      snprintf(line, sizeof(line), "WARN usb_bus_reset n=%lu\n", static_cast<unsigned long>(usb_diag.bus_resets));
      txLine(line);
    }
  }

  if (ui_dirty) {
    render();
    ui_dirty = false;
  }
}

#else  // MC_PROTO_V1

// ---------------------------------------------------------------------------------
// USB プロトコル v1（h3-control-server 接続）。USB CDC はバイナリ専用でテキストを一切出さない。
// 画面は状態の表示と、系統選択（DISARMED のときだけ）・非常停止（ラッチ / 解除）だけ。
// 手動の電源・ブレーキ・速度・自動移動のボタンは出さない（校正・端当ては従来 env で行う）。
// ---------------------------------------------------------------------------------

namespace {

V1Snapshot vsnap;
uint8_t saved_motor = 1;          // NVS に保存済みの系統。スナップショットの selected_axis が変わったら保存する
bool ui_dirty = true;
uint32_t next_snap_ms = 0;
uint32_t next_render_ms = 0;
uint32_t last_press_ms = 0;
uint32_t next_profile_save_ms = 0;  // NVS 保存に失敗したら 2 s 後に再試行（Issue #15 M6）
uint32_t nvs_save_failures = 0;     // 連続して失敗した回数（画面の「保存待ち」に添える。成功で 0）
uint8_t profile_invalid_mask = 0;   // bit n = 系統 n の NVS プロファイルが壊れていて既定値で起動した（保存し直すまで表示）
char notice[32];
uint32_t notice_until_ms = 0;
char boot_error[24] = "";         // pcnt / nvs / sprite / link の初期化失敗（USB には出さないので画面へ）

enum class V1Action : uint8_t { Motor, Estop };

struct V1Button {
  int x;
  int y;
  int w;
  int h;
  V1Action action;
};

constexpr int kMargin = 6;
V1Button v1_buttons[2];

void showNotice(const char* text) {
  snprintf(notice, sizeof(notice), "%s", text);
  notice_until_ms = millis() + 1500;
  ui_dirty = true;
}

void layoutV1(int width, int height) {
  v1_buttons[0] = {kMargin, 2, 48, 24, V1Action::Motor};
  v1_buttons[1] = {kMargin, height - 42, width - kMargin * 2, 36, V1Action::Estop};  // 上に USB 診断行と制御項の行を置く
}

const V1Button* hitV1(int x, int y) {
  for (const V1Button& b : v1_buttons) {
    if (x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) return &b;
  }
  return nullptr;
}

const char* stateName(uint8_t s) {
  switch (s) {
    case v1::state::BOOT: return "BOOT";
    case v1::state::DISARMED: return "DISARMED";
    case v1::state::ARMING: return "ARMING";
    case v1::state::HOLDING: return "HOLDING";
    case v1::state::ACTIVE: return "ACTIVE";
    case v1::state::STOPPING: return "STOPPING";
    case v1::state::DISARMING: return "DISARMING";
    case v1::state::FAULT: return "FAULT";
    case v1::state::CALIBRATING: return "CALIBRATING";
  }
  return "?";
}

const char* calStepName(uint8_t s) {
  switch (s) {
    case v1::cal_step::TO_P: return "P端へ";
    case v1::cal_step::TO_N: return "N端へ";
    case v1::cal_step::TO_NEUTRAL: return "中立へ";
  }
  return "";
}

const char* calResultName(uint8_t r) {
  switch (r) {
    case v1::cal_result::OK: return "OK";
    case v1::cal_result::FAILED: return "FAILED";
    case v1::cal_result::CANCELLED: return "CANCELLED";
  }
  return "?";
}

const char* faultName(uint16_t f) {
  switch (f) {
    case v1::fault::NONE: return "NONE";
    case v1::fault::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
    case v1::fault::FOLLOWING_ERROR: return "FOLLOWING_ERROR";
    case v1::fault::OVERSPEED: return "OVERSPEED";
    case v1::fault::SOFT_LIMIT: return "SOFT_LIMIT";
    case v1::fault::ENCODER_FAULT: return "ENCODER_FAULT";
    case v1::fault::CONTROL_OVERRUN: return "CONTROL_OVERRUN";
    case v1::fault::CONFIG_INVALID: return "CONFIG_INVALID";
    case v1::fault::STOP_TIMEOUT: return "STOP_TIMEOUT";
    case v1::fault::POWER_FAULT: return "POWER_FAULT";
    case v1::fault::HOST_FAULT: return "HOST_FAULT";
    case v1::fault::NOT_REFERENCED: return "NOT_REFERENCED";
    case v1::fault::LOCAL_ESTOP: return "LOCAL_ESTOP";
  }
  return "UNKNOWN";
}

uint16_t stateColor(uint8_t s) {
  switch (s) {
    case v1::state::HOLDING:
    case v1::state::ACTIVE: return kGreen;
    case v1::state::ARMING:
    case v1::state::STOPPING:
    case v1::state::DISARMING:
    case v1::state::CALIBRATING: return kAmber;
    case v1::state::FAULT: return kRed;
    default: return kGray;
  }
}

void onV1Button(const V1Button& b) {
  switch (b.action) {
    case V1Action::Motor:
      if (vsnap.state != v1::state::DISARMED) {
        showNotice("DISARMEDで切替");
        return;
      }
      control_v1_select_axis(static_cast<uint8_t>(vsnap.selected_axis % 3 + 1));
      break;
    case V1Action::Estop:
      control_v1_set_estop(!vsnap.estop_latched);
      showNotice(vsnap.estop_latched ? "非常停止を解除" : "非常停止");
      break;
  }
}

// 画面の非常停止は、押し続けたときだけ効かせる。2026-11-02 19:15 / 19:36 に、誰も触れていないのに #2 で
// 非常停止が入り（タッチの誤検出とみられる）、1 回のタップで入れる・外すが切り替わる作りだったので、誤検出で外れる恐れもあった。
// 入れるのは 300 ms、外すのは 1500 ms 押し続けたとき。途中で指が離れるかボタンの外へ出たら取り消す。
// 本命の非常停止は物理ボタン（主電源の遮断）で、これは補助。
constexpr uint32_t V1_ESTOP_LATCH_HOLD_MS = 300;
constexpr uint32_t V1_ESTOP_RELEASE_HOLD_MS = 1500;
bool estop_press_active = false;
bool estop_press_done = false;
uint32_t estop_press_start_ms = 0;

void pollTouchV1() {
  const auto touch = M5.Touch.getDetail();
  const uint32_t now = millis();
  if (estop_press_active) {
    const V1Button* held = touch.isPressed() ? hitV1(touch.x, touch.y) : nullptr;
    if (held == nullptr || held->action != V1Action::Estop) {
      estop_press_active = false;
      return;
    }
    const uint32_t need = vsnap.estop_latched ? V1_ESTOP_RELEASE_HOLD_MS : V1_ESTOP_LATCH_HOLD_MS;
    if (!estop_press_done && now - estop_press_start_ms >= need) {
      estop_press_done = true;
      onV1Button(*held);
    }
    return;
  }
  if (!touch.wasPressed()) return;
  // 同じ押下で wasPressed が 2 回成立することがある（実機で確認）。250 ms 以内の再押下は無視する。
  if (now - last_press_ms < 250) return;
  last_press_ms = now;
  const V1Button* b = hitV1(touch.x, touch.y);
  if (b == nullptr) return;
  if (b->action == V1Action::Estop) {
    estop_press_active = true;
    estop_press_done = false;
    estop_press_start_ms = now;
    showNotice(vsnap.estop_latched ? "1.5 秒押し続けて解除" : "押し続けて非常停止");
    return;
  }
  onV1Button(*b);
}

void renderV1() {
  if (!sprite_ok) return;
  const int width = canvas.width();
  const bool fault = vsnap.state == v1::state::FAULT;
  const bool disarmed = vsnap.state == v1::state::DISARMED;

  canvas.fillSprite(TFT_BLACK);
  canvas.fillRect(0, 0, width, 28, kDarkRed);
  canvas.drawLine(0, 28, width, 28, kOrange);
  canvas.setFont(&fonts::lgfxJapanGothic_16);

  // 系統ボタン
  {
    const V1Button& b = v1_buttons[0];
    const uint16_t edge = disarmed ? kOrange : kGray;
    canvas.fillRect(b.x, b.y, b.w, b.h, disarmed ? kDarkOrange : TFT_BLACK);
    canvas.drawRect(b.x, b.y, b.w, b.h, edge);
    canvas.setTextColor(edge, disarmed ? kDarkOrange : TFT_BLACK);
    canvas.setTextDatum(middle_center);
    char motor_label[8];
    snprintf(motor_label, sizeof(motor_label), "#%u", static_cast<unsigned>(vsnap.selected_axis));
    canvas.drawString(motor_label, b.x + b.w / 2, b.y + b.h / 2);
  }
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(kOrange, kDarkRed);
  canvas.drawString("H3 v1", 62, 14);
  canvas.setTextDatum(middle_right);
  if (notice[0] != '\0') {
    canvas.setTextColor(kRed, kDarkRed);
    canvas.drawString(notice, width - 8, 14);
  } else {
    char id_text[24];
    snprintf(id_text, sizeof(id_text), "dev %04X", static_cast<unsigned>(vsnap.device_id & 0xFFFF));
    canvas.setTextColor(TFT_WHITE, kDarkRed);
    canvas.drawString(id_text, width - 8, 14);
  }

  // 状態（FAULT は赤で異常コード名）
  canvas.setFont(&fonts::Font4);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(stateColor(vsnap.state), TFT_BLACK);
  char state_text[40];
  if (fault) {
    snprintf(state_text, sizeof(state_text), "FAULT %s", faultName(vsnap.fault_code));
  } else if (vsnap.state == v1::state::CALIBRATING) {
    snprintf(state_text, sizeof(state_text), "CALIBRATING %s", calStepName(vsnap.cal_live_step));
  } else if (vsnap.fault_code != v1::fault::NONE) {
    snprintf(state_text, sizeof(state_text), "%s %s", stateName(vsnap.state), faultName(vsnap.fault_code));
  } else {
    snprintf(state_text, sizeof(state_text), "%s", stateName(vsnap.state));
  }
  canvas.drawString(state_text, 8, 46);

  // 指令行
  canvas.setFont(&fonts::lgfxJapanGothic_16);
  char cmd_text[72];
  char boost_text[12] = "";
  if (vsnap.stall_boost_permille > 0) {  // 停止ブースト（Issue #15 H4）: 下限への上乗せ [%]。STATUS には出ないので画面だけ
    snprintf(boost_text, sizeof(boost_text), "(+%d.%d)", vsnap.stall_boost_permille / 10, vsnap.stall_boost_permille % 10);
  }
  // Duty は permille を % で小数 1 桁（Issue #25 手順 8 b）。MD13S の向きの符号
  const int duty_abs = vsnap.duty_permille < 0 ? -vsnap.duty_permille : vsnap.duty_permille;
  snprintf(cmd_text, sizeof(cmd_text), "電源 %s  ブレーキ %s  Duty %s%d.%d%%%s", vsnap.power_on_cmd ? "入" : "切",
           vsnap.brake_release_cmd ? "解除" : "作動", vsnap.duty_permille < 0 ? "-" : "+", duty_abs / 10, duty_abs % 10,
           boost_text);
  canvas.setTextColor(vsnap.power_on_cmd || vsnap.brake_release_cmd ? kGreen : kOrange, TFT_BLACK);
  canvas.drawString(cmd_text, 8, 70);

  // カウント行
  canvas.fillRect(kMargin, 84, width - kMargin * 2, 30, kDarkOrange);
  canvas.drawRect(kMargin, 84, width - kMargin * 2, 30, kOrange);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(vsnap.referenced ? TFT_WHITE : kAmber, kDarkOrange);
  char pos_text[16];
  snprintf(pos_text, sizeof(pos_text), "%ld", static_cast<long>(vsnap.actual));
  canvas.drawString(pos_text, 12, 99);
  canvas.setFont(&fonts::lgfxJapanGothic_12);
  canvas.setTextDatum(middle_right);
  char motion_text[64];
  snprintf(motion_text, sizeof(motion_text), "目標 %ld  誤差 %ld  %ld c/s", static_cast<long>(vsnap.target),
           static_cast<long>(vsnap.following_error), static_cast<long>(vsnap.velocity));
  canvas.setTextColor(kOrange, kDarkOrange);
  canvas.drawString(motion_text, width - 12, 99);

  // フラグ行
  canvas.setTextDatum(middle_left);
  canvas.setFont(&fonts::lgfxJapanGothic_12);
  char flag_text[96];
  snprintf(flag_text, sizeof(flag_text), "%s  %s  %s%s", vsnap.configured ? "CONFIG 済" : "CONFIG 未",
           vsnap.referenced ? "基準 済" : "基準 未",
           (vsnap.flags & v1::flags::POWER_CONTROLLER) ? "電源担当" : "電源担当なし",
           vsnap.encoder_fault_check_disabled ? "  無パルス検出なし" : "");  // duty_max < 10 %（Issue #15 M2）
  canvas.setTextColor(vsnap.configured && vsnap.referenced && !vsnap.encoder_fault_check_disabled ? kGreen : kAmber,
                      TFT_BLACK);
  canvas.drawString(flag_text, 8, 124);

  // リンク行
  char link_text[80];
  if (vsnap.has_rx) {
    // df = TX リングに空きが無く次の poll に回した回数、old = 古い STATUS／期限切れの保留を捨てた数（Issue #15 M7）
    snprintf(link_text, sizeof(link_text), "PC %lums前 rx_err %u stale %u tx_drop %lu df %lu old %lu",
             static_cast<unsigned long>(vsnap.last_rx_age_ms), vsnap.rx_error_count, vsnap.stale_drop_count,
             static_cast<unsigned long>(vsnap.tx_dropped), static_cast<unsigned long>(vsnap.tx_deferred),
             static_cast<unsigned long>(vsnap.tx_stale_dropped));
    canvas.setTextColor(vsnap.last_rx_age_ms < 500 ? kGreen : kAmber, TFT_BLACK);
  } else {
    snprintf(link_text, sizeof(link_text), "PC 未受信  rx_err %u", vsnap.rx_error_count);
    canvas.setTextColor(kGray, TFT_BLACK);
  }
  canvas.drawString(link_text, 8, 140);

  // 校正行: 校正中は段階と AutoMachine の理由、終わっていれば最後の CAL_RESULT（結果 / 理由 / N 端 / 中立）
  char cal_text[80];
  if (vsnap.state == v1::state::CALIBRATING) {
    snprintf(cal_text, sizeof(cal_text), "校正中 %s %s", calStepName(vsnap.cal_live_step), vsnap.cal_live_reason);
    canvas.setTextColor(kAmber, TFT_BLACK);
  } else if (vsnap.cal_result_valid) {
    char pending[24] = "";
    if (vsnap.profile_dirty) {  // 保存待ち。失敗が続いていれば回数（Issue #15 M6）
      if (nvs_save_failures > 0) {
        snprintf(pending, sizeof(pending), "  保存失敗 %lu", static_cast<unsigned long>(nvs_save_failures));
      } else {
        snprintf(pending, sizeof(pending), "  保存待ち");
      }
    }
    snprintf(cal_text, sizeof(cal_text), "校正 %s %s  N端 %ld  中立 %ld  sign %+d%s", calResultName(vsnap.cal_result),
             vsnap.cal_reason, static_cast<long>(vsnap.cal_n_end), static_cast<long>(vsnap.cal_neutral),
             static_cast<int>(vsnap.cal_sign), pending);
    canvas.setTextColor(vsnap.cal_result == v1::cal_result::OK && nvs_save_failures == 0 ? kGreen : kAmber, TFT_BLACK);
  } else if (profile_invalid_mask) {
    // NVS のプロファイルが壊れていた系統（既定値 = 未校正・sign +1 で起動している。校正して保存し直すまで出す）
    snprintf(cal_text, sizeof(cal_text), "校正 未実施  NVS 破損→既定値:%s%s%s", (profile_invalid_mask & 0x02) ? " #1" : "",
             (profile_invalid_mask & 0x04) ? " #2" : "", (profile_invalid_mask & 0x08) ? " #3" : "");
    canvas.setTextColor(kRed, TFT_BLACK);
  } else {
    snprintf(cal_text, sizeof(cal_text), "校正 この起動では未実施");
    canvas.setTextColor(kGray, TFT_BLACK);
  }
  canvas.drawString(cal_text, 8, 155);

  // tick 行（周期の守りの確認用）
  char tick_text[64];
  snprintf(tick_text, sizeof(tick_text), "tick %lu  最大間隔 %lu us%s%s", static_cast<unsigned long>(vsnap.tick_count),
           static_cast<unsigned long>(vsnap.max_tick_gap_us), boot_error[0] ? "  ERR " : "", boot_error);
  canvas.setTextColor(boot_error[0] ? kRed : kGray, TFT_BLACK);
  canvas.drawString(tick_text, 8, 170);

  // USB 診断行（Issue #11）。6x8 の ASCII フォント。意味は usb_diag_format() のコメント。
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(usbDiagColor(usb_diag), TFT_BLACK);
  canvas.drawString(usb_diag_text, 8, 182);

  // 制御項の行（Issue #25 手順 8）。単位は permille（0.1 %）、符号はカウントが増える向きが正（STATUS の pwm_permille と同じ）。
  // FF = 軌道速度 / ff、P = 誤差 × kp / 100、I = 積分（保持バイアス。Issue #25 手順 9「F」）、V = 軌道の目標速度 [count/s]、
  // F = 床の適用状態（1 = ON。Issue #25 手順 8 c）、A = 追従中の動き出し補助（1 = 床を掛けている。Issue #25 手順 9「T」）、
  // B = 停止ブーストの上乗せ、
  // O = 実際に出している値（pwm_permille）。通常 ≤ 50 文字（6 px × 53 = 318 < 320。超えれば切れる）
  // Issue #25 r6（V1_FF_DIRECTIONAL）: `FF +76 P +12 I +8 V +300 MOVE O +96`。F / A / B の代わりに状態 MOVE / HOLD / BRK。FF は方向別
  // r9（Issue #35、V1_VEL_DAMPING）: 状態の後に減衰項 D（u_d、±10 permille）を足す: `FF +76 P +12 I +8 V +300 MOVE D -3 O +93`。
  // 速い速度推定 v_fast は行に入らない（53 文字を超える）ので V1Snapshot.terms.v_fast_count_s にだけ置く
  {
    const V1DutyTerms& tm = vsnap.terms;
    char terms_text[64];
    // Issue #25 r6（V1_FF_DIRECTIONAL）: 床・補助・ブーストが無いので代わりに状態 M（MOVE / HOLD / BRK = 動き出しパルス）を出す
    const char* mode = tm.mode == V1DutyMode::HOLD ? "HOLD" : (tm.mode == V1DutyMode::BRK ? "BRK" : "MOVE");
    if (V1_FF_DIRECTIONAL) {
      snprintf(terms_text, sizeof(terms_text), "FF %+d P %+d I %+d V %+d %s D %+d O %+d", static_cast<int>(tm.ff_permille),
               static_cast<int>(tm.p_permille), static_cast<int>(tm.i_permille), static_cast<int>(tm.vel_count_s), mode,
               static_cast<int>(tm.d_permille), static_cast<int>(vsnap.pwm_permille));
    } else {
      snprintf(terms_text, sizeof(terms_text), "FF %+d P %+d I %+d V %+d F%d A%d B %d O %+d", static_cast<int>(tm.ff_permille),
               static_cast<int>(tm.p_permille), static_cast<int>(tm.i_permille), static_cast<int>(tm.vel_count_s),
               tm.floor_on ? 1 : 0, tm.assist_on ? 1 : 0, static_cast<int>(tm.boost_permille),
               static_cast<int>(vsnap.pwm_permille));
    }
    canvas.setTextColor(tm.in_band ? kGray : ((tm.floor_on || tm.assist_on || tm.mode == V1DutyMode::BRK) ? kAmber : kGreen),
                        TFT_BLACK);
    canvas.drawString(terms_text, 8, 192);
  }

  // 非常停止ボタン（ラッチ中は「解除」）
  {
    const V1Button& b = v1_buttons[1];
    const uint16_t fill = vsnap.estop_latched ? kRed : kDarkRed;
    canvas.fillRect(b.x, b.y, b.w, b.h, fill);
    canvas.drawRect(b.x, b.y, b.w, b.h, kRed);
    canvas.setFont(&fonts::lgfxJapanGothic_24);
    canvas.setTextColor(TFT_WHITE, fill);
    canvas.setTextDatum(middle_center);
    canvas.drawString(vsnap.estop_latched ? "非常停止 解除" : "非常停止", b.x + b.w / 2, b.y + b.h / 2);
  }

  canvas.pushSprite(0, 0);
}

bool snapChanged(const V1Snapshot& a, const V1Snapshot& b) {
  return a.state != b.state || a.fault_code != b.fault_code || a.flags != b.flags ||
         a.selected_axis != b.selected_axis || a.power_on_cmd != b.power_on_cmd ||
         a.brake_release_cmd != b.brake_release_cmd || (a.duty_permille / 10) != (b.duty_permille / 10) ||
         a.stall_boost_permille != b.stall_boost_permille ||
         // 制御項は 5 ms ごとに変わり得る。1 %（10 permille）刻みで見る（描き直しが受信を止めるのは df / old と同じ）
         (a.terms.ff_permille / 10) != (b.terms.ff_permille / 10) || (a.terms.p_permille / 10) != (b.terms.p_permille / 10) ||
         (a.terms.i_permille / 10) != (b.terms.i_permille / 10) || a.terms.assist_on != b.terms.assist_on ||
         (a.terms.vel_count_s / 50) != (b.terms.vel_count_s / 50) || a.terms.floor_on != b.terms.floor_on ||
         a.terms.in_band != b.terms.in_band || a.terms.boost_permille != b.terms.boost_permille || a.terms.mode != b.terms.mode ||
         (a.terms.d_permille / 5) != (b.terms.d_permille / 5) ||  // r9 の減衰項（±10 permille）は 0.5 % 刻みで見る
         (a.pwm_permille / 10) != (b.pwm_permille / 10) ||
         a.encoder_fault_check_disabled != b.encoder_fault_check_disabled || a.actual != b.actual ||
         a.target != b.target || a.velocity != b.velocity || a.following_error != b.following_error ||
         a.configured != b.configured || a.referenced != b.referenced || a.estop_latched != b.estop_latched ||
         a.has_rx != b.has_rx || a.rx_error_count != b.rx_error_count || a.stale_drop_count != b.stale_drop_count ||
         a.tx_dropped != b.tx_dropped ||
         // df / old は背圧の間 poll ごとに増える（約 1 kHz）。毎回描き直すと描画自体が受信を止めるので 50 刻みで
         (a.tx_deferred / 50) != (b.tx_deferred / 50) || (a.tx_stale_dropped / 50) != (b.tx_stale_dropped / 50) ||
         (a.last_rx_age_ms / 100) != (b.last_rx_age_ms / 100) ||
         (a.tick_count / 200) != (b.tick_count / 200) || a.cal_live_step != b.cal_live_step ||
         strcmp(a.cal_live_reason, b.cal_live_reason) != 0 || a.cal_result_valid != b.cal_result_valid ||
         a.cal_result != b.cal_result || a.cal_n_end != b.cal_n_end || a.cal_neutral != b.cal_neutral ||
         a.profile_dirty != b.profile_dirty;
}

void noteBootError(const char* what) {
  if (boot_error[0] == '\0') {
    snprintf(boot_error, sizeof(boot_error), "%s", what);
  } else {
    char merged[24];
    snprintf(merged, sizeof(merged), "%s,%s", boot_error, what);
    snprintf(boot_error, sizeof(boot_error), "%s", merged);
  }
}

}  // namespace

void setup() {
  // SET_POSITION は 100 Hz × 44 byte。画面の pushSprite（数十 ms）の間に HWCDC の既定 256 byte が溢れないよう広げる。
  // begin() より前でないと効かない。
  Serial.setRxBufferSize(1024);
  // TX リングも広げる（既定 256 byte ≈ STATUS 5 本）。書く前に空きを見て入らなければ次の poll に回す（Issue #15 M7、v1_tx_pump.h）
  // ので、広いほど PC が一瞬読まないときの保留・STATUS 捨てが減る。
  Serial.setTxBufferSize(V1_TX_RING_BYTES);
  // USB CDC は v1 のバイナリ専用。IDF / M5Unified のログも出さない（COBS の区切り 0x00 と混ざる）。
  // M5.begin() より前に止める。Arduino core の log_e() は CORE_DEBUG_LEVEL=0（platformio.ini）でコンパイル時に消す。
  esp_log_level_set("*", ESP_LOG_NONE);
  boardBegin();
  // v1 は書く前に TX リングの空きを見る（v1_tx_pump.h）ので通常 write は待たない。保留が続いて空きを見ずに書く（WriteThrough）
  // ときだけ HWCDC が tx timeout × 最大 20 回待つ。boardBegin の 20 ms（従来 env 向け）だと 400 ms になるので 1 ms（最大 20 ms）にする。
  Serial.setTxTimeoutMs(V1_TX_TIMEOUT_MS);
  motor_output_begin();
  if (!encoder_begin()) {
    noteBootError("pcnt");
  }
  if (!profile_store_begin()) {
    noteBootError("nvs");
  }
  if (!v1_link_begin()) {
    noteBootError("link");
  }
  saved_motor = profile_store_selected_motor();
  MotorProfile profiles[4];
  for (uint8_t m = 1; m <= 3; ++m) {
    bool invalid = false;
    profiles[m] = profile_store_load(m, &invalid);
    if (invalid) profile_invalid_mask |= static_cast<uint8_t>(1u << m);
  }
  if (profile_invalid_mask) noteBootError("prof");  // 壊れた系統は校正行に出す。校正して保存し直せば消える
  control_v1_begin(profiles, saved_motor);
  vsnap.selected_axis = saved_motor;

  if (!spriteBegin()) {
    noteBootError("sprite");
  }
  layoutV1(M5.Display.width(), M5.Display.height());
  ui_dirty = true;
}

void loop() {
  M5.update();
  v1_link_poll();  // USB の受信 → 受信キュー、送信キュー → USB（描画の前後で呼び、描画中の取りこぼしを減らす）
  pollTouchV1();

  if (notice[0] != '\0' && static_cast<int32_t>(millis() - notice_until_ms) >= 0) {
    notice[0] = '\0';
    ui_dirty = true;
  }

  if (static_cast<int32_t>(millis() - next_snap_ms) >= 0) {
    next_snap_ms = millis() + 50;
    const V1Snapshot previous = vsnap;
    vsnap = control_v1_snapshot();
    // 系統の選択は制御タスク（V1Axis）が DISARMED のときだけ受け付ける。受け付けられた値だけ NVS に保存する。
    if (vsnap.selected_axis != saved_motor && profile_motor_ok(vsnap.selected_axis)) {
      profile_store_select_motor(vsnap.selected_axis);
      saved_motor = vsnap.selected_axis;
    }
    // 校正に成功した系統プロファイル（行程・中立・符号）を NVS に保存する。従来 env と同じく loop() 側で書く
    //（制御タスクで Preferences を書くと 5 ms 周期が崩れる）。失敗したら flag を残し 2 s 後に再試行する
    //（Issue #15 M6。以前は成否に関わらず下ろしていた）。保存は 1 キーの blob なので途中で切れても混ざらない（profile_blob.h）。
    if (vsnap.profile_dirty && static_cast<int32_t>(millis() - next_profile_save_ms) >= 0) {
      if (profile_store_save(vsnap.profile)) {
        showNotice("校正を保存");
        control_v1_clear_profile_dirty();
        nvs_save_failures = 0;
        if (profile_motor_ok(vsnap.profile.motor)) profile_invalid_mask &= static_cast<uint8_t>(~(1u << vsnap.profile.motor));
        next_profile_save_ms = millis() + 200;
      } else {
        nvs_save_failures += 1;
        showNotice("NVS 保存失敗 再試行");
        next_profile_save_ms = millis() + 2000;
        ui_dirty = true;
      }
    }
    if (snapChanged(previous, vsnap)) {
      ui_dirty = true;
    }
  }
  {
    // Issue #11: USB-Serial/JTAG の状態（500 ms ごと）。v1 の USB はバイナリ専用なので bus reset はログに出さない。
    bool bus_reset_seen = false;
    if (usbDiagTick(&bus_reset_seen)) {
      ui_dirty = true;
    }
  }

  // 描画は 100 ms に 1 回まで（pushSprite は数十 ms。count が毎 tick 変わると描画し続けてしまう）
  if (ui_dirty && static_cast<int32_t>(millis() - next_render_ms) >= 0) {
    next_render_ms = millis() + 100;
    renderV1();
    ui_dirty = false;
    v1_link_poll();
  }
}

#endif  // MC_PROTO_V1
