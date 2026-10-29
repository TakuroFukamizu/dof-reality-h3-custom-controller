#include <M5Unified.h>

#include <cstdlib>
#include <cstring>
#include <strings.h>

#include <driver/gpio.h>

#include "config.h"
#include "pmic_log.h"

namespace {

constexpr uint32_t kDutyMax = (1u << PWM_BITS) - 1;

int requested_speed = 0;
int applied_speed = 0;
bool relay_on = false;
bool brake_on = false;
uint32_t relay_on_ms = 0;
bool was_live = false;
bool ui_dirty = true;
bool sprite_ok = false;

char line_buf[64];
size_t line_len = 0;
uint32_t next_repeat_ms = 0;

M5Canvas canvas(&M5.Display);

int relayOffLevel() { return RELAY_ON_LEVEL ? 0 : 1; }

int oppositeLevel(int level) { return level ? 0 : 1; }

bool outputLive() {
  return relay_on && (millis() - relay_on_ms >= RELAY_SETTLE_MS);
}

void driveOutput(gpio_num_t pin, int level) {
  gpio_set_level(pin, level);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  gpio_set_level(pin, level);
}

void writePwm(uint32_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_MD13S_PWM, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
}

void setupPwm() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(PIN_MD13S_PWM, PWM_FREQ_HZ, PWM_BITS);
  const uint32_t actual_hz = PWM_FREQ_HZ;
#else
  const uint32_t actual_hz = ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(PIN_MD13S_PWM, PWM_CHANNEL);
#endif
  writePwm(0);
  Serial.printf("pwm %u Hz requested, timer %u Hz, %u-bit\n", PWM_FREQ_HZ,
                actual_hz, PWM_BITS);
}

// 反転するときだけ PWM を落としてから DIR を変える。同じ出力なら戻る。
void writeMotor(int percent) {
  if (percent == applied_speed) {
    return;
  }
  percent = constrain(percent, SPEED_MIN, SPEED_MAX);

  const int new_dir =
      (percent >= 0) ? DIR_POSITIVE_LEVEL : oppositeLevel(DIR_POSITIVE_LEVEL);
  const int old_dir = (applied_speed >= 0) ? DIR_POSITIVE_LEVEL
                                           : oppositeLevel(DIR_POSITIVE_LEVEL);
  const bool reversing =
      percent != 0 && applied_speed != 0 && new_dir != old_dir;

  if (reversing || percent == 0) {
    writePwm(0);
  }
  if (reversing) {
    delay(DIR_COAST_MS);
  }
  if (percent != 0) {
    digitalWrite(PIN_MD13S_DIR, new_dir);
    const int magnitude = percent < 0 ? -percent : percent;
    writePwm(static_cast<uint32_t>(magnitude) * kDutyMax / 100u);
  }

  applied_speed = percent;
  ui_dirty = true;
}

void applyOutput() { writeMotor(outputLive() ? requested_speed : 0); }

void publish(const char* prefix) {
  Serial.printf("%s R=%d B=%d S=%d OUT=%d DIR=%d PWR_PIN=%d BRK_PIN=%d LIVE=%d\n",
                prefix, relay_on ? 1 : 0, brake_on ? 1 : 0, requested_speed,
                applied_speed, digitalRead(PIN_MD13S_DIR),
                digitalRead(PIN_POWER_RELAY), digitalRead(PIN_MOTOR_BRAKE),
                outputLive() ? 1 : 0);
}

void setRelay(bool on) {
  if (on == relay_on) {
    publish("OK");
    return;
  }
  if (!on) {
    requested_speed = 0;
    writeMotor(0);
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
  applyOutput();
  publish("OK");
  ui_dirty = true;
  return true;
}

void estop() {
  requested_speed = 0;
  writeMotor(0);
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
  Serial.println("  ? or STATUS");
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
    return;
  }
  if (strcasecmp(cmd, "X") == 0 || strcasecmp(cmd, "STOP") == 0) {
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

}  // namespace

void setup() {
  driveOutput(static_cast<gpio_num_t>(PIN_POWER_RELAY), relayOffLevel());
  driveOutput(static_cast<gpio_num_t>(PIN_MOTOR_BRAKE), relayOffLevel());
  driveOutput(static_cast<gpio_num_t>(PIN_MD13S_DIR), DIR_POSITIVE_LEVEL);
  driveOutput(static_cast<gpio_num_t>(PIN_MD13S_PWM), 0);

  auto cfg = M5.config();
  cfg.clear_display = true;
#if MC_PMIC_LOG
  // Issue #1 の計測は、不具合が出た構成と同じく DIN Base の 5V を受ける（CoreS3 から BUS 5V を出さない）。
  cfg.output_power = false;
#else
  cfg.output_power = true;
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
#if MC_PMIC_LOG
  pmic_log_boot();  // 前回の電源断要因（REG21）を、ほかの処理より先に読む
  M5.Power.setUsbOutput(false);  // CoreS3 から PC 側への USB 給電を無効化（不具合が出た構成と同じ）
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
  setupPwm();

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
  if (live != was_live) {
    was_live = live;
    applyOutput();
    publish("STAT");
    ui_dirty = true;
  }

  if (ui_dirty) {
    render();
    ui_dirty = false;
  }
}
