#include <M5Unified.h>

#include <stdio.h>

#include "config.h"
#include "control.h"
#include "format_util.h"
#include "ui.h"

namespace {

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

enum class Page : uint8_t { Drive, Encoder, Visual, Config };

struct Rect {
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;

  bool hit(int px, int py) const {
    return w > 0 && h > 0 && px >= x && px < x + w && py >= y && py < y + h;
  }
};

M5Canvas canvas(&M5.Display);
bool sprite_ok = false;
Page page = Page::Drive;
Snapshot snap{};
uint32_t next_draw_ms = 0;
bool draw_soon = true;

Rect tabs[4];
Rect estop_rect{};
Rect stop_rect{};
Rect arm_rect{};
Rect jog_pos_rect{};
Rect jog_neg_rect{};
Rect zero_rect{};
Rect vis_ref_rect{};
Rect vis_finish_rect{};
Rect vis_pos_rect{};
Rect vis_neg_rect{};
Rect vis_stop_rect{};
Rect vis_cancel_rect{};

void refresh() { control_copy(&snap); }

void place_row(Rect* out, int count, int y, int h) {
  const int width = canvas.width() > 0 ? canvas.width() : 320;
  const int gap = 4;
  const int margin = 4;
  const int span = width - margin * 2 - gap * (count - 1);
  const int w = count > 0 ? span / count : 0;
  for (int i = 0; i < count; ++i) {
    out[i] = Rect{margin + i * (w + gap), y, w, h};
  }
}

void layout_chrome() {
  const int width = M5.Display.width() > 0 ? M5.Display.width() : 320;
  const int tab_w = width / 4;
  for (int i = 0; i < 4; ++i) {
    tabs[i] = Rect{i * tab_w, 22, tab_w, 20};
  }
  estop_rect = Rect{0, 206, width / 2, 32};
  stop_rect = Rect{width / 2, 206, width - width / 2, 34};
}

void clear_page_rects() {
  arm_rect = Rect{};
  jog_pos_rect = Rect{};
  jog_neg_rect = Rect{};
  zero_rect = Rect{};
  vis_ref_rect = Rect{};
  vis_finish_rect = Rect{};
  vis_pos_rect = Rect{};
  vis_neg_rect = Rect{};
  vis_stop_rect = Rect{};
  vis_cancel_rect = Rect{};
}

void draw_button(const Rect& rect, const char* label, uint16_t fill, uint16_t text) {
  if (rect.w <= 0) {
    return;
  }
  canvas.fillRect(rect.x, rect.y, rect.w, rect.h, fill);
  canvas.drawRect(rect.x, rect.y, rect.w, rect.h, kOrange);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(text, fill);
  canvas.drawString(label, rect.x + rect.w / 2, rect.y + rect.h / 2);
}

void draw_line(int y, const char* text, uint16_t color) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(color, TFT_BLACK);
  canvas.drawString(text, 4, y);
}

uint16_t state_color(DriveState state) {
  if (state == DriveState::Fault) {
    return kRed;
  }
  if (state == DriveState::Running) {
    return kGreen;
  }
  if (state == DriveState::Disarmed) {
    return kGray;
  }
  return kAmber;
}

void draw_header() {
  const int width = canvas.width();
  canvas.fillRect(0, 0, width, 22, kDarkRed);
  canvas.setFont(&fonts::Font2);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(state_color(snap.state), kDarkRed);
  canvas.drawString(drive_state_name(snap.state), 4, 11);
  canvas.setTextDatum(middle_right);
  canvas.setTextColor(kOrange, kDarkRed);
  canvas.drawString(snap.usb_authority ? "USB" : "LOCAL", width - 4, 11);

  canvas.setFont(&fonts::Font0);
  const char* names[4] = {"DRV", "ENC", "VIS", "CFG"};
  for (int i = 0; i < 4; ++i) {
    const bool on = static_cast<int>(page) == i;
    canvas.fillRect(tabs[i].x, tabs[i].y, tabs[i].w, tabs[i].h, on ? kDarkOrange : TFT_BLACK);
    canvas.drawRect(tabs[i].x, tabs[i].y, tabs[i].w, tabs[i].h, kOrange);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(on ? kOrange : kGray, on ? kDarkOrange : TFT_BLACK);
    canvas.drawString(names[i], tabs[i].x + tabs[i].w / 2, tabs[i].y + tabs[i].h / 2);
  }
}

void draw_footer(bool estop_down) {
  const int width = canvas.width();
  canvas.fillRect(0, 188, width, 18, TFT_BLACK);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_left);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  char foot[72];
  snprintf(foot, sizeof(foot), "cnt %lld   %d c/s", static_cast<long long>(snap.count_display), snap.count_per_s);
  canvas.drawString(foot, 4, 197);

  canvas.setFont(&fonts::Font2);
  draw_button(estop_rect, "ESTOP", estop_down ? kRed : kDarkRed, TFT_WHITE);
  draw_button(stop_rect, "STOP", kDarkOrange, kOrange);
}

void draw_drive() {
  char line[80];
  snprintf(line, sizeof(line), "%s  gpio=%d", snap.power_on_cmd ? "POWER ON CMD" : "POWER OFF CMD", snap.power_gpio);
  draw_line(46, line, kOrange);
  snprintf(line, sizeof(line), "%s  gpio=%d", snap.brake_release_cmd ? "BRAKE RELEASE CMD" : "BRAKE APPLY CMD",
           snap.brake_gpio);
  draw_line(58, line, kOrange);
  snprintf(line, sizeof(line), "REQ %+d  APPL %d%%  RAW %d/%d", snap.requested_signed, snap.applied_duty, snap.pwm_raw,
           PWM_FULL_SCALE);
  draw_line(70, line, TFT_WHITE);
  snprintf(line, sizeof(line), "%d Hz  DIR %d", snap.pwm_hz, snap.dir_level);
  draw_line(82, line, TFT_WHITE);
  snprintf(line, sizeof(line), "trial %d  left %lu ms  ready %lu ms", snap.trial_id,
           static_cast<unsigned long>(snap.remaining_ms), static_cast<unsigned long>(snap.ready_remaining_ms));
  draw_line(94, line, TFT_WHITE);
  snprintf(line, sizeof(line), "move %d / %d", snap.travel_abs, snap.max_travel);
  draw_line(106, line, TFT_WHITE);
  if (snap.fault[0] != '\0') {
    snprintf(line, sizeof(line), "FAULT %s", snap.fault);
    draw_line(118, line, kRed);
  } else if (snap.reject[0] != '\0') {
    snprintf(line, sizeof(line), "reject %s", snap.reject);
    draw_line(118, line, kAmber);
  } else if (snap.lock_reason[0] != '\0') {
    snprintf(line, sizeof(line), "ARM lock %s", snap.lock_reason);
    draw_line(118, line, kAmber);
  } else {
    draw_line(118, snap.count_stable ? "count stable" : "count not stable", kGray);
  }
  if (snap.jog_duty >= 1) {
    snprintf(line, sizeof(line), "JOG %d%% of 1023", snap.jog_duty);
  } else {
    snprintf(line, sizeof(line), "JOG DUTY UNSET");
  }
  draw_line(130, line, TFT_WHITE);

  canvas.setFont(&fonts::Font2);
  Rect row[3];
  place_row(row, 3, 146, 40);
  if (snap.state == DriveState::Fault) {
    arm_rect = row[0];
    draw_button(arm_rect, "RST", kDarkRed, TFT_WHITE);
  } else if (snap.lock_reason[0] == '\0' && !snap.usb_authority) {
    arm_rect = row[0];
    draw_button(arm_rect, "ARM", kDarkOrange, kOrange);
  } else {
    draw_button(row[0], "LOCK", kGray, TFT_BLACK);
  }
  if (!snap.usb_authority) {
    jog_pos_rect = row[1];
    jog_neg_rect = row[2];
    draw_button(jog_pos_rect, "JOG+", kDarkOrange, kGreen);
    draw_button(jog_neg_rect, "JOG-", kDarkOrange, kAmber);
  }
}

void draw_encoder() {
  canvas.setFont(&fonts::Font4);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  char count_text[24];
  snprintf(count_text, sizeof(count_text), "%lld", static_cast<long long>(snap.count_display));
  canvas.drawString(count_text, canvas.width() / 2, 78);

  char line[80];
  snprintf(line, sizeof(line), "raw %lld  trial %lld  A %d  B %d", static_cast<long long>(snap.count_signed),
           static_cast<long long>(snap.count_trial), snap.a_level, snap.b_level);
  draw_line(104, line, TFT_WHITE);
  snprintf(line, sizeof(line), "sign %+d  %s  ppr %s", snap.sign, snap.count_stable ? "stable" : "not stable",
           snap.ppr_text);
  draw_line(116, line, kGray);
  draw_line(128, "CURRENT MONITOR: NOT CONNECTED", kRed);

  canvas.setFont(&fonts::Font2);
  Rect row[1];
  place_row(row, 1, 146, 40);
  zero_rect = row[0];
  draw_button(zero_rect, "ZERO DISPLAY", kDarkOrange, kOrange);
}

void draw_visual() {
  char line[80];
  snprintf(line, sizeof(line), "%s  shaft %s", visual_state_name(snap.visual_state),
           snap.visual_shaft[0] != '\0' ? snap.visual_shaft : "-");
  draw_line(46, line, kOrange);
  if (snap.visual_have_delta) {
    snprintf(line, sizeof(line), "delta %lld", static_cast<long long>(snap.visual_delta));
  } else {
    snprintf(line, sizeof(line), "delta none");
  }
  draw_line(58, line, TFT_WHITE);
  if (snap.visual_valid_n > 0) {
    char mean[16];
    char lo[16];
    char hi[16];
    char spread[16];
    fmt_fixed(mean, sizeof(mean), snap.visual_mean);
    fmt_fixed(lo, sizeof(lo), snap.visual_min);
    fmt_fixed(hi, sizeof(hi), snap.visual_max);
    fmt_fixed(spread, sizeof(spread), snap.visual_spread);
    snprintf(line, sizeof(line), "n %d  mean %s", snap.visual_valid_n, mean);
    draw_line(70, line, TFT_WHITE);
    snprintf(line, sizeof(line), "min %s max %s spr %s", lo, hi, spread);
    draw_line(82, line, TFT_WHITE);
  } else {
    draw_line(70, "valid n 0", kGray);
    draw_line(82, "cpr UNKNOWN", kGray);
  }
  if (snap.visual_reason[0] != '\0') {
    snprintf(line, sizeof(line), "reason %s", snap.visual_reason);
    draw_line(94, line, kAmber);
  } else if (snap.visual_runs[0] != '\0') {
    draw_line(94, snap.visual_runs, kGray);
  }

  canvas.setFont(&fonts::lgfxJapanGothic_12);
  Rect top[2];
  Rect bottom[4];
  place_row(top, 2, 108, 34);
  place_row(bottom, 4, 146, 40);
  vis_ref_rect = top[0];
  vis_finish_rect = top[1];
  vis_stop_rect = bottom[0];
  vis_pos_rect = bottom[1];
  vis_neg_rect = bottom[2];
  vis_cancel_rect = bottom[3];
  draw_button(vis_ref_rect, "基準を記録", kDarkOrange, kOrange);
  draw_button(vis_finish_rect, "1回転完了を記録", kDarkOrange, kOrange);
  draw_button(vis_stop_rect, "停止", kDarkRed, TFT_WHITE);
  draw_button(vis_pos_rect, "正ジョグ", kDarkOrange, kGreen);
  draw_button(vis_neg_rect, "逆ジョグ", kDarkOrange, kAmber);
  draw_button(vis_cancel_rect, "破棄", kGray, TFT_WHITE);
}

void draw_config() {
  canvas.setFont(&fonts::Font0);
  canvas.setTextWrap(true, false);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setClipRect(0, 42, canvas.width(), 144);
  canvas.setCursor(4, 46);
  canvas.printf("LOCK %s\n", snap.lock_reason[0] != '\0' ? snap.lock_reason : "none");
  canvas.printf("MISSING %s\n", snap.missing[0] != '\0' ? snap.missing : "none");
  canvas.printf("PINS PWR G%d BRK G%d PWM G%d DIR G%d\n", POWER_GPIO, BRAKE_GPIO, PWM_GPIO, DIR_GPIO);
  canvas.printf("ENC A G%d B G%d  sign %+d  glitch %d ns\n", ENCODER_A_GPIO, ENCODER_B_GPIO, snap.sign,
                snap.glitch_ns);
  canvas.setTextColor(kAmber, TFT_BLACK);
  canvas.println("not a physical e-stop");
  canvas.println("CURRENT MONITOR: NOT CONNECTED");
  canvas.println("G2 may feed all axes");
  canvas.println("one CoreS3 only");
  canvas.setTextColor(kGray, TFT_BLACK);
  canvas.println("CMD is not a measured voltage");
  canvas.clearClipRect();
}

void draw_page() {
  clear_page_rects();
  if (page == Page::Drive) {
    draw_drive();
  } else if (page == Page::Encoder) {
    draw_encoder();
  } else if (page == Page::Visual) {
    draw_visual();
  } else {
    draw_config();
  }
}

}  // namespace

void ui_begin() {
  if (M5.Display.width() < M5.Display.height()) {
    M5.Display.setRotation(M5.Display.getRotation() ^ 1);
  }
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
  layout_chrome();
  refresh();
  draw_soon = true;
}

void ui_poll() {
  refresh();
  const auto touch = M5.Touch.getDetail();
  const bool down = touch.isPressed();
  const int x = touch.x;
  const int y = touch.y;
  const bool estop = down && estop_rect.hit(x, y);
  bool jog_pos = false;
  bool jog_neg = false;
  if (down && !estop && !snap.usb_authority) {
    if (page == Page::Drive) {
      jog_pos = jog_pos_rect.hit(x, y);
      jog_neg = jog_neg_rect.hit(x, y);
    } else if (page == Page::Visual) {
      jog_pos = vis_pos_rect.hit(x, y);
      jog_neg = vis_neg_rect.hit(x, y);
    }
  }
  control_set_inputs(millis(), estop, jog_pos, jog_neg);

  if (!touch.wasPressed()) {
    return;
  }
  if (stop_rect.hit(x, y) || (page == Page::Visual && vis_stop_rect.hit(x, y))) {
    control_request_stop(false);
  }
  for (int i = 0; i < 4; ++i) {
    if (tabs[i].hit(x, y)) {
      page = static_cast<Page>(i);
      draw_soon = true;
    }
  }
  if (page == Page::Drive && arm_rect.hit(x, y)) {
    if (snap.state == DriveState::Fault && !estop) {
      control_request_fault_reset(false);
    } else if (snap.lock_reason[0] == '\0' && !snap.usb_authority) {
      control_request_arm(false);
    }
  }
  if (page == Page::Encoder && zero_rect.hit(x, y)) {
    control_request_zero(false);
  }
  if (page == Page::Visual) {
    if (vis_ref_rect.hit(x, y)) {
      control_request_visual(ReplyKind::VisualRef, false);
    } else if (vis_finish_rect.hit(x, y)) {
      control_request_visual(ReplyKind::VisualFinish, false);
    } else if (vis_cancel_rect.hit(x, y)) {
      control_request_visual(ReplyKind::VisualCancel, false);
    }
  }
}

void ui_draw() {
  if (!sprite_ok) {
    return;
  }
  const uint32_t now = millis();
  if (!draw_soon && static_cast<int32_t>(now - next_draw_ms) < 0) {
    return;
  }
  draw_soon = false;
  next_draw_ms = now + UI_PERIOD_MS;
  refresh();

  bool estop_down = false;
  const auto touch = M5.Touch.getDetail();
  if (touch.isPressed() && estop_rect.hit(touch.x, touch.y)) {
    estop_down = true;
  }

  canvas.fillSprite(TFT_BLACK);
  draw_header();
  draw_page();
  draw_footer(estop_down);
  canvas.pushSprite(0, 0);
}
