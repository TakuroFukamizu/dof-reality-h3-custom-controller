#include <M5Unified.h>

#include "brake_control.h"
#include "commands.h"
#include "config.h"
#include "control.h"
#include "encoder_counter.h"
#include "motor_driver.h"
#include "power_control.h"
#include "relay_trace.h"
#include "telemetry.h"
#include "ui.h"

static_assert(pwm_timing_ok(), "LEDC timing");
static_assert(drive_pins_unique(), "drive pins overlap");

namespace {

bool stall_latched = false;

}  // namespace

void setup() {
  // M5.begin は Port.A（G2 = 主電源リレー）の I2C 探索や、Module 検出で G13（= ブレーキ解除リレー）を
  // I2S DOUT に取ることがある。先にリレーと PWM をオフにし、cfg で探索を切ってから始める。
  power_force_off();
  brake_force_off();
  motor_force_safe();

  auto cfg = M5.config();
  cfg.clear_display = true;
#if ENCODER_TESTER_USB_POWER
  // env:m5stack-cores3-usbpower。USB のみで動かし、CoreS3 が Grove 5V を供給する（10/22 と同じ構成）。
  // DIN Base 12V と USB の同時接続で約 85 秒後に電源断する問題（Issue #1）の回避用。Bottom の DC は接続しない。
  // Bottom は Base15 に変更した。Base15 の DC と USB の同時接続は未検証。
  cfg.output_power = true;
#else
  cfg.output_power = false;  // Bottom（Base15 の DC ジャック → MP1584 → BUS 5V）から 5V を受ける設定
#endif
  cfg.internal_imu = false;
  cfg.internal_mic = false;
  cfg.internal_spk = false;
  cfg.external_imu = false;
  cfg.external_rtc = false;
  // G2（主電源）は Port.A の I2C 線、G13（ブレーキ）は Module Display/RCA 検出時の I2S DOUT。どちらも探索させない。
  cfg.external_display_value = 0;
  cfg.external_speaker_value = 0;
  M5.begin(cfg);
#if !ENCODER_TESTER_USB_POWER
  M5.Power.setUsbOutput(false);  // CoreS3からPC側へのUSB給電を無効化
#endif
  Serial.begin(115200);

  if (M5.Display.width() < M5.Display.height()) {
    M5.Display.setRotation(M5.Display.getRotation() ^ 1);
  }

  power_force_off();
  brake_force_off();
  motor_force_safe();
  power_begin();
  brake_begin();
  const bool motor_ok = motor_begin();
  const bool pcnt_ok = encoder_begin();
  const bool electrical = motor_ok && drive_pins_unique() && pwm_timing_ok();
  control_begin(pcnt_ok, electrical);
  ui_begin();

  Serial.println("BOOT state=DISARMED");
  Serial.println("BOOT ARM locked until SET");
  Serial.println("BOOT outputs off");
  Serial.println("BOOT CURRENT MONITOR NOT CONNECTED");
  Serial.println("BOOT not a physical e-stop");
  Serial.println("BOOT opening USB resets this chip");
  Serial.println("HELP");
}

void loop() {
  M5.update();
  ui_poll();
  commands_poll();
  telemetry_flush(6);
  relay_trace_flush(4);
  ui_draw();
  if (!control_alive(millis())) {
    outputs_safe();
    if (!stall_latched) {
      Serial.println("ERR control_task_stall");
      stall_latched = true;
    }
  } else if (stall_latched) {
    Serial.println("OK control_task");
    stall_latched = false;
  }
  delay(1);
}
