#include "control_task.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

#include "config.h"
#include "encoder_pcnt.h"
#include "manual_guard.h"
#include "motor_output.h"

#if MC_PROTO_V1
#include <esp_timer.h>

#include <cstring>

#include "pmic_log.h"
#include "v1_axis.h"
#include "v1_link.h"
#endif

namespace {

constexpr uint32_t kPeriodMs = 5;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

#if !MC_PROTO_V1

struct Inputs {
  bool relay_live = false;
  bool brake_released = false;
  int manual_speed = 0;
  bool stop_pulse = false;
  bool estop_pulse = false;
  bool request_pulse = false;
  AutoKind request = AutoKind::Idle;
  bool profile_pulse = false;
  MotorProfile profile{};
  bool clear_dirty_pulse = false;
  uint32_t fed_ms = 0;  // main が入力を更新した時刻。古ければ出力を止める
};

constexpr uint32_t kInputStaleMs = 200;
constexpr uint32_t kVelocityWindowMs = 100;

Inputs g_in;
ControlSnapshot g_snap;
MotorProfile g_initial;

void copy_text(char* dest, size_t size, const char* text) {
  strncpy(dest, text, size - 1);
  dest[size - 1] = '\0';
}

void task_main(void*) {
  AutoMachine machine;
  machine.set_profile(g_initial);
  ManualGuard manual_guard;  // 手動速度中にセンサーの計数が止まったら出力 0
  int64_t vel_anchor_count = 0;
  uint32_t vel_anchor_ms = millis();
  int count_per_s = 0;
  uint32_t request_seq = 0;
  char request_error[16] = "";
  uint32_t ticks = 0;

  for (;;) {
    Inputs in;
    portENTER_CRITICAL(&g_mux);
    in = g_in;
    g_in.stop_pulse = false;
    g_in.estop_pulse = false;
    g_in.request_pulse = false;
    g_in.profile_pulse = false;
    g_in.clear_dirty_pulse = false;
    portEXIT_CRITICAL(&g_mux);

    if (in.profile_pulse) {
      machine.set_profile(in.profile);
    }
    if (in.clear_dirty_pulse) {
      machine.profile_dirty = false;
    }

    const uint32_t now = millis();
    const int64_t raw = encoder_count();
    // UI ループが止まると入力が更新されない。古い入力は「主電源 OFF・手動 0」として扱い、出力を止める。
    const bool inputs_fresh = now - in.fed_ms <= kInputStaleMs;
    if (!inputs_fresh) {
      in.relay_live = false;
      in.manual_speed = 0;
    }
    AutoInputs ai;
    ai.now_ms = now;
    ai.raw_count = raw;
    ai.relay_live = in.relay_live;
    ai.brake_released = in.brake_released;
    ai.manual_speed_nonzero = in.manual_speed != 0;
    ai.stop = in.stop_pulse;
    ai.estop = in.estop_pulse;

    if (in.estop_pulse) {
      machine.invalidate_origin();  // 自動移動中でなくても非常停止で原点は無効
    }
    if (in.request_pulse) {
      const char* err = "";
      request_seq += 1;
      if (machine.request(in.request, ai, &err)) {
        request_error[0] = '\0';
      } else {
        copy_text(request_error, sizeof(request_error), err);
      }
    }
    machine.tick(ai);

    // 手動速度は、自動移動中でなくても無パルス／遅すぎで 0 にラッチする（センサー断線のまま動かさない）。
#if MC_NO_MANUAL_GUARD
    // 試験専用（env:m5stack-cores3-noguard）: フィードバックセンサーを無視して手動速度をそのまま出す。
    // センサー断線でも止まらないので、機構の切り分け以外では使わない。
    const int manual_out = machine.active() ? 0 : in.manual_speed;
    (void)manual_guard;
#else
    const int manual_out = manual_guard.tick(now, raw, machine.active() ? 0 : in.manual_speed, in.relay_live);
#endif
    const int out = machine.active() ? machine.duty_signed : manual_out;
    motor_output_write(out);

    if (now - vel_anchor_ms >= kVelocityWindowMs) {
      const int64_t dc = raw - vel_anchor_count;
      count_per_s = static_cast<int>((dc * 1000) / static_cast<int64_t>(now - vel_anchor_ms));
      vel_anchor_count = raw;
      vel_anchor_ms = now;
    }

    ControlSnapshot snap;
    snap.raw_count = raw;
    snap.origin_valid = machine.origin_valid;
    snap.position = machine.origin_valid ? machine.position(raw) : static_cast<int32_t>(raw * machine.working_sign);
    snap.count_per_s = count_per_s;
    snap.a_level = encoder_a_level();
    snap.b_level = encoder_b_level();
    snap.applied_speed = motor_output_applied();
    snap.kind = machine.kind;
    snap.phase = machine.phase;
    snap.cal_step = machine.cal_step;
    copy_text(snap.reason, sizeof(snap.reason), machine.reason);
    snap.profile = machine.profile;
    snap.profile_dirty = machine.profile_dirty;
    snap.last_measured_stroke = machine.last_measured_stroke;
    snap.working_sign = machine.working_sign;
    snap.request_seq = request_seq;
    copy_text(snap.request_error, sizeof(snap.request_error), request_error);
    snap.manual_fault_seq = manual_guard.fault_seq;
    copy_text(snap.manual_fault_reason, sizeof(snap.manual_fault_reason), manual_guard.reason);
    ticks += 1;
    snap.tick_count = ticks;

    portENTER_CRITICAL(&g_mux);
    g_snap = snap;
    portEXIT_CRITICAL(&g_mux);

    vTaskDelay(pdMS_TO_TICKS(kPeriodMs));
  }
}

#endif  // !MC_PROTO_V1

}  // namespace

#if !MC_PROTO_V1

void control_begin(const MotorProfile& initial) {
  g_initial = initial;
  g_snap.profile = initial;
  g_snap.working_sign = initial.sign;
  xTaskCreatePinnedToCore(task_main, "control", 8192, nullptr, 5, nullptr, 1);
}

void control_set_relay_live(bool live) {
  portENTER_CRITICAL(&g_mux);
  g_in.relay_live = live;
  g_in.fed_ms = millis();
  portEXIT_CRITICAL(&g_mux);
}

void control_set_brake_released(bool released) {
  portENTER_CRITICAL(&g_mux);
  g_in.brake_released = released;
  portEXIT_CRITICAL(&g_mux);
}

void control_set_manual_speed(int percent) {
  portENTER_CRITICAL(&g_mux);
  g_in.manual_speed = percent;
  portEXIT_CRITICAL(&g_mux);
}

void control_pulse_stop() {
  portENTER_CRITICAL(&g_mux);
  g_in.stop_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_pulse_estop() {
  portENTER_CRITICAL(&g_mux);
  g_in.estop_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_request(AutoKind kind) {
  portENTER_CRITICAL(&g_mux);
  g_in.request = kind;
  g_in.request_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_set_profile(const MotorProfile& p) {
  portENTER_CRITICAL(&g_mux);
  g_in.profile = p;
  g_in.profile_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_clear_profile_dirty() {
  portENTER_CRITICAL(&g_mux);
  g_in.clear_dirty_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

ControlSnapshot control_snapshot() {
  ControlSnapshot snap;
  portENTER_CRITICAL(&g_mux);
  snap = g_snap;
  portEXIT_CRITICAL(&g_mux);
  return snap;
}

#else  // MC_PROTO_V1

// ---------------------------------------------------------------------------------
// USB プロトコル v1: 5 ms 周期で V1Axis を回し、GPIO（主電源 G2・ブレーキ G13）と PWM を制御タスクから直接書く。
// 受信フレームは v1_link の受信キューから、出力メッセージは送信キューへ（ACK → STATUS の順を保つ）。
// 手動速度・自動移動（AutoMachine・ManualGuard）は使わない。モーター出力は V1Axis だけが書く。
// ---------------------------------------------------------------------------------

namespace {

struct V1Inputs {
  bool select_pulse = false;
  uint8_t select_axis = 0;
  bool estop_pulse = false;
  bool estop_value = false;
  bool clear_dirty_pulse = false;
};

void copy_text(char* dest, size_t size, const char* text) {
  strncpy(dest, text, size - 1);
  dest[size - 1] = '\0';
}

V1Axis g_axis;  // 大きいのでタスクのスタックではなく静的領域に置く
V1Inputs g_v1_in;
V1Snapshot g_v1_snap;
int8_t g_signs[4] = {1, 1, 1, 1};
MotorProfile g_profiles[4];  // NVS の各系統のプロファイル（校正の端当てで行程の見積もりに使う。校正成功で更新）
uint8_t g_selected = 1;

int relay_off_level() { return RELAY_ON_LEVEL ? 0 : 1; }
int brake_off_level() { return BRAKE_ON_LEVEL ? 0 : 1; }

// レジスタ 1 バイトを PMIC_REPORT の正規の位置へ。読めなかったものは載せない（valid ビットも 0）。
void pmic_put_reg(v1::PmicReport* r, uint8_t addr, uint8_t val, bool ok) {
  if (!ok) return;
  for (int i = 0; i < v1::kPmicRegDefined; ++i) {
    if (v1::kPmicRegAddr[i] != addr) continue;
    r->regs[i] = val;
    r->reg_valid |= 1ull << i;
    return;
  }
}

// どの kind でも共通: リセット理由、RTC の前回値、POR の目印、I2C 失敗回数。
// 目印 REG04〜07 は mark_ok のとき全 kind に載せる（サーバが POR を 1 本で判定できる）。
void pmic_report_base(v1::PmicReport* r, uint8_t kind) {
  memset(r, 0, sizeof(*r));
  r->kind = kind;
  r->por = 0xFF;
  PmicBootSnapshot boot{};
  if (pmic_copy_boot(&boot)) {
    r->reset_reason = static_cast<uint8_t>(boot.reset_reason);
    if (boot.rtc_ok) {
      r->flags = static_cast<uint8_t>(r->flags | v1::pmic_flags::RTC_OK);
      r->last_up_s = boot.last_up;
      r->last_vbus = boot.last_vbus;
      r->last_vsys = boot.last_vsys;
      r->last_vbat = boot.last_vbat;
      r->last_s0 = boot.last_s0;
      r->last_s1 = boot.last_s1;
    }
  }
  PmicPreSnapshot pre{};
  if (pmic_copy_pre(&pre) && pre.mark_ok) {
    r->flags = static_cast<uint8_t>(r->flags | v1::pmic_flags::MARK_OK);
    r->por = memcmp(pre.mark, kPmicMark, sizeof(kPmicMark)) == 0 ? 0 : 1;
    for (int i = 0; i < 4; ++i) pmic_put_reg(r, static_cast<uint8_t>(0x04 + i), pre.mark[i], true);
  }
  const uint32_t err = pmic_i2c_error_count();
  r->i2c_err = err > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(err);
}

bool pmic_make_pre(v1::McuMsg* m) {
  PmicPreSnapshot pre{};
  if (!pmic_copy_pre(&pre)) return false;
  memset(m, 0, sizeof(*m));
  m->type = v1::msg::PMIC_REPORT;
  pmic_report_base(&m->pmic_report, v1::pmic_kind::PREBOOT);
  m->pmic_report.seq = 0;
  m->pmic_report.mcu_time_us = pre.t_us;
  m->pmic_report.read_ok = pre.ok;
  m->pmic_report.read_tried = pre.tried;
  for (int i = 0; i < kPmicPreRegCount; ++i) {
    pmic_put_reg(&m->pmic_report, kPmicPreRegs[i], pre.regs[i], pre.regs_ok[i]);
  }
  return true;
}

bool pmic_make_boot(v1::McuMsg* m) {
  PmicBootSnapshot boot{};
  if (!pmic_copy_boot(&boot)) return false;
  memset(m, 0, sizeof(*m));
  m->type = v1::msg::PMIC_REPORT;
  pmic_report_base(&m->pmic_report, v1::pmic_kind::BOOT);
  m->pmic_report.seq = 0;
  m->pmic_report.mcu_time_us = boot.t_us;
  m->pmic_report.read_ok = boot.read_ok;
  m->pmic_report.read_tried = boot.read_tried;
  pmic_put_reg(&m->pmic_report, 0x20, boot.on, boot.on_ok);
  pmic_put_reg(&m->pmic_report, 0x21, boot.off, boot.off_ok);
  pmic_put_reg(&m->pmic_report, 0x48, boot.irq48, boot.irq48_ok);
  pmic_put_reg(&m->pmic_report, 0x49, boot.irq49, boot.irq49_ok);
  pmic_put_reg(&m->pmic_report, 0x4A, boot.irq4a, boot.irq4a_ok);
  for (int i = 0; i < kPmicBaselineRegCount; ++i) {
    pmic_put_reg(&m->pmic_report, kPmicBaselineRegs[i], boot.baseline[i], boot.baseline_ok[i]);
  }
  return true;
}

bool pmic_make_periodic(v1::McuMsg* m, const PmicPeriodic& p) {
  PmicBootSnapshot boot{};
  const bool has_boot = pmic_copy_boot(&boot);
  memset(m, 0, sizeof(*m));
  m->type = v1::msg::PMIC_REPORT;
  pmic_report_base(&m->pmic_report, v1::pmic_kind::PERIODIC);
  v1::PmicReport& r = m->pmic_report;
  r.seq = p.seq;
  r.mcu_time_us = p.t_us;
  r.read_ok = p.read_ok;
  r.read_tried = p.read_tried;
  r.vbus = p.vbus;
  r.vsys = p.vsys;
  r.vbat = p.vbat;
  r.ts = p.ts;
  r.tdie = static_cast<int16_t>(p.tdie_deci);
  if (p.vbus_ok) r.flags = static_cast<uint8_t>(r.flags | v1::pmic_flags::VBUS_OK);
  if (p.vsys_ok) r.flags = static_cast<uint8_t>(r.flags | v1::pmic_flags::VSYS_OK);
  if (p.vbat_ok) r.flags = static_cast<uint8_t>(r.flags | v1::pmic_flags::VBAT_OK);
  if (p.ts_ok) r.flags = static_cast<uint8_t>(r.flags | v1::pmic_flags::TS_OK);
  if (p.tdie_ok) r.flags = static_cast<uint8_t>(r.flags | v1::pmic_flags::TDIE_OK);
  pmic_put_reg(&r, 0x00, p.s0, p.s0_ok);
  pmic_put_reg(&r, 0x01, p.s1, p.s1_ok);
  pmic_put_reg(&r, 0x48, p.i48, p.i48_ok);
  pmic_put_reg(&r, 0x49, p.i49, p.i49_ok);
  pmic_put_reg(&r, 0x4A, p.i4a, p.i4a_ok);
  pmic_put_reg(&r, 0x18, p.r18, p.r18_ok);
  pmic_put_reg(&r, 0x68, p.r68, p.r68_ok);
  pmic_put_reg(&r, 0x14, p.r14, p.r14_ok);
  pmic_put_reg(&r, 0x23, p.r23, p.r23_ok);
  // REG20 / REG21 は起動時にラッチした値（1 秒読みでは読み直さない。従来の PMIC 行と同じ）。
  if (has_boot) {
    pmic_put_reg(&r, 0x20, boot.on, boot.on_ok);
    pmic_put_reg(&r, 0x21, boot.off, boot.off_ok);
  }
  return true;
}

// outbox を送ったあと。HELLO のたびに preboot → boot を 1 回ずつ。キューに入らなければ次の周期で再試行。
// 1 秒読みは最初の HELLO のあと、起動している間ずっと（ホストが黙っている展示の待機でも電源を見る）。
// どちらも I2C はしない。TX ポンプが空き不足の PMIC_REPORT を捨てるので、ACK の順番はここでも崩さない。
void pmic_tx() {
  static uint32_t hello_done = 0;
  static uint8_t stage = 0;  // 0 なし、1 preboot、2 boot
  static uint32_t sent_seq = 0;

  const uint32_t seen = g_axis.hello_seen();
  if (seen != hello_done) {
    hello_done = seen;
    stage = 1;
  }
  if (stage == 1) {
    v1::McuMsg m{};
    if (!pmic_make_pre(&m)) {
      stage = 2;
    } else if (!v1_link_tx_push(m)) {
      return;
    } else {
      stage = 2;
    }
  }
  if (stage == 2) {
    v1::McuMsg m{};
    if (!pmic_make_boot(&m)) {
      stage = 0;
    } else if (!v1_link_tx_push(m)) {
      return;
    } else {
      stage = 0;
    }
  }
  if (seen == 0) return;
  PmicPeriodic p{};
  if (!pmic_copy_periodic(&p) || p.seq == 0 || p.seq == sent_seq) return;
  v1::McuMsg m{};
  pmic_make_periodic(&m, p);
  if (v1_link_tx_push(m)) sent_seq = p.seq;
}

// 主電源は電源担当軸（CONFIG role_flags bit0）のときだけ V1Axis が ON を指令する。それ以外は常に OFF レベル。
void write_outputs(bool power_on, bool brake_release, int duty_permille) {
  digitalWrite(PIN_POWER_RELAY, power_on ? RELAY_ON_LEVEL : relay_off_level());
  digitalWrite(PIN_MOTOR_BRAKE, brake_release ? BRAKE_ON_LEVEL : brake_off_level());
  motor_output_write_permille(duty_permille);
}

void v1_task_main(void*) {
  V1Identity id;
  id.device_id = v1_link_device_id();
  id.boot_id = v1_link_boot_id();
  g_axis.begin(id, g_selected, g_signs[g_selected], static_cast<uint64_t>(esp_timer_get_time()), encoder_count());
  g_axis.set_profile_hint(g_profiles[g_selected]);

  TickType_t last_wake = xTaskGetTickCount();
  uint32_t ticks = 0;
  uint32_t max_gap_us = 0;
  uint64_t prev_now = static_cast<uint64_t>(esp_timer_get_time());

  for (;;) {
    V1Inputs in;
    portENTER_CRITICAL(&g_mux);
    in = g_v1_in;
    g_v1_in.select_pulse = false;
    g_v1_in.estop_pulse = false;
    g_v1_in.clear_dirty_pulse = false;
    portEXIT_CRITICAL(&g_mux);

    if (in.clear_dirty_pulse) {
      g_axis.clear_profile_dirty();
    }

    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t gap = now - prev_now;
    prev_now = now;
    if (gap > max_gap_us) max_gap_us = static_cast<uint32_t>(gap);

    // 画面の非常停止は受信フレームより先に（この周期の出力に必ず反映する）
    if (in.estop_pulse) {
      g_axis.set_local_estop(in.estop_value, now);
    }
    // メッセージ層より下で捨てた受信（300 byte 超・受信キュー満杯）
    for (uint32_t d = v1_link_take_rx_dropped(); d > 0; --d) {
      g_axis.record_rx_error();
    }
    V1RxFrame frame;
    while (v1_link_rx_pop(&frame)) {
      g_axis.handle_frame(frame.data, frame.len, now);
    }
    if (in.select_pulse) {
      if (g_axis.select_axis(in.select_axis, now)) {
        g_axis.set_sign(g_signs[in.select_axis]);
        g_axis.set_profile_hint(g_profiles[in.select_axis]);
      }
    }

    g_axis.tick(now, encoder_count());
    write_outputs(g_axis.power_on_cmd(), g_axis.brake_release_cmd(), g_axis.duty_permille());

    // 校正に成功した系統の符号を、次の系統切り替え（set_sign）に反映する。NVS への保存は loop() が行う
    //（Preferences の書き込みは数十 ms 止まることがあり、5 ms の制御タスクでは CONTROL_OVERRUN になる）。
    if (g_axis.profile_dirty()) {
      const MotorProfile& p = g_axis.dirty_profile();
      if (profile_motor_ok(p.motor)) {
        g_signs[p.motor] = p.sign < 0 ? -1 : 1;
        g_profiles[p.motor] = p;
        if (p.motor == g_axis.selected_axis()) g_axis.set_profile_hint(p);
      }
    }

    // outbox → 送信キュー。順序を保つ: 満杯なら STATUS は捨て（次の周期で送る）、それ以外は保留して次の周期に再試行。
    while (const v1::McuMsg* m = g_axis.peek()) {
      if (v1_link_tx_push(*m)) {
        g_axis.pop();
      } else if (m->type == v1::msg::STATUS) {
        g_axis.pop();
      } else {
        break;
      }
    }
    // ACK / STATUS を積んだあと。PMIC は outbox に入れない（満杯のとき STATUS を押し出さない）。
    pmic_tx();

    V1Snapshot snap;
    snap.state = g_axis.state();
    snap.fault_code = g_axis.fault_code();
    snap.flags = g_axis.flags();
    snap.selected_axis = g_axis.selected_axis();
    snap.power_on_cmd = g_axis.power_on_cmd();
    snap.brake_release_cmd = g_axis.brake_release_cmd();
    snap.duty_permille = g_axis.duty_permille();
    snap.stall_boost_permille = g_axis.stall_boost_permille();
    snap.terms = g_axis.duty_terms();
    snap.pwm_permille = g_axis.pwm_permille();
    snap.encoder_fault_check_disabled = g_axis.encoder_fault_check_disabled();
    snap.actual = g_axis.actual_count();
    snap.target = g_axis.target_count();
    snap.velocity = g_axis.velocity_count_s();
    snap.following_error = g_axis.following_error_count();
    snap.configured = g_axis.configured();
    snap.referenced = g_axis.referenced();
    snap.estop_latched = g_axis.estop_latched();
    snap.has_rx = g_axis.has_rx();
    snap.last_rx_age_ms = g_axis.has_rx() ? static_cast<uint32_t>((now - g_axis.last_rx_us()) / 1000) : 0;
    snap.rx_error_count = g_axis.rx_error_count();
    snap.stale_drop_count = g_axis.stale_drop_count();
    snap.tx_dropped = v1_link_tx_dropped();
    snap.tx_deferred = v1_link_tx_deferred();
    snap.tx_stale_dropped = v1_link_tx_stale_dropped();
    ticks += 1;
    snap.tick_count = ticks;
    snap.max_tick_gap_us = max_gap_us;
    snap.device_id = id.device_id;
    snap.boot_id = id.boot_id;
    snap.cal_live_step = g_axis.cal_live_step();
    copy_text(snap.cal_live_reason, sizeof(snap.cal_live_reason), g_axis.cal_live_reason());
    snap.cal_result_valid = g_axis.has_cal_result();
    if (snap.cal_result_valid) {
      const v1::CalResult& r = g_axis.last_cal_result();
      snap.cal_result = r.result;
      snap.cal_step = r.step;
      snap.cal_n_end = r.n_end_count;
      snap.cal_neutral = r.neutral_count;
      snap.cal_sign = r.sign;
      memcpy(snap.cal_reason, r.reason, v1::CAL_REASON_LEN);
      snap.cal_reason[v1::CAL_REASON_LEN] = '\0';
    }
    snap.profile_dirty = g_axis.profile_dirty();
    snap.profile = g_axis.dirty_profile();

    portENTER_CRITICAL(&g_mux);
    g_v1_snap = snap;
    portEXIT_CRITICAL(&g_mux);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kPeriodMs));
  }
}

}  // namespace

void control_v1_begin(const MotorProfile profiles[4], uint8_t selected_axis) {
  for (int i = 0; i < 4; ++i) {
    g_profiles[i] = profiles[i];
    g_signs[i] = profiles[i].sign < 0 ? -1 : 1;
  }
  g_selected = (selected_axis >= 1 && selected_axis <= 3) ? selected_axis : 1;
  g_v1_snap.selected_axis = g_selected;
  g_v1_snap.state = 0;  // BOOT（タスクが begin するまで）
  xTaskCreatePinnedToCore(v1_task_main, "control_v1", 8192, nullptr, 5, nullptr, 1);
}

void control_v1_select_axis(uint8_t axis) {
  if (axis < 1 || axis > 3) return;
  portENTER_CRITICAL(&g_mux);
  g_v1_in.select_axis = axis;
  g_v1_in.select_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_v1_set_estop(bool latched) {
  portENTER_CRITICAL(&g_mux);
  g_v1_in.estop_value = latched;
  g_v1_in.estop_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_v1_clear_profile_dirty() {
  portENTER_CRITICAL(&g_mux);
  g_v1_in.clear_dirty_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

V1Snapshot control_v1_snapshot() {
  V1Snapshot snap;
  portENTER_CRITICAL(&g_mux);
  snap = g_v1_snap;
  portEXIT_CRITICAL(&g_mux);
  return snap;
}

#endif  // MC_PROTO_V1
