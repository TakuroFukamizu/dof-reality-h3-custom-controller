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
  bool reset_stats_pulse = false;  // tick の計器（処理時間・ACTIVE 中の間隔）を 0 に戻す（画面の tick 行をタップ）
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
  // PR #49 レビュー P2c: 制御 tick の処理時間（入口の now から出力・送信キューまで）と、ACTIVE 中の tick 間隔の最大。
  // 画面の tick 行に出し、タップで 0 に戻す（プロトコルは変えない）。recent = 直近 1 s（200 tick）の最大
  uint32_t exec_max_us = 0;
  uint32_t exec_win_max_us = 0;
  uint32_t exec_recent_us = 0;
  uint32_t exec_win_ticks = 0;
  uint32_t active_max_gap_us = 0;

  for (;;) {
    V1Inputs in;
    portENTER_CRITICAL(&g_mux);
    in = g_v1_in;
    g_v1_in.select_pulse = false;
    g_v1_in.estop_pulse = false;
    g_v1_in.clear_dirty_pulse = false;
    g_v1_in.reset_stats_pulse = false;
    portEXIT_CRITICAL(&g_mux);

    if (in.reset_stats_pulse) {
      max_gap_us = 0;
      exec_max_us = 0;
      exec_win_max_us = 0;
      exec_recent_us = 0;
      exec_win_ticks = 0;
      active_max_gap_us = 0;
    }

    if (in.clear_dirty_pulse) {
      g_axis.clear_profile_dirty();
    }

    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t gap = now - prev_now;
    prev_now = now;
    if (gap > max_gap_us) max_gap_us = static_cast<uint32_t>(gap);
    if (g_axis.state() == v1::state::ACTIVE && gap > active_max_gap_us) active_max_gap_us = static_cast<uint32_t>(gap);

    // 画面の非常停止は受信フレームより先に（この周期の出力に必ず反映する）
    if (in.estop_pulse) {
      g_axis.set_local_estop(in.estop_value, now);
    }
    // メッセージ層より下で捨てた受信（300 byte 超・受信キュー満杯）
    for (uint32_t d = v1_link_take_rx_dropped(); d > 0; --d) {
      g_axis.record_rx_error();
    }
    // Issue #45: 溜まったフレームも 1 つずつ受信時刻つきで再生バッファへ（途中の目標を捨てない）。期限・lease・応答は従来どおり now。
    // now を取った後に積まれたフレームは受信時刻が now より新しいことがあるので now で頭打ち（V1Axis::handle も同じく頭打ちする）
    V1RxFrame frame;
    while (v1_link_rx_pop(&frame)) {
      g_axis.handle_frame(frame.data, frame.len, now, frame.rx_us < now ? frame.rx_us : now);
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
    snap.sup = g_axis.supervisor_info();
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
    {
      const uint64_t done = static_cast<uint64_t>(esp_timer_get_time());
      const uint32_t exec_us = static_cast<uint32_t>(done - now);  // 反転時の delay(DIR_COAST_MS) も入る
      if (exec_us > exec_max_us) exec_max_us = exec_us;
      if (exec_us > exec_win_max_us) exec_win_max_us = exec_us;
      if (++exec_win_ticks >= 200) {
        exec_recent_us = exec_win_max_us;
        exec_win_max_us = 0;
        exec_win_ticks = 0;
      }
    }
    snap.tick_count = ticks;
    snap.max_tick_gap_us = max_gap_us;
    snap.active_max_tick_gap_us = active_max_gap_us;
    snap.exec_max_us = exec_max_us;
    snap.exec_recent_us = exec_recent_us > exec_win_max_us ? exec_recent_us : exec_win_max_us;
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

void control_v1_reset_tick_stats() {
  portENTER_CRITICAL(&g_mux);
  g_v1_in.reset_stats_pulse = true;
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
