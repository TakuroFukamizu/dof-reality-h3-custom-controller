#pragma once

// 計測専用（-DV1_DIAG_BRAKE_TEST=1、env:m5stack-cores3-v1-diag だけ）: ブレーキの効き始めと、ブレーキ解除・PWM 0 の沈みを
// ブレーキ指令・PWM・エンコーダ・時刻を揃えて記録する。r10 の監視（PR #50）の仮値（沈み 200 count/s、ブレーキ作動 150 ms、
// 解除待ち 300 ms、保持の包絡線 150 count/s）を実測で置き換えるための道具で、機能ではない。
// 手順と安全の注意: moter-controler/docs/brake_sag_measurement.md
//
// 純粋（Arduino 非依存、ヘッダのみ）。V1Axis（v1_axis.h）が受理条件・状態（v1::state::DIAG）・主電源・異常のラッチを受け持ち、
// ここは「解除 → 記録 → 作動 → 後の記録」の段階と、止める判定と、記録・要約だけを持つ。
// 座標は actual_count（基準からの位置。符号は掛けない）。pwm_permille はカウント増の向きが正（STATUS と同じ）。
// 量産の組み立て（V1_DIAG_BRAKE_TEST=0）では中身が空になる。

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "v1_codec.h"

#if V1_DIAG_BRAKE_TEST

// --- 上限（DIAG_START がこれを超えたら INVALID_PARAM で拒否する。黙って丸めない） ---
constexpr uint16_t V1_DIAG_SAG_T_MAX_CAP_MS = 500;            // SAG: ブレーキ解除の最長
constexpr uint16_t V1_DIAG_SAG_D_MAX_CAP_COUNT = 200;         // SAG: |変位| の上限
constexpr uint16_t V1_DIAG_ENGAGE_DUTY_MIN_PERMILLE = 100;    // ENGAGE: これ未満は機構が動かないことがある（無パルスで ENCODER_FAULT になる）
constexpr uint16_t V1_DIAG_ENGAGE_DUTY_CAP_PERMILLE = 250;    // ENGAGE: |Duty| の上限 25 %
constexpr uint16_t V1_DIAG_ENGAGE_D_RUN_CAP_COUNT = 600;      // ENGAGE: 走行距離の上限
constexpr uint16_t V1_DIAG_ENGAGE_V_TARGET_CAP_COUNT_S = 3000;
constexpr uint16_t V1_DIAG_ENGAGE_DELAY_CAP_MS = 200;         // ENGAGE: 解除指令から走り始めるまでの待ちの上限
constexpr uint32_t V1_DIAG_POWER_HOLD_CAP_MS = 120000;        // POWER_HOLD の最長（CAL_POWER_HOLD_TIMEOUT_MS と同じ）
// --- 固定の値 ---
constexpr uint32_t V1_DIAG_RUN_MAX_MS = 300;                  // ENGAGE: 走行時間の上限（記録が 1 s に収まるように）
constexpr uint32_t V1_DIAG_POWER_HOLD_RX_TIMEOUT_MS = 2000;   // POWER_HOLD: PC から何も来ないまま経ったら主電源を切る（道具が落ちたとき）
constexpr uint32_t V1_DIAG_POST_SAG_MS = 400;                 // SAG: ブレーキ作動指令の後に記録する時間
constexpr uint32_t V1_DIAG_POST_ENGAGE_MS = 500;              // ENGAGE: 同（≥ BRAKE_APPLY_WAIT_MS。終われば主電源 OFF）
constexpr int32_t V1_DIAG_END_MARGIN_COUNT = V1_SOFT_RESERVE_COUNT;  // ソフトリミットの内側に残す余裕（再生バッファと同じ 100）
constexpr uint32_t V1_DIAG_LOOKAHEAD_MS = 200;                // 端の判定の先読み: いまの速度でこの時間進んだ位置で見る（ブレーキ作動の仮値 150 ms + 余裕）
constexpr int32_t V1_DIAG_ENGAGE_STOP_ALLOW_COUNT = 300;      // ENGAGE の開始条件: 走行の向きに D_run + これだけ余裕が要る（作動中の滑り）
constexpr int32_t V1_DIAG_ENGAGE_SAG_GUARD_COUNT = 100;       // ENGAGE: 走り始める前の待ちの間に、これだけ沈んだら止める
constexpr int32_t V1_DIAG_BEHIND_ROOM_COUNT = V1_DIAG_SAG_D_MAX_CAP_COUNT;  // ENGAGE の開始条件: 走行の反対側に要る余裕
constexpr uint32_t V1_DIAG_NO_PULSE_MS = 150;                 // ENGAGE: 走らせてこの時間カウントが動かなければ無パルス
constexpr int32_t V1_DIAG_NO_PULSE_MIN_MOVE = 3;              // 同、「動いた」とみなす量（±1 のちらつきを除く）
constexpr int32_t V1_DIAG_REVERSE_COUNT = 20;                 // ENGAGE: 指令と逆向きにこれだけ動いたら止める
constexpr uint32_t V1_DIAG_VEL_WINDOW_US = 20000;             // 速度の窓（20 ms。1 ms 記録で 1 count = 50 count/s の分解能）
constexpr uint32_t V1_DIAG_VEL_MIN_SPAN_US = 10000;           // 速度を出すのに要る最短の履歴
constexpr uint32_t V1_DIAG_SETTLE_WINDOW_US = 20000;          // 静止の判定: この時間 ±V1_DIAG_SETTLE_BAND_COUNT に収まる
constexpr int32_t V1_DIAG_SETTLE_BAND_COUNT = 1;              // （20 ms で 1 count 以下 ≈ |v| < 50 count/s）
constexpr size_t V1_DIAG_MAX_SAMPLES = 1024;                  // SAG ≤ 500 + 400 点、ENGAGE ≤ 200 + 300 + 500 点（1 ms 記録）
constexpr uint16_t V1_DIAG_SAMPLE_PERIOD_US = 1000;           // 公称の記録周期（制御タスクが記録中だけ 1 ms ごとに回す）

// DIAG_START の値の検査（状態・位置の検査は V1Axis）。受理なら ack_reason::NONE
inline uint16_t v1_diag_check_params(const v1::DiagStart& d) {
  namespace dm = v1::diag_mode;
  const uint16_t bad = v1::ack_reason::INVALID_PARAM;
  switch (d.mode) {
    case dm::CANCEL: return v1::ack_reason::NONE;
    case dm::SAG:
      if (d.t_max_ms == 0 || d.t_max_ms > V1_DIAG_SAG_T_MAX_CAP_MS) return bad;
      if (d.d_max_count == 0 || d.d_max_count > V1_DIAG_SAG_D_MAX_CAP_COUNT) return bad;
      return v1::ack_reason::NONE;
    case dm::ENGAGE:
      if (d.dir != 1 && d.dir != -1) return bad;
      if (d.duty_permille < V1_DIAG_ENGAGE_DUTY_MIN_PERMILLE || d.duty_permille > V1_DIAG_ENGAGE_DUTY_CAP_PERMILLE) return bad;
      if (d.d_max_count == 0 || d.d_max_count > V1_DIAG_ENGAGE_D_RUN_CAP_COUNT) return bad;
      if (d.v_target_count_s == 0 || d.v_target_count_s > V1_DIAG_ENGAGE_V_TARGET_CAP_COUNT_S) return bad;
      if (d.delay_ms > V1_DIAG_ENGAGE_DELAY_CAP_MS) return bad;
      return v1::ack_reason::NONE;
    case dm::POWER_HOLD:
      if (d.hold_ms == 0 || d.hold_ms > V1_DIAG_POWER_HOLD_CAP_MS) return bad;
      return v1::ack_reason::NONE;
    default: return bad;
  }
}

// 開始位置に余裕があるか。[lo, hi] = ソフトリミットから V1_DIAG_END_MARGIN_COUNT 内側。
// SAG は向きが分からない（荷重の向きは軸と座席の姿勢で決まる）ので両側に d_max。ENGAGE は走行の向きに D_run + 作動中の滑り、
// 反対側に SAG の上限ぶん。
inline bool v1_diag_room_ok(const v1::DiagStart& d, int32_t a0, int32_t lo, int32_t hi) {
  const int64_t a = a0;
  if (d.mode == v1::diag_mode::SAG) {
    return a - d.d_max_count >= lo && a + d.d_max_count <= hi;
  }
  if (d.mode == v1::diag_mode::ENGAGE) {
    const int64_t ahead = static_cast<int64_t>(d.d_max_count) + V1_DIAG_ENGAGE_STOP_ALLOW_COUNT;
    const int64_t behind = V1_DIAG_BEHIND_ROOM_COUNT;
    if (d.dir > 0) return a + ahead <= hi && a - behind >= lo;
    return a - ahead >= lo && a + behind <= hi;
  }
  return true;
}

class V1DiagBrake {
 public:
  enum class Phase : uint8_t { Idle, PowerSettle, PowerHold, ReleaseWait, Sag, Drive, Post, Done };

  // 受理した DIAG_START で始める。lo / hi は端の余裕の内側、settle_ms は主電源 ON 指令からブレーキ解除までの待ち
  //（ARMING と同じ tuning.power_settle_ms）、overspeed は CONFIG の overspeed_count_s。
  void start(const v1::DiagStart& p, uint64_t now, int32_t lo, int32_t hi, uint32_t settle_ms, uint32_t overspeed) {
    // *this = V1DiagBrake{} は記録（16 KB）の一時オブジェクトを制御タスクのスタック（8 KB）に作るので使わない
    brake_release_ = false;
    pwm_permille_ = 0;
    t0_us_ = 0;
    a0_ = 0;
    has_drive_ = false;
    t_drive_us_ = 0;
    a_drive_ = 0;
    has_engage_ = false;
    t_engage_us_ = 0;
    a_engage_ = 0;
    v_engage_ = 0;
    engage_idx_ = 0;
    engage_recorded_ = false;
    stop_reason_ = v1::diag_stop::NONE;
    result_ = v1::diag_result::OK;
    fault_code_ = v1::fault::NONE;
    fault_request_ = v1::fault::NONE;
    ring_head_ = 0;
    ring_count_ = 0;
    vel_valid_ = false;
    n_ = 0;
    truncated_ = false;
    max_gap_us_ = 0;
    peak_disp_ = 0;
    result_ready_ = false;
    last_actual_ = 0;
    p_ = p;
    lo_ = lo;
    hi_ = hi;
    settle_ms_ = settle_ms;
    overspeed_ = overspeed;
    phase_ = p.mode == v1::diag_mode::POWER_HOLD ? Phase::PowerHold : Phase::PowerSettle;
    phase_start_us_ = now;
  }

  // 制御タスクから: 主電源待ち・POWER_HOLD の間は 5 ms ごと、記録中（fast()）は 1 ms ごと。actual は actual_count、
  // adc は VIOUT の生値、power_on はこの周期の主電源 ON 指令（記録の flags に入れるだけ）。
  void step(uint64_t now, int32_t actual, uint16_t adc, bool power_on) {
    last_actual_ = actual;
    switch (phase_) {
      case Phase::Idle:
      case Phase::Done: return;
      case Phase::PowerSettle:
        if (now - phase_start_us_ >= static_cast<uint64_t>(settle_ms_) * 1000ull) begin_release(now, actual, adc, power_on);
        return;
      case Phase::PowerHold:
        if (now - phase_start_us_ >= static_cast<uint64_t>(p_.hold_ms) * 1000ull) finish(v1::diag_result::OK);
        return;
      case Phase::ReleaseWait:
      case Phase::Sag:
      case Phase::Drive:
      case Phase::Post: break;
    }
    ring_push(now, actual);
    const int32_t v = velocity();
    const int32_t disp = actual - a0_;
    const uint64_t elapsed = now - t0_us_;
    switch (phase_) {
      case Phase::Sag: {
        uint8_t why = v1::diag_stop::NONE;
        if (near_end(actual, v)) why = v1::diag_stop::END_MARGIN;
        else if (abs32(disp) >= p_.d_max_count) why = v1::diag_stop::D_MAX;
        else if (elapsed >= static_cast<uint64_t>(p_.t_max_ms) * 1000ull) why = v1::diag_stop::T_MAX;
        if (why != v1::diag_stop::NONE) engage(now, actual, v, why);
        break;
      }
      case Phase::ReleaseWait: {
        if (near_end(actual, v)) engage(now, actual, v, v1::diag_stop::END_MARGIN);
        else if (abs32(disp) >= V1_DIAG_ENGAGE_SAG_GUARD_COUNT) engage(now, actual, v, v1::diag_stop::SAG_GUARD);
        else if (elapsed >= static_cast<uint64_t>(p_.delay_ms) * 1000ull) begin_drive(now, actual);
        break;
      }
      case Phase::Drive: {
        const int32_t travel = (actual - a_drive_) * p_.dir;  // 走行の向きが正
        const int32_t v_dir = v * p_.dir;
        const uint64_t run = now - t_drive_us_;
        uint8_t why = v1::diag_stop::NONE;
        if (near_end(actual, v)) why = v1::diag_stop::END_MARGIN;
        else if (static_cast<uint32_t>(abs32(v)) > overspeed_) why = v1::diag_stop::OVERSPEED;
        else if (travel <= -V1_DIAG_REVERSE_COUNT) why = v1::diag_stop::REVERSE;
        else if (travel >= static_cast<int32_t>(p_.d_max_count)) why = v1::diag_stop::D_RUN;
        else if (vel_valid_ && v_dir >= static_cast<int32_t>(p_.v_target_count_s)) why = v1::diag_stop::V_TARGET;
        else if (run >= V1_DIAG_NO_PULSE_MS * 1000ull && abs32(actual - a_drive_) < V1_DIAG_NO_PULSE_MIN_MOVE) {
          why = v1::diag_stop::NO_PULSE;
          fault_request_ = v1::fault::ENCODER_FAULT;  // V1Axis がラッチする（既存の意味: FAULT_RESET で基準が消える）
        } else if (run >= V1_DIAG_RUN_MAX_MS * 1000ull) {
          why = v1::diag_stop::RUN_TIMEOUT;
        }
        if (why != v1::diag_stop::NONE) engage(now, actual, v, why);
        break;
      }
      case Phase::Post:
        break;
      default: break;
    }
    record(now, actual, adc, power_on);
    if (phase_ == Phase::Post && now - t_engage_us_ >= post_us()) finish(result_);
  }

  // DIAG_START(CANCEL) / DISARM / POWER_HOLD の受信途絶。記録中ならブレーキ作動 → 後の記録を続けてから CANCELLED。
  void cancel(uint64_t now) {
    switch (phase_) {
      case Phase::PowerSettle:
      case Phase::PowerHold:
        stop_reason_ = v1::diag_stop::CANCEL;
        finish(v1::diag_result::CANCELLED);
        break;
      case Phase::ReleaseWait:
      case Phase::Sag:
      case Phase::Drive:
        engage(now, last_actual_, velocity(), v1::diag_stop::CANCEL);
        result_ = v1::diag_result::CANCELLED;
        break;
      default: break;
    }
  }

  // V1Axis の latch_fault から（出力は V1Axis が既に落としている）。記録中なら作動後の記録は続ける（状態は FAULT のまま）。
  void fault(uint16_t code, uint64_t now) {
    switch (phase_) {
      case Phase::PowerSettle:
      case Phase::PowerHold:
        stop_reason_ = v1::diag_stop::FAULT;
        result_ = v1::diag_result::FAULT;
        fault_code_ = code;
        finish(v1::diag_result::FAULT);
        break;
      case Phase::ReleaseWait:
      case Phase::Sag:
      case Phase::Drive:
        engage(now, last_actual_, velocity(), v1::diag_stop::FAULT);
        result_ = v1::diag_result::FAULT;
        fault_code_ = code;
        break;
      case Phase::Post:
        if (result_ != v1::diag_result::FAULT) {
          result_ = v1::diag_result::FAULT;
          fault_code_ = code;
        }
        break;
      default: break;
    }
  }

  // --- 出力 ---
  bool busy() const { return phase_ != Phase::Idle && phase_ != Phase::Done; }
  // 記録中（制御タスクが 1 ms ごとに step を呼ぶ）
  bool fast() const {
    return phase_ == Phase::ReleaseWait || phase_ == Phase::Sag || phase_ == Phase::Drive || phase_ == Phase::Post;
  }
  Phase phase() const { return phase_; }
  bool brake_release() const { return brake_release_; }
  int pwm_permille() const { return pwm_permille_; }  // カウント増の向きが正
  uint32_t test_id() const { return p_.request_id; }
  uint8_t mode() const { return p_.mode; }
  size_t sample_count() const { return n_; }
  const v1::DiagSample& sample(size_t i) const { return samples_[i]; }

  // 走行中の無パルス: V1Axis が 1 回だけ取り出して ENCODER_FAULT をラッチする
  bool take_fault_request(uint16_t* code) {
    if (fault_request_ == v1::fault::NONE) return false;
    *code = fault_request_;
    fault_request_ = v1::fault::NONE;
    return true;
  }
  // 終わった計測の要約（1 回だけ）。boot_id・referenced_after・sign は V1Axis が埋める
  bool take_result(v1::DiagResult* out) {
    if (!result_ready_) return false;
    result_ready_ = false;
    *out = summary_;
    return true;
  }

 private:
  struct RingEntry {
    uint64_t us;
    int32_t count;
  };
  static constexpr size_t kRing = 64;

  v1::DiagStart p_{};
  Phase phase_ = Phase::Idle;
  int32_t lo_ = 0;
  int32_t hi_ = 0;
  uint32_t settle_ms_ = 0;
  uint32_t overspeed_ = 0;
  uint64_t phase_start_us_ = 0;
  int32_t last_actual_ = 0;

  bool brake_release_ = false;
  int pwm_permille_ = 0;

  uint64_t t0_us_ = 0;     // ブレーキ解除指令（t = 0）
  int32_t a0_ = 0;
  bool has_drive_ = false;
  uint64_t t_drive_us_ = 0;
  int32_t a_drive_ = 0;
  bool has_engage_ = false;
  uint64_t t_engage_us_ = 0;
  int32_t a_engage_ = 0;
  int32_t v_engage_ = 0;
  size_t engage_idx_ = 0;  // ブレーキ作動指令の点の番号（その点の flags は作動）
  bool engage_recorded_ = false;
  uint8_t stop_reason_ = v1::diag_stop::NONE;
  uint8_t result_ = v1::diag_result::OK;
  uint16_t fault_code_ = v1::fault::NONE;
  uint16_t fault_request_ = v1::fault::NONE;

  RingEntry ring_[kRing]{};
  size_t ring_head_ = 0;
  size_t ring_count_ = 0;
  bool vel_valid_ = false;

  v1::DiagSample samples_[V1_DIAG_MAX_SAMPLES]{};
  size_t n_ = 0;
  bool truncated_ = false;
  uint32_t max_gap_us_ = 0;
  int32_t peak_disp_ = 0;

  v1::DiagResult summary_{};
  bool result_ready_ = false;

  static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

  uint64_t post_us() const {
    return (p_.mode == v1::diag_mode::SAG ? V1_DIAG_POST_SAG_MS : V1_DIAG_POST_ENGAGE_MS) * 1000ull;
  }

  void begin_release(uint64_t now, int32_t actual, uint16_t adc, bool power_on) {
    t0_us_ = now;
    a0_ = actual;
    brake_release_ = true;
    pwm_permille_ = 0;
    phase_ = p_.mode == v1::diag_mode::SAG ? Phase::Sag : Phase::ReleaseWait;
    ring_count_ = 0;
    ring_head_ = 0;
    ring_push(now, actual);
    if (phase_ == Phase::ReleaseWait && p_.delay_ms == 0) begin_drive(now, actual);
    record(now, actual, adc, power_on);
  }

  void begin_drive(uint64_t now, int32_t actual) {
    phase_ = Phase::Drive;
    has_drive_ = true;
    t_drive_us_ = now;
    a_drive_ = actual;
    pwm_permille_ = p_.dir * static_cast<int>(p_.duty_permille);
  }

  // PWM 0 とブレーキ作動を同じ周期で指令する
  void engage(uint64_t now, int32_t actual, int32_t v, uint8_t why) {
    brake_release_ = false;
    pwm_permille_ = 0;
    has_engage_ = true;
    t_engage_us_ = now;
    a_engage_ = actual;
    v_engage_ = v;
    stop_reason_ = why;
    engage_idx_ = n_;  // 次に記録する点（この周期の点）
    engage_recorded_ = n_ < V1_DIAG_MAX_SAMPLES;
    phase_ = Phase::Post;
  }

  // 予測位置（いまの速度で V1_DIAG_LOOKAHEAD_MS 進んだ位置）か現在位置が [lo, hi] の外
  bool near_end(int32_t actual, int32_t v) const {
    const int64_t ahead = static_cast<int64_t>(actual) + static_cast<int64_t>(v) * V1_DIAG_LOOKAHEAD_MS / 1000;
    return actual < lo_ || actual > hi_ || ahead < lo_ || ahead > hi_;
  }

  void ring_push(uint64_t now, int32_t c) {
    ring_[ring_head_] = RingEntry{now, c};
    ring_head_ = (ring_head_ + 1) % kRing;
    if (ring_count_ < kRing) ring_count_ += 1;
  }

  // 直近の点と、V1_DIAG_VEL_WINDOW_US 以上前の最も新しい点（無ければ最古の点）の差 [count/s]。履歴が短ければ 0（vel_valid_ = false）
  int32_t velocity() {
    vel_valid_ = false;
    if (ring_count_ < 2) return 0;
    const RingEntry& last = ring_[(ring_head_ + kRing - 1) % kRing];
    const RingEntry* ref = nullptr;
    for (size_t k = 1; k < ring_count_; ++k) {
      const RingEntry& e = ring_[(ring_head_ + kRing - 1 - k) % kRing];
      ref = &e;
      if (last.us - e.us >= V1_DIAG_VEL_WINDOW_US) break;
    }
    const uint64_t dt = last.us - ref->us;
    if (dt < V1_DIAG_VEL_MIN_SPAN_US) return 0;
    vel_valid_ = true;
    return static_cast<int32_t>((static_cast<int64_t>(last.count - ref->count) * 1000000) / static_cast<int64_t>(dt));
  }

  void record(uint64_t now, int32_t actual, uint16_t adc, bool power_on) {
    const uint32_t t = static_cast<uint32_t>(now - t0_us_);
    if (n_ > 0) {
      const uint32_t gap = t - samples_[n_ - 1].t_us;
      if (gap > max_gap_us_) max_gap_us_ = gap;
    }
    const int32_t disp = actual - a0_;
    if (abs32(disp) > abs32(peak_disp_)) peak_disp_ = disp;
    if (n_ >= V1_DIAG_MAX_SAMPLES) {
      truncated_ = true;
      return;
    }
    v1::DiagSample& s = samples_[n_++];
    s.t_us = t;
    s.count = actual;
    s.pwm_permille = static_cast<int16_t>(pwm_permille_);
    s.adc_raw = adc;
    s.flags = static_cast<uint8_t>((brake_release_ ? v1::diag_flags::BRAKE_RELEASE : 0) |
                                   (power_on ? v1::diag_flags::POWER_ON : 0));
  }

  // ブレーキ作動指令の点から、V1_DIAG_SETTLE_WINDOW_US の間 ±V1_DIAG_SETTLE_BAND_COUNT に収まり始めた最初の点。無ければ n_
  size_t settle_index() const {
    if (!has_engage_ || !engage_recorded_) return n_;
    for (size_t i = engage_idx_; i < n_; ++i) {
      if (samples_[n_ - 1].t_us - samples_[i].t_us < V1_DIAG_SETTLE_WINDOW_US) return n_;  // 窓が記録の終わりを越える
      bool ok = true;
      for (size_t j = i + 1; j < n_ && samples_[j].t_us - samples_[i].t_us <= V1_DIAG_SETTLE_WINDOW_US; ++j) {
        if (abs32(samples_[j].count - samples_[i].count) > V1_DIAG_SETTLE_BAND_COUNT) {
          ok = false;
          break;
        }
      }
      if (ok) return i;
    }
    return n_;
  }

  void finish(uint8_t result) {
    brake_release_ = false;
    pwm_permille_ = 0;
    v1::DiagResult& r = summary_;
    memset(&r, 0, sizeof(r));
    r.test_id = p_.request_id;
    r.mode = p_.mode;
    r.result = result;
    r.stop_reason = stop_reason_;
    r.fault_code = fault_code_;
    r.sample_count = static_cast<uint16_t>(n_);
    r.sample_period_us = V1_DIAG_SAMPLE_PERIOD_US;
    r.max_sample_gap_us = static_cast<uint16_t>(max_gap_us_ > 0xFFFF ? 0xFFFF : max_gap_us_);
    r.start_count = a0_;
    r.t_drive_us = has_drive_ ? static_cast<uint32_t>(t_drive_us_ - t0_us_) : v1::DIAG_TIME_NONE;
    r.t_engage_us = has_engage_ ? static_cast<uint32_t>(t_engage_us_ - t0_us_) : v1::DIAG_TIME_NONE;
    r.disp_at_engage = has_engage_ ? a_engage_ - a0_ : 0;
    r.final_disp = n_ > 0 ? samples_[n_ - 1].count - a0_ : 0;
    r.peak_disp = peak_disp_;
    r.v_at_engage_count_s = has_engage_ ? v_engage_ : 0;
    r.t_settle_us = v1::DIAG_TIME_NONE;
    const size_t si = settle_index();
    if (si < n_) {
      r.t_settle_us = samples_[si].t_us - samples_[engage_idx_].t_us;
      r.disp_engage_to_settle = samples_[si].count - samples_[engage_idx_].count;
    } else if (has_engage_) {
      r.flags |= v1::diag_flags::NOT_SETTLED;
    }
    if (truncated_) r.flags |= v1::diag_flags::TRUNCATED;
    r.adc_first = n_ > 0 ? samples_[0].adc_raw : 0;
    r.drive_pwm_permille = static_cast<int16_t>(p_.mode == v1::diag_mode::ENGAGE ? p_.dir * static_cast<int>(p_.duty_permille) : 0);
    phase_ = Phase::Done;
    result_ready_ = true;
  }
};

#endif  // V1_DIAG_BRAKE_TEST
