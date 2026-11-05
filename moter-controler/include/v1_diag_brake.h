#pragma once

// 計測専用（-DV1_DIAG_BRAKE_TEST=1、env:m5stack-cores3-v1-diag だけ）: ブレーキの効き始めと、ブレーキ解除・PWM 0 の沈みを
// ブレーキ指令・PWM・エンコーダ・時刻を揃えて記録する。r10 の監視（PR #50）の仮値（沈み 200 count/s、ブレーキ作動 150 ms、
// 解除待ち 300 ms、保持の包絡線 150 count/s）を実測で置き換えるための道具で、機能ではない。
// 手順と安全の注意: moter-controler/docs/brake_sag_measurement.md
//
// 純粋（Arduino 非依存、ヘッダのみ）。V1Axis（v1_axis.h）が受理条件・状態（v1::state::DIAG）・主電源・異常のラッチを受け持ち、
// ここは「区間（主電源 ON）→ 電源待ち → 解除 → 記録 → 作動 → 後の記録 → 待機」の段階と、止める判定と、記録・要約だけを持つ。
// 座標は actual_count（基準からの位置。符号は掛けない）。pwm_permille はカウント増の向きが正（STATUS と同じ）。
// 量産の組み立て（V1_DIAG_BRAKE_TEST=0）では中身が空になる。
//
// 区間（2026-11-02 の見直し、PR #64 の指摘 2）: DIAG に入った時刻を主電源 ON 指令の始まりとし、最初の計測は区間の始まりから
// V1_DIAG_POWER_SETTLE_MS（≥ 5 s、校正と同じ）経ってからブレーキを解除する。計測が普通に終われば DISARMED に戻らず「待機」
// （主電源 ON・ブレーキ作動・PWM 0）に残り、同じ区間の次の計測は電源待ちなしで始まる。区間を閉じる（主電源 OFF）のは
// DIAG_START(CANCEL)・DISARM・受信途絶（待機・電源待ちの間に 2 s）・待機の期限・異常・作動後に端の余裕を越えたとき。

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "v1_codec.h"

#if V1_DIAG_BRAKE_TEST

// --- 上限（DIAG_START がこれを超えたら INVALID_PARAM で拒否する。黙って丸めない） ---
constexpr uint16_t V1_DIAG_SAG_T_MAX_CAP_MS = 500;            // SAG: ブレーキ解除の最長
constexpr uint16_t V1_DIAG_SAG_D_MAX_CAP_COUNT = 200;         // SAG: ブレーキ作動を指令する |変位|（の上限。止まる位置ではない）
constexpr uint16_t V1_DIAG_ENGAGE_DUTY_MIN_PERMILLE = 100;    // ENGAGE: これ未満は機構が動かないことがある（無パルスで ENCODER_FAULT になる）
constexpr uint16_t V1_DIAG_ENGAGE_DUTY_CAP_PERMILLE = 250;    // ENGAGE: |Duty| の上限 25 %
constexpr uint16_t V1_DIAG_ENGAGE_D_RUN_CAP_COUNT = 600;      // ENGAGE: ブレーキ作動を指令する走行距離（の上限）
constexpr uint16_t V1_DIAG_ENGAGE_V_TARGET_CAP_COUNT_S = 3000;
constexpr uint16_t V1_DIAG_ENGAGE_DELAY_CAP_MS = 200;         // ENGAGE: 解除指令から走り始めるまでの待ちの上限
constexpr uint32_t V1_DIAG_POWER_HOLD_CAP_MS = 120000;        // POWER_HOLD の最長（CAL_POWER_HOLD_TIMEOUT_MS と同じ）
// --- 固定の値 ---
constexpr uint32_t V1_DIAG_POWER_SETTLE_MS = 5000;            // 区間の始まり（主電源 ON 指令）から最初のブレーキ解除まで（校正と同じ。下限）
static_assert(CAL_POWER_SETTLE_MS >= V1_DIAG_POWER_SETTLE_MS, "計測の電源待ちは校正（CAL_POWER_SETTLE_MS）と同じ 5 s 以上");
constexpr uint32_t V1_DIAG_SESSION_IDLE_MS = 120000;          // 計測が終わった後の待機の最長（その後は主電源 OFF）
constexpr uint32_t V1_DIAG_RUN_MAX_MS = 300;                  // ENGAGE: 走行時間の上限（記録が 1 s に収まるように）
constexpr uint32_t V1_DIAG_RX_TIMEOUT_MS = 2000;              // 待機・電源待ちの間、PC から何も来ないまま経ったら区間を閉じる（主電源 OFF）
constexpr uint32_t V1_DIAG_POWER_HOLD_RX_TIMEOUT_MS = V1_DIAG_RX_TIMEOUT_MS;
constexpr uint32_t V1_DIAG_POST_SAG_MS = 400;                 // SAG: ブレーキ作動指令の後に記録する時間
constexpr uint32_t V1_DIAG_POST_ENGAGE_MS = 500;              // ENGAGE: 同（≥ BRAKE_APPLY_WAIT_MS）
constexpr int32_t V1_DIAG_END_MARGIN_COUNT = V1_SOFT_RESERVE_COUNT;  // ソフトリミットの内側に残す余裕（再生バッファと同じ 100）
// 止まるまでの見積もり（PR #64 の指摘 5）: ブレーキ作動の時間は分かっていない（仮値 150 ms）ので 300 ms とし、その間に重力で
// 加速しうる（向きは姿勢次第）として |v|·t + ½·a·t²（a = 5000 count/s²）。端の余裕（100 count）の手前で、両向きに見る。
constexpr uint32_t V1_DIAG_STOP_TIME_MS = 300;
constexpr uint32_t V1_DIAG_STOP_ACCEL_COUNT_S2 = 5000;
constexpr int32_t V1_DIAG_ENGAGE_SAG_GUARD_COUNT = 100;       // ENGAGE: 走り始める前の待ちの間に、これだけ沈んだら止める
// 無パルス（PR #64 の指摘 1）: 走行中（|PWM| ≥ 100 permille）、最後にカウントが 1 以上変わってから（走り始めからも）この時間
// 変わらなければ止めて ENCODER_FAULT。加えて走り始めから同じ時間で 3 count 動いていなければ同じ扱い（±1 のちらつきを除く）
constexpr uint32_t V1_DIAG_NO_PULSE_MS = 150;
constexpr int32_t V1_DIAG_NO_PULSE_MIN_MOVE = 3;
constexpr int32_t V1_DIAG_REVERSE_COUNT = 20;                 // ENGAGE: 指令と逆向きにこれだけ動いたら止める
constexpr uint32_t V1_DIAG_VEL_WINDOW_US = 20000;             // 速度の窓（20 ms。1 ms 記録で 1 count = 50 count/s の分解能）
constexpr uint32_t V1_DIAG_VEL_MIN_SPAN_US = 10000;           // V_TARGET・要約の速度に要る最短の履歴（端の見積もりは短い履歴でも出す）
constexpr uint32_t V1_DIAG_SETTLE_WINDOW_US = 20000;          // 静止の判定: この時間 ±V1_DIAG_SETTLE_BAND_COUNT に収まる
constexpr int32_t V1_DIAG_SETTLE_BAND_COUNT = 1;              // （20 ms で 1 count 以下 ≈ |v| < 50 count/s）
constexpr size_t V1_DIAG_MAX_SAMPLES = 1024;                  // SAG ≤ 500 + 400 点、ENGAGE ≤ 200 + 300 + 500 点（1 ms 記録）
constexpr uint16_t V1_DIAG_SAMPLE_PERIOD_US = 1000;           // 公称の記録周期（制御タスクが記録中だけ 1 ms ごとに回す）

// 止まるまでの見積もり [count]: |v|·t + ½·a·t²（v = 0 でも 225 count）
inline int64_t v1_diag_stop_projection(int64_t v) {
  const int64_t av = v < 0 ? -v : v;
  const int64_t t = V1_DIAG_STOP_TIME_MS;
  return av * t / 1000 + static_cast<int64_t>(V1_DIAG_STOP_ACCEL_COUNT_S2) * t * t / 2000000;
}

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

// 開始位置に余裕があるか。[lo, hi] = ソフトリミットから V1_DIAG_END_MARGIN_COUNT 内側。受理のときと、ブレーキ解除の直前（指摘 3）に見る。
// SAG は向きが分からない（荷重の向きは軸と座席の姿勢で決まる）ので両側に d_max + 止まるまでの見積もり（v = 0: 225）。
// ENGAGE は走行の向きに D_run + 見積もり（v = v_target）、反対側に待ちの間の沈みの上限（100）+ 見積もり（v = 0）。
inline bool v1_diag_room_ok(const v1::DiagStart& d, int32_t a0, int32_t lo, int32_t hi) {
  const int64_t a = a0;
  if (d.mode == v1::diag_mode::SAG) {
    const int64_t need = static_cast<int64_t>(d.d_max_count) + v1_diag_stop_projection(0);
    return a - need >= lo && a + need <= hi;
  }
  if (d.mode == v1::diag_mode::ENGAGE) {
    const int64_t ahead = static_cast<int64_t>(d.d_max_count) + v1_diag_stop_projection(d.v_target_count_s);
    const int64_t behind = V1_DIAG_ENGAGE_SAG_GUARD_COUNT + v1_diag_stop_projection(0);
    if (d.dir > 0) return a + ahead <= hi && a - behind >= lo;
    return a - ahead >= lo && a + behind <= hi;
  }
  return true;
}

class V1DiagBrake {
 public:
  // Off: 区間なし（DISARMED）。Hold: 待機（主電源 ON・ブレーキ作動・PWM 0）。Settle: 計測を受けて電源待ち。
  // ReleaseWait / Sag / Drive / Post: 記録中（ブレーキ解除から作動後の記録の終わりまで）。
  enum class Phase : uint8_t { Off, Hold, Settle, ReleaseWait, Sag, Drive, Post };

  // 区間を開く（V1Axis が DISARMED → DIAG にした時刻 = 主電源 ON 指令の始まり）。settle_ms は V1_DIAG_POWER_SETTLE_MS 未満なら引き上げる
  void open_session(uint64_t now, uint32_t settle_ms) {
    session_start_us_ = now;
    settle_us_ = static_cast<uint64_t>(settle_ms < V1_DIAG_POWER_SETTLE_MS ? V1_DIAG_POWER_SETTLE_MS : settle_ms) * 1000ull;
    hold_deadline_us_ = now + static_cast<uint64_t>(V1_DIAG_SESSION_IDLE_MS) * 1000ull;
    hold_req_id_ = 0;
    end_after_post_ = false;
    fault_request_ = v1::fault::NONE;
    phase_ = Phase::Hold;
  }

  // 受理した DIAG_START（区間が開いていて待機中のときだけ。V1Axis が確かめる）。lo / hi は端の余裕の内側、overspeed は CONFIG の値。
  // POWER_HOLD は待機の期限を延ばすだけ（記録は消さない）。SAG / ENGAGE は記録を消して電源待ちへ（区間の電源待ちが済んでいれば次の周期に解除）。
  void start(const v1::DiagStart& p, uint64_t now, int32_t lo, int32_t hi, uint32_t overspeed) {
    if (p.mode == v1::diag_mode::POWER_HOLD) {
      hold_req_id_ = p.request_id;
      hold_deadline_us_ = now + static_cast<uint64_t>(p.hold_ms) * 1000ull;
      phase_ = Phase::Hold;
      return;
    }
    // *this = V1DiagBrake{} は記録（16 KB）の一時オブジェクトを制御タスクのスタック（8 KB）に作るので使わない
    hold_req_id_ = 0;  // 電源待ちの POWER_HOLD は、この計測に引き継ぐ（区間の終わりに POWER_HOLD の結果は出さない）
    brake_release_ = false;
    pwm_permille_ = 0;
    t0_us_ = now;
    a0_ = 0;
    session_ms_at_release_ = 0;
    has_drive_ = false;
    t_drive_us_ = 0;
    a_drive_ = 0;
    last_move_us_ = 0;
    last_move_count_ = 0;
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
    post_margin_ = false;
    end_after_post_ = false;
    ring_head_ = 0;
    ring_count_ = 0;
    vel_valid_ = false;
    n_ = 0;
    truncated_ = false;
    max_gap_us_ = 0;
    peak_disp_ = 0;
    p_ = p;
    rec_id_ = p.request_id;
    lo_ = lo;
    hi_ = hi;
    overspeed_ = overspeed;
    phase_ = Phase::Settle;
  }

  // 制御タスクから: 待機・電源待ちの間は 5 ms ごと、記録中（fast()）は 1 ms ごと。actual は actual_count、
  // adc は VIOUT の生値、power_on はこの周期の主電源 ON 指令（記録の flags に入れるだけ）。
  void step(uint64_t now, int32_t actual, uint16_t adc, bool power_on) {
    last_actual_ = actual;
    switch (phase_) {
      case Phase::Off: return;
      case Phase::Hold:
        if (now >= hold_deadline_us_) end_session(v1::diag_result::OK, v1::diag_stop::NONE, v1::fault::NONE);
        return;
      case Phase::Settle:
        if (power_settled(now)) begin_release(now, actual, adc, power_on);
        return;
      case Phase::ReleaseWait:
      case Phase::Sag:
      case Phase::Drive:
      case Phase::Post: break;
    }
    ring_push(now, actual);
    const int32_t v = velocity();
    const int32_t v_guard = guard_velocity();
    const int32_t disp = actual - a0_;
    const uint64_t elapsed = now - t0_us_;
    switch (phase_) {
      case Phase::Sag: {
        uint8_t why = v1::diag_stop::NONE;
        if (near_end(actual, v_guard)) why = v1::diag_stop::END_MARGIN;
        else if (abs32(disp) >= p_.d_max_count) why = v1::diag_stop::D_MAX;
        else if (elapsed >= static_cast<uint64_t>(p_.t_max_ms) * 1000ull) why = v1::diag_stop::T_MAX;
        if (why != v1::diag_stop::NONE) engage(now, actual, v, why);
        break;
      }
      case Phase::ReleaseWait: {
        if (near_end(actual, v_guard)) engage(now, actual, v, v1::diag_stop::END_MARGIN);
        else if (abs32(disp) >= V1_DIAG_ENGAGE_SAG_GUARD_COUNT) engage(now, actual, v, v1::diag_stop::SAG_GUARD);
        else if (elapsed >= static_cast<uint64_t>(p_.delay_ms) * 1000ull) begin_drive(now, actual);
        break;
      }
      case Phase::Drive: {
        const int32_t travel = (actual - a_drive_) * p_.dir;  // 走行の向きが正
        const int32_t v_dir = v * p_.dir;
        const uint64_t run = now - t_drive_us_;
        if (actual != last_move_count_) {  // 1 count でも変われば「動いている」の起点を更新する
          last_move_count_ = actual;
          last_move_us_ = now;
        }
        uint8_t why = v1::diag_stop::NONE;
        if (near_end(actual, v_guard)) why = v1::diag_stop::END_MARGIN;
        else if (static_cast<uint32_t>(abs32(v)) > overspeed_) why = v1::diag_stop::OVERSPEED;
        else if (travel <= -V1_DIAG_REVERSE_COUNT) why = v1::diag_stop::REVERSE;
        else if (travel >= static_cast<int32_t>(p_.d_max_count)) why = v1::diag_stop::D_RUN;
        else if (vel_valid_ && v_dir >= static_cast<int32_t>(p_.v_target_count_s)) why = v1::diag_stop::V_TARGET;
        else if (no_pulse(now, actual, run)) {
          why = v1::diag_stop::NO_PULSE;
          fault_request_ = v1::fault::ENCODER_FAULT;  // V1Axis がラッチし、その場で基準を消す
        } else if (run >= V1_DIAG_RUN_MAX_MS * 1000ull) {
          why = v1::diag_stop::RUN_TIMEOUT;
        }
        if (why != v1::diag_stop::NONE) engage(now, actual, v, why);
        break;
      }
      case Phase::Post:
        // 作動の後に端の余裕を越えた（止まるまでの見積もりが外れた）: 結果を不成功にし、待機せず区間を閉じる（主電源 OFF）
        if (actual < lo_ || actual > hi_) post_margin_ = true;
        break;
      default: break;
    }
    if (!record(now, actual, adc, power_on)) return;  // 記録が満杯: ブレーキ作動・PWM 0 にして終わった
    if (phase_ == Phase::Post && now - t_engage_us_ >= post_us()) finish_test(result_, now);
  }

  // DIAG_START(CANCEL) / DISARM / 受信途絶: 区間を閉じる。記録中ならブレーキ作動 → 後の記録を続けてから CANCELLED で閉じる。
  void cancel(uint64_t now) {
    switch (phase_) {
      case Phase::Hold:
        end_session(v1::diag_result::CANCELLED, v1::diag_stop::CANCEL, v1::fault::NONE);
        break;
      case Phase::Settle:
        stop_reason_ = v1::diag_stop::CANCEL;
        end_after_post_ = true;
        finish_test(v1::diag_result::CANCELLED, now);
        break;
      case Phase::ReleaseWait:
      case Phase::Sag:
      case Phase::Drive:
        engage(now, last_actual_, velocity(), v1::diag_stop::CANCEL);
        result_ = v1::diag_result::CANCELLED;
        end_after_post_ = true;
        break;
      case Phase::Post:
        end_after_post_ = true;
        break;
      default: break;
    }
  }

  // V1Axis の latch_fault から（出力は V1Axis が既に落としている。主電源も OFF）。記録中なら作動後の記録は続け、その後に区間を閉じる。
  void fault(uint16_t code, uint64_t now) {
    switch (phase_) {
      case Phase::Hold:
        end_session(v1::diag_result::FAULT, v1::diag_stop::FAULT, code);
        break;
      case Phase::Settle:
        stop_reason_ = v1::diag_stop::FAULT;
        fault_code_ = code;
        end_after_post_ = true;
        finish_test(v1::diag_result::FAULT, now);
        break;
      case Phase::ReleaseWait:
      case Phase::Sag:
      case Phase::Drive:
        engage(now, last_actual_, velocity(), v1::diag_stop::FAULT);
        result_ = v1::diag_result::FAULT;
        fault_code_ = code;
        end_after_post_ = true;
        break;
      case Phase::Post:
        if (result_ != v1::diag_result::FAULT) {
          result_ = v1::diag_result::FAULT;
          fault_code_ = code;
        }
        end_after_post_ = true;
        break;
      default: break;
    }
  }

  // --- 出力 ---
  bool busy() const { return phase_ != Phase::Off; }           // 区間が開いている（V1Axis の state は DIAG）
  bool idle() const { return phase_ == Phase::Hold; }          // 待機中（次の DIAG_START を受けられる）
  bool test_busy() const { return busy() && phase_ != Phase::Hold; }
  // 記録中（制御タスクが 1 ms ごとに step を呼ぶ）
  bool fast() const {
    return phase_ == Phase::ReleaseWait || phase_ == Phase::Sag || phase_ == Phase::Drive || phase_ == Phase::Post;
  }
  Phase phase() const { return phase_; }
  bool brake_release() const { return brake_release_; }
  int pwm_permille() const { return pwm_permille_; }  // カウント増の向きが正
  uint32_t test_id() const { return rec_id_; }        // 記録の持ち主（最後の SAG / ENGAGE の request_id）
  uint8_t mode() const { return p_.mode; }
  size_t sample_count() const { return n_; }
  const v1::DiagSample& sample(size_t i) const { return samples_[i]; }
  bool power_settled(uint64_t now) const { return busy() && now - session_start_us_ >= settle_us_; }
  uint32_t session_ms(uint64_t now) const {
    if (!busy()) return 0;
    return static_cast<uint32_t>((now - session_start_us_) / 1000ull);
  }

  // 走行中の無パルス: V1Axis が 1 回だけ取り出して ENCODER_FAULT をラッチする（基準もその場で消す）
  bool take_fault_request(uint16_t* code) {
    if (fault_request_ == v1::fault::NONE) return false;
    *code = fault_request_;
    fault_request_ = v1::fault::NONE;
    return true;
  }
  // 終わった計測（または POWER_HOLD の区間）の要約（1 回だけ）。boot_id・referenced_after・sign は V1Axis が埋める
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
  uint32_t rec_id_ = 0;
  Phase phase_ = Phase::Off;
  int32_t lo_ = 0;
  int32_t hi_ = 0;
  uint32_t overspeed_ = 0;
  int32_t last_actual_ = 0;

  // 区間
  uint64_t session_start_us_ = 0;
  uint64_t settle_us_ = 0;
  uint64_t hold_deadline_us_ = 0;
  uint32_t hold_req_id_ = 0;  // 待機中の POWER_HOLD の request_id（区間の終わりに DIAG_RESULT(POWER_HOLD) を出す）。0 = なし
  bool end_after_post_ = false;  // 後の記録が終わったら待機せず区間を閉じる（中止・異常）

  bool brake_release_ = false;
  int pwm_permille_ = 0;

  uint64_t t0_us_ = 0;     // ブレーキ解除指令（t = 0）
  int32_t a0_ = 0;
  uint16_t session_ms_at_release_ = 0;
  bool has_drive_ = false;
  uint64_t t_drive_us_ = 0;
  int32_t a_drive_ = 0;
  uint64_t last_move_us_ = 0;
  int32_t last_move_count_ = 0;
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
  bool post_margin_ = false;

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

  // 電源待ちが済んだ。解除の直前に位置を見直す（受理は数秒前のことがある。指摘 3）。余裕が無ければ解除せず ABORTED（待機に戻る）
  void begin_release(uint64_t now, int32_t actual, uint16_t adc, bool power_on) {
    t0_us_ = now;
    a0_ = actual;
    const uint32_t sm = session_ms(now);
    session_ms_at_release_ = static_cast<uint16_t>(sm > 0xFFFF ? 0xFFFF : sm);
    if (!v1_diag_room_ok(p_, actual, lo_, hi_)) {
      stop_reason_ = v1::diag_stop::ROOM_LOST;
      session_ms_at_release_ = 0;
      finish_test(v1::diag_result::ABORTED, now);
      return;
    }
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
    last_move_us_ = now;
    last_move_count_ = actual;
    pwm_permille_ = p_.dir * static_cast<int>(p_.duty_permille);
  }

  // 走行中の無パルス: 最後にカウントが変わってから（走り始めからも）V1_DIAG_NO_PULSE_MS 変わらない、または走り始めから
  // V1_DIAG_NO_PULSE_MS 経っても V1_DIAG_NO_PULSE_MIN_MOVE 動いていない（±1 のちらつきだけ）。Drive の |PWM| は常に ≥ 100 permille
  bool no_pulse(uint64_t now, int32_t actual, uint64_t run) const {
    const uint64_t win = static_cast<uint64_t>(V1_DIAG_NO_PULSE_MS) * 1000ull;
    if (now - last_move_us_ >= win) return true;
    return run >= win && abs32(actual - a_drive_) < V1_DIAG_NO_PULSE_MIN_MOVE;
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

  // 止まるまでの見積もり（|v|·t ± ½·a·t²、両向き）か現在位置が [lo, hi] の外
  bool near_end(int32_t actual, int32_t v) const {
    const int64_t a = actual;
    const int64_t drift = static_cast<int64_t>(v) * V1_DIAG_STOP_TIME_MS / 1000;
    const int64_t grav = v1_diag_stop_projection(0);
    return a < lo_ || a > hi_ || a + drift + grav > hi_ || a + drift - grav < lo_;
  }

  void ring_push(uint64_t now, int32_t c) {
    ring_[ring_head_] = RingEntry{now, c};
    ring_head_ = (ring_head_ + 1) % kRing;
    if (ring_count_ < kRing) ring_count_ += 1;
  }

  // 直近の点と、V1_DIAG_VEL_WINDOW_US 以上前の最も新しい点（無ければ最古の点）の差 [count/s]。span は実際の時間差
  int32_t ring_velocity(uint64_t* span) const {
    *span = 0;
    if (ring_count_ < 2) return 0;
    const RingEntry& last = ring_[(ring_head_ + kRing - 1) % kRing];
    const RingEntry* ref = nullptr;
    for (size_t k = 1; k < ring_count_; ++k) {
      const RingEntry& e = ring_[(ring_head_ + kRing - 1 - k) % kRing];
      ref = &e;
      if (last.us - e.us >= V1_DIAG_VEL_WINDOW_US) break;
    }
    const uint64_t dt = last.us - ref->us;
    *span = dt;
    if (dt == 0) return 0;
    return static_cast<int32_t>((static_cast<int64_t>(last.count - ref->count) * 1000000) / static_cast<int64_t>(dt));
  }

  // V_TARGET・要約に使う速度。履歴が V1_DIAG_VEL_MIN_SPAN_US 未満なら 0（vel_valid_ = false）
  int32_t velocity() {
    uint64_t span = 0;
    const int32_t v = ring_velocity(&span);
    vel_valid_ = span >= V1_DIAG_VEL_MIN_SPAN_US;
    return vel_valid_ ? v : 0;
  }

  // 端の見積もりに使う速度: 履歴が短くても（解除直後の 10 ms も）ある分で出す（1 count / 1 ms = 1000 count/s と粗く大きめに出る側）
  int32_t guard_velocity() const {
    uint64_t span = 0;
    return ring_velocity(&span);
  }

  // 1 点記録する。満杯なら（指摘 4）ブレーキ作動・PWM 0 にして不成功（ABORTED、TRUNCATED）で終わり、false を返す
  bool record(uint64_t now, int32_t actual, uint16_t adc, bool power_on) {
    const uint32_t t = static_cast<uint32_t>(now - t0_us_);
    if (n_ > 0) {
      const uint32_t gap = t - samples_[n_ - 1].t_us;
      if (gap > max_gap_us_) max_gap_us_ = gap;
    }
    const int32_t disp = actual - a0_;
    if (abs32(disp) > abs32(peak_disp_)) peak_disp_ = disp;
    if (n_ >= V1_DIAG_MAX_SAMPLES) {
      truncated_ = true;
      if (phase_ != Phase::Post) engage(now, actual, velocity(), v1::diag_stop::BUFFER_FULL);
      finish_test(result_ == v1::diag_result::FAULT ? result_ : v1::diag_result::ABORTED, now);
      return false;
    }
    v1::DiagSample& s = samples_[n_++];
    s.t_us = t;
    s.count = actual;
    s.pwm_permille = static_cast<int16_t>(pwm_permille_);
    s.adc_raw = adc;
    s.flags = static_cast<uint8_t>((brake_release_ ? v1::diag_flags::BRAKE_RELEASE : 0) |
                                   (power_on ? v1::diag_flags::POWER_ON : 0));
    return true;
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

  // 区間を閉じる（待機・電源待ちから）。待機中の POWER_HOLD があれば、その DIAG_RESULT を出す
  void end_session(uint8_t result, uint8_t why, uint16_t code) {
    brake_release_ = false;
    pwm_permille_ = 0;
    if (hold_req_id_ != 0) {
      v1::DiagResult& r = summary_;
      memset(&r, 0, sizeof(r));
      r.test_id = hold_req_id_;
      r.mode = v1::diag_mode::POWER_HOLD;
      r.result = result;
      r.stop_reason = why;
      r.fault_code = code;
      r.sample_period_us = V1_DIAG_SAMPLE_PERIOD_US;
      r.t_drive_us = v1::DIAG_TIME_NONE;
      r.t_engage_us = v1::DIAG_TIME_NONE;
      r.t_settle_us = v1::DIAG_TIME_NONE;
      result_ready_ = true;
    }
    hold_req_id_ = 0;
    end_after_post_ = false;
    phase_ = Phase::Off;
  }

  // SAG / ENGAGE の終わり: 要約を作り、待機に戻る（普通に終わった・解除しなかった・満杯）か区間を閉じる（中止・異常・作動後に端を越えた）
  void finish_test(uint8_t result, uint64_t now) {
    brake_release_ = false;
    pwm_permille_ = 0;
    if (post_margin_ && result == v1::diag_result::OK) result = v1::diag_result::ABORTED;
    const bool keep = !end_after_post_ && !post_margin_ && result != v1::diag_result::FAULT &&
                      result != v1::diag_result::CANCELLED;
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
    if (post_margin_) r.flags |= v1::diag_flags::POST_MARGIN;
    if (keep) r.flags |= v1::diag_flags::SESSION_KEPT;
    r.adc_first = n_ > 0 ? samples_[0].adc_raw : 0;
    r.drive_pwm_permille = static_cast<int16_t>(p_.mode == v1::diag_mode::ENGAGE ? p_.dir * static_cast<int>(p_.duty_permille) : 0);
    r.session_ms_at_release = session_ms_at_release_;
    result_ready_ = true;
    end_after_post_ = false;
    if (keep) {
      phase_ = Phase::Hold;
      hold_deadline_us_ = now + static_cast<uint64_t>(V1_DIAG_SESSION_IDLE_MS) * 1000ull;
    } else {
      phase_ = Phase::Off;
    }
  }
};

#endif  // V1_DIAG_BRAKE_TEST
