#pragma once

// r10（Issue #44・#46）: 位置制御の保護の監視 V1Supervisor。V1Axis（v1_axis.h）が方向別の経路（V1Tuning::ff_directional）で使う。
// GPIO・時計を持たない（時刻・カウント・指令を注入する）。
//
// 置き換えるもの（Issue #44）: 「Duty を出しているのにカウントが進まない → ENCODER_FAULT」の 2 つの判定（走行中の無パルス A、
// 静止 2 s の監視 B）。電流検出が無いので「エンコーダは生きているが乗員の荷重で完全に止まっている」と「エンコーダ断」は区別できない。
// そこで ENCODER_FAULT はラッチせず、次の 2 つで「見えない走行」と過負荷を抑える。
//
// 1. 見えない走行の予算（FEEDBACK_UNCERTAIN。常に優先）
//    - 包絡線: 実際に出した PWM（前の tick の出力。この tick までの区間に効いていた）の向きへ、v_env(|u|) × dt だけ「見えないまま
//      動けたかもしれない」距離を積む。v_env = V1_SUP_ENV_COUNT_S × |u| / V1_SUP_ENV_REF_PERMILLE（ref = 0 なら一律 2500）。
//      上（カウント増）と下を別々に積む。向きの反転・PWM 0 の隙間では減らさない（反転や隙間で予算を洗い直せない）。
//    - 2 つの上限（どちらかを超える出力は出さない）:
//        回の上限  env_ep[d]  ≤ V1_SUP_BUDGET_MAX_COUNT（1000）                         … DISARMED まで戻せば 0 に戻る
//        位置の上限 env_cum[d] ≤ max(D_d − 400, min(50, max(0, D_d − 20)), 0)          … 「確かな動き」か基準の取り直しまで戻らない
//      最初の回の予算は min(1000, max(0, D − 400)) = Issue #44 の式。中央付近（D ≈ 2750）なら再 ARM で 2 回強まで出せ、その後は
//      確かな動きか基準の取り直し（SET_REFERENCE / 校正）が要る
//      D_d = 最後に信用できた位置（trusted）から向き d の機械端までの距離。機械端 = CONFIG のソフトリミット ± V1_MECH_DECEL_ZONE_COUNT。
//      位置の上限を再 ARM で戻さないので、エンコーダが死んだまま ARM を繰り返しても「端から 400 count 以内へ見えないまま入る」ことは無い
//     （最小予算 50 の分を除く。下記）。荷重で押さえられただけなら、再 ARM の後に一度でも確かに動けば全部戻る。
//    - 判定は先読み: この tick の出力を出すと次の tick までに上限を超える（env + v_env × max(dt, 5 ms) > 上限）なら出力 0 にして
//      FEEDBACK_UNCERTAIN を立てる。以後は位置制御の出力を常に 0（ブレーキは触らない。下の「未決」）。
//    - 戻す唯一の根拠（確かな動き）: 出している向きへ、窓（200 ms）の中で最も戻った所から 2 count 以上の正味の移動。±1 のちらつき・
//      片相のトグル（n ↔ n+1）は 1 にしかならない。出力と逆向きの動き（押し戻された）は戻さない（Issue #44 の指示）。±2 以上の
//      大きい往復は戻してしまう（残る穴。PCNT のフィルタと A/B 遷移の妥当性の計数は後続 = encoder_pcnt.cpp:56）。
//    - FEEDBACK_UNCERTAIN の解除は明示的な操作だけ: DISARM（ブレーキ作動 → DISARMED）か FAULT → FAULT_RESET で DISARMED に戻ったとき
//     （= 再 ARM が要る）。確かな動きでは自動で解除しない（出力 0 なので確かな動きは起こらない）。
//    - 最小予算（仕様からの逸脱、sup.min_budget_count = 0 で仕様どおり）: D ≤ 400 で 0 だと端から 400 以内では端へ向かう出力を
//      1 tick も出せず、PC の目標（soft + 150〜350）に着けない。
// 2. 過負荷（LOAD_LIMITED、ラッチしない）: 飽和（|出力| ≥ duty_max × 99 %、誤差の向き）のまま 100 ms の窓ごとの「進み」（向きへ 5 count
//    または |誤差| が 3 減る）が無いまま 1 s → 出力 0。|誤差| ≤ hold_exit・誤差の向きの反転・2 s 休んだら解除。
//    中央付近で完全に押さえられた場合は 400 ms で予算が先に尽きる（FEEDBACK_UNCERTAIN）。LOAD_LIMITED は「向きへ少しずつは動いている
//    （予算は戻る）が誤差を詰められない」場面のもの。
// 3. 端の手前の制動（END_BRAKING、Issue #46、ラッチしない）: 実測速度で止まれない（v² / 2a > ソフトリミットまでの残り）なら反対向きに
//    duty_max。実測で動いていることが前提なので、この出力は包絡線に積まない（エンコーダが死ねば v_fast は 40 ms で 0 になり解除）。
// 4. ソフトリミットの外（LIMIT_RECOVERY、Issue #46）: 外向きの出力は 0 にし、内側の目標へ戻す。ラッチの判定は V1Axis::check_motion_faults
//   （decide_soft_limit）。
// 5. FOLLOWING_ERROR の猶予（LIMIT_RECOVERY、Issue #46）: 上限を超えてもエンコーダが生きていて |誤差| が減っている間は最長 2 s ラッチしない
//   （fe_may_wait。超えた直後の 200 ms は減り始めるのを待つ。上限 × 1.5 で猶予なし）。
//
// 未決（実機）: FEEDBACK_UNCERTAIN / LOAD_LIMITED の退避先は PWM 0 だけで、ブレーキは解除指令のまま（ACTIVE 中にブレーキを切り替えると
// 動き出しパルス等の条件と STATUS の BRAKE_RELEASE_CMD の意味が変わり、戻すときに解除待ちが要る）。乗員が乗ったまま PWM 0 で下がるかは要確認。

#include <math.h>
#include <stdint.h>

#include "config.h"

struct V1SupTuning {
  int32_t env_count_s = V1_SUP_ENV_COUNT_S;
  int env_ref_permille = V1_SUP_ENV_REF_PERMILLE;  // 0 で一律（|u| > 0 なら env_count_s）
  int32_t budget_max_count = V1_SUP_BUDGET_MAX_COUNT;
  int32_t end_reserve_count = V1_SUP_END_RESERVE_COUNT;
  int32_t min_budget_count = V1_SUP_MIN_BUDGET_COUNT;
  int32_t mech_decel_zone_count = V1_MECH_DECEL_ZONE_COUNT;
  int32_t end_hard_margin_count = V1_END_HARD_MARGIN_COUNT;
  int32_t renew_move_count = V1_SUP_RENEW_MOVE_COUNT;
  uint32_t renew_window_ms = V1_SUP_RENEW_WINDOW_MS;
  uint32_t projection_ms = 5;  // 先読みの最小の刻み（制御周期）
  int load_sat_ratio_permille = V1_LOAD_SAT_RATIO_PERMILLE;
  uint32_t load_sat_max_ms = V1_LOAD_SAT_MAX_MS;
  uint32_t load_window_ms = V1_LOAD_WINDOW_MS;
  int32_t load_progress_count = V1_LOAD_PROGRESS_COUNT;
  int32_t load_err_drop_count = V1_LOAD_ERR_DROP_COUNT;
  uint32_t load_rest_ms = V1_LOAD_REST_MS;
  int32_t brake_acc_count_s2 = V1_BRAKE_GUARD_ACC_COUNT_S2;  // 0 で制動の監視なし
  int32_t brake_min_vel_count_s = V1_BRAKE_GUARD_MIN_VEL_COUNT_S;
  int32_t brake_hyst_count = V1_BRAKE_GUARD_HYST_COUNT;
  int32_t soft_out_vel_count_s = V1_SOFT_OUT_VEL_COUNT_S;
  uint32_t soft_outward_grace_ms = V1_SOFT_OUTWARD_GRACE_MS;
  uint32_t soft_recover_max_ms = V1_SOFT_RECOVER_MAX_MS;
  uint32_t fe_activity_ms = V1_FE_ACTIVITY_MS;
  uint32_t fe_window_ms = V1_FE_WINDOW_MS;
  int32_t fe_min_drop_count = V1_FE_MIN_DROP_COUNT;
  uint32_t fe_grace_max_ms = V1_FE_GRACE_MAX_MS;  // 0 で猶予なし（従来どおり即ラッチ）
  uint32_t fe_turn_ms = V1_FE_TURN_MS;            // 超えた直後、減り始めるのを待つ時間（超えた瞬間は必ず増えている）
  int fe_hard_ratio_percent = V1_FE_HARD_RATIO_PERCENT;  // |誤差| ≥ 上限 × これ / 100 なら猶予なし
};

// 画面・計器用（STATUS には flags のビットだけ載る）
struct V1SupInfo {
  bool enabled = false;
  bool feedback_uncertain = false;
  bool load_limited = false;
  bool end_braking = false;
  bool soft_recovery = false;
  bool fe_recovery = false;
  uint32_t activity_age_ms = 0;  // 最後にカウントが（どちら向きでも）変わってから
  uint32_t saturation_ms = 0;    // 飽和が続いている時間（進めば取り直す）
  int32_t budget_up = 0;         // 上（カウント増）へ見えないまま出せる残り [count]（回と位置の上限の小さい方）
  int32_t budget_down = 0;
};

// SOFT_LIMIT の判定（decide_soft_limit の結果）
enum class V1SoftDecision : uint8_t { Inside = 0, Recover = 1, Latch = 2 };

class V1Supervisor {
 public:
  // 位置制御の 1 tick の入力（カウントは V1Axis の actual_count の座標。速度・出力はカウント増が正）
  struct Ctx {
    uint64_t now_us = 0;
    int64_t raw = 0;             // 生カウント
    int64_t ref_offset = 0;      // actual = raw − ref_offset
    int64_t err = 0;             // 目標 − 実位置
    int32_t soft_min = 0;
    int32_t soft_max = 0;
    double vel = 0.0;            // 実測速度 [count/s]
    long duty_max_pm = 0;
    int32_t hold_exit = 0;
  };

  V1SupTuning t;

  // 基準を取り直した（begin / SET_REFERENCE / 校正の成功 / 系統の切替）: 信用できる位置 = いま、包絡線をすべて 0、FEEDBACK_UNCERTAIN も解除
  void reset_trust(int64_t raw, uint64_t now_us) {
    trusted_raw_ = raw;
    for (int i = 0; i < 2; ++i) { env_ep_[i] = 0.0; env_cum_[i] = 0.0; }
    feedback_uncertain_ = false;
    last_raw_ = raw;
    last_change_us_ = now_us;
    ren_dir_ = 0;
  }
  // DISARMED に戻った（DISARM の完了・FAULT_RESET）: 明示的な解除。FEEDBACK_UNCERTAIN と回の上限を戻す（位置の上限は戻さない）
  void clear_episode() {
    feedback_uncertain_ = false;
    env_ep_[0] = env_ep_[1] = 0.0;
  }
  // 位置制御が止まった（DISARMED / FAULT / 校正 / ARMING のブレーキ解除前）: その回だけの状態を捨てる
  void end_session(int64_t raw, uint64_t now_us) {
    last_out_ = 0;
    last_out_guard_ = false;
    have_last_us_ = false;
    ren_dir_ = 0;
    sat_running_ = false;
    load_limited_ = false;
    braking_ = false;
    soft_out_ = false;
    fe_over_ = false;
    fe_has_ref_ = false;
    soft_recovery_ = false;
    fe_recovery_ = false;
    if (raw != last_raw_) last_change_us_ = now_us;
    last_raw_ = raw;
  }

  // 位置制御中の tick ごと、異常検出（check_motion_faults）より前に 1 回: 前の tick の出力の包絡線を積み、確かな動きなら予算を戻す。
  void observe(const Ctx& c) {
    uint64_t dt_us = 0;
    if (have_last_us_ && c.now_us > last_us_) dt_us = c.now_us - last_us_;
    last_us_ = c.now_us;
    have_last_us_ = true;
    last_dt_us_ = dt_us;
    const int64_t raw_before = last_raw_;
    if (c.raw != last_raw_) {
      last_change_us_ = c.now_us;
      last_raw_ = c.raw;
    }
    // 包絡線（制動の出力は積まない: 実測で動いていることが前提の出力）
    const int d = last_out_ > 0 ? 1 : (last_out_ < 0 ? -1 : 0);
    if (d != 0 && !last_out_guard_ && dt_us > 0) {
      const double add = env_speed(last_out_) * static_cast<double>(dt_us) / 1e6;
      env_ep_[idx(d)] += add;
      env_cum_[idx(d)] += add;
    }
    // 確かな動き: 出している向きへ、窓の中で最も戻った所から renew_move 以上
    if (d == 0) {
      ren_dir_ = 0;
    } else {
      if (ren_dir_ != d || c.now_us - ren_start_us_ >= ms(t.renew_window_ms)) {
        ren_dir_ = d;
        ren_start_us_ = c.now_us;
        ren_ext_ = raw_before;  // この区間の始まりのカウント（区間の動きはこの出力の結果）
      }
      if (static_cast<int64_t>(d) * (c.raw - ren_ext_) >= t.renew_move_count) {
        trusted_raw_ = c.raw;
        for (int i = 0; i < 2; ++i) { env_ep_[i] = 0.0; env_cum_[i] = 0.0; }
        ren_start_us_ = c.now_us;
        ren_ext_ = c.raw;
      } else if ((d > 0 && c.raw < ren_ext_) || (d < 0 && c.raw > ren_ext_)) {
        ren_ext_ = c.raw;  // 最も戻った所
      }
    }
    // FOLLOWING_ERROR の猶予に使う |誤差| の傾向（fe_window ごとに比べる）
    const int64_t fe_abs = c.err < 0 ? -c.err : c.err;
    if (!fe_has_ref_) {
      fe_has_ref_ = true;
      fe_ref_abs_ = fe_abs;
      fe_ref_us_ = c.now_us;
      fe_decreasing_ = false;
    } else if (c.now_us - fe_ref_us_ >= ms(t.fe_window_ms)) {
      fe_decreasing_ = fe_abs <= fe_ref_abs_ - t.fe_min_drop_count;
      fe_ref_abs_ = fe_abs;
      fe_ref_us_ = c.now_us;
    }
  }

  // FOLLOWING_ERROR: |誤差| が上限を超えた tick に呼ぶ。true ならこの tick はラッチしない（LIMIT_RECOVERY）。
  // 上限内に戻った tick は fe_within() を呼ぶ。
  // 猶予の条件: エンコーダが生きている（直近 fe_activity_ms にカウント変化）・FEEDBACK_UNCERTAIN でない・|誤差| < 上限 × 150 % ・
  // 超えてから fe_grace_max_ms 未満、かつ「超えてから fe_turn_ms 以内（減り始めるのを待つ）」か「|誤差| が減っている」。
  bool fe_may_wait(const Ctx& c, int64_t fe_abs, uint32_t limit) {
    if (!fe_over_) {
      fe_over_ = true;
      fe_over_since_us_ = c.now_us;
    }
    const uint64_t over = c.now_us - fe_over_since_us_;
    const bool alive = c.now_us - last_change_us_ <= ms(t.fe_activity_ms);
    const bool below_hard = fe_abs * 100 < static_cast<int64_t>(limit) * t.fe_hard_ratio_percent;
    const bool ok = t.fe_grace_max_ms > 0 && alive && !feedback_uncertain_ && below_hard &&
                    over < ms(t.fe_grace_max_ms) && (over < ms(t.fe_turn_ms) || fe_decreasing_);
    fe_recovery_ = ok;
    return ok;
  }
  void fe_within() {
    fe_over_ = false;
    fe_recovery_ = false;
  }

  // SOFT_LIMIT（Issue #46）: 実位置 actual がソフトリミットの外なら、内側へ戻してよいか・ラッチか。target は制御の目標。
  V1SoftDecision decide_soft_limit(const Ctx& c, int32_t actual, int32_t target) {
    const bool below = actual < c.soft_min;
    const bool above = actual > c.soft_max;
    if (!below && !above) {
      soft_out_ = false;
      soft_recovery_ = false;
      return V1SoftDecision::Inside;
    }
    if (!soft_out_) {
      soft_out_ = true;
      soft_out_since_us_ = c.now_us;
      outward_running_ = false;
    }
    const int out_dir = above ? 1 : -1;
    const double v_out = static_cast<double>(out_dir) * c.vel;
    if (v_out > static_cast<double>(t.soft_out_vel_count_s)) {
      if (!outward_running_) {
        outward_running_ = true;
        outward_since_us_ = c.now_us;
      }
    } else {
      outward_running_ = false;
    }
    const int64_t mech_lo = static_cast<int64_t>(c.soft_min) - t.mech_decel_zone_count;
    const int64_t mech_hi = static_cast<int64_t>(c.soft_max) + t.mech_decel_zone_count;
    const bool near_end = below ? (actual <= mech_lo + t.end_hard_margin_count)
                                : (actual >= mech_hi - t.end_hard_margin_count);
    const bool target_outside = target < c.soft_min || target > c.soft_max;
    const bool outward_too_long = outward_running_ && c.now_us - outward_since_us_ >= ms(t.soft_outward_grace_ms);
    const bool outside_too_long = c.now_us - soft_out_since_us_ >= ms(t.soft_recover_max_ms);
    if (near_end || target_outside || outward_too_long || outside_too_long) {
      soft_recovery_ = false;
      return V1SoftDecision::Latch;
    }
    soft_recovery_ = true;
    return V1SoftDecision::Recover;
  }

  // 位置ループの指令 cmd（permille、カウント増が正）に保護を掛けた出力を返す。observe の後に 1 回。
  int filter(const Ctx& c, int cmd) {
    const int32_t actual = clamp32(c.raw - c.ref_offset);
    int out = cmd;
    bool guard = false;
    if (feedback_uncertain_) {
      out = 0;
      braking_ = false;
      sat_running_ = false;
    } else {
      // 端の手前の制動（実測速度）
      update_braking(c, actual);
      if (braking_) {
        out = static_cast<int>(-brake_dir_ * c.duty_max_pm);
        guard = true;
        sat_running_ = false;
      } else {
        // ソフトリミットの外: 外向きには出さない
        if ((actual > c.soft_max && out > 0) || (actual < c.soft_min && out < 0)) out = 0;
        out = update_load(c, out);
        // 見えない走行の予算（先読み）
        if (out != 0) {
          const int d = out > 0 ? 1 : -1;
          uint64_t step_us = last_dt_us_;
          if (step_us < ms(t.projection_ms)) step_us = ms(t.projection_ms);
          const double proj = env_speed(out) * static_cast<double>(step_us) / 1e6;
          const double eps = 1e-9;
          if (env_ep_[idx(d)] + proj > static_cast<double>(t.budget_max_count) + eps ||
              env_cum_[idx(d)] + proj > static_cast<double>(position_limit(d, c)) + eps) {
            feedback_uncertain_ = true;
            out = 0;
            sat_running_ = false;
            load_limited_ = false;
          }
        }
      }
    }
    last_out_ = out;
    last_out_guard_ = guard;
    last_ctx_ = c;
    has_ctx_ = true;
    return out;
  }

  bool feedback_uncertain() const { return feedback_uncertain_; }
  bool load_limited() const { return load_limited_; }
  bool end_braking() const { return braking_; }
  bool soft_recovery() const { return soft_recovery_; }
  bool fe_recovery() const { return fe_recovery_; }
  int64_t trusted_raw() const { return trusted_raw_; }
  double envelope_episode(int dir) const { return env_ep_[idx(dir)]; }
  double envelope_cumulative(int dir) const { return env_cum_[idx(dir)]; }

  V1SupInfo info(uint64_t now_us, bool enabled) const {
    V1SupInfo s;
    s.enabled = enabled;
    if (!enabled) return s;
    s.feedback_uncertain = feedback_uncertain_;
    s.load_limited = load_limited_;
    s.end_braking = braking_;
    s.soft_recovery = soft_recovery_;
    s.fe_recovery = fe_recovery_;
    s.activity_age_ms = now_us > last_change_us_ ? static_cast<uint32_t>((now_us - last_change_us_) / 1000) : 0;
    s.saturation_ms = sat_running_ && now_us > sat_start_us_ ? static_cast<uint32_t>((now_us - sat_start_us_) / 1000) : 0;
    if (has_ctx_) {
      s.budget_up = remaining(1, last_ctx_);
      s.budget_down = remaining(-1, last_ctx_);
    }
    return s;
  }

  // 位置の上限（向き d）[count]: max(max(0, D − reserve), min(min_budget, max(0, D − hard_margin)))。回の上限（budget_max）との小さい方が
  // その回の予算 = min(1000, max(0, D − 400))（最小予算を除く）
  int32_t position_limit(int d, const Ctx& c) const {
    const int64_t trusted_actual = trusted_raw_ - c.ref_offset;
    const int64_t mech = d > 0 ? static_cast<int64_t>(c.soft_max) + t.mech_decel_zone_count
                               : static_cast<int64_t>(c.soft_min) - t.mech_decel_zone_count;
    const int64_t dist = d > 0 ? mech - trusted_actual : trusted_actual - mech;
    int64_t base = dist - t.end_reserve_count;  // 1000 の頭打ちは回の上限が持つ（位置の上限は端の余裕だけを守る）
    if (base < 0) base = 0;
    int64_t floor_b = dist - t.end_hard_margin_count;
    if (floor_b < 0) floor_b = 0;
    if (floor_b > t.min_budget_count) floor_b = t.min_budget_count;
    return static_cast<int32_t>(base > floor_b ? base : floor_b);
  }

 private:
  // --- 信用（DISARM をまたいで残る） ---
  int64_t trusted_raw_ = 0;
  double env_ep_[2] = {0.0, 0.0};   // [0] = 上（カウント増）、[1] = 下
  double env_cum_[2] = {0.0, 0.0};
  bool feedback_uncertain_ = false;
  // --- 活動 ---
  int64_t last_raw_ = 0;
  uint64_t last_change_us_ = 0;
  // --- その回だけ ---
  int last_out_ = 0;
  bool last_out_guard_ = false;
  bool have_last_us_ = false;
  uint64_t last_us_ = 0;
  uint64_t last_dt_us_ = 0;
  int ren_dir_ = 0;
  uint64_t ren_start_us_ = 0;
  int64_t ren_ext_ = 0;
  bool sat_running_ = false;
  uint64_t sat_start_us_ = 0;
  uint64_t load_win_start_us_ = 0;
  int64_t load_win_raw_ = 0;
  int64_t load_win_err_abs_ = 0;
  bool load_limited_ = false;
  uint64_t load_since_us_ = 0;
  int load_dir_ = 0;
  bool braking_ = false;
  int brake_dir_ = 0;  // 動いている向き（制動はその反対）
  bool soft_out_ = false;
  uint64_t soft_out_since_us_ = 0;
  bool outward_running_ = false;
  uint64_t outward_since_us_ = 0;
  bool soft_recovery_ = false;
  bool fe_has_ref_ = false;
  int64_t fe_ref_abs_ = 0;
  uint64_t fe_ref_us_ = 0;
  bool fe_decreasing_ = false;
  bool fe_over_ = false;
  uint64_t fe_over_since_us_ = 0;
  bool fe_recovery_ = false;
  Ctx last_ctx_{};
  bool has_ctx_ = false;

  static int idx(int d) { return d > 0 ? 0 : 1; }
  static uint64_t ms(uint64_t v) { return v * 1000ull; }
  static int32_t clamp32(int64_t v) {
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return static_cast<int32_t>(v);
  }

  double env_speed(int out) const {
    const double mag = out < 0 ? -static_cast<double>(out) : static_cast<double>(out);
    if (mag == 0.0) return 0.0;
    if (t.env_ref_permille <= 0) return static_cast<double>(t.env_count_s);
    return static_cast<double>(t.env_count_s) * mag / static_cast<double>(t.env_ref_permille);
  }

  int32_t remaining(int d, const Ctx& c) const {
    const double ep = static_cast<double>(t.budget_max_count) - env_ep_[idx(d)];
    const double cum = static_cast<double>(position_limit(d, c)) - env_cum_[idx(d)];
    double r = ep < cum ? ep : cum;
    if (r < 0.0) r = 0.0;
    return static_cast<int32_t>(floor(r));
  }

  void update_braking(const Ctx& c, int32_t actual) {
    if (t.brake_acc_count_s2 <= 0) { braking_ = false; return; }
    const double v = c.vel;
    const double v_abs = v < 0.0 ? -v : v;
    const int dir = v > 0.0 ? 1 : (v < 0.0 ? -1 : 0);
    const double dist = dir > 0 ? static_cast<double>(c.soft_max) - actual : static_cast<double>(actual) - c.soft_min;
    const double stop = v_abs * v_abs / (2.0 * static_cast<double>(t.brake_acc_count_s2));
    if (braking_) {
      if (dir != brake_dir_ || v_abs < static_cast<double>(t.brake_min_vel_count_s) / 2.0 ||
          stop <= dist - static_cast<double>(t.brake_hyst_count)) {
        braking_ = false;
      }
    } else if (dir != 0 && v_abs >= static_cast<double>(t.brake_min_vel_count_s) && stop > dist) {
      braking_ = true;
      brake_dir_ = dir;
    }
  }

  // 過負荷（LOAD_LIMITED）。out はこの tick の候補。返り値が実際の候補
  int update_load(const Ctx& c, int out) {
    const int64_t err_abs = c.err < 0 ? -c.err : c.err;
    const int err_dir = c.err > 0 ? 1 : (c.err < 0 ? -1 : 0);
    if (load_limited_) {
      if (err_abs <= c.hold_exit || err_dir != load_dir_ || c.now_us - load_since_us_ >= ms(t.load_rest_ms)) {
        load_limited_ = false;  // 誤差が無くなった・向きが変わった・休んだ: もう一度 1 s まで許す
        sat_running_ = false;
      } else {
        return 0;
      }
    }
    const long mag = out < 0 ? -out : out;
    const int out_dir = out > 0 ? 1 : (out < 0 ? -1 : 0);
    const bool sat = out_dir != 0 && out_dir == err_dir &&
                     static_cast<long>(mag) * 1000 >= c.duty_max_pm * static_cast<long>(t.load_sat_ratio_permille);
    if (!sat) {
      sat_running_ = false;
      return out;
    }
    if (!sat_running_) {
      sat_running_ = true;
      sat_start_us_ = c.now_us;
      load_win_start_us_ = c.now_us;
      load_win_raw_ = c.raw;
      load_win_err_abs_ = err_abs;
      return out;
    }
    if (c.now_us - load_win_start_us_ >= ms(t.load_window_ms)) {
      const bool progress = static_cast<int64_t>(err_dir) * (c.raw - load_win_raw_) >= t.load_progress_count ||
                            err_abs <= load_win_err_abs_ - t.load_err_drop_count;
      if (progress) sat_start_us_ = c.now_us;
      load_win_start_us_ = c.now_us;
      load_win_raw_ = c.raw;
      load_win_err_abs_ = err_abs;
    }
    if (c.now_us - sat_start_us_ >= ms(t.load_sat_max_ms)) {
      load_limited_ = true;
      load_since_us_ = c.now_us;
      load_dir_ = err_dir;
      sat_running_ = false;
      return 0;
    }
    return out;
  }
};
