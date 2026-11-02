#pragma once

// USB プロトコル v1 の 1 軸状態機械 V1Axis。
// GPIO・時計・NVS・Serial を持たない（auto_move.h と同じ流儀）。制御タスクが
//   handle_frame(受信フレーム, now_us) / tick(now_us, 生の PCNT カウント)
// を呼び、出力（power_on_cmd / brake_release_cmd / duty_permille）と送信メッセージ列（outbox）を読む。
// 振る舞いは docs/control-server/protocol.md §3〜§6 と、PC 側の参照モデル control-server/src/sim_axis.rs に合わせる。
// 設計: docs/superpowers/specs/2026-10-31-moter-controler-v1-protocol-design.md §3。
//
// 座標: actual_count = raw − reference_offset。系統の符号は掛けない（符号は PC 側の校正）。
// Duty の向き: NVS プロファイルの sign（+1 なら速度 − でカウント増）から決める。
//   カウントを増やしたいなら duty_signed = −sign × duty。
// 時刻はマイクロ秒で注入する（MCU では esp_timer_get_time()）。
// 校正（protocol.md §1.2）: CALIBRATE で従来 env と同じ端当て状態機械 AutoMachine（auto_move.h）を内蔵のまま回し、
//   成功で P 端 = 0 の基準を自分で確立する。Duty は AutoMachine の duty_signed（MD13S の符号付き %）を permille にして出す。
// 分解能（Issue #25 手順 8 b）: 位置ループの指令は v1_duty_command から motor_output まで permille（0.1 %）。V1Tuning の
//   duty_min / duty_max / stall_duty は SET_LIMITS の線上と同じ整数 % のままで、使う所で v1_permille() で換算する。

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "auto_move.h"
#include "config.h"
#include "motor_profile.h"
#include "v1_codec.h"
#include "v1_supervisor.h"
#include "v1_playout.h"

// 調整値。既定は config.h（実機で詰めるまでの仮値）。テストで差し替えられるよう構造体にしている。
struct V1Tuning {
  int kp_percent_per_100 = V1_KP_PERCENT_PER_100;
  int ff_count_s_per_percent = V1_FF_COUNT_S_PER_PERCENT;
  int32_t deadband = V1_DEADBAND;
  int32_t deadband_out = V1_DEADBAND_OUT;
  int duty_min = V1_DUTY_MIN;
  int duty_max = V1_DUTY_MAX;
  int stall_duty = V1_STALL_DUTY;
  int stall_duty_ratio_percent = V1_STALL_DUTY_RATIO_PERCENT;  // ENCODER_FAULT のしきい値 = max(stall_duty, duty_max × これ / 100)
  uint32_t stall_ms = STALL_MS;
  uint32_t stall_boost_ms = V1_STALL_BOOST_MS;        // 停止ブーストの窓（0 で無効）
  uint32_t stall_boost_max_ms = V1_STALL_BOOST_MAX_MS;  // 対象のまま動かずにこれを超えたら ENCODER_FAULT（0 で時間の上限なし。頭打ちの上限は残る）
  int32_t stall_boost_min_move = V1_STALL_BOOST_MIN_MOVE;
  int32_t floor_off = V1_FLOOR_OFF_COUNT;              // 床の適用状態を OFF にする |誤差|（ON は deadband_out 超。Issue #25 手順 8 c）
  // 有界の積分（Issue #25 手順 9「F」。config.h V1_INTEGRAL）。ki = 0 で無効（I は常に 0）
  double ki_permille_per_count_s = V1_INTEGRAL ? V1_KI_PERMILLE_PER_COUNT_S : 0.0;
  int i_max_permille = V1_I_MAX_PERMILLE;            // |I| の上限
  int i_rate_permille_per_s = V1_I_RATE_PERMILLE_PER_S;  // |dI/dt| の上限
  // 整定判定（Issue #25 手順 10「S」。config.h V1_SETTLED_MEASURED）。settled_ms = 0 で従来どおり |誤差| ≤ deadband で即入る
  uint32_t settled_ms = V1_SETTLED_MEASURED ? V1_SETTLED_MS : 0;
  int32_t settled_meas_vel = V1_SETTLED_MEAS_VEL_COUNT_S;  // 整定とみなす |実測速度| の上限 [count/s]
  // 追従中の動き出し補助（Issue #25 手順 9「T」。config.h V1_TRACK_ASSIST）。assist_ms = 0 で無効
  uint32_t assist_ms = V1_TRACK_ASSIST ? V1_ASSIST_MS : 0;
  int32_t assist_vel = V1_ASSIST_VEL_COUNT_S;            // 対象になる |軌道速度| の下限（これより速い）
  int32_t assist_meas_vel = V1_ASSIST_MEAS_VEL_COUNT_S;  // 「動いていない」とみなす |実測速度|（これ未満）
  int32_t assist_release_move = V1_ASSIST_RELEASE_MOVE;  // 補助 ON の後、これ以上動けば解除（窓の間に動けば窓を取り直す）
  uint32_t overrun_ms = V1_OVERRUN_MS;
  uint32_t power_settle_ms = POWER_SETTLE_MS;
  uint32_t brake_release_wait_ms = BRAKE_RELEASE_WAIT_MS;
  uint32_t brake_apply_wait_ms = BRAKE_APPLY_WAIT_MS;
  uint32_t cal_power_settle_ms = CAL_POWER_SETTLE_MS;
  uint32_t status_period_ms = 20;
  uint32_t velocity_window_ms = 100;
  // Issue #45（config.h V1_PLAYOUT）: 再生バッファの遅れ [ms]。0 なら従来の profile_step（目標ごとに行って止まる）
  uint32_t playout_delay_ms = V1_PLAYOUT ? V1_PLAYOUT_DELAY_MS : 0;
  uint32_t playout_window_ms = V1_PLAYOUT_WINDOW_MS;
  uint32_t playout_extrap_ms = V1_PLAYOUT_EXTRAP_MS;
  double playout_kp_per_s = V1_PLAYOUT_KP_PER_S;
  // Issue #46: ソフトリミットに向かう参照速度の上限（再生バッファの経路だけ）。soft_brake_acc ≤ 0 で無効
  double soft_brake_acc_count_s2 = V1_SOFT_BRAKE_ACC_COUNT_S2;
  int32_t soft_reserve_count = V1_SOFT_RESERVE_COUNT;
  // Issue #25 r6（config.h V1_FF_DIRECTIONAL）: 方向別 FF ＋ PI、走行中の床なし、保持 2/4、動き出しパルス。
  // directional = false なら r5 までの経路（上の床 E・補助 T・不感帯 deadband/deadband_out・Ki 0.1）。
  bool ff_directional = V1_FF_DIRECTIONAL != 0;
  int ff_a_up_permille = V1_FF_A_UP_PERMILLE;
  double ff_b_up_permille_per_count_s = V1_FF_B_UP_PERMILLE_PER_COUNT_S;
  int ff_a_down_permille = V1_FF_A_DOWN_PERMILLE;  // 符号付き（負）
  double ff_b_down_permille_per_count_s = V1_FF_B_DOWN_PERMILLE_PER_COUNT_S;
  int32_t ff_taper_count_s = V1_FF_TAPER_COUNT_S;
  int32_t hold_enter = V1_HOLD_ENTER_COUNT;   // 保持に入る |誤差|（r6）
  int32_t hold_exit = V1_HOLD_EXIT_COUNT;     // 保持から出る |誤差|（r6）
  int breakaway_up_permille = V1_BREAKAWAY_UP_PERMILLE;
  int breakaway_down_permille = V1_BREAKAWAY_DOWN_PERMILLE;
  uint32_t breakaway_window_ms = V1_BREAKAWAY_WINDOW_MS;  // 0 で動き出しパルス無効
  uint32_t breakaway_pulse_ms = V1_BREAKAWAY_PULSE_MS;
  int32_t breakaway_min_move = V1_BREAKAWAY_MIN_MOVE;
  int breakaway_max_pulses = V1_BREAKAWAY_MAX_PULSES;
  int32_t breakaway_moving_min_traj_count_s = V1_BREAKAWAY_MOVING_MIN_TRAJ_COUNT_S;  // 走行中に打つ条件（Codex P1）
  int32_t breakaway_moving_max_vel_count_s = V1_BREAKAWAY_MOVING_MAX_VEL_COUNT_S;
  uint32_t breakaway_moving_window_ms = V1_BREAKAWAY_MOVING_WINDOW_MS;
  int32_t breakaway_band_lo_count_s = V1_BREAKAWAY_BAND_LO_COUNT_S;   // r8: 引っかかった時だけ短く押す帯（hi 0 で無効）
  int32_t breakaway_band_hi_count_s = V1_BREAKAWAY_BAND_HI_COUNT_S;
  uint32_t breakaway_band_window_ms = V1_BREAKAWAY_BAND_WINDOW_MS;
  uint32_t breakaway_band_pulse_ms = V1_BREAKAWAY_BAND_PULSE_MS;
  uint32_t breakaway_band_cooldown_ms = V1_BREAKAWAY_BAND_COOLDOWN_MS;
  int32_t breakaway_band_min_move = V1_BREAKAWAY_BAND_MIN_MOVE;
  // r9（Issue #35。config.h V1_VEL_DAMPING）: 速度誤差の減衰項。kd = 0 で無効（r8 と同じ出力）。方向別の経路だけで効く
  double vel_kd_permille_per_count_s = 0.0;           // Kd [permille / (count/s)]。コンストラクタで既定を入れる
  int vel_d_max_permille = V1_VEL_D_MAX_PERMILLE;     // |u_d| の上限
  uint32_t v_fast_window_ms = V1_V_FAST_WINDOW_MS;    // 速い速度推定の窓
  // r10（Issue #44・#46。config.h V1_SUPERVISOR）: 保護の監視 V1Supervisor。方向別の経路だけで効く（従来の経路では常に false 扱い）。
  // true のとき「進まない → ENCODER_FAULT」の 2 つの判定（走行中の無パルス・静止 2 s の監視）は出さず、見えない走行の予算
  //（FEEDBACK_UNCERTAIN）・過負荷（LOAD_LIMITED）・端の手前の制動・SOFT_LIMIT / FOLLOWING_ERROR の猶予に置き換わる
  bool supervisor = false;  // コンストラクタで既定を入れる
  V1SupTuning sup;
  // r10（実測 2）: 保持出力の学習（config.h V1_HOLD_LEARN）。方向別の経路だけ。false なら r6〜r9 と同じ（HOLD で出力 0・I を捨てる）
  bool hold_learn = false;  // コンストラクタで既定を入れる
  int hold_i_max_ratio_percent = V1_HOLD_I_MAX_RATIO_PERCENT;
  double hold_ki_permille_per_count_s = V1_HOLD_KI_PERMILLE_PER_COUNT_S;
  int hold_i_rate_permille_per_s = V1_HOLD_I_RATE_PERMILLE_PER_S;
  // 2026-11-02 の操作者の決定: 静止した保持のブレーキ駐車（config.h V1_PARK）。監視 ON の方向別の経路だけ
  bool park = false;  // コンストラクタで既定を入れる
  uint32_t park_still_ms = V1_PARK_STILL_MS;
  uint32_t brake_engage_ms = V1_BRAKE_ENGAGE_MS;
  int32_t park_ref_vel_count_s = V1_PARK_REF_VEL_COUNT_S;
  int32_t park_target_span_count = V1_PARK_TARGET_SPAN_COUNT;
  int32_t park_unpark_count = V1_PARK_UNPARK_COUNT;
  // r10（supervisor のときだけ）: 静止目標での力の段階的な引き上げ（step 0 で無効）と小さな残り誤差の受け入れ（band 0 で無効）。config.h
  int escalate_step_permille = V1_ESCALATE_STEP_PERMILLE;
  uint32_t escalate_window_ms = V1_ESCALATE_WINDOW_MS;
  int32_t residual_band = V1_RESIDUAL_BAND_COUNT;
  int32_t residual_exit = V1_RESIDUAL_EXIT_COUNT;
  int residual_max_pulses = V1_RESIDUAL_MAX_PULSES;

  // directional の既定はコンパイルスイッチ。テストは V1Tuning(true) / V1Tuning(false) で経路を選ぶ（r6 の派生値もここで揃える）。
  explicit V1Tuning(bool directional = V1_FF_DIRECTIONAL != 0) : ff_directional(directional) {
    if (directional) {
      ki_permille_per_count_s = V1_KI_DIRECTIONAL_PERMILLE_PER_COUNT_S;
      vel_kd_permille_per_count_s = V1_VEL_DAMPING ? V1_VEL_KD_PERMILLE_PER_COUNT_S : 0.0;
      supervisor = V1_SUPERVISOR != 0;
      hold_learn = V1_HOLD_LEARN != 0;
      park = V1_PARK != 0;
    }
  }
  // 静止の帯の入口／出口（r6 は hold_enter / hold_exit、従来は deadband / deadband_out）。v1_duty_command・整定判定・静止監視で共用
  int32_t band_enter() const { return ff_directional ? hold_enter : deadband; }
  int32_t band_exit() const { return ff_directional ? hold_exit : deadband_out; }
};

// 位置ループの状態の表示用（Issue #25 r6）。MOVE = 追従／補正（FF + P + I）、HOLD = 保持（出力 0）、BRK = 動き出しパルス
enum class V1DutyMode : uint8_t { MOVE = 0, HOLD = 1, BRK = 2 };

// 位置ループの各項の計器（Issue #25 手順 8。画面に出す。STATUS には載せない）。符号はカウントが増える向きが正。
// 単位は permille（0.1 %）: ff_permille = 軌道速度 / ff × 10、p_permille = 誤差 × kp / 100 × 10（どちらも四捨五入）。
struct V1DutyTerms {
  int16_t ff_permille = 0;     // フィードフォワード項
  int16_t p_permille = 0;      // 比例項
  int16_t vel_count_s = 0;     // 軌道の目標速度 [count/s]（int16 に飽和）
  bool floor_on = false;       // 床の適用状態が ON（|合成| が床未満なら duty_min ＋ 停止ブーストまで持ち上げる。Issue #25 手順 8 c）
  bool in_band = false;        // 静止の不感帯の中（Duty 0）
  int16_t boost_permille = 0;  // 停止ブースト（Issue #15 H4）の上乗せ。V1Axis が埋める
  int16_t i_permille = 0;      // 積分項（Issue #25 手順 9「F」）。この tick の出力に入った値（四捨五入）
  bool assist_on = false;      // 追従中の動き出し補助（Issue #25 手順 9「T」）が床を掛けている
  int16_t cmd_permille = 0;    // v1_duty_command の結果（下限・上限を掛けた後）
  V1DutyMode mode = V1DutyMode::MOVE;  // r6 の状態（HOLD は in_band と同じ。BRK は動き出しパルスがこの tick の出力を決めた）
  int16_t d_permille = 0;      // r9 の減衰項 u_d = clamp(Kd × (軌道速度 − v_fast), ±10)（四捨五入）。保持中・推定なし・kd 0 なら 0
  int16_t v_fast_count_s = 0;  // r9 の速い速度推定 [count/s]（40 ms 窓。int16 に飽和）。推定なしなら 0
  bool v_fast_valid = false;   // v_fast が使える（40 ms 分、起動直後は 20 ms 以上のサンプルがある）
};

inline int16_t v1_clamp_i16(double v) {
  if (v > 32767.0) return 32767;
  if (v < -32768.0) return -32768;
  return static_cast<int16_t>(lround(v));
}

// 整数 % → permille（V1Tuning の Duty しきい値の換算）
constexpr int v1_permille(int percent) { return percent * V1_PERMILLE_PER_PERCENT; }

// 位置ループの tick をまたぐ状態（ヒステリシス）。位置制御が止まれば両方 false に戻す。
struct V1DutyState {
  bool in_band = false;   // 静止の不感帯の中（入る側 deadband / 出る側 deadband_out）
  bool floor_on = false;  // 床（Duty 下限）の適用状態（ON: |誤差| > deadband_out かつ向き一致 / OFF: |誤差| ≤ floor_off か向き不一致）
  double i_pm = 0.0;      // 積分項 [permille]（Issue #25 手順 9「F」。|i_pm| ≤ i_max_permille。位置制御が止まれば 0）
  int64_t hold_best_abs = 0;  // r10（hold_learn）: この保持の間に一番目標に近かった |誤差|（そこから 1 count 離れたら bias を詰める）
  double bias_pm = 0.0;       // r10（hold_learn、P1a）: 保持の偏り（荷重・乗員の補償）[permille]。止まっている間だけ学ぶ。|bias| ≤ duty_max × 90 %
  int last_vel_sign = 0;      // r10（hold_learn）: 直前の軌道速度の向き（反転で追従の I を 0 に）
};

// 位置制御の指令値（カウントが増える向きを正とした permille = 0.1 %）。err = 目標 − 実位置、vel = 軌道の目標速度 [count/s]。
// フィードフォワード（vel / ff）＋比例（err × kp / 100）＋積分 I を permille で計算し、和を四捨五入（lround）してから整数の
// permille で床・上限を掛ける（5.3 % は 53 permille として出る）。
// 1. 静止の不感帯: 目標が止まっている（vel = 0）ときだけ、入る側 deadband / 出る側 deadband_out のヒステリシスで、出力は
//    **I（保持バイアス）だけ**（Issue #25 手順 9「F」。I = 0、つまり ki = 0 なら従来どおり 0）。不感帯の中では I を積まない（凍結）。
//   （到達帯の縁での往復を防ぐ。不感帯の中では床の状態も OFF に戻す）。
// 2. 床の適用状態（Issue #25 手順 8 c「E」、st.floor_on）: 合成（FF + P。I は含めない）の向きが P の向きと違えば（P = 0 のときも）即 OFF。
//    |誤差| ≤ min(floor_off（15）, deadband_out − 1) なら OFF。|誤差| > deadband_out なら ON。その間（既定 15 < |誤差| ≤ 20）は
//    前の状態を保つ。誤差 0（P = 0）は向きが無いので OFF（軌道だけが動いている立ち上がりは生の FF）。kp = 0 なら常に OFF。
// 3. ON なら FF + P の向きに、|出力| = max(床（duty_min × 10 ＋ boost_permille。duty_max × 10 を超えない）, I 込みの合成が同じ向きなら
//    その大きさ)。I が逆向きに残っていても床の向きは変わらず、床が消えもしない。OFF なら生の合成（FF + P + I）のまま
//   （0 に置き換えない。旧 Issue #15 M1 の 0 置換はこれに代わる。「誤差と逆向きに床を掛けない」規則は向きの条件として残る）。
// 4. |合成| は duty_max × 10 で飽和。
// 5. 積分の更新（Issue #25 手順 9「F」。dt_s > 0 のときだけ。出力を決めた後、次の tick のために積む）: dI = ki × err × dt を
//    |dI| ≤ i_rate × dt に制限し、|I| を減らす向き（戻す）なら常に積む。|I| を増やす向きは、実出力が (a) duty_max で飽和して
//    いて向きが誤差と同じ、(b) 床が合成を持ち上げている、(c) 床の向き不一致（E の抑制中）のどれかなら積まない（アンチ
//    ワインドアップは実出力で見る）。|I| ≤ i_max_permille。
// boost_permille は停止ブースト（Issue #15 H4）の上乗せ（床が ON のときだけ効く）。terms を渡せば各項を書く（計器用）。
// dt_s は前の tick からの時間 [s]（0 なら積分は動かない。直接呼ぶテストはそのまま）。
// settle_ok = 不感帯に**入って**よい（Issue #25 手順 10「S」: V1Axis が「軌道速度 0・|誤差| ≤ deadband・|実測速度| ≤ 10 が
// settled_ms 続いた」で立てる）。false の間は |誤差| ≤ deadband でも入らず、生の P + I（床は |誤差| ≤ 15 で OFF）を出す。
// すでに中なら出る側の判定だけ（settle_ok は見ない）。
// assist = 追従中の動き出し補助（Issue #25 手順 9「T」。V1Axis の update_track_assist が立てる）: この tick だけ、床の適用状態
// （st.floor_on）に依らず床（duty_min ＋ boost）を掛ける。st.floor_on には書かない（E のヒステリシスに残さない）。合成が軌道速度と
// 同じ向きのときだけ（軌道と逆向きに押し当てない）。合成 0 なら何もしない。
// Issue #25 r6: 方向別 FF [permille]。d = 向き（カウント増が「上」）、w = min(|v| / taper, 1)、u = w × a_d + b_d × v。v = 0 なら 0。
// a_down は符号付き（負）の切片なのでそのまま足す（下向きの v は負で b × v も負）。
inline double v1_ff_directional_pm(double vel, const V1Tuning& t) {
  if (vel == 0.0) return 0.0;
  const double v_abs = vel < 0.0 ? -vel : vel;
  const double taper = t.ff_taper_count_s > 0 ? static_cast<double>(t.ff_taper_count_s) : 1.0;
  double w = v_abs / taper;
  if (w > 1.0) w = 1.0;
  if (vel > 0.0) return w * t.ff_a_up_permille + t.ff_b_up_permille_per_count_s * vel;
  return w * t.ff_a_down_permille + t.ff_b_down_permille_per_count_s * vel;
}

// Issue #25 r6 の位置ループ（t.ff_directional のとき v1_duty_command がここへ来る）。
// 1. 保持（st.in_band）: 軌道速度 0 で、中なら |誤差| ≤ hold_exit の間とどまり、外なら |誤差| ≤ hold_enter かつ settle_ok
//   （V1Axis が「軌道完了・|誤差| ≤ hold_enter・|実測速度| ≤ 10 が 200 ms」で立てる）で入る。入ったら I = 0、出力 0（積まない）。
// 2. 走行／補正: u = FF（方向別）+ P + I。床は無い（1〜5 % の生の値がそのまま出る）。|u| は duty_max で 1 回だけ飽和、四捨五入は 1 回。
// 3. 動き出しパルス（breakaway。V1Axis の update_breakaway が立てる）: 向きは誤差の向き（update_breakaway と同じ。古い I で u が
//    逆を向いていても目標から離れる側へ蹴らない）、|出力| = max(u が同じ向きなら |u|, 方向別の動き出し Duty)、duty_max 以内。この tick は I を凍結。
// 4. 積分: dI = Ki × err × dt を |dI| ≤ rate × dt に制限。飽和（|u| ≥ duty_max）していて dI が u と同じ向きなら積まない（それ以外は
//    常に積む。戻す向きは飽和中も許す）。|I| ≤ i_max。
// 5. r9（Issue #35）の減衰項: has_v_fast（V1Axis の 40 ms 窓の速度推定が使える）かつ kd > 0 なら
//    u_d = clamp(kd × (vel − v_fast), ±vel_d_max) を合成に足す（u = FF + P + I + u_d）。保持の中では足さない（出力 0 のまま）。
//    飽和・アンチワインドアップ・動き出しパルスの「合成」は u_d 込みの u で見る（四捨五入は合成で 1 回）。
// 6. r10（Issue #44 の実測 2、t.hold_learn）: 保持出力の学習。PR #50 の Codex 事前レビュー P1a で「保持の偏り bias」と「追従の I」を分けた:
//    - bias: 軌道速度 0 のとき（保持の中を含む）だけ Ki_hold / hold_i_rate で積む。|bias| ≤ duty_max × hold_i_max_ratio。走行中は凍結。
//      保持の中では、その保持で一番近かった所から 1 count 離れた（沈んだ）ときだけ詰める
//    - 追従の I: 走行中だけ従来の Ki / i_rate で積み、|I| ≤ i_max_permille（3 %）。|I| を減らす向きは V1_TRACK_I_UNWIND_RATE で速く。
//      軌道速度の向きが反転したら 0。止まっている間は同じ速さで 0 へ戻す
//    - 出力 = FF + P + bias + I + u_d（保持の中は bias + I）。動き出しパルスの tick は両方凍結
inline int v1_duty_command_directional(int64_t err, double vel, V1DutyState& st, const V1Tuning& t, V1DutyTerms& o,
                                       double dt_s, bool settle_ok, bool breakaway, bool has_v_fast, double v_fast) {
  const int64_t err_abs = err < 0 ? -err : err;
  const long duty_max_pm = v1_permille(t.duty_max);
  st.floor_on = false;
  o.v_fast_valid = has_v_fast;
  o.v_fast_count_s = has_v_fast ? v1_clamp_i16(v_fast) : 0;
  if (vel == 0.0) {
    if (st.in_band) {
      if (err_abs > t.hold_exit) st.in_band = false;
    } else if (err_abs <= t.hold_enter && settle_ok) {
      st.in_band = true;
      st.hold_best_abs = err_abs;
      if (!t.hold_learn) st.i_pm = 0.0;  // r6〜r9: 保持に入ったら I を捨てる（#1 は無負荷なら中立付近で自己保持する。出力 0）
    }
  } else {
    st.in_band = false;
  }
  if (t.hold_learn) {
    // 軌道速度の向きの反転: 追従の I を 0（古い向きの誤差を持ち越さない）
    const int vs = vel > 0.0 ? 1 : (vel < 0.0 ? -1 : 0);
    if (vs != 0 && st.last_vel_sign != 0 && vs != st.last_vel_sign) st.i_pm = 0.0;
    if (vs != 0) st.last_vel_sign = vs;
  }
  const double bias_cap = static_cast<double>(duty_max_pm) * t.hold_i_max_ratio_percent / 100.0;
  const double i_cap = static_cast<double>(t.i_max_permille);
  const bool still = t.hold_learn && vel == 0.0;  // 軌道が止まっている: bias を学ぶ・追従の I は 0 へ
  // 追従の I を 0 へ戻す（止まっている間、速い戻し）
  auto decay_i = [&]() {
    if (dt_s <= 0.0) return;
    const double step = static_cast<double>(V1_TRACK_I_UNWIND_RATE_PERMILLE_PER_S) * dt_s;
    if (st.i_pm > step) st.i_pm -= step;
    else if (st.i_pm < -step) st.i_pm += step;
    else st.i_pm = 0.0;
  };
  if (st.in_band) {
    o.in_band = true;
    o.mode = V1DutyMode::HOLD;
    if (!t.hold_learn) {
      o.cmd_permille = 0;
      return 0;
    }
    // r10: 保持の出力 = bias（＋戻りきっていない追従の I）。この保持の間に一番近かった所から 1 count 以上離れた（荷重で沈んだ）ときだけ
    // bias を詰める（静止摩擦で目標の 1〜2 count 手前に止まっている無負荷の軸では育てない）
    if (err_abs < st.hold_best_abs) st.hold_best_abs = err_abs;
    if (dt_s > 0.0 && t.hold_ki_permille_per_count_s > 0.0 && err_abs >= 1 && err_abs >= st.hold_best_abs + 1) {
      double db = t.hold_ki_permille_per_count_s * static_cast<double>(err) * dt_s;
      const double db_max = static_cast<double>(t.hold_i_rate_permille_per_s) * dt_s;
      if (db > db_max) db = db_max;
      if (db < -db_max) db = -db_max;
      st.bias_pm += db;
      if (st.bias_pm > bias_cap) st.bias_pm = bias_cap;
      if (st.bias_pm < -bias_cap) st.bias_pm = -bias_cap;
    }
    decay_i();
    long h = lround(st.bias_pm + st.i_pm);
    if (h > duty_max_pm) h = duty_max_pm;
    if (h < -duty_max_pm) h = -duty_max_pm;
    o.i_permille = v1_clamp_i16(st.i_pm + st.bias_pm);
    o.cmd_permille = static_cast<int16_t>(h);
    return static_cast<int>(h);
  }
  const double p_pm = static_cast<double>(err * t.kp_percent_per_100) * V1_PERMILLE_PER_PERCENT / 100.0;
  const double ff_pm = v1_ff_directional_pm(vel, t);
  o.p_permille = v1_clamp_i16(p_pm);
  o.ff_permille = v1_clamp_i16(ff_pm);
  o.i_permille = v1_clamp_i16(st.i_pm + st.bias_pm);  // 計器の I は bias 込み
  // r9: 速度誤差の減衰項（保持の外だけ。上の HOLD で返した後なので保持中は出ない）
  double d_pm = 0.0;
  if (has_v_fast && t.vel_kd_permille_per_count_s > 0.0) {
    const double d_max = static_cast<double>(t.vel_d_max_permille < 0 ? -t.vel_d_max_permille : t.vel_d_max_permille);
    d_pm = t.vel_kd_permille_per_count_s * (vel - v_fast);
    if (d_pm > d_max) d_pm = d_max;
    if (d_pm < -d_max) d_pm = -d_max;
  }
  o.d_permille = v1_clamp_i16(d_pm);
  const double u_raw = ff_pm + p_pm + st.i_pm + st.bias_pm + d_pm;
  const long u = lround(u_raw);  // 四捨五入は 1 回（u_d 込み）
  const bool saturated = (u >= duty_max_pm) || (u <= -duty_max_pm);
  long mag = u < 0 ? -u : u;
  if (mag > duty_max_pm) mag = duty_max_pm;
  int dir = u > 0 ? 1 : (u < 0 ? -1 : 0);
  const int err_dir = err > 0 ? 1 : (err < 0 ? -1 : 0);
  if (breakaway && err_dir != 0) {
    // パルスの向きは常に誤差の向き（update_breakaway が進みを数える向きと同じ）。古い I で合成が逆を向いていても目標から
    // 離れる側へ蹴らない。|出力| = max(合成が同じ向きならその大きさ, 方向別の動き出し Duty)、duty_max 以内
    dir = err_dir;
    long brk = dir > 0 ? t.breakaway_up_permille : t.breakaway_down_permille;
    if (brk < 0) brk = -brk;
    if (brk > duty_max_pm) brk = duty_max_pm;
    const bool u_same_dir = u != 0 && ((u > 0) == (dir > 0));
    if (!u_same_dir) mag = 0;
    if (mag < brk) mag = brk;
    o.mode = V1DutyMode::BRK;
  } else {
    o.mode = V1DutyMode::MOVE;
  }
  const int out = static_cast<int>(dir * mag);
  o.cmd_permille = static_cast<int16_t>(out);
  // 積分の更新（次の tick のため）。パルス中は凍結
  if (dt_s > 0.0 && err != 0 && o.mode != V1DutyMode::BRK) {
    if (still) {
      // bias（止まっている間）: Ki_hold・hold_i_rate。飽和した向きへは積み増さない
      double db = t.hold_ki_permille_per_count_s * static_cast<double>(err) * dt_s;
      const double db_max = static_cast<double>(t.hold_i_rate_permille_per_s) * dt_s;
      if (db > db_max) db = db_max;
      if (db < -db_max) db = -db_max;
      const bool deeper = saturated && ((db > 0.0) == (u > 0));
      if (!deeper) {
        st.bias_pm += db;
        if (st.bias_pm > bias_cap) st.bias_pm = bias_cap;
        if (st.bias_pm < -bias_cap) st.bias_pm = -bias_cap;
      }
    } else if (t.ki_permille_per_count_s > 0.0) {
      // 追従の I（走行中、または学習 OFF のとき）: 増やす向きは i_rate、戻す向きは（学習 ON なら）速く
      double di = t.ki_permille_per_count_s * static_cast<double>(err) * dt_s;
      const bool unwinding = (st.i_pm > 0.0 && di < 0.0) || (st.i_pm < 0.0 && di > 0.0);  // |I| を減らす向きは常に許す
      const double rate = (unwinding && t.hold_learn) ? static_cast<double>(V1_TRACK_I_UNWIND_RATE_PERMILLE_PER_S)
                                                       : static_cast<double>(t.i_rate_permille_per_s);
      const double di_max = rate * dt_s;
      if (di > di_max) di = di_max;
      if (di < -di_max) di = -di_max;
      const bool deeper = saturated && ((di > 0.0) == (u > 0));  // 飽和した向きへ積み増す
      if (!deeper || unwinding) {
        st.i_pm += di;
        if (st.i_pm > i_cap) st.i_pm = i_cap;
        if (st.i_pm < -i_cap) st.i_pm = -i_cap;
      }
    }
  }
  if (still && o.mode != V1DutyMode::BRK) decay_i();
  return out;
}

// has_v_fast / v_fast: r9（Issue #35）の速い速度推定 [count/s]（V1Axis の 40 ms 窓）。方向別の経路の減衰項だけが使う
//（従来の経路では無視）。has_v_fast = false（既定）なら減衰項は 0（直接呼ぶテストは r8 までと同じ出力）。
inline int v1_duty_command(int64_t err, double vel, V1DutyState& st, const V1Tuning& t, int boost_permille = 0,
                           V1DutyTerms* terms = nullptr, double dt_s = 0.0, bool settle_ok = true,
                           bool assist = false, bool has_v_fast = false, double v_fast = 0.0) {
  V1DutyTerms scratch;
  V1DutyTerms& o = terms ? *terms : scratch;
  o = V1DutyTerms{};
  o.vel_count_s = v1_clamp_i16(vel);
  o.boost_permille = static_cast<int16_t>(boost_permille);
  // Issue #25 r6: 方向別 FF ＋ PI、床なし。boost は使わず、assist は動き出しパルス（V1Axis::update_breakaway）の意味になる
  if (t.ff_directional) {
    return v1_duty_command_directional(err, vel, st, t, o, dt_s, settle_ok, assist, has_v_fast, v_fast);
  }
  const int64_t err_abs = err < 0 ? -err : err;
  const long duty_max_pm = v1_permille(t.duty_max);
  // 不感帯の中: 出力は I（保持バイアス）だけ。積まない。
  auto hold_output = [&]() -> int {
    st.floor_on = false;
    o.in_band = true;
    o.mode = V1DutyMode::HOLD;
    long i_out = lround(st.i_pm);
    if (i_out > duty_max_pm) i_out = duty_max_pm;
    if (i_out < -duty_max_pm) i_out = -duty_max_pm;
    o.i_permille = static_cast<int16_t>(i_out);
    o.cmd_permille = static_cast<int16_t>(i_out);
    return static_cast<int>(i_out);
  };
  if (vel == 0.0) {
    if (st.in_band) {
      if (err_abs <= t.deadband_out) return hold_output();
      st.in_band = false;
    } else if (err_abs <= t.deadband && settle_ok) {
      st.in_band = true;
      return hold_output();
    }
  } else {
    st.in_band = false;
  }
  // 各項を permille で。P = err × kp / 100 × 10（err × kp は整数なので 10 の倍数の分子を 100 で割る: 誤差 345・kp 2 → 69.0 ちょうど）
  const double p_pm = static_cast<double>(err * t.kp_percent_per_100) * V1_PERMILLE_PER_PERCENT / 100.0;
  const double ff_pm = t.ff_count_s_per_percent > 0 ? vel * V1_PERMILLE_PER_PERCENT / t.ff_count_s_per_percent : 0.0;
  o.p_permille = v1_clamp_i16(p_pm);
  o.ff_permille = v1_clamp_i16(ff_pm);
  o.i_permille = v1_clamp_i16(st.i_pm);
  const long base = lround(p_pm + ff_pm);           // I を含まない合成（床の向きと ON/OFF はこれで決める）
  const long cmd = lround(p_pm + ff_pm + st.i_pm);  // 四捨五入（0.5 は 0 から遠い側）。以後は整数の permille
  // 床の適用状態（ヒステリシス）。向きは P と合成（FF + P。**I は含めない**: 静止で I が誤差と逆向きに残っていても床を
  // 消さない・逆向きにしない。I だけで床の向きが変わると停止ブーストが対象のまま床が消え、2 s で ENCODER_FAULT になる）で
  // 比べる（kp = 0 なら P = 0 で向きが無く、FF だけに床を掛けない。Codex レビュー P2-1）。合成が 0 でも OFF。
  // OFF のしきい値は min(floor_off, deadband_out − 1): SET_LIMITS で deadband_out ≤ floor_off にされても ON（> deadband_out）と
  // OFF の領域が重ならない（Codex レビュー P2-2）
  const bool sign_match = base != 0 && p_pm != 0.0 && (base < 0) == (p_pm < 0.0);
  const int32_t floor_off = t.floor_off < t.deadband_out ? t.floor_off : t.deadband_out - 1;
  if (!sign_match || err_abs <= floor_off) {
    st.floor_on = false;
  } else if (err_abs > t.deadband_out) {
    st.floor_on = true;
  }
  o.floor_on = st.floor_on;
  const bool assist_now = assist && base != 0 && vel != 0.0 && ((base < 0) == (vel < 0.0));
  o.assist_on = assist_now;
  bool floor_lifted = false;
  int out;
  long mag;
  if (st.floor_on || assist_now) {
    // 床を掛ける: 向きは FF + P（base ≠ 0）。|出力| = max(床, I 込みの合成が同じ向きならその大きさ、逆向きなら 0)
    long floor_pm = v1_permille(t.duty_min) + boost_permille;
    if (floor_pm > duty_max_pm) floor_pm = duty_max_pm;
    const bool cmd_same_dir = cmd != 0 && (cmd < 0) == (base < 0);
    mag = cmd_same_dir ? (cmd < 0 ? -cmd : cmd) : 0;
    if (mag < floor_pm) { mag = floor_pm; floor_lifted = true; }
    if (mag > duty_max_pm) mag = duty_max_pm;
    out = static_cast<int>(base < 0 ? -mag : mag);
  } else {
    mag = cmd < 0 ? -cmd : cmd;
    if (mag > duty_max_pm) mag = duty_max_pm;
    out = cmd == 0 ? 0 : static_cast<int>(cmd < 0 ? -mag : mag);
  }
  o.cmd_permille = static_cast<int16_t>(out);
  // 積分の更新（次の tick のため）
  if (dt_s > 0.0 && t.ki_permille_per_count_s > 0.0 && err != 0) {
    double di = t.ki_permille_per_count_s * static_cast<double>(err) * dt_s;
    const double di_max = static_cast<double>(t.i_rate_permille_per_s) * dt_s;
    if (di > di_max) di = di_max;
    if (di < -di_max) di = -di_max;
    const bool unwinding = (st.i_pm > 0.0 && di < 0.0) || (st.i_pm < 0.0 && di > 0.0);
    bool allow = true;
    if (!unwinding) {
      const bool saturated = out != 0 && mag >= duty_max_pm && ((out < 0) == (err < 0));
      const bool sign_mismatch = base != 0 && p_pm != 0.0 && (base < 0) != (p_pm < 0.0);  // E の抑制中（FF + P が P と逆）
      if (saturated || floor_lifted || sign_mismatch) allow = false;
    }
    if (allow) {
      st.i_pm += di;
      const double i_max = static_cast<double>(t.i_max_permille);
      if (st.i_pm > i_max) st.i_pm = i_max;
      if (st.i_pm < -i_max) st.i_pm = -i_max;
    }
  }
  return out;
}

// v1 の校正に使う AutoConfig。端当て Duty だけ従来 env より高い（V1_CAL_HOME_DUTY）。
inline AutoConfig v1_cal_config() {
  AutoConfig c;
  c.home_duty = V1_CAL_HOME_DUTY;
  return c;
}

struct V1Identity {
  uint64_t device_id = 0;  // eFuse MAC から作る（上位 16 bit は 0x5131）。v1_device_id_from_mac
  uint32_t boot_id = 0;    // 起動ごとの乱数
  uint8_t fw_major = 0;
  uint8_t fw_minor = 1;
  uint8_t fw_patch = 0;
};

// device_id = 0x5131 << 48 | MAC を表記順（mac[0] が先頭オクテット）に並べた 48 bit。
// 例: 30:ED:A0:0B:20:40 → 0x513130EDA00B2040。下 16 bit（画面の `dev XXXX`）は末尾 2 オクテット。
// ESP.getEfuseMac() は先頭オクテットを最下位に置く（逆順）ので使わない。
inline uint64_t v1_device_id_from_mac(const uint8_t mac[6]) {
  uint64_t id = 0x5131ull << 48;
  for (int i = 0; i < 6; ++i) {
    id |= static_cast<uint64_t>(mac[i]) << (8 * (5 - i));
  }
  return id;
}

class V1Axis {
 public:
  static constexpr size_t kOutboxCap = 40;   // 1 周期の最大: 受信 16 件分の ACK（SET_LIMITS なら各 + LIMITS）+ STATUS + FAULT_EVENT + AXIS_SELECT + CAL_RESULT
  static constexpr size_t kDedupeDepth = 16;

  V1Tuning tuning;
  AutoConfig cal_config = v1_cal_config();  // 端当て校正の調整値（既定は config.h。テストで差し替えられる）

  // BOOT → DISARMED。selected_axis は NVS の motor、sign はその系統のプロファイルの符号。
  void begin(const V1Identity& id, uint8_t selected_axis, int8_t sign, uint64_t now_us, int64_t raw) {
    id_ = id;
    selected_axis_ = selected_axis;
    sign_ = sign < 0 ? -1 : 1;
    boot_origin_us_ = now_us;
    last_tick_us_ = now_us;
    now_us_ = now_us;
    raw_ = raw;
    reference_offset_ = 0;
    vel_anchor_raw_ = raw;
    vel_anchor_us_ = now_us;
    reset_v_fast();
    sup_.reset_trust(raw, now_us);
    state_ = v1::state::DISARMED;
    cal_phase_ = CalPhase::Idle;
    cal_.cfg = cal_config;
    cal_.set_profile(MotorProfile{});
    hold_here();
    began_ = true;
  }

  bool began() const { return began_; }
  void set_sign(int8_t sign) { sign_ = sign < 0 ? -1 : 1; }
  // NVS に保存されている選択中の系統のプロファイル（stroke / neutral / calibrated）。校正の端当てで
  // 「端まで遠いときは AUTO_DUTY_MAX、近づいたら HOME_DUTY」の判定（auto_move.h の end_distance）に使う。
  // 無いと全行程を HOME_DUTY（10 %）で動くことになり、#2 の持ち上げ側は下限から動き出せないことがある
  //（2026-11-01 の実機: N 端へ向かう行程が開始直後に無パルスで失敗）。行程・中立は校正が測り直して上書きする。
  void set_profile_hint(const MotorProfile& p) { profile_hint_ = p; }

  // --- 校正の出力（制御タスク・画面向け） ------------------------------------------
  // 成功した校正のプロファイル（motor = 校正時の系統）。profile_dirty() の間は NVS に未保存。保存したら clear_profile_dirty()。
  bool profile_dirty() const { return profile_dirty_; }
  const MotorProfile& dirty_profile() const { return dirty_profile_; }
  void clear_profile_dirty() { profile_dirty_ = false; }
  // この boot で最後に送った CAL_RESULT（画面の表示用）
  bool has_cal_result() const { return has_cal_result_; }
  const v1::CalResult& last_cal_result() const { return last_cal_result_; }
  // 校正中の段階（CALIBRATING 以外では NONE）と AutoMachine の直近の理由（"retreat" など）
  uint8_t cal_live_step() const {
    if (state_ != v1::state::CALIBRATING) return v1::cal_step::NONE;
    return cal_phase_ == CalPhase::PowerHold ? v1::cal_step::POWER_HOLD : cal_step_code();
  }
  const char* cal_live_reason() const { return cal_.reason; }

  // --- 入力 ---------------------------------------------------------------

  // COBS 符号化済み（区切りなし）の 1 フレーム。復号できなければ rx_error を数えるだけで応答しない。
  // rx_us（Issue #45）: リンク層がフレームを受け取った時刻（≤ now_us）。再生バッファの時刻印だけに使う。期限（valid_for）・lease・
  // 応答の時刻は従来どおり now_us（制御タスクが取り出した時刻）で数える（バッファに残っているからといって鮮度は延ばさない）。
  void handle_frame(const uint8_t* encoded, size_t n, uint64_t now_us) { handle_frame(encoded, n, now_us, now_us); }
  void handle_frame(const uint8_t* encoded, size_t n, uint64_t now_us, uint64_t rx_us) {
    v1::HostMsg m;
    if (!v1::decode_host_encoded(encoded, n, &m)) {
      record_rx_error();
      return;
    }
    handle(m, now_us, rx_us);
  }

  // 復号済みメッセージ。物理は進めない（tick が進める）。ACK → STATUS の順で outbox に積む。
  void handle(const v1::HostMsg& m, uint64_t now_us) { handle(m, now_us, now_us); }
  void handle(const v1::HostMsg& m, uint64_t now_us, uint64_t rx_us) {
    now_us_ = now_us;
    dispatch(m, now_us, rx_us > now_us ? now_us : rx_us);
    emit_status_if_changed();
  }

  // 5 ms ごと。タイマー → 軌道 → 異常検出 → Duty → STATUS。
  void tick(uint64_t now_us, int64_t raw) {
    if (!began_) return;
    const uint64_t dt_us = now_us > last_tick_us_ ? now_us - last_tick_us_ : 0;
    last_tick_us_ = now_us;
    now_us_ = now_us;
    raw_ = raw;
    update_velocity(now_us);
    update_v_fast(now_us);  // r9（Issue #35）: 40 ms 窓の速い速度推定。tick ごと

    // 制御周期の逸脱。止まっている状態（DISARMED / FAULT）では見ない（起動直後や待機中の間隔は制御に影響しない）。
    if (!parked() && dt_us > static_cast<uint64_t>(tuning.overrun_ms) * 1000ull) {
      latch_fault(v1::fault::CONTROL_OVERRUN, now_us);
    }

    check_timers(now_us);
    if (position_control_on()) {
      if (park_ == ParkPhase::None) {
        update_setpoint(static_cast<double>(dt_us) / 1e6);
      } else {
        setpoint_vel_ = 0.0;  // 駐車中・ブレーキ解除待ちは参照を駐車した位置で止める（再生バッファにはフレームが溜まる）
      }
      check_motion_faults(now_us);
    }
    if (state_ == v1::state::STOPPING && setpoint_vel_ == 0.0) {
      finish_stop(now_us);
    }
    update_duty(now_us, dt_us);
    tick_calibration(now_us);  // CALIBRATING のときだけ Duty を書く（update_duty の後）
    emit_status_if_changed();
    emit_periodic_status(now_us);
  }

  // 系統選択（protocol.md §1.1）。DISARMED のときだけ。CONFIGURED / POSITION_REFERENCED / calibration_hash を消し、
  // AXIS_SELECT を 1 回。受け付けたら true。
  bool select_axis(uint8_t axis, uint64_t now_us) {
    now_us_ = now_us;
    if (state_ != v1::state::DISARMED) return false;
    if (axis == selected_axis_) return true;
    const uint8_t prev = selected_axis_;
    selected_axis_ = axis;
    has_config_ = false;
    playout_clock_.reset();
    playout_.clear();
    referenced_ = false;
    sup_.reset_trust(raw_, now_us);  // r10: 別の系統の位置は信用の起点にならない
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::AXIS_SELECT;
    m.axis_select.boot_id = id_.boot_id;
    m.axis_select.selected_axis = axis;
    m.axis_select.prev_selected_axis = prev;
    m.axis_select.mcu_state = state_;
    push(m);
    emit_status_if_changed();
    return true;
  }

  // 画面の非常停止。ラッチで即 FAULT(LOCAL_ESTOP)（減速中の異常コードがあればそれを保つ）。
  // ラッチ中は FAULT_RESET が FAULT_CAUSE_PRESENT。解除しても FAULT は FAULT_RESET まで残る。
  void set_local_estop(bool latched, uint64_t now_us) {
    if (latched == estop_latched_) return;
    now_us_ = now_us;
    estop_latched_ = latched;
    if (latched) {
      const uint16_t code = (state_ == v1::state::STOPPING && stop_kind_ == StopKind::Fault)
                                ? stop_fault_code_
                                : v1::fault::LOCAL_ESTOP;
      latch_fault(code, now_us);
      emit_status_if_changed();
    }
  }

  // メッセージ層より下で捨てたフレーム（COBS・300 byte 超・受信キュー満杯）を数える。
  void record_rx_error() {
    if (rx_error_count_ < 0xFFFF) rx_error_count_ += 1;
  }

  // --- 出力（送信メッセージ） -------------------------------------------------

  size_t pending() const { return out_count_; }
  const v1::McuMsg* peek() const { return out_count_ ? &outbox_[out_head_] : nullptr; }
  void pop() {
    if (out_count_ == 0) return;
    out_head_ = (out_head_ + 1) % kOutboxCap;
    out_count_ -= 1;
  }
  bool pop(v1::McuMsg* m) {
    if (out_count_ == 0) return false;
    *m = outbox_[out_head_];
    pop();
    return true;
  }

  // --- 出力（アクチュエータ指令） ---------------------------------------------

  bool power_on_cmd() const { return power_controller() && powered_state(); }
  bool brake_release_cmd() const { return brake_release_cmd_; }
  int duty_permille() const { return duty_permille_; }     // 符号付き permille（MD13S の向き。motor_output_write_permille へそのまま）
  int16_t pwm_permille() const {                            // 正が count の増える向き。実際に出している permille そのもの
    return static_cast<int16_t>(-static_cast<int>(effective_sign()) * duty_permille_);
  }

  // --- 参照 -----------------------------------------------------------------

  uint8_t state() const { return state_; }
  // STATUS に出す異常コード（FAULT、または COMMAND_TIMEOUT による減速中）
  uint16_t fault_code() const {
    if (state_ == v1::state::FAULT) return fault_code_;
    if (state_ == v1::state::STOPPING && stop_kind_ == StopKind::Fault) return stop_fault_code_;
    return v1::fault::NONE;
  }
  int32_t actual_count() const { return clamp_count(raw_ - reference_offset_); }
  int32_t target_count() const { return to_count(setpoint_); }
  int32_t control_target_count() const { return to_count(control_target_); }
  int32_t velocity_count_s() const { return velocity_; }
  // 参照（軌道）の速度 [count/s]。FF・r9 の減衰・保持／整定の判定・動き出しパルスが使う値そのもの（丸めない）
  double setpoint_vel() const { return setpoint_vel_; }
  double setpoint_pos() const { return setpoint_; }
  // Issue #45: 直近の tick の再生バッファの推定（has / starved / pos / vel / 点の数）と時刻の付け直しの周期 [us]（0 = 未確定）
  const V1PlayoutFit& playout_fit() const { return playout_fit_; }
  double playout_period_us() const { return playout_clock_.valid() ? playout_clock_.period_us() : 0.0; }
  uint32_t playout_reseeds() const { return playout_clock_.reseeds(); }  // 周期の変化で時刻の付け直しを取り直した回数
  // r9（Issue #35）: 40 ms 窓の速い速度推定 [count/s]（生カウントの向き = カウント増が正）。推定が無ければ valid = false・値 0
  bool v_fast_valid() const { return v_fast_valid_; }
  double v_fast_count_s() const { return v_fast_valid_ ? v_fast_ : 0.0; }
  int32_t following_error_count() const {
    return clamp_count(static_cast<int64_t>(actual_count()) - static_cast<int64_t>(target_count()));
  }
  bool configured() const { return has_config_; }
  bool referenced() const { return referenced_; }
  const v1::Config* config() const { return has_config_ ? &config_ : nullptr; }
  uint8_t selected_axis() const { return selected_axis_; }
  int8_t sign() const { return sign_; }
  bool estop_latched() const { return estop_latched_; }
  bool has_rx() const { return has_rx_; }
  uint64_t last_rx_us() const { return last_rx_us_; }
  uint16_t rx_error_count() const { return rx_error_count_; }
  uint16_t stale_drop_count() const { return stale_drop_count_; }
  uint32_t last_rx_seq() const { return last_rx_seq_; }
  uint32_t last_applied_seq() const { return last_applied_seq_; }
  uint32_t last_applied_frame() const { return last_applied_frame_; }
  uint32_t calibration_hash() const { return has_config_ ? config_.calibration_hash : 0; }
  uint32_t boot_id() const { return id_.boot_id; }
  uint64_t device_id() const { return id_.device_id; }
  uint16_t last_host_fault_code() const { return last_host_fault_code_; }
  uint64_t mcu_time_us(uint64_t now_us) const { return now_us > boot_origin_us_ ? now_us - boot_origin_us_ : 0; }
  // 停止ブースト（Issue #15 H4）: いま Duty 下限に上乗せしている permille（0 = なし）。STATUS には載せない（空きフィールドが無い）。画面用
  int stall_boost_permille() const { return stall_boost_; }
  // 積分項（Issue #25 手順 9「F」）の現在値 [permille]（四捨五入前の値を丸めたもの）。画面の計器行 `I`。STATUS には載せない
  int integral_permille() const { return static_cast<int>(lround(duty_state_.i_pm)); }
  // r10（hold_learn、P1a）: 保持の偏り（荷重・乗員の補償）[permille]。画面の I は追従の I と bias の和
  int bias_permille() const { return static_cast<int>(lround(duty_state_.bias_pm)); }
  // r10: 静止目標での引き上げ [permille]（0 = なし）と、小さな残り誤差を受け入れて出力 0 にしているか（画面・テスト用）
  int escalation_permille() const { return esc_level_; }
  bool residual_accepted() const { return residual_accepted_; }
  // 動き出しパルス（Issue #25 r6）: 進まないまま打った数と、いま出しているか（画面・テスト用）
  int breakaway_pulses() const { return brk_pulses_; }
  bool breakaway_active() const { return brk_active_; }
  // 位置ループの各項（Issue #25 手順 8 の計器）。直近の update_duty で計算したもの。位置制御が動いていなければ全部 0
  const V1DutyTerms& duty_terms() const { return terms_; }
  // r10（Issue #44・#46）: 保護の監視が効いているか（方向別の経路かつ tuning.supervisor）と、その計器（画面・テスト用）
  bool supervisor_on() const { return tuning.ff_directional && tuning.supervisor; }
  V1SupInfo supervisor_info() const {
    V1SupInfo s = sup_.info(now_us_, supervisor_on());
    s.parked = brake_parked();
    return s;
  }
  // ブレーキ駐車（PARKED）: 入りかけ（ブレーキが掴むまで保持出力）と駐車中。戻りかけ（ブレーキ解除待ち）は含めない
  bool brake_parked() const { return park_ == ParkPhase::Engaging || park_ == ParkPhase::Parked; }
  bool unparking() const { return park_ == ParkPhase::Releasing; }
  int park_count() const { return park_count_; }
  // 持続する停止（FEEDBACK_UNCERTAIN / LOAD_LIMITED）: PWM 0 ＋ ブレーキ作動（主電源は保つ）。DISARMED に戻すまで続く
  bool supervisor_inhibited() const { return supervisor_on() && sup_.inhibited(); }
  const V1Supervisor& supervisor() const { return sup_; }
  // ENCODER_FAULT を判定し始める |Duty| [permille]（Issue #15 M2）= max(stall_duty, duty_max × ratio / 100) × 10。
  // 整数 % で切り捨ててから permille にするので効き目は整数 % のときと同じ: duty_max 20 → 150、14 → 10 % → 100（下限）
  int encoder_fault_duty_threshold_permille() const {
    const int th = tuning.duty_max * tuning.stall_duty_ratio_percent / 100;
    return v1_permille(th < tuning.stall_duty ? tuning.stall_duty : th);
  }
  // duty_max がしきい値より低く、無パルスの ENCODER_FAULT が決して出ない（SET_LIMITS は受理するが画面で警告する）
  bool encoder_fault_check_disabled() const {
    if (supervisor_on()) return false;  // r10: 無パルスの ENCODER_FAULT 自体が無い（見えない走行の予算が代わりに守る）
    return encoder_fault_duty_threshold_permille() > v1_permille(tuning.duty_max);
  }

  uint16_t flags() const {
    uint16_t f = 0;
    if (power_controller()) {
      f |= v1::flags::POWER_CONTROLLER;
      if (powered_state()) f |= v1::flags::POWER_ON_CMD;
    }
    if (brake_release_cmd_) f |= v1::flags::BRAKE_RELEASE_CMD;
    if (referenced_) f |= v1::flags::POSITION_REFERENCED;
    if (has_config_) f |= v1::flags::CONFIGURED;
    if (supervisor_on()) {  // r10（Issue #44・#46）
      if (sup_.load_limited()) f |= v1::flags::LOAD_LIMITED;
      if (sup_.feedback_uncertain()) f |= v1::flags::FEEDBACK_UNCERTAIN;
      if (brake_parked()) f |= v1::flags::PARKED;
      if (sup_.end_braking()) f |= v1::flags::END_BRAKING;
      if (sup_.soft_recovery() || sup_.fe_recovery()) f |= v1::flags::LIMIT_RECOVERY;
    }
    return f;
  }

  // いま送るとしたら、の STATUS。
  v1::Status status(uint64_t now_us) const {
    v1::Status s;
    memset(&s, 0, sizeof(s));
    s.boot_id = id_.boot_id;
    s.mcu_time_us = mcu_time_us(now_us);
    s.mcu_state = state_;
    s.selected_axis = selected_axis_;
    s.fault_code = fault_code();
    s.flags = flags();
    s.pwm_permille = pwm_permille();
    s.actual_count = actual_count();
    s.target_count = target_count();
    s.velocity_count_s = velocity_;
    s.following_error_count = following_error_count();
    s.last_applied_frame = last_applied_frame_;
    s.last_applied_seq = last_applied_seq_;
    s.last_rx_seq = last_rx_seq_;
    s.current_ma = 0;  // 電流計測なし（CURRENT_VALID は立てない）
    s.rx_error_count = rx_error_count_;
    s.stale_drop_count = stale_drop_count_;
    return s;
  }

 private:
  enum class StopKind : uint8_t { Hold, Disarm, Fault };
  // 校正の段階（protocol.md §1.2 の手順）: 主電源 ON 指令 → cal_power_settle → ブレーキ解除指令 → brake_release_wait →
  // AutoMachine（Cal）→ 終了手順（ブレーキ給電 OFF → brake_apply_wait → CAL_RESULT → DISARMED）
  enum class CalPhase : uint8_t { Idle, PowerSettle, BrakeRelease, Running, Shutdown, PowerHold };

  struct Accepted {
    uint8_t type;
    uint32_t request_id;
    uint8_t len;
    uint8_t payload[48];  // CONFIG の 44 byte が最大
  };

  // --- 識別・選択 ---
  V1Identity id_{};
  uint8_t selected_axis_ = 0;
  int8_t sign_ = 1;
  bool began_ = false;
  uint64_t boot_origin_us_ = 0;
  uint64_t last_tick_us_ = 0;
  uint64_t now_us_ = 0;  // 直近に handle / tick へ渡された時刻（即時 STATUS の mcu_time_us 用）

  // --- 状態 ---
  uint8_t state_ = v1::state::BOOT;
  uint16_t fault_code_ = v1::fault::NONE;
  StopKind stop_kind_ = StopKind::Hold;
  uint16_t stop_fault_code_ = v1::fault::NONE;
  uint16_t last_host_fault_code_ = v1::fault::NONE;
  bool estop_latched_ = false;

  v1::Config config_{};
  bool has_config_ = false;
  bool referenced_ = false;

  // --- タイマー（絶対 us） ---
  uint64_t arming_started_us_ = 0;
  uint64_t brake_release_at_us_ = 0;
  uint64_t disarming_started_us_ = 0;
  uint64_t stopping_started_us_ = 0;
  uint64_t lease_us_ = 0;
  uint64_t valid_until_us_ = 0;
  bool brake_release_cmd_ = false;

  // --- SET_POSITION ---
  uint32_t last_rx_seq_ = 0;
  uint32_t last_applied_seq_ = 0;
  uint32_t last_applied_frame_ = 0;
  uint16_t rx_error_count_ = 0;
  uint16_t stale_drop_count_ = 0;
  bool has_rx_ = false;
  uint64_t last_rx_us_ = 0;

  // --- 位置・軌道 ---
  int64_t raw_ = 0;
  int64_t reference_offset_ = 0;
  double setpoint_ = 0.0;
  double setpoint_vel_ = 0.0;
  double control_target_ = 0.0;
  double frame_max_vel_ = 0.0;
  double frame_max_acc_ = 0.0;
  V1PlayoutBuffer playout_{};  // Issue #45: ACTIVE で受けた目標（受信時刻つき）。START・hold_here で空にする
  V1PlayoutFit playout_fit_{};  // 直近の tick の軌道の推定（計器・テスト用）
  V1PlayoutClock playout_clock_{};  // Issue #45: 有効な SET_POSITION 全部（HOLDING 中も）の (seq, 受信時刻) から時刻を付け直す
  int duty_permille_ = 0;  // MD13S の向きの符号付き permille
  V1DutyTerms terms_{};  // 直近の位置ループの各項（計器）

  // 速度（100 ms 窓）
  int64_t vel_anchor_raw_ = 0;
  uint64_t vel_anchor_us_ = 0;
  int32_t velocity_ = 0;

  // 速い速度推定（r9、Issue #35。update_v_fast）: tick ごとの (時刻, 生カウント) のリング。5 ms 周期なら 155 ms 分
  struct VFastSample {
    uint64_t us;
    int64_t raw;
  };
  static constexpr size_t kVFastCap = 32;
  VFastSample vf_buf_[kVFastCap]{};
  size_t vf_head_ = 0;   // 次に書く位置
  size_t vf_count_ = 0;
  bool v_fast_valid_ = false;
  double v_fast_ = 0.0;  // [count/s]

  // r10（Issue #44・#46）: 保護の監視
  V1Supervisor sup_{};
  // r10: 静止目標での引き上げ（update_escalation）と小さな残り誤差の受け入れ
  bool esc_running_ = false;
  int esc_dir_ = 0;
  int esc_level_ = 0;  // [permille]
  uint64_t esc_win_start_us_ = 0;
  int64_t esc_anchor_raw_ = 0;
  int last_combined_pm_ = 0;  // 直近の v1_duty_command の結果（引き上げの起点）
  bool residual_accepted_ = false;
  bool hold_pulses_spent_ = false;
  // ブレーキ駐車（2026-11-02 の操作者の決定）
  enum class ParkPhase : uint8_t { None, Engaging, Parked, Releasing };
  ParkPhase park_ = ParkPhase::None;
  uint64_t park_phase_us_ = 0;
  int park_hold_out_ = 0;        // 入るときの保持出力 [permille、カウント増が正]。戻るときもこの値を出してからブレーキを解除する
  bool park_release_armed_ = false;  // 戻りかけ: 保持出力を出した次の tick でブレーキを解除する
  double park_target_ = 0.0;     // 駐車した目標（フレームの目標の幅の中央）
  bool still_running_ = false;   // 静止の窓を数えている
  uint64_t still_start_us_ = 0;
  double still_min_ = 0.0, still_max_ = 0.0;
  uint64_t park_cooldown_until_us_ = 0;
  int park_count_ = 0;  // r10（hold_learn）: 保持の帯の近くで進まないパルスを打ち切った（以後は I が詰める）

  // 無パルス（ENCODER_FAULT）
  bool stall_running_ = false;
  V1DutyState duty_state_{};  // 不感帯と床のヒステリシス（位置制御が止まれば戻す）
  uint64_t settle_dwell_us_ = 0;  // 整定判定（Issue #25 手順 10「S」）: 条件が続いている時間

  // 追従中の動き出し補助（Issue #25 手順 9「T」）
  bool assist_active_ = false;          // 床を掛けている
  bool assist_window_running_ = false;  // 対象になってから assist_ms を数えている
  uint64_t assist_window_start_us_ = 0;
  int64_t assist_anchor_raw_ = 0;       // 窓の開始時（ON の後は ON 時）のカウント
  int64_t assist_err_anchor_abs_ = 0;   // 窓の開始時の |誤差|（増え続けていることの基準）
  // 動き出しパルス（Issue #25 r6。update_breakaway）
  bool brk_active_ = false;          // パルスを出している
  bool brk_running_ = false;         // 要求が続いていて窓／パルスを数えている
  bool brk_exhausted_ = false;       // 進まないまま max_pulses 打った（要求が続く間は打たない。進めば戻る）
  int brk_dir_ = 0;                  // 要求の向き（+1 = カウント増）
  int brk_pulses_ = 0;               // 進まないまま打ったパルスの数
  uint64_t brk_phase_start_us_ = 0;  // 窓／パルスの開始
  int64_t brk_anchor_raw_ = 0;       // 窓／パルスの開始時のカウント
  int brk_mode_ = 0;                 // 0 = 静止目標、1 = 走行中（帯の外）、2 = 走行中で軌道速度が帯の中（r8）
  uint64_t brk_cooldown_until_us_ = 0;  // 帯の中で押した後の休みの終わり（r8）
  int64_t brk_err_anchor_abs_ = 0;   // 窓の開始時の |誤差|（走行中: 増えていなければ打たない）
  uint64_t stall_start_us_ = 0;
  uint64_t stall_last_change_us_ = 0;
  int64_t stall_last_raw_ = 0;

  // 停止ブースト（Issue #15 H4）
  int stall_boost_ = 0;              // duty_min への上乗せ [permille]（V1_STALL_BOOST_STEP_PERMILLE 刻み）
  bool boost_eligible_ = false;      // 前の tick で対象だった
  bool boost_window_running_ = false;
  int8_t boost_err_sign_ = 0;        // 直近に評価した誤差の符号（反転で 0 に戻す）
  uint64_t boost_window_start_us_ = 0;
  uint64_t boost_active_since_us_ = 0;  // 対象になってから（動けば取り直す）。stall_boost_max_ms の起点
  int64_t boost_anchor_raw_ = 0;     // 窓の開始時のカウント

  // --- 校正（CALIBRATING） ---
  AutoMachine cal_{};
  CalPhase cal_phase_ = CalPhase::Idle;
  bool cal_keep_power_ = false;  // KEEP_POWER 付きで開始した（電源担当軸のみ有効）
  uint64_t cal_phase_started_us_ = 0;  // いまの段階に入った時刻
  v1::CalResult pending_cal_result_{};  // 終了手順の後に送る結果
  bool has_cal_result_ = false;
  v1::CalResult last_cal_result_{};
  MotorProfile dirty_profile_{};
  MotorProfile profile_hint_{};
  bool profile_dirty_ = false;

  // --- DUPLICATE 判定（受理した命令、古い順） ---
  Accepted accepted_[kDedupeDepth]{};
  size_t accepted_count_ = 0;

  // --- outbox（リング） ---
  v1::McuMsg outbox_[kOutboxCap]{};
  size_t out_head_ = 0;
  size_t out_count_ = 0;

  // --- STATUS ---
  bool has_next_status_ = false;
  uint64_t next_status_us_ = 0;
  bool has_last_key_ = false;
  uint8_t key_state_ = 0;
  uint16_t key_fault_ = 0;
  uint16_t key_flags_ = 0;
  uint8_t key_axis_ = 0;

  // ------------------------------------------------------------------------
  // 補助
  // ------------------------------------------------------------------------

  static int32_t clamp_count(int64_t v) {
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return static_cast<int32_t>(v);
  }
  static int32_t to_count(double v) { return clamp_count(static_cast<int64_t>(llround(v))); }
  static uint64_t ms_to_us(uint64_t ms) { return ms * 1000ull; }

  bool parked() const {
    return state_ == v1::state::DISARMED || state_ == v1::state::FAULT || state_ == v1::state::BOOT;
  }
  bool powered_state() const {
    return state_ == v1::state::ARMING || state_ == v1::state::HOLDING || state_ == v1::state::ACTIVE ||
           state_ == v1::state::STOPPING || state_ == v1::state::DISARMING || state_ == v1::state::CALIBRATING;
  }
  // pwm_permille の向きに使う符号。校正で P 端に当たって向きが分かった後はその値（NVS の sign は未校正だと既定値のまま）。
  int8_t effective_sign() const {
    if (state_ == v1::state::CALIBRATING && cal_.origin_valid) return cal_.working_sign;
    return sign_;
  }
  uint8_t cal_step_code() const {
    switch (cal_.cal_step) {
      case CalStep::ToP: return v1::cal_step::TO_P;
      case CalStep::ToN: return v1::cal_step::TO_N;
      case CalStep::ToNeutral: return v1::cal_step::TO_NEUTRAL;
      default: return v1::cal_step::NONE;
    }
  }
  bool power_controller() const {
    return has_config_ && (config_.role_flags & v1::role_flags::POWER_CONTROLLER) != 0;
  }
  // 位置ループが動いている（ARMING のブレーキ解除以降・HOLDING・ACTIVE・STOPPING・DISARMING）
  bool position_control_on() const {
    switch (state_) {
      case v1::state::ARMING: return brake_release_cmd_;
      case v1::state::HOLDING:
      case v1::state::ACTIVE:
      case v1::state::STOPPING:
      case v1::state::DISARMING: return true;
      default: return false;
    }
  }
  bool inside_soft_window() const {
    const int32_t a = actual_count();
    return a >= config_.soft_min_count && a <= config_.soft_max_count;
  }
  // FAULT_RESET を拒む「残る原因」。v1 では画面の非常停止ボタンのラッチだけ（設計 §3）。
  // ENCODER_FAULT の無パルスは FAULT 中（ブレーキ作動・PWM 0）には観測できないので原因として残さず、解除は通す
  // （POSITION_REFERENCED は消える）。
  bool fault_cause_present() const { return estop_latched_; }

  void hold_here() {
    const double a = static_cast<double>(actual_count());
    setpoint_ = a;
    setpoint_vel_ = 0.0;
    control_target_ = a;
    playout_.clear();
  }

  // ------------------------------------------------------------------------
  // 命令の振り分け（protocol.md §4）
  // ------------------------------------------------------------------------

  void dispatch(const v1::HostMsg& m, uint64_t now, uint64_t rx_us) {
    has_rx_ = true;
    last_rx_us_ = now;
    switch (m.type) {
      case v1::msg::HELLO_REQ: on_hello_req(m.hello_req.request_id); break;
      case v1::msg::CLOCK_SYNC: on_clock_sync(m.clock_sync, now); break;
      case v1::msg::SET_POSITION: on_set_position(m.set_position, now, rx_us); break;
      case v1::msg::HOST_FAULT: on_host_fault(m.host_fault, now); break;
      case v1::msg::CONFIG: command(m, m.config.request_id, now); break;
      case v1::msg::ARM_PREPARE: command(m, m.arm_prepare.request_id, now); break;
      case v1::msg::START: command(m, m.start.request_id, now); break;
      case v1::msg::CONTROLLED_STOP: command(m, m.controlled_stop.request_id, now); break;
      case v1::msg::DISARM: command(m, m.disarm.request_id, now); break;
      case v1::msg::FAULT_RESET: command(m, m.fault_reset.request_id, now); break;
      case v1::msg::SET_REFERENCE: command(m, m.set_reference.request_id, now); break;
      case v1::msg::CALIBRATE: command(m, m.calibrate.request_id, now); break;
      case v1::msg::SET_LIMITS: command(m, m.set_limits.request_id, now); break;
      default: break;  // decode 済みなので来ない
    }
  }

  // 命令ごとの条件。受理なら ack_reason::NONE、拒否なら理由。
  uint16_t run_command(const v1::HostMsg& m, uint64_t now) {
    switch (m.type) {
      case v1::msg::CONFIG: return on_config(m.config);
      case v1::msg::ARM_PREPARE: return on_arm_prepare(m.arm_prepare.session_epoch, now);
      case v1::msg::START: return on_start(m.start.session_epoch, now);
      case v1::msg::CONTROLLED_STOP: return on_controlled_stop(m.controlled_stop.session_epoch, now);
      case v1::msg::DISARM: return on_disarm(now);
      case v1::msg::FAULT_RESET: return on_fault_reset();
      case v1::msg::SET_REFERENCE: return on_set_reference(m.set_reference.reference_count);
      case v1::msg::CALIBRATE: return on_calibrate(m.calibrate.mode, m.calibrate.flags, now);
      case v1::msg::SET_LIMITS: return on_set_limits(m.set_limits);
      default: return v1::ack_reason::BAD_STATE;
    }
  }

  // ACK を返す命令の共通処理（HOST_FAULT 以外）: request_id 0 → DUPLICATE → 異常ゲート → 命令ごとの条件。
  void command(const v1::HostMsg& m, uint32_t request_id, uint64_t now) {
    if (request_id == 0) {
      push_ack(request_id, m.type, v1::ack_result::REJECTED, v1::ack_reason::INVALID_PARAM);
      return;
    }
    uint8_t payload[v1::MAX_PAYLOAD_LEN];
    const size_t plen = v1::encode_host_payload(m, payload, sizeof(payload));
    if (is_duplicate(m.type, request_id, payload, plen)) {
      push_ack(request_id, m.type, v1::ack_result::DUPLICATE, v1::ack_reason::NONE);
      return;
    }
    uint16_t reason;
    if (m.type != v1::msg::FAULT_RESET && fault_code() != v1::fault::NONE) {
      reason = v1::ack_reason::FAULT_ACTIVE;
    } else {
      reason = run_command(m, now);
    }
    if (reason == v1::ack_reason::NONE) {
      remember(m.type, request_id, payload, plen);
      push_ack(request_id, m.type, v1::ack_result::ACCEPTED, v1::ack_reason::NONE);
      if (m.type == v1::msg::SET_LIMITS) push_limits();  // 受理した直後だけ、ACK の後に 1 回（DUPLICATE では送らない）
    } else {
      // 拒否は副作用が無いので記憶しない（再送は改めて評価する）
      push_ack(request_id, m.type, v1::ack_result::REJECTED, reason);
    }
  }

  bool is_duplicate(uint8_t type, uint32_t request_id, const uint8_t* payload, size_t plen) const {
    for (size_t i = 0; i < accepted_count_; ++i) {
      const Accepted& a = accepted_[i];
      if (a.type == type && a.request_id == request_id && a.len == plen && memcmp(a.payload, payload, plen) == 0) {
        return true;
      }
    }
    return false;
  }

  // 同じ request_id の古い記憶は置き換え、満杯なら最古を捨てる。
  void remember(uint8_t type, uint32_t request_id, const uint8_t* payload, size_t plen) {
    size_t w = 0;
    for (size_t r = 0; r < accepted_count_; ++r) {
      if (accepted_[r].request_id != request_id) {
        if (w != r) accepted_[w] = accepted_[r];
        w += 1;
      }
    }
    accepted_count_ = w;
    if (accepted_count_ >= kDedupeDepth) {
      for (size_t i = 1; i < accepted_count_; ++i) accepted_[i - 1] = accepted_[i];
      accepted_count_ -= 1;
    }
    Accepted& a = accepted_[accepted_count_++];
    a.type = type;
    a.request_id = request_id;
    a.len = static_cast<uint8_t>(plen > sizeof(a.payload) ? sizeof(a.payload) : plen);
    memset(a.payload, 0, sizeof(a.payload));
    memcpy(a.payload, payload, a.len);
  }

  void on_hello_req(uint32_t request_id) {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::HELLO;
    m.hello.request_id = request_id;
    m.hello.protocol_version = v1::PROTOCOL_VERSION;
    m.hello.selected_axis = selected_axis_;
    m.hello.mcu_state = state_;
    m.hello.device_id = id_.device_id;
    m.hello.boot_id = id_.boot_id;
    m.hello.fw_major = id_.fw_major;
    m.hello.fw_minor = id_.fw_minor;
    m.hello.fw_patch = id_.fw_patch;
    m.hello.capabilities = v1::capabilities::POSITION_MODE | v1::capabilities::LIMITS;
    m.hello.calibration_hash = calibration_hash();
    push(m);
  }

  void on_clock_sync(const v1::ClockSync& c, uint64_t now) {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::CLOCK_SYNC_REPLY;
    m.clock_sync_reply.request_id = c.request_id;
    m.clock_sync_reply.pc_time_us = c.pc_time_us;
    m.clock_sync_reply.mcu_rx_us = mcu_time_us(now);
    m.clock_sync_reply.mcu_tx_us = mcu_time_us(now);
    push(m);
  }

  // CONFIG: DISARMED のときだけ。§4 の表の検査だけ（0 は拒否）。速度・加速度の上限は絶対上限で黙ってクランプして保存する
  // （PC は CONFIG に絶対上限を入れ、毎フレームの SET_POSITION で実際の上限を決める。motion-limits 設計 §4.4）。
  // epoch が変われば last_rx_seq = 0。
  uint16_t on_config(const v1::Config& c) {
    if (state_ != v1::state::DISARMED) return v1::ack_reason::BAD_STATE;
    const bool valid = c.soft_min_count < c.soft_max_count && c.max_vel_count_s > 0 && c.max_acc_count_s2 > 0 &&
                       c.following_error_limit_count > 0 && c.overspeed_count_s > 0 && c.command_timeout_ms > 0 &&
                       c.stop_timeout_ms > 0;
    if (!valid) return v1::ack_reason::INVALID_PARAM;
    if (!has_config_ || config_.session_epoch != c.session_epoch) {
      last_rx_seq_ = 0;  // seq の単調性は epoch ごと
    }
    config_ = c;
    playout_clock_.reset();  // Issue #45（PR #49 レビュー P2b）: セッションが変わったら時刻の付け直しを取り直す
    playout_.clear();
    if (config_.max_vel_count_s > V1_ABS_MAX_VEL_COUNT_S) config_.max_vel_count_s = V1_ABS_MAX_VEL_COUNT_S;
    if (config_.max_acc_count_s2 > V1_ABS_MAX_ACC_COUNT_S2) config_.max_acc_count_s2 = V1_ABS_MAX_ACC_COUNT_S2;
    has_config_ = true;
    return v1::ack_reason::NONE;
  }

  // SET_LIMITS（motion-limits 設計 §4.2）: 状態は問わない（ACTIVE 中も受理）。検査に通れば tuning の該当項目を置き換え、
  // Duty の上下限は絶対上限 V1_ABS_DUTY_MAX でクランプする。拒否なら tuning は変えない。LIMITS は command() が ACK の後に送る。
  uint16_t on_set_limits(const v1::SetLimits& l) {
    if (l.duty_max_percent == 0 || l.duty_min_percent > l.duty_max_percent || l.ff_count_s_per_percent == 0 ||
        l.deadband_out_count < l.deadband_count) {
      return v1::ack_reason::INVALID_PARAM;
    }
    int duty_max = l.duty_max_percent;
    if (duty_max > V1_ABS_DUTY_MAX) duty_max = V1_ABS_DUTY_MAX;
    int duty_min = l.duty_min_percent;
    if (duty_min > duty_max) duty_min = duty_max;
    tuning.duty_max = duty_max;
    tuning.duty_min = duty_min;
    tuning.kp_percent_per_100 = l.kp_percent_per_100;
    tuning.ff_count_s_per_percent = l.ff_count_s_per_percent;
    tuning.deadband = l.deadband_count;
    tuning.deadband_out = l.deadband_out_count;
    return v1::ack_reason::NONE;
  }

  // ARM_PREPARE: 拒否理由の順序は §4 の表。受理で ARMING（主電源 ON 指令・ブレーキ作動のまま）。
  uint16_t on_arm_prepare(uint32_t epoch, uint64_t now) {
    if (state_ != v1::state::DISARMED) return v1::ack_reason::BAD_STATE;
    if (!has_config_) return v1::ack_reason::NOT_CONFIGURED;
    if (!referenced_) return v1::ack_reason::NOT_REFERENCED;
    if (config_.session_epoch != epoch) return v1::ack_reason::EPOCH_MISMATCH;
    if (estop_latched_) return v1::ack_reason::BAD_STATE;  // ローカル操作中（設計 §3。通常は FAULT 側で弾かれる）
    if (!inside_soft_window()) return v1::ack_reason::BAD_STATE;  // 解除しても SOFT_LIMIT になるだけ
    state_ = v1::state::ARMING;
    arming_started_us_ = now;
    lease_us_ = now;
    valid_until_us_ = 0;  // START に数えるのは以後に受けたフレームだけ
    brake_release_cmd_ = false;
    hold_here();
    return v1::ack_reason::NONE;
  }

  // START: HOLDING・epoch 一致・期限内の SET_POSITION。目標は次の SET_POSITION まで保持位置のまま。
  uint16_t on_start(uint32_t epoch, uint64_t now) {
    if (state_ != v1::state::HOLDING) return v1::ack_reason::BAD_STATE;
    if (!has_config_) return v1::ack_reason::NOT_CONFIGURED;
    if (config_.session_epoch != epoch) return v1::ack_reason::EPOCH_MISMATCH;
    if (now > valid_until_us_) return v1::ack_reason::BAD_STATE;  // 受理しても次の tick で期限切れになる
    state_ = v1::state::ACTIVE;
    control_target_ = setpoint_;
    playout_.clear();  // 前の ACTIVE の残りや HOLDING 中のフレームは再生しない（目標は次の SET_POSITION まで保持位置）
    frame_max_vel_ = static_cast<double>(config_.max_vel_count_s);
    frame_max_acc_ = static_cast<double>(config_.max_acc_count_s2);
    return v1::ack_reason::NONE;
  }

  // SET_POSITION: ACK なし。古い → stale_drop、apply_at ≠ 0 → rx_error、有効 → lease。ACTIVE でだけ目標に反映。
  // 校正中は捨てる（数えない。protocol.md §1.2）。
  void on_set_position(const v1::SetPosition& m, uint64_t now, uint64_t rx_us) {
    if (state_ == v1::state::CALIBRATING) return;
    if (!has_config_ || m.session_epoch != config_.session_epoch || m.seq <= last_rx_seq_) {
      if (stale_drop_count_ < 0xFFFF) stale_drop_count_ += 1;
      return;
    }
    if (m.apply_at_us != 0) {
      record_rx_error();  // v1 では時刻同期が無いので予約適用はできない。lease も延長しない
      return;
    }
    last_rx_seq_ = m.seq;
    playout_clock_.push(m.seq, rx_us);  // Issue #45: START 前（HOLDING）から周期を掴んでおく
    lease_us_ = now;
    valid_until_us_ = now + ms_to_us(m.valid_for_ms);
    if (state_ == v1::state::ACTIVE) {
      int32_t target = m.target_count;
      if (target < config_.soft_min_count) target = config_.soft_min_count;
      if (target > config_.soft_max_count) target = config_.soft_max_count;
      control_target_ = static_cast<double>(target);
      playout_.push(m.seq, rx_us, target);  // Issue #45: 受信時刻で時刻印（再生バッファの経路だけが読む）
      const uint32_t vmax = m.max_vel_count_s < config_.max_vel_count_s ? m.max_vel_count_s : config_.max_vel_count_s;
      const uint32_t amax =
          m.max_acc_count_s2 < config_.max_acc_count_s2 ? m.max_acc_count_s2 : config_.max_acc_count_s2;
      frame_max_vel_ = static_cast<double>(vmax);
      frame_max_acc_ = static_cast<double>(amax);
      last_applied_seq_ = m.seq;
      last_applied_frame_ = m.group_frame_id;
    }
  }

  // CONTROLLED_STOP: epoch 一致。ACTIVE → STOPPING、それ以外は何もせず受理。
  uint16_t on_controlled_stop(uint32_t epoch, uint64_t now) {
    if (!has_config_ || config_.session_epoch != epoch) return v1::ack_reason::EPOCH_MISMATCH;
    if (state_ == v1::state::ACTIVE) begin_stop(StopKind::Hold, now);
    return v1::ack_reason::NONE;
  }

  // DISARM: 動いていれば減速してから DISARMING。DISARMED / DISARMING では何もしない。
  uint16_t on_disarm(uint64_t now) {
    switch (state_) {
      case v1::state::DISARMED:
      case v1::state::DISARMING:
      case v1::state::BOOT: break;
      case v1::state::ARMING:
      case v1::state::HOLDING: enter_disarming(now); break;
      case v1::state::ACTIVE: begin_stop(StopKind::Disarm, now); break;
      case v1::state::STOPPING: stop_kind_ = StopKind::Disarm; break;
      case v1::state::CALIBRATING: cal_cancel(now); break;  // 校正の中止（終了手順を経て DISARMED）
      case v1::state::FAULT: return v1::ack_reason::FAULT_ACTIVE;  // 異常ゲートで先に弾かれる
      default: break;
    }
    return v1::ack_reason::NONE;
  }

  // FAULT_RESET: FAULT のときだけ。原因が残っていれば FAULT_CAUSE_PRESENT。解除後 DISARMED。
  uint16_t on_fault_reset() {
    if (state_ != v1::state::FAULT) return v1::ack_reason::BAD_STATE;
    if (fault_cause_present()) return v1::ack_reason::FAULT_CAUSE_PRESENT;
    if (fault_code_ == v1::fault::ENCODER_FAULT) referenced_ = false;
    sup_.clear_episode();  // r10: FAULT_RESET は FEEDBACK_UNCERTAIN の明示的な解除のひとつ（位置の上限は戻さない）
    state_ = v1::state::DISARMED;
    fault_code_ = v1::fault::NONE;
    stop_kind_ = StopKind::Hold;
    hold_here();
    return v1::ack_reason::NONE;
  }

  // HOST_FAULT: 共通規則を飛ばして毎回実行、常に ACCEPTED。記憶しない。
  void on_host_fault(const v1::HostFault& h, uint64_t now) {
    last_host_fault_code_ = h.fault_code;
    const uint16_t code = (state_ == v1::state::STOPPING && stop_kind_ == StopKind::Fault) ? stop_fault_code_
                                                                                            : v1::fault::HOST_FAULT;
    latch_fault(code, now);
    push_ack(h.request_id, v1::msg::HOST_FAULT, v1::ack_result::ACCEPTED, v1::ack_reason::NONE);
  }

  // SET_REFERENCE: DISARMED のときだけ。現在の生カウントを reference_count と定義する（符号は掛けない）。
  uint16_t on_set_reference(int32_t reference_count) {
    if (state_ != v1::state::DISARMED) return v1::ack_reason::BAD_STATE;
    reference_offset_ = raw_ - static_cast<int64_t>(reference_count);
    hold_here();
    referenced_ = true;
    sup_.reset_trust(raw_, now_us_);  // r10: 基準の取り直しは信用の取り直し（見えない走行の包絡線をすべて 0 に）
    return v1::ack_reason::NONE;
  }

  // CALIBRATE（protocol.md §4、§1.2）: mode 1 は DISARMED のときだけ（CONFIG・基準は不要）。受理で CALIBRATING。
  // mode 0 は CALIBRATING なら中止、それ以外は何もせず受理。0・1 以外は INVALID_PARAM。
  uint16_t on_calibrate(uint8_t mode, uint8_t flags, uint64_t now) {
    if (mode != v1::cal_mode::START && mode != v1::cal_mode::CANCEL) return v1::ack_reason::INVALID_PARAM;
    if (mode == v1::cal_mode::CANCEL) {
      if (state_ == v1::state::CALIBRATING) cal_cancel(now);
      return v1::ack_reason::NONE;
    }
    if (state_ != v1::state::DISARMED) return v1::ack_reason::BAD_STATE;
    if (estop_latched_) return v1::ack_reason::BAD_STATE;  // ローカル操作中（通常は FAULT 側で弾かれる）
    cal_keep_power_ = power_controller() && (flags & v1::cal_flags::KEEP_POWER) != 0;
    state_ = v1::state::CALIBRATING;
    cal_phase_ = CalPhase::PowerSettle;  // 主電源 ON 指令（電源担当軸のみ。powered_state で立つ）
    cal_phase_started_us_ = now;
    brake_release_cmd_ = false;
    duty_permille_ = 0;
    referenced_ = false;  // 端当てで原点を取り直す。成功したときだけ P 端 = 0 で立て直す
    cal_.cfg = cal_config;
    MotorProfile p;
    p.motor = selected_axis_;
    p.sign = sign_;
    if (profile_hint_.motor == selected_axis_ && profile_hint_.calibrated && profile_hint_.stroke > 0) {
      p.stroke = profile_hint_.stroke;      // 端までの距離の見積もりに使う（速い区間と遅い区間の切り替え）
      p.neutral = profile_hint_.neutral;
      p.calibrated = true;
    }
    cal_.set_profile(p);  // origin_valid = false・Idle に戻る
    cal_.profile_dirty = false;
    memset(&pending_cal_result_, 0, sizeof(pending_cal_result_));
    return v1::ack_reason::NONE;
  }

  // ------------------------------------------------------------------------
  // 遷移
  // ------------------------------------------------------------------------

  void begin_stop(StopKind kind, uint64_t now) {
    state_ = v1::state::STOPPING;
    stop_kind_ = kind;
    stopping_started_us_ = now;
  }

  // 減速してからラッチする異常（COMMAND_TIMEOUT）。止まっていれば即ラッチ。
  void begin_fault_stop(uint16_t code, uint64_t now) {
    if (setpoint_vel_ == 0.0) {
      latch_fault(code, now);
    } else if (state_ == v1::state::STOPPING) {
      stop_kind_ = StopKind::Fault;  // 減速の開始時刻はそのまま（stop_timeout の起点を保つ）
      stop_fault_code_ = code;
    } else {
      begin_stop(StopKind::Fault, now);
      stop_fault_code_ = code;
    }
  }

  void finish_stop(uint64_t now) {
    switch (stop_kind_) {
      case StopKind::Hold:
        state_ = v1::state::HOLDING;
        control_target_ = setpoint_;
        break;
      case StopKind::Disarm: enter_disarming(now); break;
      case StopKind::Fault: latch_fault(stop_fault_code_, now); break;
    }
  }

  // ブレーキ給電 OFF → BRAKE_APPLY_WAIT_MS → DISARMED
  void enter_disarming(uint64_t now) {
    state_ = v1::state::DISARMING;
    disarming_started_us_ = now;
    brake_release_cmd_ = false;
    stop_kind_ = StopKind::Hold;
  }

  // FAULT ラッチ: PWM 0・ブレーキ作動・主電源 OFF 指令。最初のコードが残る。FAULT_EVENT を 1 回。
  // 校正中なら校正も終わる: CAL_RESULT(CANCELLED) を FAULT_EVENT より前に 1 回（基準は立てない。終了手順は踏まず即停止）。
  void latch_fault(uint16_t code, uint64_t now) {
    if (state_ == v1::state::FAULT) return;
    const uint8_t before = state_;
    if (state_ == v1::state::CALIBRATING) {
      cal_abort_for_fault(code == v1::fault::LOCAL_ESTOP ? "estop" : "fault", now);
    }
    state_ = v1::state::FAULT;
    fault_code_ = code;
    stop_kind_ = StopKind::Hold;
    brake_release_cmd_ = false;
    duty_permille_ = 0;  // 次の tick を待たず PWM 0
    terms_ = V1DutyTerms{};
    duty_state_ = V1DutyState{};  // 積分（保持バイアス）も捨てる（Issue #25 手順 9「F」）
    reset_v_fast();               // r9: 速い速度推定も取り直す（積分と同じ契機）
    stall_running_ = false;
    hold_here();
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::FAULT_EVENT;
    m.fault_event.boot_id = id_.boot_id;
    m.fault_event.mcu_time_us = mcu_time_us(now);
    m.fault_event.fault_code = code;
    m.fault_event.mcu_state_before = before;
    m.fault_event.actual_count = actual_count();
    push(m);
  }

  // ------------------------------------------------------------------------
  // tick の中身
  // ------------------------------------------------------------------------

  void update_velocity(uint64_t now) {
    const uint64_t window = ms_to_us(tuning.velocity_window_ms);
    if (now >= vel_anchor_us_ + window) {
      const int64_t dc = raw_ - vel_anchor_raw_;
      const uint64_t dt = now - vel_anchor_us_;
      velocity_ = clamp_count((dc * 1000000) / static_cast<int64_t>(dt));
      vel_anchor_raw_ = raw_;
      vel_anchor_us_ = now;
    }
  }

  void reset_v_fast() {
    vf_head_ = 0;
    vf_count_ = 0;
    v_fast_valid_ = false;
    v_fast_ = 0.0;
  }

  // r9（Issue #35）: 速い速度推定 v_fast = (いまの生カウント − 約 window 前の生カウント) / 実際の経過秒。tick ごとに 1 サンプル積む。
  // 基準は「window 以上前のサンプルのうち最も新しいもの」。まだ無ければ（位置制御の開始直後・FAULT の後）最も古いサンプルが
  // window / 2 以上前ならそれ、それも無ければ推定なし（valid = false → 減衰項 0）。前のサンプルから 2 × window を超えて空いた
  //（または時刻が戻った）ら古いサンプルを捨てる。経過が 2 × window を超える基準は使わない（古すぎる平均で減衰させない）。
  // 生カウント（reference_offset を引く前）で差を取るので SET_REFERENCE の座標移動は速度に出ない。
  void update_v_fast(uint64_t now) {
    const uint64_t win = ms_to_us(tuning.v_fast_window_ms);
    if (win == 0) { reset_v_fast(); return; }
    if (vf_count_ > 0) {
      const VFastSample& last = vf_buf_[(vf_head_ + kVFastCap - 1) % kVFastCap];
      if (now < last.us || now - last.us > 2 * win) reset_v_fast();
    }
    vf_buf_[vf_head_] = VFastSample{now, raw_};
    vf_head_ = (vf_head_ + 1) % kVFastCap;
    if (vf_count_ < kVFastCap) ++vf_count_;
    v_fast_valid_ = false;
    v_fast_ = 0.0;
    const VFastSample* base = nullptr;
    for (size_t k = 1; k < vf_count_; ++k) {  // k = 0 はいま積んだサンプル
      const VFastSample& smp = vf_buf_[(vf_head_ + kVFastCap - 1 - k) % kVFastCap];
      if (now - smp.us >= win) { base = &smp; break; }
    }
    if (!base && vf_count_ >= 2) {
      const VFastSample& oldest = vf_buf_[(vf_head_ + kVFastCap - vf_count_) % kVFastCap];
      if (now - oldest.us >= win / 2) base = &oldest;
    }
    if (!base) return;
    const uint64_t dt = now - base->us;
    if (dt == 0 || dt > 2 * win) return;
    v_fast_ = static_cast<double>(raw_ - base->raw) * 1e6 / static_cast<double>(dt);
    v_fast_valid_ = true;
  }

  void check_timers(uint64_t now) {
    if (state_ == v1::state::ARMING) {
      const uint64_t t = now - arming_started_us_;
      // r10: 持続する停止の間はブレーキを解除し直さない（解除は DISARMED → ARM_PREPARE の手順だけ）
      if (!brake_release_cmd_ && !supervisor_inhibited() && t >= ms_to_us(tuning.power_settle_ms)) {
        // 電源が落ち着いた。ソフトリミット外ではブレーキを解除しない（解除直後に SOFT_LIMIT になるパルスを避ける）
        if (!inside_soft_window()) {
          latch_fault(v1::fault::SOFT_LIMIT, now);
          return;
        }
        hold_here();  // 現在位置を保持目標にして位置制御開始
        brake_release_cmd_ = true;
        brake_release_at_us_ = now;
      }
      if (brake_release_cmd_ && now - brake_release_at_us_ >= ms_to_us(tuning.brake_release_wait_ms)) {
        state_ = v1::state::HOLDING;
      }
    } else if (state_ == v1::state::DISARMING) {
      if (now - disarming_started_us_ >= ms_to_us(tuning.brake_apply_wait_ms)) {
        state_ = v1::state::DISARMED;  // ブレーキ作動済み: PWM 0・主電源 OFF
        hold_here();
      }
    }
    if (!has_config_) return;

    // lease: command_timeout_ms の間、有効な SET_POSITION が無い
    bool leased = false;
    switch (state_) {
      case v1::state::ARMING:
      case v1::state::HOLDING:
      case v1::state::ACTIVE: leased = true; break;
      case v1::state::STOPPING: leased = stop_kind_ == StopKind::Hold; break;
      default: break;
    }
    if (leased && now - lease_us_ > ms_to_us(config_.command_timeout_ms)) {
      begin_fault_stop(v1::fault::COMMAND_TIMEOUT, now);
    }
    // 目標の期限: ACTIVE → STOPPING → HOLDING
    if (state_ == v1::state::ACTIVE && now > valid_until_us_) {
      begin_stop(StopKind::Hold, now);
    }
    // 減速は stop_timeout_ms 内に終わる
    if (state_ == v1::state::STOPPING && now - stopping_started_us_ > ms_to_us(config_.stop_timeout_ms)) {
      latch_fault(stop_kind_ == StopKind::Fault ? stop_fault_code_ : v1::fault::STOP_TIMEOUT, now);
    }
  }

  // 速度・加速度制限つきの setpoint（sim_axis.rs の update_setpoint / profile_step と同じ。ただし Issue #45 の再生バッファ（既定）では
  // ACTIVE の参照を playout_step で作るので、PC 側の参照モデルとは ACTIVE の軌道の時刻・形が違う（STOPPING 以降は同じ）
  void update_setpoint(double dt) {
    if (!has_config_) return;
    playout_clock_.refit_if_dirty();  // Issue #45: 時刻の付け直しの当てはめは 1 tick に 1 回まで（push は O(1)）
    const double lo = static_cast<double>(config_.soft_min_count);
    const double hi = static_cast<double>(config_.soft_max_count);
    if (state_ == v1::state::ACTIVE && tuning.playout_delay_ms > 0) {
      playout_step(dt, lo, hi);  // Issue #45: 再生バッファから参照を作る（既定）
    } else if (state_ == v1::state::ACTIVE) {
      double p = setpoint_;
      double v = setpoint_vel_;
      profile_step(control_target_, frame_max_vel_, frame_max_acc_, dt, &p, &v);
      setpoint_ = p < lo ? lo : (p > hi ? hi : p);
      setpoint_vel_ = v;
    } else if (state_ == v1::state::STOPPING) {
      // 減速は最後に適用したフレームの max_acc で（CONFIG には絶対上限が入るので、そちらでは 40000 count/s² の急減速になる。
      // motion-limits 設計 §4.4）。frame_max_acc_ は START で CONFIG の値から初期化され SET_POSITION ごとに更新されるが、
      // フレームが max_acc = 0 を運ぶと 0 になり減速できないので、そのときだけ CONFIG の値に戻す。
      // PR #49 レビュー P1b: ごく小さい max_acc（1 など）でも止まれるよう V1_MIN_DECEL_COUNT_S2 を下限にする
      double amax = frame_max_acc_ > 0.0 ? frame_max_acc_ : static_cast<double>(config_.max_acc_count_s2);
      if (amax < static_cast<double>(V1_MIN_DECEL_COUNT_S2)) amax = static_cast<double>(V1_MIN_DECEL_COUNT_S2);
      const double dv = amax * dt;
      double v = setpoint_vel_;
      if (fabs(v) <= dv) {
        v = 0.0;
      } else {
        v -= dv * (v > 0 ? 1.0 : -1.0);
      }
      setpoint_vel_ = v;
      const double p = setpoint_ + v * dt;
      setpoint_ = p < lo ? lo : (p > hi ? hi : p);
    } else {
      setpoint_vel_ = 0.0;  // ARMING / HOLDING / DISARMING は保持
    }
  }

  // 再生バッファの参照が減速に使う加速度 [count/s²]: 最後のフレームの max_acc（CONFIG で頭打ち済み）。0・ごく小さい値でも
  // 止まれるよう V1_MIN_DECEL_COUNT_S2 を下限にする（PR #49 レビュー P1b）。加速はフレームの max_acc のまま（0 なら加速しない）
  float playout_decel() const {
    const float a = frame_max_acc_ > 0.0 ? static_cast<float>(frame_max_acc_) : 0.0f;
    const float lo = static_cast<float>(V1_MIN_DECEL_COUNT_S2);
    return a > lo ? a : lo;
  }

  // Issue #45: 再生バッファの参照（ACTIVE、playout_delay_ms > 0）。playout 時刻 tp = いま − 遅れ での軌道の推定
  //（V1PlayoutBuffer::fit。位置 p_t と傾き v_t）に、参照（setpoint_ / setpoint_vel_）を速度・加速度の制限の中で追従させる:
  //   v_des = v_t + 補正、補正 = 向き(e) × min(kp × |e|, sqrt(a_dec × |e|))（= 減速度 / 2 の制動曲線）、e = p_t − 参照の次の位置
  //  （p_t はソフトリミットで clamp。kp は目標が止まっていれば KP_FLAT、動いていれば playout_kp_per_s）
  //   v_des を ±max_vel と、次の 2 つの制動曲線で切る（どちらも減速度の 8 割。加速度制限いっぱいだと 1 tick 遅れて越える）:
  //    - 途絶の上限（PR #49 レビュー P1a）: 受けた最新の目標 + V1_PLAYOUT_LOSS_MARGIN_COUNT の先へは止まれる速さまで。
  //      フレームが途切れても（外挿 → 減速、期限切れ → STOPPING）参照は最新の目標を約 MARGIN 以上越えない。遅い軌道では
  //      最新の目標は遅れ（40 ms）の分だけ先にあり効かないが、速い軌道（≳ 270 count/s）では参照の遅れが v² / (2 × 0.8 × a_dec)
  //      まで増える（600 count/s・2000 count/s² で約 110 count。従来の profile_step も約 90 count 遅れていた）
  //    - Issue #46: ソフトリミットに向かう速度は v ≤ sqrt(2 × a_brake × max(0, d − reserve))（d = 参照から進む向きの端まで）。
  //      参照は端の reserve 手前で止まる（その内側にいれば端に向かっては動かない。離れる向きは制限しない）。
  //   Δv は加速なら ±max_acc × dt、減速なら ±a_dec × dt に制限して積分する（速度を一気に 0 にしない）。
  // 目標が止まった（窓の中の目標がすべて同じ → v_t がちょうど 0）・参照が p_t の SNAP 以内・1 tick で止まれる速さなら、
  // 参照を p_t に乗せて速度をちょうど 0 にする（保持・整定・停止の判定と、PR #48 の静止の監視は setpoint_vel_ == 0.0 を見る）。
  // まだ tp に届くフレームが無い（START 直後の遅れの間）・新しいフレームが外挿の上限を越えて途切れた → v_des = 0（a_dec で減速）。
  // 期限（valid_for）・lease・STOP・FAULT はこれまでどおり状態機械が決める（バッファにフレームが残っていても鮮度は延びない）。
  // 単精度（ESP32-S3 の FPU）。位置は参照からの差で float にする。
  void playout_step(double dt, double lo, double hi) {
    const uint64_t delay = ms_to_us(tuning.playout_delay_ms);
    const uint64_t tp = now_us_ > delay ? now_us_ - delay : 0;
    const V1PlayoutFit f =
        playout_.fit(tp, ms_to_us(tuning.playout_window_ms), ms_to_us(tuning.playout_extrap_ms), playout_clock_);
    playout_fit_ = f;
    const float dtf = static_cast<float>(dt);
    const float vmax = frame_max_vel_ > 0.0 ? static_cast<float>(frame_max_vel_) : 0.0f;
    const float amax = frame_max_acc_ > 0.0 ? static_cast<float>(frame_max_acc_) : 0.0f;
    const float adec = playout_decel();
    const double p0 = setpoint_;
    float v = static_cast<float>(setpoint_vel_);
    const float v_before = v;
    const bool tracking = f.has && !f.starved;
    double p_t = p0;
    float v_des = 0.0f;
    if (tracking) {
      p_t = f.pos < lo ? lo : (f.pos > hi ? hi : f.pos);
      const float e = static_cast<float>(p_t - p0) - v * dtf;  // この tick で進む分を見込んで比べる（参照が 1 tick 先行しないように）
      const float ea = fabsf(e);
      // 目標が止まっている（傾きちょうど 0）なら硬いゲインで寄せる（指数の尾を短く。ステップ目標でも従来の軌道と同程度で着く）。
      // 動いている間は推定の量子化の揺れを拾わないよう柔らかいゲイン
      const float kp = f.vel == 0.0 ? static_cast<float>(V1_PLAYOUT_KP_FLAT_PER_S) : static_cast<float>(tuning.playout_kp_per_s);
      float corr = kp * ea;
      const float brake = sqrtf(adec * ea);
      if (brake < corr) corr = brake;
      v_des = static_cast<float>(f.vel) + (e >= 0.0f ? corr : -corr);
    }
    if (v_des > vmax) v_des = vmax;
    if (v_des < -vmax) v_des = -vmax;
    const float a_cap = 0.8f * adec;
    // 途絶の上限（P1a）: 受けた最新の目標 ± MARGIN まで止まれる速さ
    if (!playout_.empty()) {
      const float y = static_cast<float>(static_cast<double>(playout_.newest_pos()) - p0);
      const float m = static_cast<float>(V1_PLAYOUT_LOSS_MARGIN_COUNT);
      const float d_up = y + m;
      const float d_dn = m - y;
      const float cap_up = d_up > 0.0f ? sqrtf(2.0f * a_cap * d_up) : 0.0f;
      const float cap_dn = d_dn > 0.0f ? sqrtf(2.0f * a_cap * d_dn) : 0.0f;
      if (v_des > cap_up) v_des = cap_up;
      if (v_des < -cap_dn) v_des = -cap_dn;
    }
    // Issue #46: ソフトリミットの手前
    if (tuning.soft_brake_acc_count_s2 > 0.0) {
      const float sb = static_cast<float>(tuning.soft_brake_acc_count_s2);
      const float ab = sb < a_cap ? sb : a_cap;
      const float r = static_cast<float>(tuning.soft_reserve_count);
      const float d_up = static_cast<float>(hi - p0) - r;
      const float d_dn = static_cast<float>(p0 - lo) - r;
      const float cap_up = d_up > 0.0f ? sqrtf(2.0f * ab * d_up) : 0.0f;
      const float cap_dn = d_dn > 0.0f ? sqrtf(2.0f * ab * d_dn) : 0.0f;
      if (v_des > cap_up) v_des = cap_up;
      if (v_des < -cap_dn) v_des = -cap_dn;
    }
    float dv = v_des - v;
    const bool decelerating = (v > 0.0f && dv < 0.0f) || (v < 0.0f && dv > 0.0f);
    const float dv_max = (decelerating ? adec : amax) * dtf;
    if (dv > dv_max) dv = dv_max;
    if (dv < -dv_max) dv = -dv_max;
    float v_new = v + dv;
    if (decelerating && v_new != 0.0f && ((v_new > 0.0f) != (v > 0.0f))) {
      // 0 を越えた分は逆向きの加速: max_acc × dt まで（max_acc 0 ならちょうど 0 で止まる。小さな残りの速度が消えなくなるのを防ぐ）
      const float acc_lim = amax * dtf;
      if (v_new > acc_lim) v_new = acc_lim;
      if (v_new < -acc_lim) v_new = -acc_lim;
    }
    v = v_new;
    double p = p0 + static_cast<double>(v) * dt;
    if (tracking && f.vel == 0.0 && fabs(p_t - p) <= V1_PLAYOUT_SNAP_COUNT && fabsf(v_before) <= adec * dtf) {
      p = p_t;  // 目標が止まった: 参照を乗せて速度をちょうど 0（1 tick で止まれる速さのときだけ。減速度の制限の中）
      v = 0.0f;
    }
    if (p < lo) {
      p = lo;
      if (v < 0.0f) v = 0.0f;
    }
    if (p > hi) {
      p = hi;
      if (v > 0.0f) v = 0.0f;
    }
    setpoint_ = p;
    setpoint_vel_ = static_cast<double>(v);
  }

  // 目標へ向かって加速度制限で加減速、速度制限を超えない、到達（通過）で目標に乗せて速度 0。
  // Issue #45: playout_delay_ms = 0（-DV1_PLAYOUT=0）のときだけ使う従来の経路。100 Hz の整数の目標ではゆっくりの区間で
  // 毎フレーム追いついて速度が 0 に落ちる（FF が抜ける）。
  static void profile_step(double target, double vmax, double amax, double dt, double* pos, double* vel) {
    const double err = target - *pos;
    if (vmax < 0) vmax = 0;
    if (amax < 0) amax = 0;
    double v_des = 0.0;
    if (err != 0.0) {
      const double brake_v = sqrt(2.0 * amax * fabs(err));
      v_des = (err > 0 ? 1.0 : -1.0) * (vmax < brake_v ? vmax : brake_v);
    }
    double dv = v_des - *vel;
    const double dv_max = amax * dt;
    if (dv > dv_max) dv = dv_max;
    if (dv < -dv_max) dv = -dv_max;
    const double v = *vel + dv;
    const double p = *pos + v * dt;
    const bool settled = (err == 0.0) ? (v == 0.0) : ((target - p) * err <= 0.0);
    if (settled) {
      *pos = target;
      *vel = 0.0;
    } else {
      *pos = p;
      *vel = v;
    }
  }

  // r10: 保護の監視への 1 tick の入力（座標は actual_count、速度は v_fast が使えればそれ、無ければ 100 ms 窓）
  V1Supervisor::Ctx sup_ctx(uint64_t now) const {
    V1Supervisor::Ctx c;
    c.now_us = now;
    c.raw = raw_;
    c.ref_offset = reference_offset_;
    c.err = static_cast<int64_t>(target_count()) - static_cast<int64_t>(actual_count());
    c.soft_min = config_.soft_min_count;
    c.soft_max = config_.soft_max_count;
    c.vel = v_fast_valid_ ? v_fast_ : static_cast<double>(velocity_);
    c.duty_max_pm = v1_permille(tuning.duty_max);
    c.hold_exit = tuning.hold_exit;
    // r10（実測 2）: 保持出力の扱い。軌道が止まっていて目標の近くなら、学んだ保持出力（I）の分は包絡線に積まない（V1Supervisor）
    c.i_pm = duty_state_.bias_pm;  // P1a: 保持の偏り（追従の I は含めない）
    // 駐車の入りかけ（ブレーキ作動の指令から V1_BRAKE_ENGAGE_MS）はまだ掴んでいないとみなし、露出を積む（Codex 最終レビュー P1）
    c.brake_released = brake_release_cmd_ || park_ == ParkPhase::Engaging;
    const int64_t ea = c.err < 0 ? -c.err : c.err;
    // 目標の近くで止まりかけ（参照速度 ≤ 駐車の入口 30 count/s・|誤差| ≤ 6）: 整定・保持の小さな出力（学んだ保持出力 + 4 % まで）は
    // 積まない（実機 12712ac の FAIL 1: 下の参照の止まる所への寄せの終わりで、参照速度が 0 になる前の小さな出力を一律の包絡線で積み、
    // 端の余裕 250 を使い切って停止した）
    const double sv_abs = setpoint_vel_ < 0.0 ? -setpoint_vel_ : setpoint_vel_;
    c.stationary_near = tuning.hold_learn && sv_abs <= static_cast<double>(tuning.park_ref_vel_count_s) && ea <= escalation_floor();
    c.park_ok = park_conditions(ea);
    // 駐車から戻りかけ（ブレーキ解除待ち、参照は駐車した位置で止めている）: 動かないのが正しい。出力（駐車に入ったときの保持出力）は
    // 学んだ保持出力として扱い（|誤差| が帯を超えていても）、期限の露出は積まない（実機 27eaf6b: 駐車中に 9 count ずれた軸が、
    // 戻りかけの −1 permille を一律の包絡線で積まれて 0.3 s で FEEDBACK_UNCERTAIN → 参照だけ進んで FOLLOWING_ERROR）
    if (park_ == ParkPhase::Releasing) {
      c.stationary_near = tuning.hold_learn;
      c.deadline_hold = true;
    }
    return c;
  }

  // FOLLOWING_ERROR → OVERSPEED → SOFT_LIMIT の順。r10（Issue #46、supervisor_on のときだけ。どちらの猶予も既定 OFF = 従来どおり即ラッチ。
  // -DV1_FE_GRACE=1 / -DV1_SOFT_GRACE=1、または tuning.sup.fe_grace_max_ms / soft_grace で ON。PR #48 の Codex 事前レビュー P1c）:
  //  - FOLLOWING_ERROR（ON のとき）: 上限そのものは変えない。超えていても、エンコーダが生きていて（直近 100 ms にカウント変化）|誤差| が減っている
  //    （100 ms ごとに 2 以上）間は最長 2 s ラッチしない（LIMIT_RECOVERY）。超えた直後の 200 ms は減り始めるのを待つ（超えた瞬間は
  //    必ず増えている）。減らなくなった・2 s 経った・|誤差| ≥ 上限 × 1.5・カウントが 100 ms 動かない・FEEDBACK_UNCERTAIN ならラッチ。
  //  - SOFT_LIMIT（ON のとき）: 少し越えただけで目標が内側なら内側へ戻す（外向きの出力は V1Supervisor::filter が 0 にする。LIMIT_RECOVERY）。
  //    機械端（ソフトリミット ∓ 50）から 20 以内・外向きに 200 ms 動き続けた・2 s 外に居続けた・目標が外ならラッチ。
  //  - OVERSPEED は変えない。
  void check_motion_faults(uint64_t now) {
    if (!has_config_) return;
    const bool sup = supervisor_on();
    V1Supervisor::Ctx c;
    if (sup) {
      sup_.t = tuning.sup;  // 調整値は V1Tuning が持つ（テスト・将来の SET_LIMITS で差し替え）
      c = sup_ctx(now);
      sup_.observe(c);  // 前の tick の出力の包絡線を積み、確かな動きなら予算を戻す（異常判定より前）
    }
    const int64_t fe = static_cast<int64_t>(actual_count()) - static_cast<int64_t>(target_count());
    const int64_t fe_abs = fe < 0 ? -fe : fe;
    const int64_t vel_abs = velocity_ < 0 ? -static_cast<int64_t>(velocity_) : velocity_;
    bool fe_over = fe_abs > static_cast<int64_t>(config_.following_error_limit_count);
    if (sup) {
      if (fe_over && sup_.fe_may_wait(c, fe_abs, config_.following_error_limit_count)) {
        fe_over = false;
      } else if (!fe_over) {
        sup_.fe_within();
      }
    }
    if (fe_over) {
      latch_fault(v1::fault::FOLLOWING_ERROR, now);
    } else if (vel_abs > static_cast<int64_t>(config_.overspeed_count_s)) {
      latch_fault(v1::fault::OVERSPEED, now);
    } else if (sup && tuning.sup.soft_grace) {
      if (sup_.decide_soft_limit(c, actual_count(), target_count()) == V1SoftDecision::Latch) {
        latch_fault(v1::fault::SOFT_LIMIT, now);
      }
    } else if (!inside_soft_window()) {
      latch_fault(v1::fault::SOFT_LIMIT, now);
    }
  }

  // ------------------------------------------------------------------------
  // 校正（CALIBRATING）
  // ------------------------------------------------------------------------

  AutoInputs cal_inputs(uint64_t now) const {
    AutoInputs in;
    in.now_ms = static_cast<uint32_t>(now / 1000);
    in.raw_count = raw_;
    in.relay_live = true;  // 電源 settle 済み（電源担当でない軸も同じ時間を待った）
    in.brake_released = brake_release_cmd_;
    in.manual_speed_nonzero = false;
    in.stop = false;
    in.estop = false;
    return in;
  }

  // 校正の各段階を進める。tick の update_duty の後に呼び、CALIBRATING の間だけ duty_permille_ を書く。
  void tick_calibration(uint64_t now) {
    if (state_ != v1::state::CALIBRATING) return;
    const uint64_t elapsed = now - cal_phase_started_us_;
    switch (cal_phase_) {
      case CalPhase::PowerSettle:
        duty_permille_ = 0;
        if (elapsed >= ms_to_us(tuning.cal_power_settle_ms)) {
          brake_release_cmd_ = true;
          cal_phase_ = CalPhase::BrakeRelease;
          cal_phase_started_us_ = now;
        }
        break;
      case CalPhase::BrakeRelease:
        duty_permille_ = 0;
        if (elapsed >= ms_to_us(tuning.brake_release_wait_ms)) {
          const char* err = "";
          const AutoInputs in = cal_inputs(now);
          if (cal_.request(AutoKind::Cal, in, &err)) {
            cal_phase_ = CalPhase::Running;
            cal_phase_started_us_ = now;
            duty_permille_ = v1_permille(cal_.duty_signed);  // AutoMachine は整数 %
          } else {
            cal_finish(v1::cal_result::FAILED, err, now);  // 起こらないはず（busy / not_live / brake は満たしている）
          }
        }
        break;
      case CalPhase::Running: {
        const AutoInputs in = cal_inputs(now);
        cal_.tick(in);
        duty_permille_ = v1_permille(cal_.duty_signed);  // AutoMachine は整数 %
        if (!cal_.active()) {
          uint8_t result = v1::cal_result::FAILED;
          if (cal_.phase == AutoPhase::Done) result = v1::cal_result::OK;
          if (cal_.phase == AutoPhase::Cancelled) result = v1::cal_result::CANCELLED;
          cal_finish(result, cal_.reason, now);
        }
        break;
      }
      case CalPhase::Shutdown:
        duty_permille_ = 0;  // AutoMachine の finish で既に 0。ブレーキ作動待ちの間も 0
        if (elapsed >= ms_to_us(tuning.brake_apply_wait_ms)) {
          push_cal_result();
          if (cal_keep_power_) {  // 他の軸の校正が終わるまで主電源を保つ（CALIBRATE(0) か期限で DISARMED）
            cal_phase_ = CalPhase::PowerHold;
            cal_phase_started_us_ = now;
            duty_permille_ = 0;
          } else {
            cal_phase_ = CalPhase::Idle;
            state_ = v1::state::DISARMED;  // ここで主電源 OFF 指令（powered_state）
            hold_here();
          }
        }
        break;
      case CalPhase::PowerHold:
        duty_permille_ = 0;
        if (elapsed >= ms_to_us(CAL_POWER_HOLD_TIMEOUT_MS)) end_power_hold();
        break;
      case CalPhase::Idle: break;
    }
  }

  // 校正の終わり（成功・失敗・中止）: 結果を組み、成功なら基準・符号・プロファイルを確定し、終了手順へ。
  void cal_finish(uint8_t result, const char* reason, uint64_t now) {
    v1::CalResult& r = pending_cal_result_;
    memset(&r, 0, sizeof(r));
    r.boot_id = id_.boot_id;
    r.result = result;
    r.step = cal_step_code();
    for (size_t i = 0; i < v1::CAL_REASON_LEN && reason[i] != '\0'; ++i) r.reason[i] = reason[i];  // NUL 埋め（8 文字ちょうどなら終端なし）
    if (result == v1::cal_result::OK) {
      const MotorProfile& p = cal_.profile;
      reference_offset_ = cal_.origin_raw;  // P 端 = 0
      referenced_ = true;
      sup_.reset_trust(raw_, now);  // r10: 端当てで基準を取り直した
      sign_ = p.sign < 0 ? -1 : 1;
      dirty_profile_ = p;
      dirty_profile_.motor = selected_axis_;
      profile_dirty_ = true;
    } else {
      referenced_ = false;
    }
    // 座標は actual_count と同じ（P 端 = 0。符号は掛けない）: n_end = raw_N − raw_P = stroke × sign、neutral も同様。
    // N 端まで測れていれば（中立への移動で失敗したときも）値を載せる。保存するのは成功だけ。
    if (cal_.profile.calibrated) {
      const int32_t s = cal_.profile.sign < 0 ? -1 : 1;
      r.n_end_count = cal_.profile.stroke * s;
      r.neutral_count = cal_.profile.neutral * s;
    }
    r.sign = sign_;
    brake_release_cmd_ = false;  // 終了手順: ブレーキ給電 OFF → brake_apply_wait → PWM 0 のまま主電源 OFF → DISARMED
    duty_permille_ = 0;
    cal_phase_ = CalPhase::Shutdown;
    cal_phase_started_us_ = now;
  }

  // POWER_HOLD の終わり（CALIBRATE(0)・DISARM・期限）。CAL_RESULT は校正終了時に送った 1 回だけで、ここでは送らない。
  void end_power_hold() {
    cal_phase_ = CalPhase::Idle;
    cal_keep_power_ = false;
    state_ = v1::state::DISARMED;  // 主電源 OFF 指令。このとき CAL_RESULT は送らない
    hold_here();
  }

  // CALIBRATE(0) / DISARM による中止。AutoMachine が動いていれば stop を渡して "stop" で止め、まだなら直接終了手順へ。
  // PC が止めた校正は KEEP_POWER 付きでも POWER_HOLD に入らず DISARMED まで行く（protocol.md §4 の mode 0。
  // PC は中止で主電源の解放予約を消すので、ここで保持に入ると期限まで CALIBRATING に留まってしまう）。
  void cal_cancel(uint64_t now) {
    cal_keep_power_ = false;
    if (cal_phase_ == CalPhase::PowerHold) { end_power_hold(); return; }
    if (cal_phase_ == CalPhase::Shutdown || cal_phase_ == CalPhase::Idle) return;  // 既に終わりかけ
    if (cal_phase_ == CalPhase::Running) {
      AutoInputs in = cal_inputs(now);
      in.stop = true;
      cal_.tick(in);
      cal_finish(v1::cal_result::CANCELLED, cal_.reason, now);
    } else {
      cal_finish(v1::cal_result::CANCELLED, "stop", now);
    }
  }

  // FAULT による打ち切り（latch_fault から）。終了手順は踏まない（FAULT が出力を落とす）。CAL_RESULT だけ即送る。
  // 終了手順（ブレーキ作動待ち）の間なら結果は確定済み: そのまま送る（成功なら基準・プロファイルも保つ。
  // FAULT は ENCODER_FAULT 以外では基準を消さない）。
  void cal_abort_for_fault(const char* reason, uint64_t now) {
    if (cal_phase_ == CalPhase::Shutdown) {
      push_cal_result();
      cal_phase_ = CalPhase::Idle;
      return;
    }
    if (cal_phase_ == CalPhase::PowerHold) {  // 結果は送信済み。FAULT が出力を落とす
      cal_phase_ = CalPhase::Idle;
      return;
    }
    if (cal_phase_ == CalPhase::Running) {
      AutoInputs in = cal_inputs(now);
      in.estop = true;
      cal_.tick(in);  // Cancelled "estop"・原点無効
    }
    if (cal_phase_ != CalPhase::Idle) {
      cal_finish(v1::cal_result::CANCELLED, reason, now);
      push_cal_result();
    }
    cal_phase_ = CalPhase::Idle;
  }

  void push_cal_result() {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::CAL_RESULT;
    m.cal_result = pending_cal_result_;
    last_cal_result_ = pending_cal_result_;
    has_cal_result_ = true;
    push(m);
  }

  // 位置制御の Duty と無パルス判定。dt_us は前の tick からの時間（積分の刻み。overrun_ms で頭打ち: 間隔が飛んだ直後に I を跳ねさせない）
  void update_duty(uint64_t now, uint64_t dt_us = 0) {
    if (!position_control_on()) {
      duty_permille_ = 0;  // CALIBRATING では直後の tick_calibration が書き直す
      sup_.end_session(raw_, now);  // r10: その回だけの状態（飽和・過負荷・制動・猶予）を捨てる
      if (state_ == v1::state::DISARMED) sup_.clear_episode();  // DISARM の完了: FEEDBACK_UNCERTAIN の明示的な解除（再 ARM が要る）
      stall_running_ = false;
      duty_state_ = V1DutyState{};
      reset_v_fast();  // r9: 位置制御が始まってから 20〜40 ms は推定なし（減衰項 0）
      settle_dwell_us_ = 0;
      reset_track_assist();
      reset_breakaway();
      reset_stall_boost();
      reset_escalation();
      residual_accepted_ = false;
      hold_pulses_spent_ = false;
      park_ = ParkPhase::None;  // STOP 以外で位置制御が止まった（DISARMED / FAULT など）: 駐車も終わり（ブレーキはどのみち作動）
      still_running_ = false;
      terms_ = V1DutyTerms{};
      return;
    }
    const int64_t err = static_cast<int64_t>(target_count()) - static_cast<int64_t>(actual_count());
    const int64_t err_abs = err < 0 ? -err : err;
    // ブレーキ駐車（2026-11-02 の操作者の決定）。駐車中（ブレーキ作動・PWM 0）だけは監視を通さない。入りかけ・戻りかけは保持出力を
    // 監視（予算・期限・停止・基準を消す）に通し、停止なら駐車をやめて即ブレーキの停止にする（Codex 最終レビュー P1）
    if (parking_enabled()) {
      int park_out = 0;
      const ParkAct pa = park_step(err_abs, now, &park_out);
      if (pa == ParkAct::Parked) {
        sup_.note_parked();
        apply_park_output(0, V1DutyMode::HOLD);
        return;
      }
      if (pa == ParkAct::Transition) {
        int out = park_out;
        if (has_config_) {
          sup_.t = tuning.sup;
          out = sup_.filter(sup_ctx(now), out);
          if (sup_.take_reference_lost()) referenced_ = false;
          if (sup_.inhibited()) {
            park_ = ParkPhase::None;  // 停止が駐車より先: ブレーキ作動・PWM 0（戻りかけでも即）
            still_running_ = false;
            out = 0;
            brake_release_cmd_ = false;
            duty_state_ = V1DutyState{};
            reset_breakaway();
            reset_track_assist();
            reset_stall_boost();
          }
        }
        apply_park_output(out, V1DutyMode::HOLD);
        return;
      }
    }
    // r10: 小さな残り誤差の受け入れの解除（|誤差| が出口を超えた・軌道が動いた・ブレーキ作動・監視 OFF）
    if (residual_accepted_ && (!supervisor_on() || err_abs > tuning.residual_exit || setpoint_vel_ != 0.0 || !brake_release_cmd_)) {
      residual_accepted_ = false;
    }
    if (hold_pulses_spent_ && (err_abs > tuning.residual_exit || setpoint_vel_ != 0.0 || duty_state_.in_band || !brake_release_cmd_)) {
      hold_pulses_spent_ = false;
    }
    // 停止ブースト（Issue #15 H4）: 対象外になった・誤差の符号が反転した tick は、Duty を計算する前に上乗せを 0 に戻す
    //（戻す前に計算すると、最大上乗せのまま反対向きや軌道の立ち上がりに 1 tick だけ強いパルスが出る）
    stall_boost_gate(err, now);
    const uint64_t dt_cap_us = ms_to_us(tuning.overrun_ms);
    const double dt_s = static_cast<double>(dt_us < dt_cap_us ? dt_us : dt_cap_us) / 1e6;
    const bool settle_ok = update_settle_dwell(err, dt_us);
    const bool assist = update_track_assist(err, now) || update_breakaway(err, now);  // r6 では後者（動き出しパルス）だけ
    update_escalation(err, now);  // r10: 静止目標での引き上げ（esc_level_）
    // r10（PR #50 の Codex 事前レビュー P1b）: 積分のアンチワインドアップは実際に出した出力で見る。引き上げ・端の手前の制動・ソフトリミットの
    // 外の外向き 0・過負荷・予算・停止が出力を決めた tick は、bias も追従の I も積まない（ここで取っておいた値に戻す）
    const double i_before = duty_state_.i_pm;
    const double bias_before = duty_state_.bias_pm;
    int cmd = v1_duty_command(err, setpoint_vel_, duty_state_, tuning, stall_boost_, &terms_, dt_s, settle_ok,
                              assist, v_fast_valid_, v_fast_);  // カウント増の向きが正 [permille]。r9 は v_fast を減衰項に
    last_combined_pm_ = cmd;
    const int cmd_ctrl = cmd;  // 位置ループが出した値（上書きの判定用）
    // r10（supervisor_on）: 静止目標（軌道速度 0・保持でない）での出力の形
    //  - |誤差| ≤ residual_band か残り誤差の受け入れ中: 動き出しパルスの間だけ出す。それ以外は 0（I も 0）。受け入れ中はパルスも無い
    //  - |誤差| > residual_band で進まない: 誤差の向きに max(合成, 引き上げ) を出す（duty_max まで）
    // （保持出力の学習 hold_learn が ON なら、小さな誤差での出力 0 は使わない: 乗員の荷重で沈むので保持出力 I を出し続ける）
    if (supervisor_on() && setpoint_vel_ == 0.0 && !duty_state_.in_band) {
      if (!tuning.hold_learn &&
          (residual_accepted_ || (tuning.residual_band > 0 && err_abs <= tuning.residual_band && !brk_active_))) {
        cmd = 0;
        duty_state_.i_pm = 0.0;
        terms_.cmd_permille = 0;
        if (residual_accepted_) terms_.mode = V1DutyMode::HOLD;
      } else if (esc_level_ > 0) {
        const int dir = err > 0 ? 1 : -1;
        long mag = (cmd != 0 && (cmd > 0) == (dir > 0)) ? (cmd < 0 ? -cmd : cmd) : 0;
        if (mag < esc_level_) mag = esc_level_;
        const long cap = v1_permille(tuning.duty_max);
        if (mag > cap) mag = cap;
        cmd = static_cast<int>(dir * mag);
        terms_.cmd_permille = static_cast<int16_t>(cmd);
        terms_.mode = V1DutyMode::BRK;
      }
    }
    // r10（Issue #44・#46）: 保護の監視が出力を決める（見えない走行の予算・過負荷・端の手前の制動・ソフトリミットの外の外向き 0）
    if (supervisor_on() && has_config_) {
      sup_.t = tuning.sup;
      cmd = sup_.filter(sup_ctx(now), cmd);
      if (sup_.take_reference_lost()) referenced_ = false;  // P0: 駐車できないまま期限の露出が余裕の全部に届いた → 基準を消す
      if (sup_.inhibited()) {
        // 持続する停止（PR #48 の Codex 事前レビュー P0）: PWM 0 ＋ ブレーキ作動（G13 を作動側へ）。主電源は保つ（powered_state のまま）。
        // 制御器の状態（積分・動き出しパルス・補助）も捨てる（再開は DISARMED → ARM_PREPARE の通常の手順で、目標は現在位置から）
        cmd = 0;
        brake_release_cmd_ = false;
        duty_state_ = V1DutyState{};
        reset_breakaway();
        reset_track_assist();
        reset_stall_boost();
        terms_.cmd_permille = 0;
      }
    }
    if (tuning.hold_learn && cmd != cmd_ctrl && !sup_.inhibited()) {  // 停止のときは上で捨てている（学習 OFF は従来どおり）
      duty_state_.i_pm = i_before;
      duty_state_.bias_pm = bias_before;
    }
    const int mag = cmd < 0 ? -cmd : cmd;
    // カウントを増やしたいなら −sign × duty
    duty_permille_ = -static_cast<int>(sign_) * cmd;
    update_stall_boost(now);
    if (state_ == v1::state::FAULT) return;  // 上乗せの上限・時間切れで ENCODER_FAULT をラッチした（出力は落ちている）

    // ENCODER_FAULT: |duty| ≥ しきい値（max(stall_duty, duty_max × 75 %) の permille。Issue #15 M2）が stall_ms 続いてカウントが
    // 変わらない（manual_guard.h の無パルス判定と同じ窓）。
    // 窓を回すのは「ブレーキ解除指令中」かつ「軌道が動いている（setpoint_vel ≠ 0）」間だけ（設計 §3）:
    //  - ブレーキ作動指令中（DISARMING）はカウントが変わらないのが正常。残った追従誤差で窓を回すと DISARM が FAULT になる。
    //  - 目標が止まっているときの静止（荷重とのつり合いで最小 Duty では動かない）は、エンコーダの死と区別できない。
    //    この場合の誤差の上限は FOLLOWING_ERROR が守る（エンコーダが生きている前提。エンコーダが死んだまま目標が
    //    止まっている間は検出できず、目標が動けば STALL_MS で出る）。
    //  - しきい値を duty_max の 75 % 以上にするのは、kp を上げたとき数百 count の誤差だけで 10 % に届き、荷重で動けない軸が
    //    FOLLOWING_ERROR の前に ENCODER_FAULT になって基準を失うのを避けるため。上限近くまで出しても止まっているときだけ異常。
    // r10（supervisor_on）: この判定は出さない（荷重で動けない軸を「エンコーダ断」と取り違える。Issue #44）。見えない走行は
    // V1Supervisor の予算が止める
    if (!supervisor_on() && mag >= encoder_fault_duty_threshold_permille() && brake_release_cmd_ && setpoint_vel_ != 0.0) {
      if (!stall_running_) {
        stall_running_ = true;
        stall_start_us_ = now;
        stall_last_change_us_ = now;
        stall_last_raw_ = raw_;
      } else {
        if (raw_ != stall_last_raw_) {
          stall_last_raw_ = raw_;
          stall_last_change_us_ = now;
        }
        const uint64_t window = ms_to_us(tuning.stall_ms);
        if (now - stall_start_us_ >= window && now - stall_last_change_us_ >= window) {
          latch_fault(v1::fault::ENCODER_FAULT, now);
          duty_permille_ = 0;
          stall_running_ = false;
        }
      }
    } else {
      stall_running_ = false;
    }
  }

  // 整定判定（Issue #25 手順 10「S」）。「軌道速度 0・|誤差| ≤ deadband・|実測速度| ≤ settled_meas_vel」が続いた時間を数え、
  // settled_ms 以上なら不感帯に入ってよい。条件が 1 tick でも崩れれば 0 から。settled_ms = 0 なら常に可（従来の振る舞い）。
  // 実測速度は 100 ms 窓（velocity_）なので、動いた直後は最長 100 ms 遅れて 0 になる → 入るまで最長 300 ms。
  // 「実測静止」だけで判定しない（追従中に止まってしまった軸の補正を止めない）: 軌道速度 0 と |誤差| ≤ deadband も要る。
  bool update_settle_dwell(int64_t err, uint64_t dt_us) {
    if (tuning.settled_ms == 0) { settle_dwell_us_ = 0; return true; }
    const int64_t err_abs = err < 0 ? -err : err;
    const int32_t v_abs = velocity_ < 0 ? -velocity_ : velocity_;
    const bool cond = setpoint_vel_ == 0.0 && err_abs <= tuning.band_enter() && v_abs <= tuning.settled_meas_vel;  // r6 は hold_enter（2）
    if (!cond) { settle_dwell_us_ = 0; return false; }
    settle_dwell_us_ += dt_us;
    return settle_dwell_us_ >= ms_to_us(tuning.settled_ms);
  }

  void reset_track_assist() {
    assist_active_ = false;
    assist_window_running_ = false;
  }

  // 追従中の動き出し補助（Issue #25 手順 9「T」）。Duty を計算する前に呼ぶ。true ならこの tick は床を掛ける。
  // 対象: |軌道速度| > assist_vel（100）・|実測速度| < assist_meas_vel（20。100 ms 窓なので実質 0 か ±10）・ブレーキ解除指令中。
  // 対象のまま assist_ms（150 ms）の間カウントが assist_release_move（3）未満しか動かず、|誤差| が窓の開始時より増えていれば ON。
  // 窓の間に動く・|誤差| が減る・対象外になれば窓を取り直す。ON の後はカウントが 3 以上動く・軌道が止まる（≤ assist_vel）・
  // ブレーキ作動で解除。停止ブースト（H4）は軌道速度 0 だけが対象なので排他。実測速度は 100 ms 窓なので、動き出しの検出は
  // 最長 100 ms 遅れる（対象になるのが遅れる方向。ON になった後の解除はカウントで見るので即）。
  // 保護との関係: 掛けるのは床（duty_min ＋ ブースト。ブーストは静止だけなので実質 duty_min）までで、ENCODER_FAULT の
  // しきい値 max(10 %, duty_max × 75 %) には届かない（FF + P がそこまで出ていれば従来どおり無パルス判定が回る）。
  // FOLLOWING_ERROR の判定（check_motion_faults）は触らない。
  bool update_track_assist(int64_t err, uint64_t now) {
    if (tuning.assist_ms == 0 || tuning.ff_directional) { reset_track_assist(); return false; }  // r6 は update_breakaway
    const double sv_abs = setpoint_vel_ < 0.0 ? -setpoint_vel_ : setpoint_vel_;
    const bool traj_moving = sv_abs > static_cast<double>(tuning.assist_vel) && brake_release_cmd_;
    const int64_t moved = raw_ - assist_anchor_raw_;
    const int64_t moved_abs = moved < 0 ? -moved : moved;
    if (assist_active_) {
      if (!traj_moving || moved_abs >= tuning.assist_release_move) { reset_track_assist(); return false; }
      return true;
    }
    const int32_t v_abs = velocity_ < 0 ? -velocity_ : velocity_;
    const bool eligible = traj_moving && v_abs < tuning.assist_meas_vel;
    if (!eligible) { reset_track_assist(); return false; }
    const int64_t err_abs = err < 0 ? -err : err;
    if (!assist_window_running_ || moved_abs >= tuning.assist_release_move || err_abs < assist_err_anchor_abs_) {
      assist_window_running_ = true;  // 窓の開始（取り直し）
      assist_window_start_us_ = now;
      assist_anchor_raw_ = raw_;
      assist_err_anchor_abs_ = err_abs;
      return false;
    }
    if (now - assist_window_start_us_ >= ms_to_us(tuning.assist_ms) && err_abs > assist_err_anchor_abs_) {
      assist_active_ = true;
      assist_anchor_raw_ = raw_;
      return true;
    }
    return false;
  }

  void reset_breakaway() {
    brk_active_ = false;
    brk_running_ = false;
    brk_exhausted_ = false;
    brk_dir_ = 0;
    brk_pulses_ = 0;
    brk_mode_ = 0;
    brk_err_anchor_abs_ = 0;
    brk_cooldown_until_us_ = 0;
  }

  // 動き出しパルス（Issue #25 r6。床の唯一の例外）。Duty を計算する前に呼ぶ。true ならこの tick は v1_duty_command が
  // 方向別の動き出し Duty（breakaway_up/down）を |合成| との大きい方で出す。
  // 要求: ブレーキ解除指令中・保持の外で |誤差| > hold_exit。向きは誤差の向き（軌道が動いていれば軌道の向きで、誤差が軌道と逆
  //（先行）なら対象外: 先行している軸を蹴らない）。進み = 向きへの符号付きカウント（自重で下がりながら上を要求されていれば進みではない）。
  // 窓 breakaway_window_ms で進みが breakaway_min_move 未満 → パルス ON。パルスは進みが min_move 以上か breakaway_pulse_ms で終わる。
  // 進まずに終わったパルスを数え、breakaway_max_pulses に達したら要求が続く間は打たない（静止 2 s の監視 ENCODER_FAULT に任せる）。
  // 進めば数を 0 に戻す。停止ブーストの監視の時計（boost_active_since_us_）には触らない。
  bool update_breakaway(int64_t err, uint64_t now) {
    if (!tuning.ff_directional || tuning.breakaway_window_ms == 0) { reset_breakaway(); return false; }
    const int64_t err_abs = err < 0 ? -err : err;
    // r10（supervisor_on）: 静止目標では、小さな残り誤差（≤ residual_band）だけパルスで詰める。それより大きい誤差は引き上げ
    //（update_escalation）が受け持ち、受け入れ中は打たない
    if (supervisor_on() && setpoint_vel_ == 0.0 &&
        (residual_accepted_ || (tuning.escalate_step_permille > 0 && err_abs > escalation_floor()))) {
      reset_breakaway();
      return false;
    }
    // r10（hold_learn）: 保持の帯の近くのパルスは、I がまだ動き出しの Duty に育っていない間だけ。荷重で目標から離れている（沈んでいる）
    // 間は打たない（パルスで持ち上げても出力が落ちればまた沈む往復になる。I が詰める）。進まないパルス 3 回で打ち切る
    if (tuning.hold_learn && setpoint_vel_ == 0.0 && err != 0) {
      const int edir = err > 0 ? 1 : -1;
      long need = edir > 0 ? tuning.breakaway_up_permille : tuning.breakaway_down_permille;
      if (need < 0) need = -need;
      const double v = v_fast_valid_ ? v_fast_ : static_cast<double>(velocity_);
      const bool sagging = static_cast<double>(edir) * v < -static_cast<double>(tuning.breakaway_moving_max_vel_count_s);
      if (hold_pulses_spent_ || static_cast<double>(edir) * duty_state_.i_pm >= static_cast<double>(need) || sagging) {
        reset_breakaway();
        return false;
      }
    }
    int dir = err > 0 ? 1 : (err < 0 ? -1 : 0);
    const bool traj_moving = setpoint_vel_ != 0.0;
    const double traj_abs = setpoint_vel_ < 0.0 ? -setpoint_vel_ : setpoint_vel_;
    // r8（Issue #25、Codex 案 d）: 軌道速度が帯 [band_lo, band_hi] の間は「引っかかった時だけ短く押す」モード。
    // 実機の解析で 50〜150 count/s の速度ムラが r3 の 1.3〜1.9 倍（静止摩擦 ≈ 9 % と動摩擦 ≈ 5 % の差で引っかかっては P が溜まって
    // 飛び出す）。帯の中では: 100 ms カウントが向きへ 1 も進まず |誤差| が増えていれば、上 90 / 下 60 を最初の 1 count か 50 ms まで。
    // 押した後 200 ms は押さない。帯の外の走行中は r7 のまま（実測 |v| ≤ 10 が 200 ms・誤差増加）。
    const bool band = traj_moving && tuning.breakaway_band_hi_count_s > 0 &&
                      traj_abs >= static_cast<double>(tuning.breakaway_band_lo_count_s) &&
                      traj_abs <= static_cast<double>(tuning.breakaway_band_hi_count_s);
    const int mode = band ? 2 : (traj_moving ? 1 : 0);
    if (traj_moving) {
      const int traj_dir = setpoint_vel_ > 0.0 ? 1 : -1;
      if (traj_dir != dir) dir = 0;  // 軌道より先行: 対象外
      // 走行中（PR #33 Codex P1）: 軌道が速い（≥ moving_min_traj）のに軸が止まっている（実測 |v| ≤ moving_max_vel）ときだけ対象。
      // 遅い軌道に普通に遅れている（30 count/s ≈ 3 count/100 ms）だけでは打たない。実測速度は 100 ms 窓（最長 100 ms 古い）。
      const int32_t v_abs = velocity_ < 0 ? -velocity_ : velocity_;
      if (!band && (traj_abs < static_cast<double>(tuning.breakaway_moving_min_traj_count_s) ||
                    v_abs > tuning.breakaway_moving_max_vel_count_s)) {
        dir = 0;
      }
    }
    // r7: 静止目標で保持に入っていなければ |誤差| > hold_enter から対象（r6 は > hold_exit で、2 < |誤差| ≤ 4 に P ＋ I が
    // 流れたまま止まる帯があった）。帯の走行中も > hold_enter（r8）。帯の外の走行中は > hold_exit のまま
    const int64_t thresh = mode == 1 ? tuning.hold_exit : tuning.hold_enter;
    const bool demand = dir != 0 && err_abs > thresh && brake_release_cmd_ && !duty_state_.in_band;
    if (!demand) { reset_breakaway(); return false; }
    if (!brk_running_ || dir != brk_dir_ || mode != brk_mode_) {
      reset_breakaway();
      brk_running_ = true;
      brk_dir_ = dir;
      brk_mode_ = mode;
      brk_phase_start_us_ = now;
      brk_anchor_raw_ = raw_;
      brk_err_anchor_abs_ = err_abs;
      return false;
    }
    const int32_t min_move = band ? tuning.breakaway_band_min_move : tuning.breakaway_min_move;
    const uint32_t pulse_ms = band ? tuning.breakaway_band_pulse_ms : tuning.breakaway_pulse_ms;
    const int64_t progress = static_cast<int64_t>(dir) * (raw_ - brk_anchor_raw_);
    if (progress >= min_move) {
      if (band && brk_active_) brk_cooldown_until_us_ = now + ms_to_us(tuning.breakaway_band_cooldown_ms);
      brk_active_ = false;  // 進んだ: 窓を取り直し、数も戻す
      brk_exhausted_ = false;
      brk_pulses_ = 0;
      brk_phase_start_us_ = now;
      brk_anchor_raw_ = raw_;
      brk_err_anchor_abs_ = err_abs;
      return false;
    }
    if (brk_exhausted_) return false;
    if (band && !brk_active_ && now < brk_cooldown_until_us_) {
      brk_phase_start_us_ = now;  // 押した直後の休み: 窓は休みが明けてから数え始める
      brk_anchor_raw_ = raw_;
      brk_err_anchor_abs_ = err_abs;
      return false;
    }
    const uint64_t elapsed = now - brk_phase_start_us_;
    if (brk_active_) {
      if (elapsed >= ms_to_us(pulse_ms)) {
        brk_active_ = false;  // 進まずに終わった
        ++brk_pulses_;
        if (brk_pulses_ >= tuning.breakaway_max_pulses) brk_exhausted_ = true;
        // r10: 静止目標の小さな残り誤差で進まないパルスが residual_max_pulses 回 → 打ち切る。hold_learn なら以後は I が詰める
        //（出力 0 で受け入れない）。hold_learn が OFF なら従来の B（受け入れて出力 0）
        if (mode == 0 && tuning.residual_max_pulses > 0 && brk_pulses_ >= tuning.residual_max_pulses) {
          if (tuning.hold_learn) {
            hold_pulses_spent_ = true;
            reset_breakaway();
            return false;
          }
          if (supervisor_on() && tuning.residual_band > 0 && err_abs <= tuning.residual_band) {
            residual_accepted_ = true;
            reset_breakaway();
            return false;
          }
        }
        if (band) brk_cooldown_until_us_ = now + ms_to_us(tuning.breakaway_band_cooldown_ms);
        brk_phase_start_us_ = now;
        brk_anchor_raw_ = raw_;
        brk_err_anchor_abs_ = err_abs;
        return false;
      }
      return true;
    }
    const uint32_t window_ms = mode == 2 ? tuning.breakaway_band_window_ms
                                         : (mode == 1 ? tuning.breakaway_moving_window_ms : tuning.breakaway_window_ms);
    if (elapsed >= ms_to_us(window_ms)) {
      if (traj_moving && err_abs <= brk_err_anchor_abs_) {
        brk_phase_start_us_ = now;  // 走行中で |誤差| が増えていない: 打たずに窓を取り直す
        brk_anchor_raw_ = raw_;
        brk_err_anchor_abs_ = err_abs;
        return false;
      }
      brk_active_ = true;
      brk_phase_start_us_ = now;
      brk_anchor_raw_ = raw_;
      brk_err_anchor_abs_ = err_abs;
      return true;
    }
    return false;
  }

  bool parking_enabled() const { return supervisor_on() && tuning.park; }
  // 駐車してよい瞬間の条件（時間を除く）: ACTIVE / HOLDING・ブレーキ解除中・参照が止まっている・目標の近く・実測で止まっている・戻した直後でない・停止でない
  bool park_conditions(int64_t err_abs) const {
    if (!parking_enabled() || park_ != ParkPhase::None) return false;
    if (state_ != v1::state::ACTIVE && state_ != v1::state::HOLDING) return false;
    const double sv = setpoint_vel_ < 0.0 ? -setpoint_vel_ : setpoint_vel_;
    const int32_t v = velocity_ < 0 ? -velocity_ : velocity_;
    return brake_release_cmd_ && sv <= static_cast<double>(tuning.park_ref_vel_count_s) && err_abs <= tuning.hold_exit &&
           v <= tuning.settled_meas_vel && now_us_ >= park_cooldown_until_us_ && !sup_.inhibited();
  }
  int holding_output_permille() const {
    long h = lround(duty_state_.bias_pm + duty_state_.i_pm);
    const long cap = v1_permille(tuning.duty_max);
    if (h > cap) h = cap;
    if (h < -cap) h = -cap;
    return static_cast<int>(h);
  }
  void apply_park_output(int out_count_positive, V1DutyMode mode) {
    duty_permille_ = -static_cast<int>(sign_) * out_count_positive;
    terms_ = V1DutyTerms{};
    terms_.mode = mode;
    terms_.in_band = true;
    terms_.cmd_permille = static_cast<int16_t>(out_count_positive);
    terms_.i_permille = v1_clamp_i16(duty_state_.bias_pm + duty_state_.i_pm);
  }
  enum class ParkAct : uint8_t { None, Transition, Parked };
  void enter_park(uint64_t now) {
    park_ = ParkPhase::Engaging;
    park_phase_us_ = now;
    park_target_ = 0.5 * (still_min_ + still_max_);
    if (!still_running_) park_target_ = control_target_;
    // いまの保持出力（直前に実際に出していた値 = bias + I + P）をブレーキが掴むまで保ち、戻るときもこの値を出す（duty_max で頭打ち）
    park_hold_out_ = pwm_permille();
    const int cap = v1_permille(tuning.duty_max);
    if (park_hold_out_ > cap) park_hold_out_ = cap;
    if (park_hold_out_ < -cap) park_hold_out_ = -cap;
    brake_release_cmd_ = false;
    still_running_ = false;
    ++park_count_;
    reset_breakaway();
    reset_track_assist();
    reset_stall_boost();
    reset_escalation();
    hold_pulses_spent_ = false;
    duty_state_.in_band = false;
    settle_dwell_us_ = 0;
  }
  // 駐車の状態機械。Parked: 駐車中（PWM 0、監視を通さない）。Transition: 入りかけ・戻りかけ（*out の保持出力を監視に通して出す）。
  // None: 通常の制御。DISARMING など ACTIVE / HOLDING / STOPPING 以外では駐車をやめて何もしない（Codex 最終レビュー P1: DISARMING を
  // 上書きしない。DISARMING は従来どおりブレーキ作動のまま位置制御の出力を BRAKE_APPLY_WAIT_MS 出す）。STOPPING で戻りかけなら
  // ブレーキ解除をやめて入りかけに戻す（保持出力を出したまま掴むまで待つ）
  ParkAct park_step(int64_t err_abs, uint64_t now, int* out) {
    if (state_ != v1::state::ACTIVE && state_ != v1::state::HOLDING && state_ != v1::state::STOPPING) {
      park_ = ParkPhase::None;
      still_running_ = false;
      return ParkAct::None;
    }
    if (state_ != v1::state::ACTIVE && park_ == ParkPhase::Releasing) {
      // STOP（STOPPING、止まっていれば同じ tick で HOLDING）: 戻りかけをやめてブレーキを掴み直す（保持出力を出したまま掴むまで待つ）
      park_ = ParkPhase::Engaging;
      park_phase_us_ = now;
      brake_release_cmd_ = false;
    }
    const double ct_dev = fabs(control_target_ - park_target_);
    switch (park_) {
      case ParkPhase::None: {
        const bool ok = park_conditions(err_abs);
        if (ok) {
          if (!still_running_) {
            still_running_ = true;
            still_start_us_ = now;
            still_min_ = still_max_ = control_target_;
          } else {
            if (control_target_ < still_min_) still_min_ = control_target_;
            if (control_target_ > still_max_) still_max_ = control_target_;
            if (still_max_ - still_min_ > static_cast<double>(tuning.park_target_span_count)) {
              still_start_us_ = now;  // 目標が動いた: 窓を取り直す
              still_min_ = still_max_ = control_target_;
            }
          }
        } else {
          still_running_ = false;
        }
        const bool timed = ok && still_running_ && now - still_start_us_ >= ms_to_us(tuning.park_still_ms);
        const bool deadline = sup_.take_park_request() && ok;  // 期限の露出が半分に届いた
        if (!timed && !deadline) return ParkAct::None;
        enter_park(now);
        *out = park_hold_out_;
        return ParkAct::Transition;
      }
      case ParkPhase::Engaging:
        brake_release_cmd_ = false;
        if (now - park_phase_us_ >= ms_to_us(tuning.brake_engage_ms)) {
          park_ = ParkPhase::Parked;
          park_phase_us_ = now;
          return ParkAct::Parked;
        }
        *out = park_hold_out_;
        return ParkAct::Transition;
      case ParkPhase::Parked:
        brake_release_cmd_ = false;
        if (ct_dev > static_cast<double>(tuning.park_unpark_count) && state_ == v1::state::ACTIVE) {
          // 戻る: まず駐車に入ったときの保持出力を出す（ブレーキはこの tick はまだ作動）。次の tick でブレーキを解除する
          park_ = ParkPhase::Releasing;
          park_phase_us_ = now;
          park_release_armed_ = false;
          setpoint_vel_ = 0.0;
          *out = park_hold_out_;
          return ParkAct::Transition;
        }
        return ParkAct::Parked;
      case ParkPhase::Releasing:
        if (!park_release_armed_) {
          park_release_armed_ = true;  // 保持出力は前の tick から出ている: ここでブレーキを解除し、解除待ちを数え始める
          park_phase_us_ = now;
        }
        brake_release_cmd_ = true;
        if (now - park_phase_us_ >= ms_to_us(tuning.brake_release_wait_ms)) {
          park_ = ParkPhase::None;
          park_cooldown_until_us_ = now + ms_to_us(tuning.park_still_ms);
          still_running_ = false;
          sup_.begin_fresh_window();  // 期限は通常の制御に入ってから新しい窓で（残りの予算までに抑える）
          return ParkAct::None;  // この tick から通常の制御（参照は次の tick から追いつく）
        }
        *out = park_hold_out_;
        return ParkAct::Transition;
    }
    return ParkAct::None;
  }

  void reset_escalation() {
    esc_running_ = false;
    esc_dir_ = 0;
    esc_level_ = 0;
  }
  // 引き上げの対象になる |誤差| の下限（これを超えたら）。小さな残り誤差の帯が無効なら保持の出口
  int64_t escalation_floor() const { return tuning.residual_band > 0 ? tuning.residual_band : tuning.hold_exit; }

  // r10（Issue #44 の 14:22 の実測）: 静止目標での力の段階的な引き上げ。Duty を計算する前に呼び、esc_level_ を決める。
  // 対象: 監視 ON・軌道速度 0・保持でない・小さな残り誤差の受け入れ中でない・|誤差| > escalation_floor・ブレーキ解除指令中・停止中でない。
  // 窓（escalate_window_ms）の間に誤差の向きへ breakaway_min_move 進まなければ、0 なら max(動き出し Duty, |合成|) + step、以後は窓ごとに
  // + step（duty_max まで）。進んだ窓では保つ。向きの反転・対象外で 0。出力は V1Supervisor::filter を通る（予算・過負荷・停止）。
  void update_escalation(int64_t err, uint64_t now) {
    if (!supervisor_on() || tuning.escalate_step_permille <= 0) { reset_escalation(); return; }
    const int64_t err_abs = err < 0 ? -err : err;
    const int dir = err > 0 ? 1 : (err < 0 ? -1 : 0);
    const bool eligible = setpoint_vel_ == 0.0 && !duty_state_.in_band && !residual_accepted_ && err_abs > escalation_floor() &&
                          brake_release_cmd_ && !sup_.inhibited() && dir != 0;
    if (!eligible) { reset_escalation(); return; }
    if (!esc_running_ || dir != esc_dir_) {
      reset_escalation();
      esc_running_ = true;
      esc_dir_ = dir;
      esc_win_start_us_ = now;
      esc_anchor_raw_ = raw_;
      return;
    }
    if (now - esc_win_start_us_ < ms_to_us(tuning.escalate_window_ms)) return;
    const bool progress = static_cast<int64_t>(dir) * (raw_ - esc_anchor_raw_) >= tuning.breakaway_min_move;
    if (!progress) {
      const long cap = v1_permille(tuning.duty_max);
      long level = esc_level_;
      if (level == 0) {
        long brk = dir > 0 ? tuning.breakaway_up_permille : tuning.breakaway_down_permille;
        if (brk < 0) brk = -brk;
        const long comb = (last_combined_pm_ != 0 && (last_combined_pm_ > 0) == (dir > 0))
                              ? (last_combined_pm_ < 0 ? -last_combined_pm_ : last_combined_pm_)
                              : 0;
        level = brk > comb ? brk : comb;
      }
      level += tuning.escalate_step_permille;
      if (level > cap) level = cap;
      esc_level_ = static_cast<int>(level);
    }
    esc_win_start_us_ = now;
    esc_anchor_raw_ = raw_;
  }

  void reset_stall_boost() {
    stall_boost_ = 0;
    boost_eligible_ = false;
    boost_window_running_ = false;
    boost_err_sign_ = 0;
  }

  // 停止ブースト（Issue #15 H4）の対象判定。Duty を計算する前に呼ぶ。対象（軌道速度 0・|誤差| > deadband_out）なら床の適用状態
  // （v1_duty_command の st.floor_on。Issue #25 手順 8 c）は必ず ON になる（静止では FF + P = P で向きが一致する。積分 I は床の
  // 向きと ON/OFF に入れない。Issue #25 手順 9「F」）ので上乗せが効く。
  // 逆に 15 < |誤差| ≤ 20 では床が ON のまま残ることがあるが、対象ではないので上乗せは 0 で床 = duty_min だけ。
  // 対象は「軌道が止まっている（setpoint_vel = 0）」「|誤差| > deadband_out（不感帯の外。in_band の履歴に依らず明示的に見る。
  // 動いた後は in_band = false なので cmd ≠ 0 だけでは 10 < |誤差| ≤ 20 の静止誤差も対象になってしまう）」「ブレーキ解除指令中」。
  // 対象外になった tick と誤差の符号が反転した tick は、ここで上乗せを 0 に戻す（この tick の Duty から効く）。
  void stall_boost_gate(int64_t err, uint64_t now) {
    if (tuning.stall_boost_ms == 0) { reset_stall_boost(); return; }
    const int64_t err_abs = err < 0 ? -err : err;
    // r6 は hold_exit（4）、r7 は hold_enter（2）: 保持に入れない静止誤差は動き出しパルスとこの監視の対象（出力が流れ続ける帯を残さない）
    const int64_t gate = tuning.ff_directional ? tuning.hold_enter : tuning.band_exit();
    const bool eligible = setpoint_vel_ == 0.0 && err_abs > gate && brake_release_cmd_;
    if (!eligible) { reset_stall_boost(); return; }
    const int8_t err_sign = err > 0 ? 1 : -1;
    if (!boost_eligible_ || err_sign != boost_err_sign_) {
      reset_stall_boost();
      boost_eligible_ = true;
      boost_err_sign_ = err_sign;
      boost_window_running_ = true;  // 窓の開始
      boost_window_start_us_ = now;
      boost_active_since_us_ = now;
      boost_anchor_raw_ = raw_;
    }
  }

  // 停止ブースト（Issue #15 H4）の窓。Duty を書いた後に呼ぶ。対象のまま stall_boost_ms の窓の間にカウントが
  // stall_boost_min_move 未満しか動かなければ上乗せを V1_STALL_BOOST_STEP_PERMILLE（1 % = 10 permille）増やす
  //（(duty_max − duty_min) × 10 で頭打ち）。動けば窓と時間の起点を
  // 取り直す（上乗せは保つ）。
  // 上限: 上乗せが頭打ち（Duty = duty_max）のまま次の窓も動かない、または対象になってから stall_boost_max_ms 動かない
  //（既定 2 s）なら ENCODER_FAULT をラッチする（詰まった機構か死んだエンコーダかは区別できない。静止目標では無パルス判定が
  // 回らないので、上乗せを duty_max で出し続ける経路をここで閉じる。FAULT_RESET で基準が消えるのは protocol.md §4 のとおり）。
  // 誤差の上限は従来どおり FOLLOWING_ERROR が守る。
  void update_stall_boost(uint64_t now) {
    if (!boost_eligible_ || !boost_window_running_) return;
    const int64_t moved = raw_ - boost_anchor_raw_;
    // r6（PR #33 Codex P2）: 「動いた」は誤差の向きへの進みだけ（自重で目標から離れても時計は戻さない）。従来経路は向きを問わない
    const int64_t moved_fwd = tuning.ff_directional ? static_cast<int64_t>(boost_err_sign_) * moved
                                                    : (moved < 0 ? -moved : moved);
    if (moved_fwd >= tuning.stall_boost_min_move) {
      boost_window_start_us_ = now;  // 動いている: 窓と時間の起点を取り直す
      boost_active_since_us_ = now;
      boost_anchor_raw_ = raw_;
      return;
    }
    const int max_boost = tuning.duty_max > tuning.duty_min ? v1_permille(tuning.duty_max - tuning.duty_min) : 0;
    if (stall_boost_ > max_boost) stall_boost_ = max_boost;  // SET_LIMITS で上限が下がった
    const bool timed_out = tuning.stall_boost_max_ms != 0 &&
                           now - boost_active_since_us_ >= ms_to_us(tuning.stall_boost_max_ms);
    if (tuning.ff_directional) {
      // Issue #25 r6: 床が無いので上乗せもしない。静止 2 s の監視（対象のまま stall_boost_min_move 未満しか進まない →
      // ENCODER_FAULT）だけを残す。時計を戻すのは上の「動いた」だけで、動き出しパルス（update_breakaway）は戻さない。
      // r10（supervisor_on）: ラッチしない（押さえられた軸を ENCODER_FAULT にしない。Issue #44）。V1Supervisor が代わりに守る
      stall_boost_ = 0;
      if (timed_out && !supervisor_on() && !encoder_fault_check_disabled()) latch_fault(v1::fault::ENCODER_FAULT, now);
      return;
    }
    if (now - boost_window_start_us_ < ms_to_us(tuning.stall_boost_ms)) return;
    const bool at_cap = stall_boost_ >= max_boost;
    if (at_cap || timed_out) {
      if (!encoder_fault_check_disabled()) {
        latch_fault(v1::fault::ENCODER_FAULT, now);  // 出力 0・ブレーキ作動・主電源 OFF 指令。reset_stall_boost は次の update_duty で
        return;
      }
      // duty_max < stall_duty で無パルス判定を無効にしている構成では FAULT にせず頭打ちのまま（画面に「無パルス検出なし」）
      boost_window_start_us_ = now;
      return;
    }
    stall_boost_ += V1_STALL_BOOST_STEP_PERMILLE;
    if (stall_boost_ > max_boost) stall_boost_ = max_boost;
    boost_window_start_us_ = now;
    boost_anchor_raw_ = raw_;
  }

  // ------------------------------------------------------------------------
  // 送信
  // ------------------------------------------------------------------------

  void push_ack(uint32_t request_id, uint8_t acked_type, uint8_t result, uint16_t reason) {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::ACK;
    m.ack.request_id = request_id;
    m.ack.acked_type = acked_type;
    m.ack.result = result;
    m.ack.reason = reason;
    push(m);
  }

  // 満杯なら最古の STATUS を 1 つ捨てて入れる。STATUS が無ければ、入れるのが STATUS なら捨て、
  // それ以外なら最古を捨てる（ACK・FAULT_EVENT・AXIS_SELECT・HELLO を優先）。
  void push(const v1::McuMsg& m) {
    if (out_count_ >= kOutboxCap) {
      bool removed = false;
      for (size_t i = 0; i < out_count_; ++i) {
        const size_t idx = (out_head_ + i) % kOutboxCap;
        if (outbox_[idx].type == v1::msg::STATUS) {
          for (size_t j = i; j + 1 < out_count_; ++j) {
            outbox_[(out_head_ + j) % kOutboxCap] = outbox_[(out_head_ + j + 1) % kOutboxCap];
          }
          out_count_ -= 1;
          removed = true;
          break;
        }
      }
      if (!removed) {
        if (m.type == v1::msg::STATUS) return;
        pop();
      }
    }
    outbox_[(out_head_ + out_count_) % kOutboxCap] = m;
    out_count_ += 1;
  }

  void push_status(uint64_t now) {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::STATUS;
    m.status = status(now);
    push(m);
  }

  // LIMITS（motion-limits 設計 §4.3）: いまの tuning の有効値と config.h の絶対上限。SET_LIMITS 受理直後だけ送る。
  void push_limits() {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::LIMITS;
    m.limits.boot_id = id_.boot_id;
    m.limits.duty_max_percent = static_cast<uint8_t>(tuning.duty_max);
    m.limits.duty_min_percent = static_cast<uint8_t>(tuning.duty_min);
    m.limits.kp_percent_per_100 = static_cast<uint8_t>(tuning.kp_percent_per_100);
    m.limits.abs_duty_max_percent = static_cast<uint8_t>(V1_ABS_DUTY_MAX);
    m.limits.ff_count_s_per_percent = static_cast<uint16_t>(tuning.ff_count_s_per_percent);
    m.limits.deadband_count = static_cast<uint16_t>(tuning.deadband);
    m.limits.deadband_out_count = static_cast<uint16_t>(tuning.deadband_out);
    m.limits.abs_max_vel_count_s = V1_ABS_MAX_VEL_COUNT_S;
    m.limits.abs_max_acc_count_s2 = V1_ABS_MAX_ACC_COUNT_S2;
    push(m);
  }

  void remember_key() {
    has_last_key_ = true;
    key_state_ = state_;
    key_fault_ = fault_code();
    key_flags_ = flags();
    key_axis_ = selected_axis_;
  }

  // 状態・fault_code・flags・selected_axis が変わったら即時 STATUS
  void emit_status_if_changed() {
    const bool changed = !has_last_key_ || key_state_ != state_ || key_fault_ != fault_code() ||
                         key_flags_ != flags() || key_axis_ != selected_axis_;
    if (changed) {
      remember_key();
      push_status(now_us_);
    }
  }

  // 20 ms ごとの STATUS（固定の刻み、1 回の tick で最大 1 つ）
  void emit_periodic_status(uint64_t now) {
    const uint64_t period = ms_to_us(tuning.status_period_ms ? tuning.status_period_ms : 1);
    if (!has_next_status_) {
      has_next_status_ = true;
      next_status_us_ = now;
    }
    if (now >= next_status_us_) {
      next_status_us_ += ((now - next_status_us_) / period + 1) * period;
      remember_key();
      push_status(now);
    }
  }
};
