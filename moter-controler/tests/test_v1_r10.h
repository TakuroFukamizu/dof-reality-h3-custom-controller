// r10（Issue #44・#46）: 保護の監視 V1Supervisor のホストテスト。test_v1_axis.h の後に include する（Harness を使う）。
// プラントは Harness の pwm_plant（PWM で動く。物理位置 phys とエンコーダの値 raw を分け、enc_connected = false で断線。ブレーキの遅れ・
// 効かないブレーキ・重力／外力 ext_vel・乗員が押さえる held・荷重で遅い plant_max_vel）。
// 既定（PR #48 の Codex 事前レビュー後）: 包絡線は一律 2500 count/s、最小予算 0、先読みは最悪の tick 間隔 15 ms、SOFT_LIMIT / FOLLOWING_ERROR の
// 猶予は OFF、停止（FEEDBACK_UNCERTAIN / LOAD_LIMITED）は PWM 0 ＋ ブレーキ作動で DISARMED まで続く。
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>

#include "test_v1_axis.h"
#include "v1_supervisor.h"

namespace v1r10_test {

using namespace v1axis_test;

constexpr uint16_t SUP_FLAGS = fl::LOAD_LIMITED | fl::FEEDBACK_UNCERTAIN | fl::END_BRAKING | fl::LIMIT_RECOVERY;

// 先読み 15 ms・5 ms の tick で、予算 B [count] の間に出せる tick の数（一律 2500 count/s = 12.5 count/tick）
constexpr int ticks_for_budget(double b, double proj_count = 37.5) {
  return b < proj_count ? 0 : static_cast<int>((b - proj_count) / 12.5) + 1;
}

// r10 の軸: r6 の経路（方向別 FF）＋監視 ON。kp 30 / duty_max 20 は 11-02 の運用設定。ref を渡せば SET_REFERENCE で
// 実位置をそこにしてから（信用の起点もそこ）、ARM → HOLDING → START（ACTIVE、目標 = いま）まで進める。
// deadline = false: 確かな動きの無い期限（PR #50 の Codex 事前レビュー P0）を切る。予算・引き上げ・追従など別の仕組みだけを見る試験用
//（期限は test_r10_deadline_* で見る）
// park: ブレーキ駐車（2026-11-02）。既定 false（駐車の試験 test_r10_park_* 以外は、静止の区間で駐車しないようにして各仕組みを見る）
inline void r10_active(Harness& h, int32_t ref = 0, uint32_t fe_limit = 100000, int kp = 30, int duty_max = 20,
                       bool pwm_plant = true, bool deadline = true, bool park = false) {
  h.ax.tuning = r6_tuning(kp, duty_max);
  h.ax.tuning.supervisor = true;
  h.ax.tuning.sup.deadline = deadline;
  h.ax.tuning.park = park;
  if (park) h.ax.tuning.playout_delay_ms = V1_PLAYOUT_DELAY_MS;  // 駐車は焼く既定の参照（再生バッファ）で見る（-DV1_PLAYOUT=0 の構成でも）
  h.ax.tuning.hold_learn = true;  // r10（実測 2）: 保持出力の学習
  h.ax.tuning.vel_kd_permille_per_count_s = 0.0;  // 減衰（r9）は保護に関係しない。手で動かすカウントで v_fast が跳ねるので切る
  h.pwm_plant = pwm_plant;
  h.stall = !pwm_plant;
  if (ref != 0) Harness::accepted(h.set_reference(ref));
  h.phys = static_cast<double>(h.raw);
  v1::Config c = base_config();
  c.following_error_limit_count = fe_limit;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  CHECK(h.ax.state() == st::ACTIVE && h.ax.supervisor_on());
}

// 10 ms ごとに SET_POSITION を送りながら 5 ms ずつ進め、出力が 0 でない tick を数える
inline int drive_count_nonzero(Harness& h, uint32_t ms, int32_t target) {
  int n = 0;
  for (uint32_t t = 0; t < ms; t += 5) {
    if ((t % 10) == 0) h.set_position(target);
    h.step(5);
    if (h.ax.pwm_permille() != 0) ++n;
  }
  return n;
}

inline bool no_fault_events(const Harness& h) { return h.fault_events().empty(); }

// 持続する停止の出力: PWM 0・ブレーキ作動・主電源は保つ・状態は変えない（FAULT ではない）
inline bool braked_inhibit(const Harness& h) {
  return h.ax.supervisor_inhibited() && h.ax.pwm_permille() == 0 && h.ax.duty_permille() == 0 && !h.ax.brake_release_cmd() &&
         h.ax.power_on_cmd() && h.ax.state() != st::FAULT && (h.status().flags & fl::BRAKE_RELEASE_CMD) == 0;
}

// --- 既定値 -------------------------------------------------------------------

inline void test_r10_conservative_defaults() {
  const V1SupTuning t;
  CHECK(t.env_ref_permille == 0 && t.env_count_s == 2500);  // 一律の包絡線
  CHECK(t.min_budget_count == 0);                           // D ≤ 400 → 0
  CHECK(t.projection_ms == V1_OVERRUN_MS);                  // 最悪の tick 間隔
  CHECK(!t.soft_grace && t.fe_grace_max_ms == 0);           // SOFT_LIMIT / FOLLOWING_ERROR の猶予は OFF
  CHECK(V1Tuning(true).supervisor == (V1_SUPERVISOR != 0) && !V1Tuning(false).supervisor);
  CHECK(!t.renew_directional && t.hold_allow_trim_permille == 40);  // どちら向きでも確かな動き、保持出力 + 4 % は積まない（実測 2）
  CHECK(t.deadline && t.deadline_proportional && t.backdrive_count_s == 200 && t.certify_ms == 100);  // P0 / P1c
  CHECK(V1Tuning(true).park == (V1_PARK != 0) && !V1Tuning(false).park && V1Tuning(true).park_still_ms == 500 &&
        V1Tuning(true).brake_engage_ms == 150 && fl::PARKED == 0x800);
  CHECK(V1Tuning(true).hold_learn == (V1_HOLD_LEARN != 0) && !V1Tuning(false).hold_learn);
}

// --- V1Supervisor 単体（数を厳密に見るため先読みは 5 ms） -------------------------

struct SupRig {
  V1Supervisor s;
  V1Supervisor::Ctx c;
  SupRig() {
    s.t.projection_ms = 5;
    s.t.deadline = false;  // 予算の単体試験（期限は test_r10_deadline_unit）
    s.t.brake_acc_count_s2 = 0;  // 制動は別に見る
    c.soft_min = -2700;
    c.soft_max = 2700;
    c.duty_max_pm = 200;
    c.hold_exit = 4;
    c.err = 300;
    s.reset_trust(0, 0);
    s.end_session(0, 0);
  }
  int tick(int cmd, int64_t raw) {
    c.now_us += 5 * MS;
    c.raw = raw;
    c.err = cmd > 0 ? 300 : (cmd < 0 ? -300 : 0);
    s.observe(c);
    return s.filter(c, cmd);
  }
};

// 予算の式と位置の上限: 機械端 = ソフトリミット ± 50。D = 信用できた位置からの距離。min(1000, max(0, D − 400))、最小予算（変種）50（D − 20 まで）
inline void test_r10_budget_derivation() {
  SupRig r;
  auto lim = [&](int64_t trusted, int d) {
    r.s.reset_trust(trusted, 0);
    return r.s.position_limit(d, r.c);
  };
  CHECK(lim(2100, 1) == 250);    // D = 2750 − 2100 = 650 → 250（2500 count/s で 100 ms）
  CHECK(lim(2225, 1) == 125);    // D = 525 → 125（50 ms）
  CHECK(lim(2350, 1) == 0);      // D = 400 → 0
  CHECK(lim(-2100, -1) == 250);  // 下も同じ
  CHECK(lim(0, 1) == 2350);      // 中央: 位置の上限は 2350。その回の予算は回の上限 1000 との小さい方
  CHECK(lim(2740, -1) == 5090);  // 反対向きは広い（位置の上限は 1000 で頭打ちにしない）
  r.s.t.min_budget_count = 50;   // 変種
  CHECK(lim(2350, 1) == 50);
  CHECK(lim(2700, 1) == 30);     // D = 50 → min(50, 50 − 20)
  CHECK(lim(2740, 1) == 0);      // D = 10 → 0（機械端から 20 以内へは出さない）
}

// 確かな動きだけが予算を戻す: ±1 のちらつき・片相のトグル・PWM の隙間・出力の反転では戻らない。出力と逆向きの動き（押し戻された・
// 荷重で沈む）は、既定（どちら向きでも数える。実測 2）では戻し、renew_directional（Issue #44 の元の指示）では戻さない
inline void test_r10_renewal_rules_unit() {
  auto ticks_until_inhibit = [](std::function<int64_t(int)> raw_at, std::function<int(int)> cmd_at, int* up, int* down,
                                bool directional = false) {
    SupRig r;
    r.s.t.renew_directional = directional;
    int n = 0;
    for (int i = 0; i < 2000 && !r.s.feedback_uncertain(); ++i) {
      const int out = r.tick(cmd_at(i), raw_at(i));
      if (out > 0) ++*up;
      if (out < 0) ++*down;
      if (out != 0) ++n;
    }
    CHECK(r.s.feedback_uncertain() && r.s.inhibited());
    CHECK(r.s.envelope_episode(1) <= 1000.0 + 1e-9 && r.s.envelope_episode(-1) <= 1000.0 + 1e-9);
    return n;
  };
  int up = 0, down = 0;
  // 予算 1000 = 80 tick（12.5 count/tick）
  CHECK(ticks_until_inhibit([](int i) { return static_cast<int64_t>(i % 2); }, [](int) { return 200; }, &up, &down) == 80);  // ちらつき
  up = down = 0;
  CHECK(ticks_until_inhibit([](int i) { return static_cast<int64_t>((i / 2) % 2); }, [](int) { return 200; }, &up, &down) == 80);  // 片相
  up = down = 0;
  CHECK(ticks_until_inhibit([](int i) { return static_cast<int64_t>(-i); }, [](int) { return 200; }, &up, &down, true) == 80);  // 逆向き（向きつき）
  {  // 逆向き（既定、P1c）: 下向きの確かな動きは下の予算だけを戻す。出している上向きの予算は戻らず 80 tick で止まる
    up = down = 0;
    CHECK(ticks_until_inhibit([](int i) { return static_cast<int64_t>(-i); }, [](int) { return 200; }, &up, &down) == 80);
    SupRig r;
    for (int i = 0; i < 40; ++i) r.tick(-200, 0);  // 下に 40 tick 出してから
    r.tick(0, 0);                                  // 最後の出力の区間を積む
    const double down_env = r.s.envelope_episode(-1);
    for (int i = 1; i <= 3; ++i) r.tick(0, i);     // 上へ 3 count 動いた
    // 下の包絡線は残す（P1c）。位置はいま分かったので両向きの距離の基準はいまの位置（実機 12712ac の FAIL 2）
    CHECK(down_env > 400.0 && r.s.envelope_episode(-1) == down_env && r.s.trusted_raw(1) == 2 && r.s.trusted_raw(-1) == 2);
  }
  up = down = 0;
  CHECK(ticks_until_inhibit([](int) { return int64_t{0}; }, [](int i) { return (i / 10) % 2 == 0 ? 200 : 0; }, &up, &down) == 80);  // 隙間
  up = down = 0;
  ticks_until_inhibit([](int) { return int64_t{0}; }, [](int i) { return (i / 10) % 2 == 0 ? 200 : -200; }, &up, &down);  // 反転
  CHECK(up == 80 && down <= 80 && down >= 70);
  // 出力の向きへ 2 count 進めば戻る
  SupRig r;
  for (int i = 0; i < 40; ++i) r.tick(200, 0);
  CHECK(r.s.envelope_episode(1) > 400.0);
  r.tick(200, 1);
  CHECK(r.s.envelope_episode(1) > 400.0);  // 1 では戻らない
  r.tick(200, 2);
  CHECK(r.s.envelope_episode(1) == 0.0 && r.s.trusted_raw(1) == 2);
  // 既知の穴（PCNT のフィルタ・A/B 遷移の妥当性の計数が後続）: ±2 以上の往復は戻してしまう
  SupRig k;
  for (int i = 0; i < 40; ++i) k.tick(200, 0);
  k.tick(200, -2);
  k.tick(200, 0);
  CHECK(k.s.envelope_episode(1) == 0.0);
}

// 解除の範囲: DISARMED に戻す（clear_episode）は停止と回の上限を戻すが、位置の上限は戻さない。基準の取り直しは全部
inline void test_r10_recovery_scope_unit() {
  SupRig r;
  r.s.reset_trust(2100, 0);  // 上の位置の上限 250
  int n = 0;
  for (int i = 0; i < 100; ++i) n += r.tick(200, 2100) != 0 ? 1 : 0;
  CHECK(r.s.feedback_uncertain() && n == 20);
  r.s.end_session(2100, r.c.now_us);
  CHECK(r.s.feedback_uncertain());  // 位置制御が止まっただけ（ARMING のブレーキ作動など）では解除しない
  r.s.clear_episode();
  CHECK(!r.s.feedback_uncertain() && r.s.envelope_episode(1) == 0.0 && r.s.envelope_cumulative(1) == 250.0);
  CHECK(r.tick(200, 2100) == 0 && r.s.feedback_uncertain());  // 位置の上限は尽きたまま: 端の方へは見えないまま出せない
  r.s.end_session(2100, r.c.now_us);
  r.s.clear_episode();
  CHECK(r.tick(-200, 2100) == -200);  // 反対向きは出せる
  r.s.reset_trust(2100, r.c.now_us);  // SET_REFERENCE / 校正
  CHECK(r.s.envelope_cumulative(1) == 0.0 && r.s.envelope_cumulative(-1) == 0.0 && !r.s.feedback_uncertain());
}

// --- 軸を通して -------------------------------------------------------------

// エンコーダ断（物理位置だけが動く）、中央付近、Duty 10 / 13 / 20 %、両向き: 予算（1000 count）の中で PWM 0 ＋ ブレーキ・FEEDBACK_UNCERTAIN。
// ENCODER_FAULT はラッチしない（ACTIVE のまま）。一律の包絡線（既定）: 断線からの出力は 78 tick 以下（先読み 15 ms）。比例（変種）: 物理の移動 ≤ 1000
inline void test_r10_dead_encoder_mid_stroke_cut_within_budget() {
  for (int duty : {10, 13, 20}) {
    for (int dir : {1, -1}) {
      for (bool proportional : {false, true}) {
        Harness h;
        r10_active(h, 0, 100000, 30, duty);
        if (proportional) h.ax.tuning.sup.env_ref_permille = V1_SUP_ENV_PROP_REF_PERMILLE;
        h.pos_max_vel = 1000;  // 中央付近で端の手前の制動（停止距離 500）に掛からない速さ
        const int32_t target = dir * 2400;
        drive_count_nonzero(h, 200, target);  // 繋がったまま走り出す
        CHECK(h.ax.pwm_permille() * dir > 0 && !h.ax.supervisor().feedback_uncertain());
        h.enc_connected = false;  // 断線: 物理位置だけが動く
        const double phys0 = h.phys;
        const int n = drive_count_nonzero(h, 1500, target);
        const double blind = fabs(h.phys - phys0);
        if (std::getenv("R10_PRINT")) {
          std::printf("dead enc duty %d dir %+d env %s: nonzero %d ticks, blind %.1f count\n", duty, dir,
                      proportional ? "prop" : "flat", n, blind);
        }
        CHECK(h.ax.state() == st::ACTIVE && h.ax.fault_code() == fc::NONE && no_fault_events(h));
        CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain());
        CHECK((h.status().flags & fl::FEEDBACK_UNCERTAIN) != 0);
        CHECK(blind <= 1000.0);
        if (!proportional) CHECK(n >= 75 && n <= 78);  // 最後に確かに動いた tick からの 78 tick（断線の直前の数 tick を引く）
      }
    }
  }
}

// 端の近く（既定）: D = 650 / 525 / 400 → 18 / 8 / 0 tick（先読み 15 ms のぶん仕様の 100 / 50 / 0 ms より 1〜2 tick 短い）。
// 先読み 5 ms なら仕様どおり 20 / 10 / 0 tick（100 / 50 / 0 ms）。最小予算の変種（50）なら D = 400 で 2 tick。物理の移動は予算以下
inline void test_r10_dead_encoder_near_end_budgets() {
  struct Case { int32_t d; int32_t budget; };
  enum Mode { Default, Projection5, MinBudget };
  for (const Case& k : {Case{650, 250}, Case{525, 125}, Case{400, 0}}) {
    for (int dir : {1, -1}) {
      for (Mode mode : {Default, Projection5, MinBudget}) {
        Harness h;
        const int32_t ref = dir > 0 ? 2700 + 50 - k.d : -2700 - 50 + k.d;
        r10_active(h, ref, 100000, 30, 20, true, false);
        h.ax.tuning.soft_reserve_count = 0;  // 参照の端の手前の減速（Issue #45 / #46）を外し、監視の予算だけを見る（参照は既定では監視の帯の外で止まる）
        int expect = ticks_for_budget(k.budget);
        double bound = k.budget;
        if (mode == Projection5) {
          h.ax.tuning.sup.projection_ms = 5;
          expect = ticks_for_budget(k.budget, 12.5);
        } else if (mode == MinBudget) {
          h.ax.tuning.sup.min_budget_count = V1_SUP_MIN_BUDGET_VARIANT_COUNT;
          if (k.budget < 50) { expect = ticks_for_budget(50); bound = 50; }
        }
        CHECK(h.ax.actual_count() == ref && h.ax.pwm_permille() == 0);
        h.enc_connected = false;
        const double phys0 = h.phys;
        const int n = drive_count_nonzero(h, 500, ref + dir * 300);
        CHECK(n == expect);
        CHECK(fabs(h.phys - phys0) <= bound + 1e-9);
        CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && braked_inhibit(h));
      }
    }
  }
  CHECK(ticks_for_budget(250) == 18 && ticks_for_budget(125) == 8 && ticks_for_budget(0) == 0);
  CHECK(ticks_for_budget(250, 12.5) == 20 && ticks_for_budget(125, 12.5) == 10);
}

// 不規則な tick（5〜14 ms、CONTROL_OVERRUN の 15 ms 未満）でも、先読みが最悪の間隔を見るので物理の移動は予算を超えない
inline void test_r10_irregular_ticks_stay_within_budget() {
  const uint32_t pattern[] = {5, 9, 14, 7, 12, 6, 14, 5, 11};
  for (int32_t ref : {0, 2100}) {
    Harness h;
    r10_active(h, ref, 100000, 30, 20, true, false);
    h.ax.tuning.soft_reserve_count = 0;  // 参照の端の手前の減速を外し、監視の予算だけを見る
    h.enc_connected = false;
    const double phys0 = h.phys;
    for (int i = 0; i < 300; ++i) {
      h.set_position(ref + 400);
      h.step(pattern[i % 9]);
    }
    const double budget = ref == 0 ? 1000.0 : 250.0;
    CHECK(fabs(h.phys - phys0) <= budget + 1e-9);
    CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && braked_inhibit(h));
  }
}

// 乗員が押さえて 10 s 動かない（エンコーダは生きている、物理も止まっている）: ENCODER_FAULT にしない。出力は予算（1000 count = 78 tick）
// の後 PWM 0 ＋ ブレーキ（FEEDBACK_UNCERTAIN）。状態は ACTIVE のまま。手を離した後の DISARM → 再 ARM で目標へ動ける。位置の上限は再 ARM で残る
inline void test_r10_held_occupant_10s_no_encoder_fault() {
  Harness h;
  r10_active(h, 0, 200);  // FOLLOWING_ERROR 200 は誤差 150 では出ない
  h.held = true;
  int n = 0;
  bool load_seen = false;
  for (int i = 0; i < 1000; ++i) {  // 10 s
    n += drive_count_nonzero(h, 10, 150);
    if (h.ax.supervisor().load_limited()) load_seen = true;
  }
  CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h));
  CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain() && !load_seen);
  CHECK(n >= 76 && n <= 78);
  const V1SupInfo info = h.ax.supervisor_info();
  CHECK(info.enabled && info.feedback_uncertain && info.activity_age_ms >= 9900 && info.budget_up < 38);
  const double cum_up = h.ax.supervisor().envelope_cumulative(1);
  CHECK(cum_up > 900.0);
  // 手を離して DISARM（減速 → ブレーキ作動 → DISARMED）→ 再 ARM → START: 目標へ動き、確かな動きで予算が戻る
  h.held = false;
  Harness::accepted(h.disarm());
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED && !h.ax.supervisor_inhibited() && (h.status().flags & SUP_FLAGS) == 0);
  CHECK(h.ax.supervisor().envelope_episode(1) == 0.0 && h.ax.supervisor().envelope_cumulative(1) == cum_up);  // 位置の上限は残る
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  for (int i = 0; i < 150; ++i) drive_count_nonzero(h, 10, 150);
  CHECK(h.ax.state() == st::ACTIVE && !h.ax.supervisor_inhibited() && no_fault_events(h));
  CHECK(std::abs(h.ax.actual_count() - 150) <= 4);
  if (std::getenv("R10_PRINT")) std::printf("held: after resume env_cum up %.1f (before %.1f)\n", h.ax.supervisor().envelope_cumulative(1), cum_up);
  CHECK(h.ax.supervisor().envelope_cumulative(1) < cum_up);  // 確かな動きで戻った（その後の小さな補正の分は残りうる）
}

// 停止の後に古い目標が溜まっていても、再開は通常の ARMING（主電源 → power settle → ブレーキ解除 → 解除待ち）を通り、目標は ARM_PREPARE の
// 現在位置から（停止中に届いた遠い目標へは飛ばない）。停止中はブレーキを解除し直さない。制御器の状態（積分）は 0 から
inline void test_r10_resume_after_inhibit_discards_queued_targets() {
  Harness h;
  r10_active(h, 0, 100000);
  h.held = true;
  drive_count_nonzero(h, 600, 150);
  CHECK(braked_inhibit(h));
  for (int i = 0; i < 50; ++i) {  // 停止中も PC は目標を送り続ける（遠い目標）
    h.set_position(1500);
    h.step();
    CHECK(!h.ax.brake_release_cmd() && h.ax.pwm_permille() == 0);
  }
  h.held = false;
  Harness::accepted(h.disarm());  // 停止中も軌道は遠い目標へ走っていたので、減速（STOPPING）→ DISARMING → DISARMED
  h.idle(1000);
  CHECK(h.ax.state() == st::DISARMED);
  const int32_t here = h.ax.actual_count();
  Harness::accepted(h.arm());
  CHECK(h.ax.state() == st::ARMING && !h.ax.brake_release_cmd() && h.ax.target_count() == here);
  h.drive(POWER_SETTLE_MS - 20, here);
  CHECK(!h.ax.brake_release_cmd());  // power settle の間は作動のまま
  h.drive(40, here);
  CHECK(h.ax.brake_release_cmd() && h.ax.state() == st::ARMING);  // 解除指令、解除待ち
  h.drive(BRAKE_RELEASE_WAIT_MS + 20, here);
  CHECK(h.ax.state() == st::HOLDING && h.ax.target_count() == here && h.ax.integral_permille() == 0);
  h.set_position(here);
  Harness::accepted(h.start());
  h.step();
  CHECK(h.ax.target_count() == here && std::abs(h.ax.pwm_permille()) < 50);  // 古い 1500 へは向かわない
  CHECK(!h.ax.supervisor_inhibited() && no_fault_events(h));
}

// 重力（−300 count/s）の下で停止したとき: ブレーキが効けば流れるのは遅れ（50 / 300 ms）の間だけ。効かない（PWM 0 だけの退避）と流れ続ける
inline void test_r10_gravity_drift_with_brake_vs_without() {
  struct Case { uint32_t delay_ms; bool fails; };
  for (const Case& k : {Case{50, false}, Case{BRAKE_APPLY_WAIT_MS, false}, Case{50, true}}) {
    Harness h;
    r10_active(h, 0, 100000);
    h.brake_delay_ms = k.delay_ms;
    h.brake_fails = k.fails;
    h.ext_vel = -300.0;  // 自重（ブレーキ解除中は制御が打ち消している）
    drive_count_nonzero(h, 300, 0);
    CHECK(!h.ax.supervisor_inhibited());
    h.enc_connected = false;  // エンコーダが死んだまま上へ動かそうとする → 予算で停止
    for (int i = 0; i < 200 && !h.ax.supervisor_inhibited(); ++i) drive_count_nonzero(h, 5, 300);
    CHECK(braked_inhibit(h));
    const double at_inhibit = h.phys;
    drive_count_nonzero(h, 1000, 300);
    const double drift = fabs(h.phys - at_inhibit);
    if (std::getenv("R10_PRINT")) std::printf("gravity: delay %u fails %d drift %.1f\n", k.delay_ms, k.fails ? 1 : 0, drift);
    if (k.fails) {
      CHECK(drift >= 280.0);  // 1 s 流れる
    } else {
      CHECK(drift <= 300.0 * k.delay_ms / 1000.0 + 5.0);  // 遅れの間だけ（tick の量子化ぶん）
    }
  }
}

// 外力で押されて目標から離れる（エンコーダは生きている）→ 離せば戻る: 短い押し（誤差 < 200）はラッチしない。
// 上限 200 を越える押しは既定では従来どおり FOLLOWING_ERROR、FE の猶予（変種）を ON にすれば戻りつつある間はラッチしない
inline void test_r10_external_push_away() {
  struct Case { uint32_t push_ms; bool fe_grace; };
  for (const Case& k : {Case{60, false}, Case{120, false}, Case{120, true}}) {
    Harness h;
    r10_active(h, 0, 200);
    if (k.fe_grace) h.ax.tuning.sup.fe_grace_max_ms = V1_FE_GRACE_VARIANT_MS;
    h.ext_vel = -4000.0;
    int max_fe = 0;
    bool recovery_seen = false;
    for (uint32_t t = 0; t < 1500 && h.ax.state() != st::FAULT; t += 10) {
      if (t >= k.push_ms) h.ext_vel = 0.0;
      drive_count_nonzero(h, 10, 0);
      const int fe = std::abs(h.ax.following_error_count());
      if (fe > max_fe) max_fe = fe;
      if (h.status().flags & fl::LIMIT_RECOVERY) recovery_seen = true;
    }
    if (std::getenv("R10_PRINT")) std::printf("push %u ms grace %d: max |FE| %d\n", k.push_ms, k.fe_grace ? 1 : 0, max_fe);
    if (k.push_ms == 120 && !k.fe_grace) {
      CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::FOLLOWING_ERROR);  // 既定: 猶予なし
      continue;
    }
    CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h));
    CHECK(!h.ax.supervisor_inhibited() && (h.status().flags & SUP_FLAGS) == 0);
    CHECK(std::abs(h.ax.actual_count()) <= 4);
    if (k.fe_grace) CHECK(max_fe > 200 && max_fe < 300 && recovery_seen);
    if (k.push_ms == 60) CHECK(max_fe < 200);
  }
}

// 過負荷（LOAD_LIMITED）: 荷重で 15 count/s しか進めない（確かな動きはあるので予算は戻る）まま飽和 1 s → PWM 0 ＋ ブレーキ。自動では戻らない
//（3 s 待っても出さない）。DISARM → 再 ARM で戻り、荷重が無くなれば目標に着く
inline void test_r10_load_limited_brakes_and_stays_until_rearm() {
  Harness h;
  r10_active(h, 0, 200);
  h.plant_max_vel = 15.0;
  const uint64_t t0 = h.now;
  for (int i = 0; i < 200 && !h.ax.supervisor().load_limited(); ++i) drive_count_nonzero(h, 10, 150);
  const uint64_t t_limited = h.now - t0;
  CHECK(h.ax.supervisor().load_limited() && braked_inhibit(h) && (h.status().flags & fl::LOAD_LIMITED) != 0);
  CHECK(t_limited >= 1000 * MS && t_limited <= 1150 * MS);
  CHECK(!h.ax.supervisor().feedback_uncertain());
  h.plant_max_vel = 1e9;  // 荷重が無くなっても自動では戻らない
  const int n = drive_count_nonzero(h, 3000, 150);
  CHECK(n == 0 && braked_inhibit(h) && h.ax.supervisor().load_limited() && h.ax.state() == st::ACTIVE);
  Harness::accepted(h.disarm());
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED && !h.ax.supervisor_inhibited());
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  drive_count_nonzero(h, 1500, 150);
  CHECK(std::abs(h.ax.actual_count() - 150) <= 4 && (h.status().flags & SUP_FLAGS) == 0);
  CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h));
}

// 静止目標で誤差 10 のまま進まない（r6〜r9 は 2 s で ENCODER_FAULT）・軌道が動いているのに止まったまま（走行中の無パルス）:
// どちらも ENCODER_FAULT にしない（停止 = ブレーキ）。FE 上限があれば止まった軸は従来どおり FOLLOWING_ERROR
inline void test_r10_old_stall_scenarios_never_latch_encoder_fault() {
  Harness h;
  r10_active(h, 0, 100000, 2, 20, false);  // 手で動かすカウント（動かない）
  h.raw -= 10;
  h.drive(3000, h.ax.target_count());
  CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && braked_inhibit(h));
  Harness g;
  r10_active(g, 0, 100000, 6, 20, false);
  g.pos_max_vel = 600;
  const int32_t s0 = g.ax.actual_count();
  for (int i = 0; i < 100; ++i) g.drive(10, s0 + 5000);
  CHECK(g.ax.state() == st::ACTIVE && no_fault_events(g) && braked_inhibit(g) && g.ax.supervisor().feedback_uncertain());
  Harness k;
  r10_active(k, 0, 400, 6, 20, false);
  k.pos_max_vel = 600;
  const int32_t k0 = k.ax.actual_count();
  for (int i = 0; i < 100 && k.ax.state() != st::FAULT; ++i) k.drive(10, k0 + 5000);
  CHECK(k.ax.state() == st::FAULT && k.ax.fault_code() == fc::FOLLOWING_ERROR);
}

// 停止の解除は DISARMED に戻すときだけ（DISARM の完了か FAULT_RESET）。外から動かされても自動では解除しない
inline void test_r10_feedback_uncertain_requires_explicit_recovery() {
  Harness h;
  r10_active(h, 0, 100000);
  h.held = true;
  drive_count_nonzero(h, 600, 150);
  CHECK(braked_inhibit(h));
  h.held = false;
  h.raw += 5;  // 外から動いた（出力は 0 なので「出力の向きへの確かな動き」ではない）
  h.phys = static_cast<double>(h.raw);
  drive_count_nonzero(h, 500, 150);
  CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain());
  h.host_fault();  // HOST_FAULT → FAULT → FAULT_RESET でも解除される
  CHECK(h.ax.state() == st::FAULT && h.ax.supervisor().feedback_uncertain());
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.state() == st::DISARMED && !h.ax.supervisor_inhibited() && h.ax.referenced());
}

// ARMING のブレーキ解除の後（HOLDING の前）に停止しても、解除し直さない（ARMING のまま作動）
inline void test_r10_inhibit_during_arming_keeps_brake_applied() {
  Harness h;
  h.ax.tuning = r6_tuning(30);
  h.ax.tuning.supervisor = true;
  h.ax.tuning.vel_kd_permille_per_count_s = 0.0;
  h.stall = true;
  Harness::accepted(h.set_reference(2100));  // 上の予算 250 count（18 tick = 90 ms）。解除待ち 300 ms の間に尽きるように端の近くで
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.arm());
  h.drive(POWER_SETTLE_MS + 10, 2100);
  CHECK(h.ax.state() == st::ARMING && h.ax.brake_release_cmd());
  h.raw -= 30;  // 解除直後に誤差 +30（上へ）、カウントは動かない → 予算で停止
  h.drive(150, 2100);
  CHECK(h.ax.supervisor_inhibited() && !h.ax.brake_release_cmd());
  h.drive(1000, 2100);
  CHECK(h.ax.state() == st::ARMING && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0 && h.ax.power_on_cmd());
}

// 端の手前の制動（Issue #46）: 実測速度で止まれない（v² / 2a > ソフトリミットまでの残り）なら反対向きに duty_max。ラッチしない。
// この出力も包絡線に積む（P1a）。止まれる速さなら何もしない。動きが止まれば解除
inline void test_r10_end_braking_guard() {
  Harness h;
  r10_active(h, 2400, 100000, 6, 20, false);
  int guard_ticks = 0;
  const double env0 = h.ax.supervisor().envelope_episode(-1);
  for (int i = 0; i < 12; ++i) {  // 1200 count/s で上へ（停止距離 720 > 残り 300〜240）
    h.raw += 6;
    h.set_position(2400);
    h.step();
    if (h.ax.supervisor().end_braking()) ++guard_ticks;
  }
  CHECK(h.ax.supervisor().end_braking() && h.ax.pwm_permille() == -200 && (h.status().flags & fl::END_BRAKING) != 0);
  // 制動の出力は割り引かずに一律の包絡線で積む（PR #50 の Codex 事前レビュー P2）: 制動の tick ごとに 12.5 count（最後の tick は次で積む）。
  // 上向きの確かな動きは上の予算だけを戻すので、下（制動の向き）の包絡線は戻らない（P1c）
  CHECK(guard_ticks >= 3 && h.ax.supervisor().envelope_episode(-1) - env0 >= 12.5 * (guard_ticks - 1) - 1e-6);
  h.drive(80, 2400);  // 止まった: v_fast は 40 ms で 0
  CHECK(!h.ax.supervisor().end_braking() && h.ax.pwm_permille() != -200);
  Harness g;  // 止まれる速さ（200 count/s、停止距離 20 < 残り 300）
  r10_active(g, 2400, 100000, 6, 20, false);
  bool seen = false;
  for (int i = 0; i < 40; ++i) {
    g.raw += 1;
    g.set_position(2400);
    g.step();
    if (g.ax.supervisor().end_braking()) seen = true;
  }
  CHECK(!seen && no_fault_events(g));
  Harness k;  // 従来の経路（監視なし）には無い
  r10_active(k, 2400, 100000, 6, 20, false);
  k.ax.tuning.supervisor = false;
  for (int i = 0; i < 12; ++i) {
    k.raw += 6;
    k.set_position(2400);
    k.step();
  }
  CHECK(!k.ax.supervisor_info().enabled && (k.status().flags & fl::END_BRAKING) == 0 && k.ax.pwm_permille() > -200);
}

// 制動中にエンコーダが止まる: 制動は解除（v_fast が 0）、出力（制動を含む）は予算の中で止まり、PWM 0 ＋ ブレーキ
inline void test_r10_encoder_loss_during_braking() {
  Harness h;
  r10_active(h, 2400, 100000, 6, 20, false);
  int n = 0;
  for (int i = 0; i < 12; ++i) {
    h.raw += 6;
    h.set_position(2400);
    h.step();
    if (h.ax.pwm_permille() != 0) ++n;
  }
  CHECK(h.ax.supervisor().end_braking());
  n = 0;  // 断線の後に出した tick（動いていた間は確かな動きで予算が戻っている）
  for (int i = 0; i < 200; ++i) {  // カウントが止まる（断線）
    h.set_position(2400);
    h.step();
    if (h.ax.pwm_permille() != 0) ++n;
  }
  CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain() && !h.ax.supervisor().end_braking());
  CHECK(n <= ticks_for_budget(1000));
  CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h));
}

// SOFT_LIMIT: 既定は従来どおり外れたら即ラッチ。猶予（変種、soft_grace）を ON にしたとき: 少し越えただけで目標が内側・外へ動き続けて
// いないならラッチせず内側へ戻す（外向きの出力は出さない）。ラッチ: 機械端（ソフトリミット + 50）から 20 以内 / 外向きに 200 ms /
// 外に 2 s（戻す途中でエンコーダが止まれば予算で停止 = ブレーキ、2 s でラッチ）
inline void test_r10_soft_limit_default_latch_and_grace_variant() {
  auto grace = [](Harness& h, int32_t ref) {
    r10_active(h, ref, 100000, 6, 20, false);
    h.ax.tuning.sup.soft_grace = true;
  };
  {  // 既定: 即ラッチ
    Harness h;
    r10_active(h, 2690, 100000, 6, 20, false);
    h.raw += 20;
    h.drive(10, 2690);
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT);
  }
  {  // 変種: 少し越えて止まる → 戻す
    Harness h;
    grace(h, 2690);
    h.raw += 20;  // 2710
    bool outward = false;
    for (int i = 0; i < 60; ++i) {
      h.set_position(2690);
      h.step();
      if (h.ax.pwm_permille() > 0) outward = true;
    }
    CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && !outward);
    CHECK((h.status().flags & fl::LIMIT_RECOVERY) != 0 && h.ax.pwm_permille() < 0);
    h.raw -= 15;  // 2695 に戻った
    h.drive(20, 2690);
    CHECK((h.status().flags & fl::LIMIT_RECOVERY) == 0 && h.ax.state() == st::ACTIVE);
  }
  {  // 変種: 機械端から 20 以内（2730 = 2750 − 20）
    Harness h;
    grace(h, 2690);
    h.raw += 40;
    h.drive(10, 2690);
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT);
  }
  {  // 変種: 外へ 50 count/s で動き続ける → 外に出てから 200 ms 前後でラッチ（機械端の手前）
    Harness h;
    grace(h, 2698);
    for (int i = 0; i < 400 && h.ax.state() != st::FAULT; ++i) {
      if ((i % 4) == 0) h.raw += 1;
      h.set_position(2698);
      h.step();
    }
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT);
    CHECK(h.ax.actual_count() >= 2708 && h.ax.actual_count() < 2730);
  }
  {  // 変種: 外で動けない（エンコーダが止まっている）→ 予算で PWM 0 ＋ ブレーキ → 2 s でラッチ
    Harness h;
    grace(h, 2690);
    h.raw += 20;
    const uint64_t t0 = h.now;
    h.drive(600, 2690);
    CHECK(braked_inhibit(h) && h.ax.state() == st::ACTIVE);
    h.drive(1300, 2690);
    CHECK(h.ax.state() == st::ACTIVE);
    h.drive(200, 2690);
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT && h.now - t0 <= 2100 * MS);
  }
}

// FOLLOWING_ERROR: 既定は従来どおり越えたら即ラッチ。猶予（変種、fe_grace_max_ms）を ON にしたとき: 越えてもエンコーダが生きていて
// |誤差| が減っていればラッチしない。止まった（100 ms カウント変化なし）・増え続ける（越えてから 200 ms）・上限 × 1.5 以上ならラッチ
inline void test_r10_following_error_default_and_grace_variant() {
  auto grace = [](Harness& h) {
    r10_active(h, 0, 200, 6, 20, false);
    h.ax.tuning.sup.fe_grace_max_ms = V1_FE_GRACE_VARIANT_MS;
  };
  {  // 既定: −205 で即ラッチ
    Harness h;
    r10_active(h, 0, 200, 6, 20, false);
    for (int i = 0; i < 41 && h.ax.state() != st::FAULT; ++i) { h.raw -= 5; h.set_position(0); h.step(); }
    CHECK(h.ax.fault_code() == fc::FOLLOWING_ERROR && std::abs(h.ax.actual_count()) <= 205);
  }
  {  // 変種: 5 count/tick で −250 まで押され、1 count/tick で戻る（越えている時間 ≈ 300 ms）
    Harness h;
    grace(h);
    bool recovery_seen = false;
    for (int i = 0; i < 50; ++i) { h.raw -= 5; h.set_position(0); h.step(); }
    for (int i = 0; i < 80; ++i) {
      h.raw += 1;
      h.set_position(0);
      h.step();
      if (h.status().flags & fl::LIMIT_RECOVERY) recovery_seen = true;
    }
    CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && recovery_seen);
    h.drive(20, 0);
    CHECK((h.status().flags & fl::LIMIT_RECOVERY) == 0);
  }
  {  // 変種: −250 で止まる → カウントが 100 ms 動かなければラッチ
    Harness h;
    grace(h);
    for (int i = 0; i < 50; ++i) { h.raw -= 5; h.set_position(0); h.step(); }
    const uint64_t t0 = h.now;
    for (int i = 0; i < 60 && h.ax.state() != st::FAULT; ++i) h.drive(5, 0);
    CHECK(h.ax.fault_code() == fc::FOLLOWING_ERROR && h.now - t0 <= 160 * MS);
  }
  {  // 変種: 越えた後も 1 count/tick で増え続ける → 越えてから 200 ms でラッチ
    Harness h;
    grace(h);
    for (int i = 0; i < 40; ++i) { h.raw -= 5; h.set_position(0); h.step(); }
    uint64_t t_over = 0;
    for (int i = 0; i < 100 && h.ax.state() != st::FAULT; ++i) {
      h.raw -= 1;
      h.set_position(0);
      h.step();
      if (t_over == 0 && std::abs(h.ax.following_error_count()) > 200) t_over = h.now;
    }
    CHECK(h.ax.fault_code() == fc::FOLLOWING_ERROR && h.now - t_over >= 195 * MS && h.now - t_over <= 215 * MS);
  }
  {  // 変種: 上限 × 1.5 以上 → 猶予なし
    Harness h;
    grace(h);
    h.raw -= 301;
    h.drive(5, 0);
    CHECK(h.ax.fault_code() == fc::FOLLOWING_ERROR);
  }
}

// 既知の穴: エンコーダが死んで ±2 の雑音だけ出す（物理は出力どおり動く）と、雑音が「確かな動き」に数えられて予算が戻り続け、見えない
// 走行が予算を超える。PCNT のフィルタ（encoder_pcnt.cpp:56 で無効）と A/B 遷移の妥当性の計数を入れるまで閉じられない。この試験は穴を
// 固定するもので、塞いだら期待を反転させる。この場面では飽和したまま進まないので LOAD_LIMITED（1 s）が後から止める（飽和しない雑音
// なら止まらない）。±1 の雑音では予算どおり止まる
inline void test_r10_known_hole_noisy_pm2_feedback_renews_budget() {
  for (int amp : {1, 2}) {
    Harness h;
    r10_active(h, 0, 100000);
    h.enc_connected = false;
    const int64_t base = h.raw;
    const double phys0 = h.phys;
    for (int i = 0; i < 400; ++i) {  // 2 s
      h.raw = base + ((i % 2) ? 0 : -amp);
      h.set_position(2400);
      h.step();
    }
    const double blind = fabs(h.phys - phys0);
    if (std::getenv("R10_PRINT")) std::printf("noise ±%d: blind %.1f inhibited %d\n", amp, blind, h.ax.supervisor_inhibited() ? 1 : 0);
    if (amp == 1) {
      CHECK(blind <= 1000.0 && braked_inhibit(h));
    } else {
      CHECK(blind > 1000.0);  // 穴（期待の反転待ち）
      CHECK(h.ax.supervisor().load_limited() && !h.ax.supervisor().feedback_uncertain() && braked_inhibit(h));
    }
  }
}


// --- 静止目標での引き上げ（A）、保持出力の学習（実測 2）、小さな残り誤差の受け入れ（B、学習 OFF のときだけ） ---------------------------

// 荷重で逆に回るプラント（load_permille、静止摩擦 deadband）で、保持してから目標を err0 だけ上へ動かす。最大の出力・動いたか・結果を返す
struct EscResult {
  int max_out = 0;
  int32_t final_err = 0;
  bool moved = false;
  double blind = 0.0;
};
inline EscResult escalation_run(Harness& h, int load_pm, int deadband_pm, int32_t err0, uint32_t ms, bool disconnect = false,
                                double creep = 0.0) {
  r10_active(h, 0, 400, 30, 20, true, false);  // 引き上げと予算を見る（静止の保持で期限が先に来ないように切る）
  h.pwm_count_s_per_permille = 4.0;
  h.load_permille = load_pm;
  h.pwm_deadband_permille = deadband_pm;
  h.pwm_deadband_subtract = true;
  drive_count_nonzero(h, 3000, 0);  // 荷重の下で保持出力を学ぶ
  CHECK(!h.ax.supervisor_inhibited() && h.ax.state() == st::ACTIVE);  // 荷重の下で保持できている
  const int32_t start = h.ax.actual_count();
  const int32_t target = start + err0;
  if (disconnect) h.enc_connected = false;
  h.ext_vel = creep;
  const double phys0 = h.phys;
  EscResult r;
  for (uint32_t t = 0; t < ms && !h.ax.supervisor_inhibited(); t += 5) {
    if ((t % 10) == 0) h.set_position(target);
    h.step(5);
    const int u = std::abs(h.ax.pwm_permille());
    if (u > r.max_out) r.max_out = u;
  }
  r.final_err = target - h.ax.actual_count();
  r.moved = std::abs(h.ax.actual_count() - start) >= 10;
  r.blind = fabs(h.phys - phys0);
  return r;
}

// A: 保持 12 %・静止摩擦 3 %（持ち上げに 15 % 要る）軸を 26 count 上へ（#1 の最下端の行き過ぎ）。引き上げが 15 % を越えて動き、着いて保つ
inline void test_r10_escalation_lifts_axis_needing_15_percent() {
  Harness h;
  const EscResult r = escalation_run(h, 120, 30, 26, 3000);
  if (std::getenv("R10_PRINT")) {
    std::printf("esc 15%%: max_out %d final_err %d moved %d I %d inhib %d fu %d ll %d dl %d budget %d/%d verified %.1f\n", r.max_out, r.final_err, r.moved ? 1 : 0,
                h.ax.integral_permille(), h.ax.supervisor_inhibited() ? 1 : 0, h.ax.supervisor().feedback_uncertain() ? 1 : 0,
                h.ax.supervisor().load_limited() ? 1 : 0, h.ax.supervisor().deadline_tripped() ? 1 : 0, h.ax.supervisor_info().budget_up,
                h.ax.supervisor_info().budget_down, h.ax.supervisor().verified_hold_permille());
  }
  CHECK(r.max_out >= 150 && r.moved && std::abs(r.final_err) <= 2);
  CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && !h.ax.supervisor_inhibited());
  CHECK(h.ax.escalation_permille() == 0);  // 着いたら戻る
  // 監視 OFF・学習 OFF（r9 と同じ）では、動けないまま静止 2 s の監視で ENCODER_FAULT（14:22 の #1 の再現）。保持もできない
  Harness k;
  r10_active(k, 0, 400);
  k.ax.tuning.supervisor = false;
  k.ax.tuning.hold_learn = false;
  k.pwm_count_s_per_permille = 4.0;
  k.load_permille = 120;
  k.pwm_deadband_permille = 30;
  k.pwm_deadband_subtract = true;
  for (int i = 0; i < 1200 && k.ax.state() != st::FAULT; ++i) {
    if ((i % 2) == 0) k.set_position(26);
    k.step();
  }
  if (std::getenv("R10_PRINT")) std::printf("esc off: fault %u\n", k.ax.fault_code());
  CHECK(k.ax.state() == st::FAULT);
}

// A: 25 % 要る軸（静止摩擦 25 %。duty_max 20 % では動かない）。duty_max まで引き上げ、カウントが動かないので
// 見えない走行の予算で PWM 0 ＋ ブレーキ。にじむ（15 count/s）なら予算は戻り、duty_max の飽和 1 s で LOAD_LIMITED。ENCODER_FAULT なし
inline void test_r10_escalation_to_duty_max_then_braked_inhibit() {
  Harness h;
  const EscResult r = escalation_run(h, 0, 249, 26, 3000);
  if (std::getenv("R10_PRINT")) std::printf("esc 25%%: max_out %d moved %d\n", r.max_out, r.moved ? 1 : 0);
  CHECK(r.max_out == 200 && !r.moved);
  CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain() && no_fault_events(h));
  Harness g;
  const EscResult q = escalation_run(g, 0, 249, 40, 3000, false, 15.0);
  if (std::getenv("R10_PRINT")) std::printf("esc 25%% creep: max_out %d final_err %d\n", q.max_out, q.final_err);
  CHECK(q.max_out == 200);
  CHECK(braked_inhibit(g) && g.ax.supervisor().load_limited() && !g.ax.supervisor().feedback_uncertain() && no_fault_events(g));
}

// A: 引き上げの途中でエンコーダが死ぬ（物理は持ち上がる）→ 予算で止まる（物理の移動 ≤ 1000）
inline void test_r10_escalation_cut_by_budget_when_encoder_dead() {
  Harness h;
  const EscResult r = escalation_run(h, 120, 30, 26, 3000, true);
  if (std::getenv("R10_PRINT")) std::printf("esc dead: max_out %d blind %.1f\n", r.max_out, r.blind);
  CHECK(r.blind > 0.0 && r.blind <= 1000.0);
  CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain() && no_fault_events(h));
}

// 保持出力の学習の結果（保持の区間）
struct HoldResult {
  int pulse_onsets = 0;       // 着いてから 2 s 以降の動き出しパルスの立ち上がり
  int max_abs_err = 0;        // 着いてから 2 s 以降の |誤差| の最大
  int out_min = 1000, out_max = -1000;  // 同、出力の範囲 [permille]
  double out_mean = 0.0;
  bool inhibited = false;
  bool fault = false;
};
// 荷重 load（back-drive、静止摩擦 deadband）の軸を 0 → 300 へ動かし、hold_s 秒保持する
inline HoldResult hold_run(Harness& h, int load_pm, int deadband_pm, uint32_t hold_s, bool learn = true, bool deadline = false,
                           double sway = 0.0) {
  r10_active(h, 0, 400, 30, 20, true, deadline);
  h.ax.tuning.hold_learn = learn;
  h.pwm_count_s_per_permille = 4.0;
  h.load_permille = load_pm;
  h.pwm_deadband_permille = deadband_pm;
  h.pwm_deadband_subtract = true;
  h.pos_max_vel = 300;
  drive_count_nonzero(h, 3000, 300);  // 移動と着いてからの 2 s（学習）
  HoldResult r;
  bool prev = false;
  long sum = 0;
  int n = 0;
  for (uint32_t t = 0; t < hold_s * 1000 && h.ax.state() != st::FAULT; t += 5) {
    if ((t % 10) == 0) h.set_position(300);
    // 乗員の体重移動（sway [count/s] の外力、1.2 s 周期の矩形）: 生きたエンコーダの保持で見える小さな動き
    if (sway != 0.0) h.ext_vel = ((t / 600) % 2 == 0) ? sway : -sway;
    h.step();
    const bool brk = h.ax.duty_terms().mode == V1DutyMode::BRK;
    if (brk && !prev) ++r.pulse_onsets;
    prev = brk;
    const int e = std::abs(h.ax.following_error_count());
    if (e > r.max_abs_err) r.max_abs_err = e;
    const int u = h.ax.pwm_permille();
    if (u < r.out_min) r.out_min = u;
    if (u > r.out_max) r.out_max = u;
    sum += u;
    ++n;
    if (h.ax.supervisor_inhibited()) r.inhibited = true;
  }
  r.out_mean = n ? static_cast<double>(sum) / n : 0.0;
  r.fault = h.ax.state() == st::FAULT;
  return r;
}

// 実測 2: 乗員の荷重（6 % 未満では沈む、静止摩擦なし / 1.5 %）の下で 30 s 保持。出力は保持 Duty（≈ 60）に収まり、位置は ±2、
// パルスの往復なし、停止・FAULT なし。無負荷（静止摩擦 3 %）なら出力はほぼ 0。学習 OFF（r6〜r9）では往復が出る（14:43 の再現）
inline void test_r10_hold_learns_holding_duty_against_backdrive() {
  for (int deadband : {0, 15}) {
    Harness h;
    const HoldResult r = hold_run(h, 60, deadband, 30);
    if (std::getenv("R10_PRINT")) {
      std::printf("hold load 60 db %d: err max %d out %d..%d mean %.1f pulses %d inhibited %d fault %d I %d\n", deadband,
                  r.max_abs_err, r.out_min, r.out_max, r.out_mean, r.pulse_onsets, r.inhibited ? 1 : 0, r.fault ? 1 : 0,
                  h.ax.integral_permille());
    }
    CHECK(!r.fault && !r.inhibited && no_fault_events(h) && h.ax.state() == st::ACTIVE);
    CHECK(r.max_abs_err <= 2);
    CHECK(r.pulse_onsets == 0);
    // 保持 Duty（静止摩擦があればその帯 60 ± deadband のどこでも保てる）に収まる。出力が動くのは静止摩擦の帯の中だけ（軸は動かない）
    CHECK(std::abs(r.out_mean - 60.0) <= deadband + 5.0 && r.out_max - r.out_min <= 2 * deadband + 10);
  }
  {  // 無負荷・静止摩擦 3 %: 出力はほぼ 0 で止まっている
    Harness h;
    const HoldResult r = hold_run(h, 0, 30, 10);
    if (std::getenv("R10_PRINT")) std::printf("hold unloaded: err max %d out %d..%d mean %.1f pulses %d\n", r.max_abs_err, r.out_min, r.out_max, r.out_mean, r.pulse_onsets);
    CHECK(!r.fault && !r.inhibited && r.max_abs_err <= 2 && std::abs(r.out_min) <= 30 && std::abs(r.out_max) <= 30 && r.pulse_onsets == 0);
  }
  {  // 学習 OFF（r6〜r9 の保持）: 荷重 6 % で往復（パルスが繰り返す）か FAULT
    Harness h;
    h.ax.tuning.supervisor = true;
    const HoldResult r = hold_run(h, 60, 15, 10, false);
    if (std::getenv("R10_PRINT")) std::printf("hold no-learn: err max %d pulses %d fault %d inhibited %d\n", r.max_abs_err, r.pulse_onsets, r.fault ? 1 : 0, r.inhibited ? 1 : 0);
    CHECK(r.pulse_onsets >= 3 || r.fault || r.inhibited || r.max_abs_err > 4);
  }
}

// 保持出力の学習で追従（正弦波・反転）の誤差が悪くならないこと。荷重 6 % と無負荷で、学習 ON / OFF の FE を比べる（PR #50 のコメントの値）。
// r9 の遅れ・慣性のあるプラント（100 count/s の上り）でも比べる
inline void test_r10_hold_learn_tracking_fe_before_after() {
  for (int k = 0; k < 2; ++k) {
    const double cog = k == 0 ? 5.0 : 8.0, period = k == 0 ? 30.0 : 40.0, tau = k == 0 ? 0.050 : 0.080;
    const R9PlantResult off = r9_plant_run(V1_VEL_KD_PERMILLE_PER_COUNT_S, cog, period, tau, false);
    const R9PlantResult on = r9_plant_run(V1_VEL_KD_PERMILLE_PER_COUNT_S, cog, period, tau, true);
    if (std::getenv("R10_PRINT")) {
      std::printf("r9 plant cog %.0f: fe_rms off %.2f on %.2f / vel_err off %.1f on %.1f / rev off %d on %d\n", cog, off.fe_rms, on.fe_rms,
                  off.vel_err_rms, on.vel_err_rms, off.reversals, on.reversals);
    }
    CHECK(on.active && on.fe_rms <= off.fe_rms * 1.15 + 0.2 && on.reversals == 0);
  }
  for (int load : {0, 60}) {
    double rms[2] = {0, 0};
    int fe_max[2] = {0, 0};
    double stop_over[2] = {0, 0};
    for (int learn = 0; learn < 2; ++learn) {
      Harness h;
      r10_active(h, 0, 100000, 30, 20, true, false);  // 追従の比較（静止の区間で期限が先に来ないように切る）
      h.ax.tuning.hold_learn = learn != 0;
      h.pwm_count_s_per_permille = 4.0;
      h.load_permille = load;
      h.pwm_deadband_permille = 15;
      h.pwm_deadband_subtract = true;
      h.pos_max_vel = 1000;
      drive_count_nonzero(h, 2000, 0);
      double se = 0.0;
      int n = 0;
      const double pi = 3.14159265358979323846;
      for (int i = 0; i < 600; ++i) {  // 6 s: 振幅 300・周期 2 s の正弦波（最大 940 count/s… ではなく 300 × π ≈ 940 → 遅い側に 150 で）
        const double tgt = 150.0 * sin(2.0 * pi * i / 200.0);
        h.set_position(static_cast<int32_t>(lround(tgt)));
        h.idle(10);
        if (i >= 100) {
          const int fe = std::abs(h.ax.actual_count() - static_cast<int32_t>(lround(tgt)));
          se += static_cast<double>(fe) * fe;
          ++n;
          if (fe > fe_max[learn]) fe_max[learn] = fe;
        }
      }
      rms[learn] = sqrt(se / n);
      // 反転の後に止める: 下りの途中から 0 に止めたときの行き過ぎ
      for (int i = 0; i < 100; ++i) { h.set_position(-150 + i * 3); h.idle(10); }
      double over = 0.0;
      for (int i = 0; i < 200; ++i) {
        h.set_position(150);
        h.idle(10);
        const double o = static_cast<double>(h.ax.actual_count()) - 150.0;
        if (o > over) over = o;
      }
      stop_over[learn] = over;
      CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && !h.ax.supervisor_inhibited());
    }
    if (std::getenv("R10_PRINT")) {
      std::printf("tracking load %d: FE rms off %.2f on %.2f / max off %d on %d / stop overshoot off %.0f on %.0f\n", load, rms[0],
                  rms[1], fe_max[0], fe_max[1], stop_over[0], stop_over[1]);
    }
    CHECK(rms[1] <= rms[0] * 1.15 + 0.5);
    CHECK(stop_over[1] <= stop_over[0] + 3.0);
  }
}

// B（学習 OFF のときだけ）: 保持の近くで残り誤差 4（下向き）、プラントは 6 %（下向きのパルス 60）では動かない（#2 の 14:22）: パルスは 3 回まで、
// その後は出力 0（受け入れ）。学習 ON では使わない（乗員の荷重で沈むので保持出力を出し続ける。実測 2）
inline void test_r10_residual_pulses_limited_then_zero_output_without_learning() {
  Harness h;
  r10_active(h, 0, 200);
  h.ax.tuning.hold_learn = false;
  h.pwm_deadband_permille = 60;
  const int32_t start = h.ax.actual_count();
  int onsets = 0;
  bool prev_brk = false;
  int nonzero_after_accept = 0;
  int nonzero_non_pulse = 0;
  for (int i = 0; i < 600; ++i) {  // 3 s
    if ((i % 2) == 0) h.set_position(start - 4);
    h.step();
    const bool brk = h.ax.breakaway_active();
    if (brk && !prev_brk) ++onsets;
    prev_brk = brk;
    if (h.ax.residual_accepted() && h.ax.pwm_permille() != 0) ++nonzero_after_accept;
    if (!brk && h.ax.pwm_permille() != 0 && h.ax.duty_terms().vel_count_s == 0) ++nonzero_non_pulse;
  }
  if (std::getenv("R10_PRINT")) std::printf("residual: onsets %d accepted %d\n", onsets, h.ax.residual_accepted() ? 1 : 0);
  CHECK(onsets >= 1 && onsets <= V1_RESIDUAL_MAX_PULSES);
  CHECK(h.ax.residual_accepted() && nonzero_after_accept == 0 && nonzero_non_pulse == 0 && h.ax.pwm_permille() == 0);
  CHECK(h.ax.following_error_count() == 4 && h.ax.state() == st::ACTIVE && no_fault_events(h) && !h.ax.supervisor_inhibited());
  h.pwm_deadband_permille = 0;
  drive_count_nonzero(h, 800, start - 300);
  CHECK(!h.ax.residual_accepted() && std::abs(h.ax.actual_count() - (start - 300)) <= V1_RESIDUAL_BAND_COUNT);
}

// r10 統合（PR #48 × #49）: 端の余裕の一元化。再生バッファの参照が端へ向かうと、参照はソフトリミットから V1_SOFT_RESERVE_COUNT（= 400 − 50 + 250
// = 600）の手前で止まり、そこは監視の「端へ出せない帯」（機械端から 400）の外に 250 count の予算を残す所。PWM で動くプラント（エンコーダは
// 生きている）で ±600 count/s のランプを端の先まで送っても、停止も FAULT も無く参照の止まる所に着く。#49 の元の余裕（100）なら帯の中まで
// 進もうとして停止（ブレーキ）になり、余裕 100（450）でも寄せの終わりで止まる
inline void test_r10_integrated_playout_stops_outside_no_drive_zone() {
  CHECK(V1_SOFT_RESERVE_COUNT == V1_SUP_END_RESERVE_COUNT - V1_MECH_DECEL_ZONE_COUNT + V1_SUP_END_MARGIN_COUNT);
  CHECK(V1_SOFT_RESERVE_COUNT == 600 && V1_SOFT_BRAKE_ACC_COUNT_S2 == static_cast<double>(V1_BRAKE_GUARD_ACC_COUNT_S2));
  for (int dir : {1, -1}) {
    for (int32_t reserve : {V1_SOFT_RESERVE_COUNT, int32_t{100}, int32_t{450}}) {
      Harness h;
      r10_active(h, 0, 400, 30, 20, true, false);  // 端の余裕の一元化を見る（着いた後の静止の保持の期限は test_r10_deadline_*）
      h.ax.tuning.soft_reserve_count = reserve;
      h.ax.tuning.playout_delay_ms = 40;  // 再生バッファの参照（-DV1_PLAYOUT=0 の構成でも明示的に使う。端の手前の減速はこの経路だけ）
      h.pos_max_vel = 600;
      double tgt = 0.0;
      bool guard_seen = false;
      for (int i = 0; i < 700; ++i) {  // 7 s: 600 count/s で ±3000 まで（ソフトリミット ±2700 の先）、その後は止めて保持
        if (std::fabs(tgt) < 3000.0) tgt += dir * 6.0;
        h.set_position(static_cast<int32_t>(tgt));
        h.idle(10);
        if (h.ax.supervisor().end_braking()) guard_seen = true;
        if (h.ax.supervisor_inhibited() || h.ax.state() == st::FAULT) break;
      }
      const v1::Config c = base_config();
      const int32_t stop_at = dir > 0 ? c.soft_max_count - reserve : c.soft_min_count + reserve;
      if (std::getenv("R10_PRINT")) {
        std::printf("integrated dir %+d reserve %d: actual %d target %d inhibited %d fault %u guard %d budget %d/%d\n", dir, reserve,
                    h.ax.actual_count(), h.ax.target_count(), h.ax.supervisor_inhibited() ? 1 : 0, h.ax.fault_code(),
                    guard_seen ? 1 : 0, h.ax.supervisor_info().budget_up, h.ax.supervisor_info().budget_down);
      }
      if (reserve != 100) {  // 600（既定）と 450: 帯の外で止まり、寄せの終わり（止まりかけ）の小さな出力は積まないので着く
        CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && !h.ax.supervisor_inhibited() && !guard_seen);
        const V1SupInfo info = h.ax.supervisor_info();
        if (reserve == V1_SOFT_RESERVE_COUNT) CHECK((dir > 0 ? info.budget_up : info.budget_down) >= 100);  // 着いた後も 100 以上残る
        CHECK(std::abs(h.ax.target_count() - stop_at) <= 1 && std::abs(h.ax.actual_count() - stop_at) <= V1_RESIDUAL_BAND_COUNT);
      } else {
        // 100（#49 の元の値）: 帯の中へ進もうとして止まる
        CHECK(h.ax.supervisor_inhibited() && h.ax.supervisor().feedback_uncertain());
      }
    }
  }
}


// --- PR #50 の Codex 事前レビュー（b6a93fb）: P0 期限・P1a bias と追従の I・P1b 上書き中の凍結・P1c 認定 -------------------------------

// P0 単体（駐車の決定の後）: エンコーダが止まったまま（確かな動きなし）、PWM 0 でも下向きの逆回り（200 count/s）を露出として積む。
// 駐車できない（park_ok = false）なら余裕の全部で停止して基準を消すよう求める。駐車できる（park_ok）なら余裕の半分で駐車を求める（停止しない）。
// ±1 のちらつきでは戻らず、下へ 2 count の確かな動きで下の露出が戻る
inline void test_r10_deadline_unit() {
  auto rig = [](V1Supervisor& s, V1Supervisor::Ctx& c) {
    c.soft_min = -2700;
    c.soft_max = 2700;
    c.duty_max_pm = 200;
    c.hold_exit = 4;
    c.brake_released = true;
    s.t.brake_acc_count_s2 = 0;
    s.reset_trust(0, 0);
    s.end_session(0, 0);
  };
  {  // 駐車できない: 余裕の全部（中央 2350、200 count/s → 11.75 s）で停止 ＋ 基準を消す要求
    V1Supervisor s;
    V1Supervisor::Ctx c;
    rig(s, c);
    int ticks = 0;
    while (!s.feedback_uncertain() && ticks < 4000) {
      c.now_us += 5 * MS;
      s.observe(c);
      s.filter(c, 0);
      ++ticks;
    }
    CHECK(s.feedback_uncertain() && s.deadline_tripped() && s.take_reference_lost() && !s.take_park_request());
    CHECK(fabs(ticks * 0.005 - (2350.0 / 200.0 - 0.015)) <= 0.011 && s.exposure(-1) <= 2350.0 + 1e-6 && s.exposure(1) == 0.0);
    // DISARMED → 再 ARM でも露出は戻らない（PR #50 の Codex 事前レビュー e6ee0ef P1a）: 出力 0（離れる向きでない）ならすぐまた止まる
    s.end_session(0, c.now_us);
    s.clear_episode();
    CHECK(!s.feedback_uncertain() && s.exposure(-1) > 2300.0);
    c.now_us += 5 * MS;
    s.observe(c);
    s.filter(c, 0);
    CHECK(s.feedback_uncertain());
    // 離れる向き（上へ 10 %）の出力なら、下の露出が尽きていても止めない
    s.end_session(0, c.now_us);
    s.clear_episode();
    c.now_us += 5 * MS;
    s.observe(c);
    CHECK(s.filter(c, 100) == 100 && !s.feedback_uncertain());
  }
  {  // 駐車できる: 余裕の半分（5.875 s）で駐車を求める。停止はしない
    V1Supervisor s;
    V1Supervisor::Ctx c;
    rig(s, c);
    c.park_ok = true;
    int ticks = 0;
    bool req = false;
    while (!req && ticks < 4000) {
      c.now_us += 5 * MS;
      s.observe(c);
      s.filter(c, 0);
      req = s.take_park_request();
      ++ticks;
    }
    CHECK(req && !s.feedback_uncertain() && fabs(ticks * 0.005 - (1175.0 / 200.0 - 0.015)) <= 0.011);
  }
  {  // ±1 のちらつき・時間の経過では戻らない。下へ 2 count の確かな動きで下の露出が戻る
    V1Supervisor q;
    V1Supervisor::Ctx d;
    rig(q, d);
    q.t.deadline = false;
    for (int i = 0; i < 400; ++i) {
      d.now_us += 5 * MS;
      d.raw = i % 2;
      q.observe(d);
      q.filter(d, 0);
    }
    CHECK(q.exposure(-1) >= 400 * 0.005 * 200.0 - 2.0);
    d.raw = -2;
    d.now_us += 5 * MS;
    q.observe(d);
    CHECK(q.exposure(-1) == 0.0);
  }
}

// P0 軸: 荷重で沈む軸（外力 −150 count/s）が保持中にエンコーダを失う。駐車なし: 期限（余裕の全部）で PWM 0 ＋ ブレーキ・基準を消す。
// 見えない走行は余裕以下。駐車あり: 静止の窓（500 ms）で駐車してブレーキが沈みを止める
inline void test_r10_deadline_bounds_dead_encoder_hold() {
  {  // 駐車なし
    Harness h;
    r10_active(h, 0, 100000, 30, 20, true, false);
    h.pwm_count_s_per_permille = 4.0;
    drive_count_nonzero(h, 500, 0);
    h.ax.tuning.sup.deadline = true;
    h.enc_connected = false;
    h.ext_vel = -150.0;
    const double phys0 = h.phys;
    int n = 0;
    while (!h.ax.supervisor_inhibited() && n < 4000) { drive_count_nonzero(h, 5, 0); ++n; }
    const double blind = fabs(h.phys - phys0);
    if (std::getenv("R10_PRINT")) std::printf("deadline sag (no park): %.2f s blind %.1f\n", n * 0.005, blind);
    CHECK(braked_inhibit(h) && h.ax.supervisor().deadline_tripped() && !h.ax.referenced() && no_fault_events(h));
    CHECK(blind <= 2750.0 - 400.0);
  }
  {  // 駐車あり: 沈みはブレーキが止める（ブレーキが掴むまでの間だけ）
    Harness h;
    r10_active(h, 0, 100000, 30, 20, true, true, true);
    h.pwm_count_s_per_permille = 4.0;
    drive_count_nonzero(h, 300, 0);
    h.enc_connected = false;
    h.ext_vel = -150.0;
    const double phys0 = h.phys;
    drive_count_nonzero(h, 5000, 0);
    const double blind = fabs(h.phys - phys0);
    if (std::getenv("R10_PRINT")) std::printf("deadline sag (park): parked %d blind %.1f\n", h.ax.brake_parked() ? 1 : 0, blind);
    CHECK(h.ax.brake_parked() && !h.ax.supervisor_inhibited() && h.ax.referenced() && no_fault_events(h));
    CHECK(blind <= 150.0 * 0.7);  // 静止の窓 500 ms ＋ 掴むまで（ブレーキの遅れ 0 の模型）
  }
}

// P0 軸（駐車なし）: 生きたエンコーダでも完全に静止した保持は、駐車が無ければ期限（余裕の全部）で停止して基準を消す。ARM_PREPARE は NOT_REFERENCED
inline void test_r10_deadline_static_hold_without_parking() {
  Harness h;
  r10_active(h, 0, 100000);
  int n = 0;
  while (!h.ax.supervisor_inhibited() && n < 6000) { drive_count_nonzero(h, 5, 0); ++n; }
  if (std::getenv("R10_PRINT")) std::printf("deadline static hold (no park): %.2f s\n", n * 0.005);
  CHECK(braked_inhibit(h) && h.ax.supervisor().deadline_tripped() && h.ax.state() == st::ACTIVE && !h.ax.referenced());
  Harness::accepted(h.disarm());
  h.idle(400);
  Harness::rejected(h.arm(), rs::NOT_REFERENCED);
}

// --- ブレーキ駐車（2026-11-02 の操作者の決定） ------------------------------------------------------------

struct ParkRun {
  int parks = 0;
  double first_park_s = -1.0;  // 静止してから駐車（ブレーキ指令）まで
  double max_sag = 0.0;        // 駐車中の物理位置のずれ
  bool inhibited = false;
  bool fault = false;
};

// 着座（荷重 6 %、沈む）・無負荷で 60 s 静止。ブレーキの遅れ brake_delay（brake_engage 150 ms 以下なら沈まない）
inline ParkRun park_still_run(Harness& h, int load, uint32_t brake_delay, uint32_t seconds) {
  r10_active(h, 0, 400, 30, 20, true, true, true);
  h.pwm_count_s_per_permille = 4.0;
  h.load_permille = load;
  h.pwm_deadband_permille = 15;
  h.pwm_deadband_subtract = true;
  h.brake_delay_ms = brake_delay;
  // 荷重の下で保持出力を学ぶ間は体重移動で確かな動きを出す（駐車しない）
  for (int i = 0; i < 300; ++i) {
    h.ext_vel = ((i / 30) % 2 == 0) ? 40.0 : -40.0;
    drive_count_nonzero(h, 10, 0);
  }
  h.ext_vel = 0.0;
  ParkRun r;
  const uint64_t t0 = h.now;
  double park_pos = 0.0;
  bool was = h.ax.brake_parked();
  if (was) park_pos = h.phys;
  for (uint32_t t = 0; t < seconds * 1000; t += 10) {
    drive_count_nonzero(h, 10, 0);
    const bool p = h.ax.brake_parked();
    if (p && !was) {
      ++r.parks;
      park_pos = h.phys;
      if (r.first_park_s < 0) r.first_park_s = (h.now - t0) / 1e6;
    }
    if (p) r.max_sag = std::max(r.max_sag, fabs(h.phys - park_pos));
    was = p;
    if (h.ax.supervisor_inhibited()) r.inhibited = true;
  }
  r.fault = h.ax.state() == st::FAULT || !no_fault_events(h);
  return r;
}

inline void test_r10_park_still_hold_seated_and_unloaded() {
  struct Case { int load; uint32_t delay; };
  for (const Case& k : {Case{60, 0}, Case{60, 100}, Case{60, 150}, Case{0, 100}}) {
    Harness h;
    const ParkRun r = park_still_run(h, k.load, k.delay, 60);
    if (std::getenv("R10_PRINT")) {
      std::printf("park still load %d brake delay %u: first park %.2f s parks %d sag %.2f inhibited %d fault %d bias %d\n", k.load,
                  k.delay, r.first_park_s, r.parks, r.max_sag, r.inhibited ? 1 : 0, r.fault ? 1 : 0, h.ax.bias_permille());
    }
    CHECK(r.first_park_s >= 0.0 && r.first_park_s <= 0.8);  // 落ち着いてから 500 ms（＋ 速度窓の遅れ）
    CHECK(r.parks == 1 && !r.inhibited && !r.fault && h.ax.brake_parked() && h.ax.power_on_cmd() && !h.ax.brake_release_cmd());
    CHECK((h.status().flags & fl::PARKED) != 0 && h.ax.state() == st::ACTIVE && h.ax.pwm_permille() == 0);
    CHECK(r.max_sag <= 1.0);  // ブレーキが掴むまで（遅れ ≤ 150 ms）は保持出力を出しているので沈まない
    if (k.load == 60) CHECK(std::abs(h.ax.bias_permille() - 60) <= 20);  // 学んだ bias は捨てない
  }
  {  // ブレーキの遅れ 300 ms > 掴むまでの 150 ms: 150 ms 分だけ沈む（荷重 6 % の不足分 × 4 count/s/permille）
    Harness h;
    const ParkRun r = park_still_run(h, 60, 300, 5);
    if (std::getenv("R10_PRINT")) std::printf("park still brake delay 300: sag %.1f parks %d first %.2f\n", r.max_sag, r.parks, r.first_park_s);
    CHECK(r.max_sag > 1.0 && r.max_sag <= 60.0 * 4.0 * 0.15 + 2.0 && !r.inhibited);
  }
}

// 駐車中に目標が動く: 保持出力を出してブレーキ解除 → 300 ms は参照を止めたまま → 参照が追いつく。遅れは 300 ms ＋ 追いつき
inline void test_r10_park_unpark_latency_and_tracking() {
  Harness h;
  r10_active(h, 0, 400, 30, 20, true, true, true);
  h.pwm_count_s_per_permille = 4.0;
  h.load_permille = 60;
  h.pos_max_vel = 300;
  drive_count_nonzero(h, 3000, 0);
  drive_count_nonzero(h, 7000, 0);
  CHECK(h.ax.brake_parked());
  const uint64_t t0 = h.now;
  double t_release = -1, t_move = -1;
  int32_t ref_at_release_end = 0;
  bool released_seen = false;
  int max_fe_after = 0;
  for (int i = 0; i < 300; ++i) {
    h.set_position(i < 2 ? 3 : 300);  // ±2 を超えて動く
    h.idle(10);
    if (t_release < 0 && h.ax.brake_release_cmd()) t_release = (h.now - t0) / 1e6;
    if (h.ax.unparking()) {
      released_seen = true;
      ref_at_release_end = h.ax.target_count();
    }
    if (t_move < 0 && h.ax.actual_count() >= 5) t_move = (h.now - t0) / 1e6;
    if (t_move >= 0) max_fe_after = std::max(max_fe_after, std::abs(h.ax.following_error_count()));
  }
  if (std::getenv("R10_PRINT")) std::printf("unpark: release %.3f s, motion %.3f s, ref during release %d, FE max %d, final %d\n", t_release, t_move, ref_at_release_end, max_fe_after, h.ax.actual_count());
  CHECK(released_seen && ref_at_release_end == 0);  // 解除待ちの間は参照を駐車した位置に止める
  CHECK(t_release >= 0.0 && t_release <= 0.03);
  CHECK(t_move >= 0.3 && t_move <= 0.45);
  CHECK(std::abs(h.ax.actual_count() - 300) <= 4 && !h.ax.supervisor_inhibited() && no_fault_events(h));
  CHECK(max_fe_after <= 120);
}

// 駐車中にエンコーダが死んでから目標が動く: 戻ってから動かそうとしても確かな動きが無いので、予算で PWM 0 ＋ ブレーキ（見えない走行 ≤ 1000）
inline void test_r10_park_unpark_with_dead_encoder() {
  Harness h;
  r10_active(h, 0, 100000, 30, 20, true, true, true);
  drive_count_nonzero(h, 2000, 0);
  CHECK(h.ax.brake_parked());
  h.enc_connected = false;
  const double phys0 = h.phys;
  for (int i = 0; i < 300 && !h.ax.supervisor_inhibited(); ++i) { h.set_position(800); h.idle(10); }
  const double blind = fabs(h.phys - phys0);
  if (std::getenv("R10_PRINT")) std::printf("unpark dead: blind %.1f\n", blind);
  CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain() && blind <= 1000.0 && no_fault_events(h));
}

// ゲームのような動き（正弦波、400 ms 以下の止まり）では駐車しない。±1 の揺れのある静止は 1 回だけ駐車して戻らない。
// ゆっくり流れる目標（0.5 count/s）は、駐車 → 2 count を超えたら戻る、を繰り返すが、間隔は静止の窓＋解除待ちより長い
inline void test_r10_park_hysteresis() {
  {  // 正弦波と 400 ms の止まり
    Harness h;
    r10_active(h, 0, 100000, 30, 20, true, true, true);
    const double pi = 3.14159265358979323846;
    int parks = 0;
    bool was = false;
    double tgt = 0;
    for (int i = 0; i < 1000; ++i) {  // 10 s: 1 s 動いて 0.4 s 止まる
      const int ph = i % 140;
      if (ph < 100) tgt = 200.0 * sin(2.0 * pi * i / 300.0);
      h.set_position(static_cast<int32_t>(lround(tgt)));
      h.idle(10);
      const bool p = h.ax.brake_parked();
      if (p && !was) ++parks;
      was = p;
    }
    if (std::getenv("R10_PRINT")) std::printf("park game-like: parks %d\n", parks);
    CHECK(parks == 0 && !h.ax.supervisor_inhibited());
  }
  {  // ±1 の揺れ
    Harness h;
    r10_active(h, 0, 100000, 30, 20, true, true, true);
    int parks = 0, unparks = 0;
    bool was = false;
    for (int i = 0; i < 2000; ++i) {  // 20 s
      h.set_position((i * 7 % 3) - 1);
      h.idle(10);
      const bool p = h.ax.brake_parked();
      if (p && !was) ++parks;
      if (!p && was) ++unparks;
      was = p;
    }
    if (std::getenv("R10_PRINT")) std::printf("park noise: parks %d unparks %d\n", parks, unparks);
    CHECK(parks == 1 && unparks == 0 && !h.ax.supervisor_inhibited());
  }
  {  // 0.5 count/s で流れる目標
    Harness h;
    r10_active(h, 0, 100000, 30, 20, true, true, true);
    int parks = 0;
    bool was = false;
    uint64_t last_park = 0, min_gap = UINT64_MAX;
    for (int i = 0; i < 3000; ++i) {  // 30 s
      h.set_position(static_cast<int32_t>(i * 0.005));
      h.idle(10);
      const bool p = h.ax.brake_parked();
      if (p && !was) {
        if (parks > 0) min_gap = std::min(min_gap, h.now - last_park);
        last_park = h.now;
        ++parks;
      }
      was = p;
    }
    if (std::getenv("R10_PRINT")) std::printf("park drift: parks %d min gap %.2f s\n", parks, min_gap == UINT64_MAX ? 0.0 : min_gap / 1e6);
    CHECK(parks >= 1 && parks <= 6 && (parks < 2 || min_gap >= 800 * MS) && !h.ax.supervisor_inhibited() && no_fault_events(h));
  }
}

// 期限からの駐車: 静止の窓を長くしても（駐車の時刻にしない）、期限の露出が半分に届いた所で停止ではなく駐車する
inline void test_r10_park_from_deadline_instead_of_inhibit() {
  Harness h;
  r10_active(h, 0, 100000, 30, 20, true, true, true);
  h.ax.tuning.park_still_ms = 100000;
  int n = 0;
  while (!h.ax.brake_parked() && !h.ax.supervisor_inhibited() && n < 4000) { drive_count_nonzero(h, 5, 0); ++n; }
  if (std::getenv("R10_PRINT")) std::printf("park from deadline: %.2f s\n", n * 0.005);
  CHECK(h.ax.brake_parked() && !h.ax.supervisor_inhibited() && h.ax.referenced());
}

// 駐車中の STOP / DISARM / FAULT は従来どおり（ブレーキはどのみち作動）
inline void test_r10_park_stop_disarm_fault() {
  Harness h;
  r10_active(h, 0, 100000, 30, 20, true, true, true);
  drive_count_nonzero(h, 1500, 0);
  CHECK(h.ax.brake_parked());
  Harness::accepted(h.stop());
  h.drive(200, 0);
  CHECK(h.ax.state() == st::HOLDING && !h.ax.brake_release_cmd());
  Harness::accepted(h.disarm());
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED && !h.ax.brake_parked() && (h.status().flags & fl::PARKED) == 0);
  Harness g;
  r10_active(g, 0, 100000, 30, 20, true, true, true);
  drive_count_nonzero(g, 1500, 0);
  CHECK(g.ax.brake_parked());
  g.host_fault();
  g.step();
  CHECK(g.ax.state() == st::FAULT && !g.ax.brake_parked() && !g.ax.brake_release_cmd());
}

// P1a: bias（8 %）を学んだ後に反転・大きな目標の跳び。追従の I は反転で 0 になり、古い向きの I が残る tick は無い。FE の山を出す
inline void test_r10_bias_and_tracking_i_separate_on_reversal() {
  Harness h;
  r10_active(h, 0, 100000, 30, 20, true, false);
  h.pwm_count_s_per_permille = 4.0;
  h.load_permille = 80;
  h.pwm_deadband_permille = 15;
  h.pwm_deadband_subtract = true;
  drive_count_nonzero(h, 3000, 0);
  const int bias0 = h.ax.bias_permille();
  CHECK(std::abs(bias0 - 80) <= 20 && std::abs(h.ax.integral_permille()) <= 1);
  h.pos_max_vel = 300;
  int stale = 0, fe_peak = 0;
  int prev_dir = 0;
  for (int i = 0; i < 300; ++i) {  // 上へ 1 s、下へ 1 s、上へ 1 s（反転 2 回）
    const int32_t tgt = i < 100 ? 600 : (i < 200 ? -600 : 600);
    h.set_position(tgt);
    h.idle(10);
    const double v = h.ax.setpoint_vel();
    const int vd = v > 0 ? 1 : (v < 0 ? -1 : 0);
    const int ip = h.ax.integral_permille();
    if (vd != 0 && prev_dir != 0 && vd == prev_dir && ip != 0 && (ip > 0) != (vd > 0) && std::abs(ip) > 5) ++stale;
    if (vd != 0) prev_dir = vd;
    fe_peak = std::max(fe_peak, std::abs(h.ax.following_error_count()));
    CHECK(std::abs(ip) <= V1_I_MAX_PERMILLE);  // 追従の I は 3 % まで
  }
  if (std::getenv("R10_PRINT")) std::printf("bias reversal: bias %d -> %d, stale I ticks %d, FE peak %d\n", bias0, h.ax.bias_permille(), stale, fe_peak);
  CHECK(h.ax.bias_permille() == bias0);  // 走行中は bias を動かさない
  CHECK(stale == 0 && h.ax.state() == st::ACTIVE && no_fault_events(h));
}

// P1b: 上書き（引き上げ・端の手前の制動）が出力を決めている間は bias も追従の I も積まない
inline void test_r10_integrators_frozen_while_overridden() {
  {  // 引き上げ: bias 80 を学んでから、静止摩擦 25 % で動かない軸の目標を 26 上へ
    Harness h;
    r10_active(h, 0, 100000, 30, 20, true, false);
    h.pwm_count_s_per_permille = 4.0;
    h.load_permille = 80;
    drive_count_nonzero(h, 3000, 0);
    const int b0 = h.ax.bias_permille();
    h.pwm_deadband_permille = 250;
    h.pwm_deadband_subtract = true;
    int esc_ticks = 0;
    int b_at_first_esc = 0;
    bool frozen = true;
    for (int i = 0; i < 300 && !h.ax.supervisor_inhibited(); ++i) {
      if ((i % 2) == 0) h.set_position(26);
      const double b_before = h.ax.supervisor_info().enabled ? h.ax.bias_permille() : 0;
      h.step();
      // 引き上げが出力を決めた tick（出力 = 引き上げの量）
      if (h.ax.escalation_permille() > 0 && h.ax.pwm_permille() == std::min(h.ax.escalation_permille(), 200)) {
        if (esc_ticks == 0) b_at_first_esc = static_cast<int>(b_before);
        ++esc_ticks;
        if (h.ax.bias_permille() != static_cast<int>(b_before)) frozen = false;
      }
    }
    if (std::getenv("R10_PRINT")) std::printf("freeze esc: bias %d -> %d at first esc, esc ticks %d\n", b0, b_at_first_esc, esc_ticks);
    CHECK(esc_ticks > 5 && frozen);
  }
  {  // 端の手前の制動: bias（荷重 6 % で学ぶ）を持ったまま制動に入って出る間、bias と I は変わらない
    Harness h;
    r10_active(h, 1800, 100000, 6, 20, true, false);  // 参照の端の手前の止まる位置（ソフトリミットから 600）より内側
    h.pwm_count_s_per_permille = 4.0;
    h.load_permille = 60;
    drive_count_nonzero(h, 3000, 1800);
    h.pwm_plant = false;  // ここからはカウントを手で動かす
    h.stall = true;
    const int b0 = h.ax.bias_permille();
    const int i0 = h.ax.integral_permille();
    bool frozen = true;
    int guard = 0;
    for (int i = 0; i < 30; ++i) {
      if (i < 12) h.raw += 8;  // 1600 count/s で上へ（停止距離 1280 > 残り 900）
      h.set_position(1800);
      const int bb = h.ax.bias_permille(), ib = h.ax.integral_permille();
      h.step();
      if (h.ax.supervisor().end_braking()) {
        ++guard;
        if (h.ax.bias_permille() != bb || h.ax.integral_permille() != ib) frozen = false;
      }
    }
    if (std::getenv("R10_PRINT")) std::printf("freeze guard: bias %d I %d guard ticks %d\n", b0, i0, guard);
    CHECK(guard >= 3 && frozen && b0 > 0);
  }
}

// P1c（実機 17d0282 の後）: 保持出力として認めるのは、確かな動きの後に落ち着いて（|誤差| ≤ 6・|実測速度| ≤ 10・位置 ±1）100 ms
// 保てたときの、実際に出していた出力。落ち着いている間は更新し続ける。動いている・目標から離れている間は認めない
inline void test_r10_hold_certification_requires_stable_window() {
  V1Supervisor s;
  V1Supervisor::Ctx c;
  c.soft_min = -2700;
  c.soft_max = 2700;
  c.duty_max_pm = 200;
  c.brake_released = true;
  s.t.deadline = false;
  s.t.brake_acc_count_s2 = 0;
  s.reset_trust(0, 0);
  s.end_session(0, 0);
  auto tick = [&](int64_t raw, int out, int64_t err) {
    c.now_us += 5 * MS;
    c.raw = raw;
    c.err = err;
    s.observe(c);
    s.filter(c, out);
  };
  tick(0, 10, 0);
  tick(2, 20, 0);  // 確かな動き: 認定待ち
  for (int i = 0; i < 40; ++i) tick(2, 30, 9);  // 誤差 9（目標の近くでない）: 認めない
  CHECK(s.verified_hold_permille() == 0.0);
  for (int i = 0; i < 18; ++i) tick(2 + (i % 2), 40, 1);  // 落ち着いて 90 ms: まだ
  CHECK(s.verified_hold_permille() == 0.0);
  for (int i = 0; i < 4; ++i) tick(2, 60, 1);  // 100 ms を越えた: 出していた出力（60）
  CHECK(s.verified_hold_permille() == 60.0);
  tick(2, 65, 1);  // 落ち着いている間は更新
  tick(2, 65, 1);
  CHECK(s.verified_hold_permille() == 65.0);
  tick(5, 90, 1);  // 動いた（±1 を越えた）: 取り直し、認定は前のまま
  tick(5, 95, 1);
  CHECK(s.verified_hold_permille() == 65.0);
}


// --- Codex 最終レビュー（e32838e）: 駐車の切り替わりの間の DISARM / STOP / FAULT・監視・保持出力の受け渡し ----------------------------

// 荷重 6 % の軸を駐車の入りかけ（engage = true）または戻りかけ（false）の先頭まで進める。入りかけなら target 0 のまま、戻りかけなら +50
inline void park_to_transition(Harness& h, bool engage) {
  r10_active(h, 0, 100000, 30, 20, true, true, true);
  h.pwm_count_s_per_permille = 4.0;
  h.load_permille = 60;
  h.pwm_deadband_permille = 15;  // 静止摩擦 1.5 %（保持できる出力の帯 45〜75）
  h.pwm_deadband_subtract = true;
  for (int i = 0; i < 300; ++i) {
    h.ext_vel = ((i / 30) % 2 == 0) ? 40.0 : -40.0;
    drive_count_nonzero(h, 10, 0);
  }
  h.ext_vel = 0.0;
  for (int i = 0; i < 400 && !h.ax.brake_parked(); ++i) {
    if ((i % 2) == 0) h.set_position(0);
    h.step();
  }
  CHECK(h.ax.brake_parked());
  if (engage) return;
  drive_count_nonzero(h, 400, 0);  // 入りかけを終えて駐車
  for (int i = 0; i < 20 && !h.ax.unparking(); ++i) {
    if ((i % 2) == 0) h.set_position(50);
    h.step();
  }
  CHECK(h.ax.unparking());
}

inline void test_r10_park_transitions_with_disarm_stop_fault() {
  for (bool engage : {true, false}) {
    for (uint32_t at_ms : engage ? std::vector<uint32_t>{5, 75, 140} : std::vector<uint32_t>{5, 100, 250}) {
      for (int cmd = 0; cmd < 3; ++cmd) {  // 0 = DISARM, 1 = STOP, 2 = FAULT
        Harness h;
        park_to_transition(h, engage);
        const int32_t tgt = engage ? 0 : 50;
        h.drive(at_ms, tgt);
        if (cmd == 0) Harness::accepted(h.disarm());
        if (cmd == 1) Harness::accepted(h.stop());
        if (cmd == 2) h.host_fault();
        bool brake_released_seen = false;
        int ticks = 0;
        for (; ticks < 200; ++ticks) {  // 1 s
          if ((ticks % 2) == 0) h.set_position(tgt);
          h.step();
          if (h.ax.brake_release_cmd()) brake_released_seen = true;
        }
        if (std::getenv("R10_PRINT")) {
          std::printf("park transition %s at %u ms cmd %d: state %u parked %d brake_rel_seen %d pwm %d\n", engage ? "engage" : "release",
                      at_ms, cmd, h.ax.state(), h.ax.brake_parked() ? 1 : 0, brake_released_seen ? 1 : 0, h.ax.pwm_permille());
        }
        CHECK(!brake_released_seen);  // DISARM / STOP / FAULT の後にブレーキを解除し直さない
        if (cmd == 0) CHECK(h.ax.state() == st::DISARMED && !h.ax.brake_parked() && h.ax.pwm_permille() == 0);
        if (cmd == 1) CHECK(h.ax.state() == st::HOLDING && h.ax.brake_parked() && h.ax.pwm_permille() == 0);
        if (cmd == 2) CHECK(h.ax.state() == st::FAULT && !h.ax.brake_parked() && h.ax.pwm_permille() == 0);
      }
    }
  }
}

// 入りかけ（掴むまで）と戻りかけは監視に通す: 露出（逆回り）を積み、駐車中（PWM 0・作動）は積まない
inline void test_r10_park_transitions_accounted() {
  Harness h;
  park_to_transition(h, true);
  const double e0 = h.ax.supervisor().exposure(-1);
  h.drive(100, 0);  // 入りかけ（150 ms の中）
  const double e1 = h.ax.supervisor().exposure(-1);
  h.drive(300, 0);  // 駐車
  const double e2 = h.ax.supervisor().exposure(-1);
  h.drive(1000, 0);
  const double e3 = h.ax.supervisor().exposure(-1);
  if (std::getenv("R10_PRINT")) std::printf("park exposure: engage +%.1f, parked +%.1f\n", e1 - e0, e3 - e2);
  CHECK(e1 - e0 >= 200.0 * 0.09 && fabs(e3 - e2) < 1e-9);
}

// 戻りかけ（ブレーキ解除待ち、参照を止めている）の間は期限を見ない（動かないのが正しい。実機 27eaf6b）。戻った後は新しい窓で数え、
// 確かな動きが出るまでは残りの予算までに抑える。下の端の近く（下の余裕 120）で、期限の露出の半分で駐車してから戻っても止まらず目標に着く
inline void test_r10_release_wait_not_tripped_by_deadline() {
  Harness h;
  r10_active(h, -2230, 100000, 30, 20, true, true, true);  // 下の余裕 = −2230 − (−2750) − 400 = 120
  h.ax.tuning.soft_reserve_count = 0;
  for (int i = 0; i < 100 && !h.ax.brake_parked(); ++i) drive_count_nonzero(h, 10, -2230);
  drive_count_nonzero(h, 300, -2230);
  CHECK(h.ax.brake_parked() && !h.ax.unparking() && !h.ax.supervisor_inhibited());
  bool released = false, inhib_in_release = false;
  for (int i = 0; i < 200; ++i) {
    if ((i % 2) == 0) h.set_position(-2200);
    h.step();
    if (h.ax.unparking()) {
      released = true;
      if (h.ax.supervisor_inhibited()) inhib_in_release = true;
    }
  }
  if (std::getenv("R10_PRINT")) std::printf("release near end: released %d inhibited %d actual %d\n", released ? 1 : 0, h.ax.supervisor_inhibited() ? 1 : 0, h.ax.actual_count());
  CHECK(released && !inhib_in_release && !h.ax.supervisor_inhibited() && no_fault_events(h));
  CHECK(std::abs(h.ax.actual_count() + 2200) <= 4);
}

// 保持出力の受け渡し: 戻るときは駐車に入ったときの保持出力を出してから（その次の tick で）ブレーキを解除する。荷重で沈む軸で解除の間に沈まない
inline void test_r10_unpark_handoff_no_sag() {
  Harness h;
  park_to_transition(h, true);
  const int hold_at_entry = h.ax.pwm_permille();
  drive_count_nonzero(h, 400, 0);
  CHECK(h.ax.brake_parked() && h.ax.pwm_permille() == 0);
  const double pos0 = h.phys;
  int out_before_release = 0;
  bool prev_brake = false;
  double min_pos = pos0;
  int i = 0;
  for (; i < 80; ++i) {  // 400 ms
    if ((i % 2) == 0) h.set_position(50);
    h.step();
    if (!prev_brake && h.ax.brake_release_cmd()) out_before_release = h.ax.pwm_permille();
    prev_brake = h.ax.brake_release_cmd();
    if (h.ax.unparking()) min_pos = std::min(min_pos, h.phys);
  }
  if (std::getenv("R10_PRINT")) std::printf("handoff: hold at entry %d, output at release %d, min pos %.2f (parked %.2f)\n", hold_at_entry, out_before_release, min_pos, pos0);
  CHECK(hold_at_entry > 30 && out_before_release == hold_at_entry);
  CHECK(min_pos >= pos0 - 1.0);
}


// 実機 27eaf6b の再現（#2）: 駐車中に 9 count ずれた（ブレーキの掴みで off-target）軸に、目標が 700 count の段（max_vel 1200 /
// max_acc 6000、再生バッファ）で来る。戻りかけ（参照を止めている）の小さな保持出力を一律の包絡線で積まれて 0.3 s で停止し、参照だけ
// 進んで FOLLOWING_ERROR になった。いまは戻って追従し、停止も FAULT も無い
inline void test_r10_park_offtarget_then_large_step_reproduction() {
  for (int dir : {-1, 1}) {
    Harness h;
    r10_active(h, dir < 0 ? 1000 : -1000, 400, 30, 20, true, true, true);  // ソフトリミット ±2700 の中で 700 の段
    const int32_t base = h.ax.actual_count();
    h.pos_max_vel = 1200;
    h.pos_max_acc = 6000;
    // 実機の #2 は保持出力 −1 permille、それまでに予算をいくらか使っていた。ここでは保持出力を −10（上へ押す 1 % の荷重）にし、
    // 解除待ちを 450 ms に延ばして、修正前の仕組み（|誤差| 9 > 6 で一律 2500 count/s に積まれて 400 ms で予算切れ）が試験で見えるようにする
    h.load_permille = -10;
    h.ax.tuning.brake_release_wait_ms = 450;
    for (int i = 0; i < 200 && !h.ax.brake_parked(); ++i) drive_count_nonzero(h, 10, base);
    drive_count_nonzero(h, 400, base);
    CHECK(h.ax.brake_parked());
    h.phys += 9.0;  // 駐車中に 9 count ずれた
    h.raw = static_cast<int64_t>(floor(h.phys));
    drive_count_nonzero(h, 200, base);
    CHECK(h.ax.brake_parked() && std::abs(h.ax.following_error_count()) >= 8);
    const int32_t tgt = base + dir * 700;
    int fe_max = 0;
    bool inhib = false;
    for (int i = 0; i < 300; ++i) {
      h.set_position(tgt);
      h.idle(10);
      fe_max = std::max(fe_max, std::abs(h.ax.following_error_count()));
      if (h.ax.supervisor_inhibited()) inhib = true;
    }
    if (std::getenv("R10_PRINT")) std::printf("rig repro dir %+d: inhibited %d fault %u FE max %d final %d (target %d)\n", dir, inhib ? 1 : 0, h.ax.fault_code(), fe_max, h.ax.actual_count(), tgt);
    CHECK(!inhib && no_fault_events(h) && h.ax.state() == st::ACTIVE);
    CHECK(std::abs(h.ax.actual_count() - tgt) <= V1_RESIDUAL_BAND_COUNT && fe_max < 400);
  }
}

// 駐車に入るのは |誤差| ≤ 4 が窓の間ずっと続いたときだけ（誤差 9 のまま静止した軸は駐車しない）
inline void test_r10_park_requires_small_error_through_window() {
  Harness h;
  r10_active(h, 0, 100000, 30, 20, false, false, true);  // カウントは手で動かす（動かない）、期限なし
  h.raw += 9;
  bool parked = false;
  int budget_at_park = 1000;
  for (int i = 0; i < 300 && !h.ax.supervisor_inhibited() && !parked; ++i) {
    h.set_position(0);
    const V1SupInfo before = h.ax.supervisor_info();
    h.step();
    if (h.ax.brake_parked()) {
      parked = true;
      budget_at_park = std::min(before.budget_up, before.budget_down);
    }
  }
  // 静止の窓（|誤差| ≤ 4 が 500 ms）では入らない。誤差 9 のまま動けず予算が尽きたときだけ、停止ではなく駐車（参照が止まっている）
  CHECK(!parked || budget_at_park < 40);
}


// 実機 12712ac の FAIL 1: 1200 / 6000（再生バッファ、kp 30、duty_max 20）で各端の参照の止まる所（ソフトリミット ∓ 600）へ寄せて、
// 10 s 保持（駐車あり）。慣性（時定数 60 ms）・静止摩擦 3 %・重力（下向き 2 %）のある PWM プラント。停止も FAULT も無く、止まる所に着く
inline void test_r10_end_stop_settle_and_hold_both_ends() {
  for (int dir : {1, -1}) {
    Harness h;
    r10_active(h, 0, 400, 30, 20, true, true, true);
    h.pwm_count_s_per_permille = 12.0;
    h.plant_tau_s = 0.06;
    h.pwm_deadband_permille = 30;
    h.pwm_deadband_subtract = true;
    h.load_permille = 20;
    h.pos_max_vel = 1200;
    h.pos_max_acc = 6000;
    const v1::Config c = base_config();
    const int32_t stop_at = dir > 0 ? c.soft_max_count - V1_SOFT_RESERVE_COUNT : c.soft_min_count + V1_SOFT_RESERVE_COUNT;
    bool inhib = false;
    int parks = 0;
    bool was = false;
    for (int i = 0; i < 1300; ++i) {  // 寄せ 3 s ＋ 保持 10 s
      h.set_position(dir * 3000);
      h.idle(10);
      if (h.ax.supervisor_inhibited()) inhib = true;
      const bool p = h.ax.brake_parked();
      if (p && !was) ++parks;
      was = p;
    }
    if (std::getenv("R10_PRINT")) {
      std::printf("end stop dir %+d: actual %d (stop %d) inhibited %d fault %u parks %d budget %d/%d\n", dir, h.ax.actual_count(), stop_at,
                  inhib ? 1 : 0, h.ax.fault_code(), parks, h.ax.supervisor_info().budget_up, h.ax.supervisor_info().budget_down);
    }
    CHECK(!inhib && no_fault_events(h) && h.ax.state() == st::ACTIVE && h.ax.referenced());
    CHECK(std::abs(h.ax.actual_count() - stop_at) <= V1_RESIDUAL_BAND_COUNT + 4 && parks >= 1);
  }
}

// 実機 12712ac の FAIL 2 の再現: 下の端の近くで端の向き（下）の予算が尽きて停止 → 目標が上へ動いて FOLLOWING_ERROR → FAULT_RESET
//（基準は残る）→ ARM → START → 上（端から離れる向き）へ動かす。上の予算は独立で、端の向きの小さな補正は 0 にするだけなので、止まらずに動く
inline void test_r10_after_near_end_inhibit_can_drive_away() {
  Harness h;
  r10_active(h, -2100, 400, 30, 20, true, true, false);  // 下の参照の止まる所（下の余裕 250）。期限あり・駐車なし
  h.ax.tuning.soft_reserve_count = 0;
  // 動けないまま下へ 6 % ほど押させる（確かな動きなし）→ 下の予算（一律 2500 count/s）が期限（出力に比例 ＋ 逆回り）より先に尽きて
  // 停止。基準は残る（実機の flags 285 と同じ）
  h.held = true;
  for (int i = 0; i < 200 && !h.ax.supervisor_inhibited(); ++i) { h.set_position(-2120); h.idle(10); }
  CHECK(h.ax.supervisor_inhibited() && h.ax.supervisor().feedback_uncertain() && h.ax.referenced());
  h.held = false;
  for (int i = 0; i < 300 && h.ax.state() != st::FAULT; ++i) { h.set_position(-2100 + i * 4); h.idle(10); }  // 目標が上へ → FE
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::FOLLOWING_ERROR);
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.state() == st::DISARMED && h.ax.referenced());
  h.reach_holding_from_configured();
  const int32_t here = h.ax.actual_count();
  h.set_position(here);
  Harness::accepted(h.start());
  bool inhib = false;
  for (int i = 0; i < 200; ++i) {
    h.set_position(here + std::min(i * 4, 400));
    h.idle(10);
    if (h.ax.supervisor_inhibited()) inhib = true;
  }
  if (std::getenv("R10_PRINT")) {
    std::printf("drive away after inhibit: inhibited %d fault %u moved %d deadline %d env_cum down %.1f env_ep down %.1f budget %d/%d ref %d\n",
                inhib ? 1 : 0, h.ax.fault_code(), h.ax.actual_count() - here, h.ax.supervisor().deadline_tripped() ? 1 : 0,
                h.ax.supervisor().envelope_cumulative(-1), h.ax.supervisor().envelope_episode(-1), h.ax.supervisor_info().budget_up,
                h.ax.supervisor_info().budget_down, h.ax.referenced() ? 1 : 0);
  }
  CHECK(!inhib && h.ax.state() == st::ACTIVE && std::abs(h.ax.actual_count() - (here + 400)) <= 6);
}


// FAIL 1 の仕組み（寄せの終わり）: 下の参照の止まる所（下の余裕 250）へ 10 count/s の遅い参照で寄せ、軸は 100 ms に 1 count ずつ付いていく
//（確かな動き 2 count まで 200 ms）。参照速度が 0 になる前の小さな出力を一律の包絡線で積むと 200 ms × 2500 = 500 > 250 で停止していた。
// いまは止まりかけ（参照速度 ≤ 30・|誤差| ≤ 6）の小さな出力は積まないので止まらない
inline void test_r10_slow_final_approach_near_end_not_charged() {
  Harness h;
  r10_active(h, -2100, 100000, 30, 20, false, false, false);  // カウントは手で動かす。期限・駐車なし（予算だけを見る）
  h.ax.tuning.soft_reserve_count = 0;
  h.ax.tuning.playout_delay_ms = 0;  // 一定の遅い参照（従来の軌道）
  h.pos_max_vel = 10;
  bool inhib = false;
  for (int i = 0; i < 150; ++i) {
    h.set_position(-2110);
    h.idle(10);
    if ((i % 10) == 9 && h.ax.actual_count() > -2110) h.raw -= 1;  // 100 ms に 1 count 付いていく
    if (h.ax.supervisor_inhibited()) inhib = true;
  }
  if (std::getenv("R10_PRINT")) std::printf("slow approach near end: inhibited %d actual %d\n", inhib ? 1 : 0, h.ax.actual_count());
  CHECK(!inhib && h.ax.state() == st::ACTIVE);
}


// PR #50 の Codex 事前レビュー（e6ee0ef）P1a: エンコーダが止まったまま（断線）、ARM → 短い保持 → DISARM を繰り返しても、期限の露出は
// 戻らず積み上がる（ARMING の待ちの分も積む）。余裕 + arm_grace を超えたら止めて基準を消し、ARM_PREPARE は NOT_REFERENCED。
// 荷重で沈む（外力 −150 count/s）物理の移動は余裕 + arm_grace 以下
inline void test_r10_frozen_feedback_repeated_short_sessions() {
  Harness h;
  r10_active(h, -2000, 100000, 30, 20, true, true, true);  // 下の余裕 = −2000 − (−2750) − 400 = 350
  h.enc_connected = false;
  h.ext_vel = -150.0;
  const double phys0 = h.phys;
  double prev_exp = h.ax.supervisor().exposure(-1);
  bool monotonic = true;
  int sessions = 0;
  bool ref_lost = false;
  for (int k = 0; k < 40; ++k) {
    drive_count_nonzero(h, 200, -2000);  // 短い保持
    Harness::accepted(h.disarm());
    h.idle(400);
    const double e = h.ax.supervisor().exposure(-1);
    if (e + 1e-9 < prev_exp) monotonic = false;
    prev_exp = e;
    if (!h.ax.referenced()) { ref_lost = true; break; }
    const v1::Ack a = h.arm();
    if (a.result != ar::ACCEPTED) break;
    ++sessions;
    h.drive(620, -2000);
    if (h.ax.state() == st::HOLDING) {
      h.set_position(h.ax.actual_count());
      h.start();
    }
  }
  const double sag = fabs(h.phys - phys0);
  if (std::getenv("R10_PRINT")) std::printf("frozen short sessions: sessions %d ref_lost %d exposure %.1f sag %.1f monotonic %d\n", sessions, ref_lost ? 1 : 0, h.ax.supervisor().exposure(-1), sag, monotonic ? 1 : 0);
  CHECK(monotonic && ref_lost && sessions < 40);
  CHECK(sag <= 350.0 + V1_SUP_ARM_GRACE_COUNT + 1.0);
  Harness::rejected(h.arm(), rs::NOT_REFERENCED);
}

// PR #50 の Codex 事前レビュー（e6ee0ef）P1b: エンコーダが止まったまま、目標の小さな出入り（+3 / 0）で駐車と戻りを繰り返しても、期限の
// 露出は戻らない（駐車中は止まるだけ）。参照が止まっていれば期限は停止ではなく駐車（ブレーキが保つ）になり、参照が動いている間に
// 尽きれば止めて基準を消す。どちらにしても露出は余裕 + arm_grace 以内
inline void test_r10_frozen_feedback_park_cycles_do_not_renew() {
  Harness h;
  r10_active(h, 0, 100000, 30, 20, true, true, true);
  drive_count_nonzero(h, 1500, 0);
  CHECK(h.ax.brake_parked());
  h.enc_connected = false;
  double prev = h.ax.supervisor().exposure(-1);
  bool monotonic = true;
  int cycles = 0;
  for (; cycles < 60 && !h.ax.supervisor_inhibited(); ++cycles) {
    drive_count_nonzero(h, 1000, 3);
    drive_count_nonzero(h, 1000, 0);
    const double e = h.ax.supervisor().exposure(-1);
    if (e + 1e-9 < prev) monotonic = false;
    prev = e;
  }
  if (std::getenv("R10_PRINT")) std::printf("frozen park cycles: cycles %d inhibited %d exposure %.1f monotonic %d\n", cycles, h.ax.supervisor_inhibited() ? 1 : 0, h.ax.supervisor().exposure(-1), monotonic ? 1 : 0);
  CHECK(monotonic && (h.ax.supervisor_inhibited() || h.ax.brake_parked()));
  CHECK(h.ax.supervisor().exposure(-1) <= 2350.0 + V1_SUP_ARM_GRACE_COUNT + 13.0);
}


// --- 系統的な回帰（実機 17d0282 の FAIL: 上の端から 1350 の位置へ着いて約 0.7 s で FEEDBACK_UNCERTAIN・基準を消す）-------------------
// PWM で動くプラント（重力 0 / 3 / 6 %、静止摩擦 2.5 %、慣性 60 ms、Duty 1 permille あたり 10 count/s）、kp 30・duty_max 20・
// 1200 / 6000・再生バッファ・駐車・期限あり（焼く既定）。ストロークの端から端まで 300 count ごとの位置を順に回り、各位置で:
// 300〜1500 の段（上下交互）→ 落ち着いて駐車 → 10 s 保持 → 小さな段（+5）で戻る → 大きな段（−800 / +800）→ 元の位置へ戻る。
// どこでも停止（FEEDBACK_UNCERTAIN / LOAD_LIMITED）・基準を消す・FAULT が無いこと（参照は端の手前 600 で止まるので、端の帯には入らない）。
struct SweepStats {
  int cases = 0;
  int inhibits = 0;
  int faults = 0;
  bool ref_lost = false;
  int min_budget = 1000000;
  double max_exposure = 0.0;
  int parks = 0;
  int worst_case = -1;
  int32_t worst_pos = 0;
};

inline SweepStats sweep_session(int load_pm, bool print_cases) {
  Harness h;
  r10_active(h, 0, 400, 30, 20, true, true, true);
  h.pwm_count_s_per_permille = 10.0;
  h.plant_tau_s = 0.06;
  h.pwm_deadband_permille = 25;
  h.pwm_deadband_subtract = true;
  h.load_permille = load_pm;
  h.pos_max_vel = 1200;
  h.pos_max_acc = 6000;
  SweepStats st;
  bool was_parked = false;
  int case_min_budget = 1000000;
  double case_max_exp = 0.0;
  auto run = [&](int32_t target, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10) {
      h.set_position(target);
      h.idle(10);
      const V1SupInfo info = h.ax.supervisor_info();
      const int b = std::min(info.budget_up, info.budget_down);
      case_min_budget = std::min(case_min_budget, b);
      const double e = std::max(h.ax.supervisor().exposure(1), h.ax.supervisor().exposure(-1));
      case_max_exp = std::max(case_max_exp, e);
      const bool p = h.ax.brake_parked();
      if (p && !was_parked) ++st.parks;
      was_parked = p;
      if (h.ax.supervisor_inhibited() || h.ax.state() == st::FAULT) {
        if (std::getenv("R10_SWEEP_DEBUG")) {
          std::printf("    stop: target %d ref %d actual %d sv %.1f err %d dl %d fault %u exp up %.1f down %.1f budget %d/%d pwm %d unparking %d parked %d\n",
                      target, h.ax.target_count(), h.ax.actual_count(), h.ax.setpoint_vel(), h.ax.following_error_count(),
                      h.ax.supervisor().deadline_tripped() ? 1 : 0, h.ax.fault_code(), h.ax.supervisor().exposure(1),
                      h.ax.supervisor().exposure(-1), h.ax.supervisor_info().budget_up, h.ax.supervisor_info().budget_down,
                      h.ax.pwm_permille(), h.ax.unparking() ? 1 : 0, h.ax.brake_parked() ? 1 : 0);
        }
        return false;
      }
    }
    return true;
  };
  const int sizes[] = {300, 600, 900, 1200, 1500};
  int k = 0;
  for (int32_t pos = -2400; pos <= 2400; pos += 300, ++k) {
    for (int dir : {1, -1}) {
      case_min_budget = 1000000;
      case_max_exp = 0.0;
      const int size = sizes[(k * 2 + (dir > 0 ? 0 : 1)) % 5];
      const int32_t step = pos + dir * size;
      bool ok = run(pos, 2500);                    // 位置へ
      ok = ok && run(step, 3000);                  // 段 → 落ち着く
      ok = ok && run(step, 10000);                 // 10 s 保持（駐車）
      ok = ok && run(step + 5, 2000);              // 小さな段で戻る
      ok = ok && run(step + 5 - dir * 800, 3000);  // 大きな段
      ok = ok && run(pos, 3000);                   // 元へ
      ++st.cases;
      if (print_cases) {
        std::printf("  load %2d pos %+5d dir %+d size %4d: min budget %4d max exposure %7.1f %s\n", load_pm, pos, dir, size,
                    case_min_budget, case_max_exp, ok ? "" : "<-- STOP");
      }
      st.min_budget = std::min(st.min_budget, case_min_budget);
      st.max_exposure = std::max(st.max_exposure, case_max_exp);
      if (!ok) {
        if (h.ax.supervisor_inhibited()) ++st.inhibits;
        if (h.ax.state() == st::FAULT) ++st.faults;
        if (!h.ax.referenced()) st.ref_lost = true;
        if (st.worst_case < 0) { st.worst_case = st.cases; st.worst_pos = h.ax.actual_count(); }
        return st;
      }
    }
  }
  st.ref_lost = !h.ax.referenced();
  return st;
}

inline void test_r10_systematic_sweep_whole_stroke() {
  const bool print = std::getenv("R10_PRINT") != nullptr;
  for (int load : {0, 30, 60}) {
    const SweepStats st = sweep_session(load, print);
    if (print) {
      std::printf("sweep load %d: cases %d inhibits %d faults %d ref_lost %d parks %d min budget %d max exposure %.1f first stop case %d at %d\n",
                  load, st.cases, st.inhibits, st.faults, st.ref_lost ? 1 : 0, st.parks, st.min_budget, st.max_exposure, st.worst_case,
                  st.worst_pos);
    }
    CHECK(st.inhibits == 0 && st.faults == 0 && !st.ref_lost && st.cases == 34);
  }
}

// 実機 17d0282 の再現: 中立（0）から上の端の 1350 手前（機械端 2750 − 1350 = 1400）へ 1200 / 6000 で段、重力 3 / 6 %、落ち着いて駐車。停止しない
inline void test_r10_step_to_upper_mid_settle_and_park() {
  for (int load : {30, 60}) {
    Harness h;
    r10_active(h, 0, 400, 30, 20, true, true, true);
    h.pwm_count_s_per_permille = 10.0;
    h.plant_tau_s = 0.06;
    h.pwm_deadband_permille = 25;
    h.pwm_deadband_subtract = true;
    h.load_permille = load;
    h.pos_max_vel = 1200;
    h.pos_max_acc = 6000;
    drive_count_nonzero(h, 3000, 0);
    bool inhib = false;
    double max_exp = 0.0;
    for (int i = 0; i < 600 && !inhib; ++i) {
      h.set_position(1400);
      h.idle(10);
      inhib = h.ax.supervisor_inhibited();
      max_exp = std::max(max_exp, h.ax.supervisor().exposure(1));
    }
    if (std::getenv("R10_PRINT")) std::printf("step to upper mid load %d: inhibited %d ref %d parked %d actual %d max up exposure %.1f\n", load, inhib ? 1 : 0, h.ax.referenced() ? 1 : 0, h.ax.brake_parked() ? 1 : 0, h.ax.actual_count(), max_exp);
    CHECK(!inhib && h.ax.referenced() && h.ax.brake_parked() && std::abs(h.ax.actual_count() - 1400) <= 6);
  }
}

// STATUS の flags（bit 7〜10）。監視が OFF（従来の経路）では立たない。flags の変化（停止・ブレーキ作動）で即時 STATUS が出る
inline void test_r10_status_flags_bits() {
  CHECK(fl::LOAD_LIMITED == 0x80 && fl::FEEDBACK_UNCERTAIN == 0x100 && fl::END_BRAKING == 0x200 && fl::LIMIT_RECOVERY == 0x400);
  Harness h;
  r10_active(h, 0, 200);
  h.held = true;
  const size_t before = h.statuses().size();
  drive_count_nonzero(h, 500, 150);
  bool immediate = false;
  const std::vector<v1::Status> s = h.statuses();
  for (size_t i = before; i < s.size(); ++i) {
    if ((s[i].flags & fl::FEEDBACK_UNCERTAIN) && !(s[i].flags & fl::BRAKE_RELEASE_CMD) && (s[i].flags & fl::POWER_ON_CMD)) {
      immediate = true;
      break;
    }
  }
  CHECK(immediate);
  Harness k;  // 従来の経路
  Harness::accepted(k.config());
  CHECK((k.status().flags & SUP_FLAGS) == 0 && !k.ax.supervisor_info().enabled);
}

inline void run_all() {
  test_r10_conservative_defaults();
  test_r10_budget_derivation();
  test_r10_renewal_rules_unit();
  test_r10_recovery_scope_unit();
  test_r10_dead_encoder_mid_stroke_cut_within_budget();
  test_r10_dead_encoder_near_end_budgets();
  test_r10_irregular_ticks_stay_within_budget();
  test_r10_held_occupant_10s_no_encoder_fault();
  test_r10_resume_after_inhibit_discards_queued_targets();
  test_r10_gravity_drift_with_brake_vs_without();
  test_r10_external_push_away();
  test_r10_load_limited_brakes_and_stays_until_rearm();
  test_r10_old_stall_scenarios_never_latch_encoder_fault();
  test_r10_feedback_uncertain_requires_explicit_recovery();
  test_r10_inhibit_during_arming_keeps_brake_applied();
  test_r10_end_braking_guard();
  test_r10_encoder_loss_during_braking();
  test_r10_soft_limit_default_latch_and_grace_variant();
  test_r10_following_error_default_and_grace_variant();
  test_r10_known_hole_noisy_pm2_feedback_renews_budget();
  test_r10_escalation_lifts_axis_needing_15_percent();
  test_r10_escalation_to_duty_max_then_braked_inhibit();
  test_r10_escalation_cut_by_budget_when_encoder_dead();
  test_r10_hold_learns_holding_duty_against_backdrive();
  test_r10_hold_learn_tracking_fe_before_after();
  test_r10_residual_pulses_limited_then_zero_output_without_learning();
  test_r10_integrated_playout_stops_outside_no_drive_zone();
  test_r10_deadline_unit();
  test_r10_deadline_bounds_dead_encoder_hold();
  test_r10_deadline_static_hold_without_parking();
  test_r10_park_still_hold_seated_and_unloaded();
  test_r10_park_unpark_latency_and_tracking();
  test_r10_park_unpark_with_dead_encoder();
  test_r10_park_hysteresis();
  test_r10_park_from_deadline_instead_of_inhibit();
  test_r10_park_stop_disarm_fault();
  test_r10_bias_and_tracking_i_separate_on_reversal();
  test_r10_integrators_frozen_while_overridden();
  test_r10_hold_certification_requires_stable_window();
  test_r10_park_transitions_with_disarm_stop_fault();
  test_r10_park_transitions_accounted();
  test_r10_release_wait_not_tripped_by_deadline();
  test_r10_unpark_handoff_no_sag();
  test_r10_park_offtarget_then_large_step_reproduction();
  test_r10_park_requires_small_error_through_window();
  test_r10_end_stop_settle_and_hold_both_ends();
  test_r10_after_near_end_inhibit_can_drive_away();
  test_r10_slow_final_approach_near_end_not_charged();
  test_r10_frozen_feedback_repeated_short_sessions();
  test_r10_frozen_feedback_park_cycles_do_not_renew();
  test_r10_step_to_upper_mid_settle_and_park();
  test_r10_systematic_sweep_whole_stroke();
  test_r10_status_flags_bits();
}

}  // namespace v1r10_test
