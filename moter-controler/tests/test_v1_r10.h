// r10（Issue #44・#46）: 保護の監視 V1Supervisor のホストテスト。test_v1_axis.h の後に include する（Harness を使う）。
// プラントは Harness の pwm_plant（PWM で動く。物理位置 phys とエンコーダの値 raw を分け、enc_connected = false で断線。ブレーキの遅れ・
// 効かないブレーキ・重力／外力 ext_vel・乗員が押さえる held・荷重で遅い plant_max_vel）。
// 既定（PR #48 の Codex 事前レビュー後）: 包絡線は一律 2500 count/s、最小予算 0、先読みは最悪の tick 間隔 15 ms、SOFT_LIMIT / FOLLOWING_ERROR の
// 猶予は OFF、停止（FEEDBACK_UNCERTAIN / LOAD_LIMITED）は PWM 0 ＋ ブレーキ作動で DISARMED まで続く。
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
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
inline void r10_active(Harness& h, int32_t ref = 0, uint32_t fe_limit = 100000, int kp = 30, int duty_max = 20,
                       bool pwm_plant = true) {
  h.ax.tuning = r6_tuning(kp, duty_max);
  h.ax.tuning.supervisor = true;
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
}

// --- V1Supervisor 単体（数を厳密に見るため先読みは 5 ms） -------------------------

struct SupRig {
  V1Supervisor s;
  V1Supervisor::Ctx c;
  SupRig() {
    s.t.projection_ms = 5;
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

// 確かな動きだけが予算を戻す: ±1 のちらつき・片相のトグル・出力と逆向きの動き・PWM の隙間・出力の反転では戻らない
inline void test_r10_renewal_rules_unit() {
  auto ticks_until_inhibit = [](std::function<int64_t(int)> raw_at, std::function<int(int)> cmd_at, int* up, int* down) {
    SupRig r;
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
  CHECK(ticks_until_inhibit([](int i) { return static_cast<int64_t>(-i); }, [](int) { return 200; }, &up, &down) == 80);  // 逆向き
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
  CHECK(r.s.envelope_episode(1) == 0.0 && r.s.trusted_raw() == 2);
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
        r10_active(h, ref);
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
    r10_active(h, ref);
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
  for (int i = 0; i < 12; ++i) {  // 1200 count/s で上へ（停止距離 720 > 残り 300〜240）
    h.raw += 6;
    h.set_position(2400);
    h.step();
  }
  CHECK(h.ax.supervisor().end_braking() && h.ax.pwm_permille() == -200 && (h.status().flags & fl::END_BRAKING) != 0);
  CHECK(h.ax.supervisor().envelope_episode(-1) > 0.0);  // 制動の出力も積む
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


// --- 静止目標での引き上げ（A）と小さな残り誤差の受け入れ（B）。Issue #44 の 14:22 の実測 -------------------------------

// 静止で |誤差| 26（#1 が最下端で行き過ぎた量）の軸を、指定の Duty（静止摩擦）を越えないと動かないプラントで戻す。最大の出力・動いたか・結果を返す
struct EscResult {
  int max_out = 0;
  int32_t final_err = 0;
  bool moved = false;
  double blind = 0.0;
};
inline EscResult escalation_run(Harness& h, int deadband_pm, int32_t err0, uint32_t ms, bool disconnect = false,
                                double creep = 0.0) {
  r10_active(h, 0, 200);
  h.pwm_deadband_permille = deadband_pm;
  h.pwm_deadband_subtract = true;
  const int32_t start = h.ax.actual_count();
  const int32_t target = start + err0;
  drive_count_nonzero(h, 60, target);  // 軌道が目標に着く（プラントは静止摩擦で動かない）
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

// A: 15 % 要る軸。合成（P 78 + I）では動かないが、引き上げが 15 % を越えて動き、目標の近く（残り誤差の帯）に着く。停止も FAULT も無い
inline void test_r10_escalation_lifts_axis_needing_15_percent() {
  Harness h;
  const EscResult r = escalation_run(h, 149, 26, 2000);
  if (std::getenv("R10_PRINT")) std::printf("esc 15%%: max_out %d final_err %d moved %d\n", r.max_out, r.final_err, r.moved ? 1 : 0);
  CHECK(r.max_out >= 150 && r.moved && std::abs(r.final_err) <= V1_RESIDUAL_BAND_COUNT + 3);
  CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && !h.ax.supervisor_inhibited());
  CHECK(h.ax.escalation_permille() == 0);  // 着いたら戻る
  // 監視 OFF（r9 と同じ）では引き上げが無く、動けないまま静止 2 s の監視で ENCODER_FAULT（14:22 の #1 の再現）
  Harness k;
  r10_active(k, 0, 200);
  k.ax.tuning.supervisor = false;
  k.pwm_deadband_permille = 149;
  k.pwm_deadband_subtract = true;
  const int32_t t0 = k.ax.actual_count() + 26;
  int max_out = 0;
  for (int i = 0; i < 500 && k.ax.state() != st::FAULT; ++i) {
    if ((i % 2) == 0) k.set_position(t0);
    k.step();
    if (std::abs(k.ax.pwm_permille()) > max_out) max_out = std::abs(k.ax.pwm_permille());
  }
  if (std::getenv("R10_PRINT")) std::printf("esc off: fault %u max_out %d\n", k.ax.fault_code(), max_out);
  CHECK(k.ax.fault_code() == fc::ENCODER_FAULT);
}

// A: 25 % 要る軸（duty_max 20 % では動かない）。duty_max まで引き上げ、カウントが全く動かないので見えない走行の予算で PWM 0 ＋ ブレーキ。
// 少しずつ動く（15 count/s のにじみ）なら予算は戻り、duty_max の飽和 1 s で LOAD_LIMITED（ブレーキ）。どちらも ENCODER_FAULT にしない
inline void test_r10_escalation_to_duty_max_then_braked_inhibit() {
  Harness h;
  const EscResult r = escalation_run(h, 249, 26, 3000);
  CHECK(r.max_out == 200 && !r.moved);
  CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain() && no_fault_events(h));
  Harness g;
  const EscResult q = escalation_run(g, 249, 40, 3000, false, 15.0);
  if (std::getenv("R10_PRINT")) std::printf("esc 25%% creep: max_out %d final_err %d\n", q.max_out, q.final_err);
  CHECK(q.max_out == 200);
  CHECK(braked_inhibit(g) && g.ax.supervisor().load_limited() && !g.ax.supervisor().feedback_uncertain() && no_fault_events(g));
}

// A: 引き上げの途中でエンコーダが死ぬ（物理は 15 % を越えて動く）→ 引き上げは duty_max まで続くが、見えない走行の予算で止まる（物理の移動 ≤ 1000）
inline void test_r10_escalation_cut_by_budget_when_encoder_dead() {
  Harness h;
  const EscResult r = escalation_run(h, 149, 26, 3000, true);
  if (std::getenv("R10_PRINT")) std::printf("esc dead: max_out %d blind %.1f\n", r.max_out, r.blind);
  CHECK(r.max_out == 200 && r.blind > 0.0 && r.blind <= 1000.0);
  CHECK(braked_inhibit(h) && h.ax.supervisor().feedback_uncertain() && no_fault_events(h));
}

// B: 保持の近くで残り誤差 4（下向き）、プラントは 6 %（下向きのパルス 60）では動かない（#2 の 14:22）: パルスは 3 回まで、その後は出力 0 の
// まま（パルスの合間も 0。「4 で止まったまま P + I が流れる」r6 の振る舞いにならない）。|誤差| が 12 を超える（目標が動く）と解除して動く
inline void test_r10_residual_pulses_limited_then_zero_output() {
  Harness h;
  r10_active(h, 0, 200);
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
  // 目標が動けば解除（遠くへ）: 動き出す
  h.pwm_deadband_permille = 0;
  drive_count_nonzero(h, 800, start - 300);
  CHECK(!h.ax.residual_accepted() && std::abs(h.ax.actual_count() - (start - 300)) <= V1_RESIDUAL_BAND_COUNT);
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
  test_r10_residual_pulses_limited_then_zero_output();
  test_r10_status_flags_bits();
}

}  // namespace v1r10_test
