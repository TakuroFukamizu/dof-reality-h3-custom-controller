// r10（Issue #44・#46）: 保護の監視 V1Supervisor のホストテスト。test_v1_axis.h の後に include する（Harness を使う）。
// プラントは Harness の pwm_plant（PWM で動く。物理位置 phys とエンコーダの値 raw を分け、enc_connected = false で断線）。
#pragma once

#include <cmath>
#include <cstdlib>
#include <functional>
#include <cstdio>
#include <vector>

#include "test_v1_axis.h"
#include "v1_supervisor.h"

namespace v1r10_test {

using namespace v1axis_test;

constexpr uint16_t SUP_FLAGS = fl::LOAD_LIMITED | fl::FEEDBACK_UNCERTAIN | fl::END_BRAKING | fl::LIMIT_RECOVERY;

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
inline int drive_count_nonzero(Harness& h, uint32_t ms, int32_t target, int* up = nullptr, int* down = nullptr) {
  int n = 0;
  for (uint32_t t = 0; t < ms; t += 5) {
    if ((t % 10) == 0) h.set_position(target);
    h.step(5);
    const int u = h.ax.pwm_permille();
    if (u != 0) ++n;
    if (u > 0 && up) ++*up;
    if (u < 0 && down) ++*down;
  }
  return n;
}

inline bool no_fault_events(const Harness& h) { return h.fault_events().empty(); }

// --- V1Supervisor 単体 -------------------------------------------------------

struct SupRig {
  V1Supervisor s;
  V1Supervisor::Ctx c;
  explicit SupRig(int env_ref = 0) {
    s.t.env_ref_permille = env_ref;  // 既定は一律 2500（仕様どおりの数で見る）
    s.t.brake_acc_count_s2 = 0;      // 制動は別に見る
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

// 予算の式と位置の上限: 機械端 = ソフトリミット ± 50。D = 信用できた位置からの距離。min(1000, max(0, D − 400))、最小予算 50（D − 20 まで）
inline void test_r10_budget_derivation() {
  SupRig r;
  auto lim = [&](int64_t trusted, int d) {
    r.s.reset_trust(trusted, 0);
    return r.s.position_limit(d, r.c);
  };
  r.s.t.min_budget_count = 0;
  CHECK(lim(2100, 1) == 250);    // D = 2750 − 2100 = 650 → 250（2500 count/s で 100 ms）
  CHECK(lim(2225, 1) == 125);    // D = 525 → 125（50 ms）
  CHECK(lim(2350, 1) == 0);      // D = 400 → 0
  CHECK(lim(-2100, -1) == 250);  // 下も同じ
  CHECK(lim(0, 1) == 2350);      // 中央: 位置の上限は 2350。その回の予算は回の上限 1000 との小さい方
  r.s.t.min_budget_count = 50;
  CHECK(lim(2350, 1) == 50);     // 最小予算
  CHECK(lim(2700, 1) == 30);     // D = 50 → min(50, 50 − 20)
  CHECK(lim(2740, 1) == 0);      // D = 10 → 0（機械端から 20 以内へは出さない）
  CHECK(lim(2740, -1) == 5090);  // 反対向きは広い（位置の上限は 1000 で頭打ちにしない。回の上限が 1000）
}

// 確かな動きだけが予算を戻す: ±1 のちらつき・片相のトグル・出力と逆向きの動き・PWM の隙間・出力の反転では戻らない
inline void test_r10_renewal_rules_unit() {
  // 予算 1000 = 80 tick（2500 count/s × 5 ms = 12.5）
  auto ticks_until_inhibit = [](std::function<int64_t(int)> raw_at, std::function<int(int)> cmd_at, int* up, int* down) {
    SupRig r;
    int n = 0;
    for (int i = 0; i < 2000 && !r.s.feedback_uncertain(); ++i) {
      const int out = r.tick(cmd_at(i), raw_at(i));
      if (out > 0) ++*up;
      if (out < 0) ++*down;
      if (out != 0) ++n;
    }
    CHECK(r.s.feedback_uncertain());
    CHECK(r.s.envelope_episode(1) <= 1000.0 + 1e-9 && r.s.envelope_episode(-1) <= 1000.0 + 1e-9);
    return n;
  };
  int up = 0, down = 0;
  // ちらつき（毎 tick n ↔ n+1）
  CHECK(ticks_until_inhibit([](int i) { return static_cast<int64_t>(i % 2); }, [](int) { return 200; }, &up, &down) == 80);
  // 片相のトグル（A だけが 2 tick ごとに変わる: n ↔ n+1）
  up = down = 0;
  CHECK(ticks_until_inhibit([](int i) { return static_cast<int64_t>((i / 2) % 2); }, [](int) { return 200; }, &up, &down) == 80);
  // 出力と逆向きの動き（押し戻されている）
  up = down = 0;
  CHECK(ticks_until_inhibit([](int i) { return static_cast<int64_t>(-i); }, [](int) { return 200; }, &up, &down) == 80);
  // PWM の隙間（10 tick 出して 10 tick 0）: 隙間で包絡線は戻らない → 出した tick の合計が 80
  up = down = 0;
  CHECK(ticks_until_inhibit([](int) { return int64_t{0}; }, [](int i) { return (i / 10) % 2 == 0 ? 200 : 0; }, &up, &down) == 80);
  // 出力の反転（10 tick ごとに ±）: 上下を別々に積む。先に尽きた向きで止まる（どちらも 80 以下）
  up = down = 0;
  ticks_until_inhibit([](int) { return int64_t{0}; }, [](int i) { return (i / 10) % 2 == 0 ? 200 : -200; }, &up, &down);
  CHECK(up == 80 && down <= 80 && down >= 70);
  // 出力の向きへ 2 count 進めば戻る
  SupRig r;
  for (int i = 0; i < 40; ++i) r.tick(200, 0);
  CHECK(r.s.envelope_episode(1) > 400.0);
  r.tick(200, 1);
  CHECK(r.s.envelope_episode(1) > 400.0);  // 1 では戻らない
  r.tick(200, 2);
  CHECK(r.s.envelope_episode(1) == 0.0 && r.s.trusted_raw() == 2);
  // 残る穴（PCNT のフィルタ・A/B 遷移の妥当性の計数が後続）: ±2 以上の往復は戻してしまう
  SupRig k;
  for (int i = 0; i < 40; ++i) k.tick(200, 0);
  k.tick(200, -2);
  k.tick(200, 0);
  CHECK(k.s.envelope_episode(1) == 0.0);
}

// 解除の範囲: DISARMED に戻す（clear_episode）は FEEDBACK_UNCERTAIN と回の上限を戻すが、位置の上限は戻さない。基準の取り直しは全部
inline void test_r10_recovery_scope_unit() {
  SupRig r;
  r.s.t.min_budget_count = 0;
  r.s.reset_trust(2100, 0);  // 上の位置の上限 250
  int n = 0;
  for (int i = 0; i < 100; ++i) n += r.tick(200, 2100) != 0 ? 1 : 0;
  CHECK(r.s.feedback_uncertain() && n == 20);
  r.s.end_session(2100, r.c.now_us);
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

// エンコーダ断（物理位置だけが動く）、中央付近、Duty 10 / 13 / 20 %、両向き: 予算（1000 count）の中で出力 0・FEEDBACK_UNCERTAIN。
// ENCODER_FAULT はラッチしない（状態は ACTIVE のまま）。比例の包絡線（既定）: 物理の移動 ≤ 1000。一律 2500（仕様の数）: 出力は 80 tick = 400 ms
inline void test_r10_dead_encoder_mid_stroke_cut_within_budget() {
  for (int duty : {10, 13, 20}) {
    for (int dir : {1, -1}) {
      for (bool constant_env : {false, true}) {
        Harness h;
        r10_active(h, 0, 100000, 30, duty);
        if (constant_env) h.ax.tuning.sup.env_ref_permille = 0;
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
                      constant_env ? "const" : "prop", n, blind);
        }
        CHECK(h.ax.state() == st::ACTIVE && h.ax.fault_code() == fc::NONE && no_fault_events(h));
        CHECK(h.ax.supervisor().feedback_uncertain() && h.ax.pwm_permille() == 0 && h.ax.duty_permille() == 0);
        CHECK((h.status().flags & fl::FEEDBACK_UNCERTAIN) != 0);
        CHECK(blind <= 1000.0);
        if (constant_env) CHECK(n >= 78 && n <= 80);  // 最後に確かに動いた tick からの 80 tick（断線の直前の数 tick を引く）
      }
    }
  }
}

// 端の近く: D = 650 / 525 / 400 → 100 / 50 / 0 ms（一律 2500・最小予算 0 = Issue #44 の仕様どおり）。既定の最小予算 50 なら D = 400 で 20 ms。
// 静止（HOLD）から端の方へ目標を出し、断線したエンコーダの前で出せた tick を数える。物理の移動は予算以下
inline void test_r10_dead_encoder_near_end_budgets() {
  struct Case { int32_t d; int ticks_spec; int ticks_default; };
  for (const Case& k : {Case{650, 20, 20}, Case{525, 10, 10}, Case{400, 0, 4}}) {
    for (int dir : {1, -1}) {
      for (bool spec : {true, false}) {
        Harness h;
        const int32_t ref = dir > 0 ? 2700 + 50 - k.d : -2700 - 50 + k.d;
        r10_active(h, ref);
        h.ax.tuning.sup.env_ref_permille = 0;
        if (spec) h.ax.tuning.sup.min_budget_count = 0;
        CHECK(h.ax.actual_count() == ref && h.ax.pwm_permille() == 0);
        h.enc_connected = false;
        const double phys0 = h.phys;
        const int n = drive_count_nonzero(h, 500, ref + dir * 300);
        const int expect = spec ? k.ticks_spec : k.ticks_default;
        CHECK(n == expect);
        CHECK(fabs(h.phys - phys0) <= expect * 12.5 + 1e-9);
        CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h) && h.ax.supervisor().feedback_uncertain());
      }
    }
  }
}

// 乗員が押さえて 10 s 動かない（エンコーダは生きている、物理も止まっている）: ENCODER_FAULT にしない。出力は予算（1000 count ≈ 400 ms）
// の後 0、FEEDBACK_UNCERTAIN（ラッチではない: 状態は ACTIVE、DISARM → 再 ARM で戻る）。手を離した後の再 ARM で目標へ動ける
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
  CHECK(h.ax.supervisor().feedback_uncertain() && h.ax.pwm_permille() == 0 && !load_seen);
  const V1SupInfo info = h.ax.supervisor_info();
  if (std::getenv("R10_PRINT")) {
    std::printf("held: nonzero %d ticks, env %.1f, age %u ms, budget_up %d\n", n, h.ax.supervisor().envelope_episode(1),
                info.activity_age_ms, info.budget_up);
  }
  // 包絡線（比例: duty_max 20 % で 2500 count/s）が 1000 count に届いた所で 0。目標の立ち上がり（軌道 → P）で出力が上がる途中の分だけ
  // 80 tick（400 ms）より長い
  CHECK(n >= 80 && n <= 120 && h.ax.supervisor().envelope_episode(1) <= 1000.0 + 1e-9);
  CHECK(info.enabled && info.feedback_uncertain && info.activity_age_ms >= 9900 && info.budget_up < 13);  // 残りは 1 tick（12.5）未満
  // 手を離して DISARM（ブレーキ作動 → DISARMED）→ 再 ARM → START: 目標へ動き、確かな動きで予算が戻る
  h.held = false;
  Harness::accepted(h.disarm());
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED && !h.ax.supervisor().feedback_uncertain() && (h.status().flags & SUP_FLAGS) == 0);
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  for (int i = 0; i < 150; ++i) drive_count_nonzero(h, 10, 150);
  CHECK(h.ax.state() == st::ACTIVE && !h.ax.supervisor().feedback_uncertain() && no_fault_events(h));
  CHECK(std::abs(h.ax.actual_count() - 150) <= 4);
  CHECK(h.ax.supervisor().envelope_cumulative(1) < 100.0);
}

// 外力で押されて目標から離れる（エンコーダは生きている）→ 離せば戻る: ラッチしない。FOLLOWING_ERROR の上限（200）を少し越えても、
// 戻りつつあれば猶予（LIMIT_RECOVERY）でラッチしない
inline void test_r10_external_push_away_does_not_latch() {
  for (uint32_t push_ms : {60u, 120u}) {
    Harness h;
    r10_active(h, 0, 200);
    h.ext_vel = -4000.0;
    int max_fe = 0;
    bool recovery_seen = false;
    for (uint32_t t = 0; t < 1500; t += 10) {
      if (t >= push_ms) h.ext_vel = 0.0;
      drive_count_nonzero(h, 10, 0);
      const int fe = std::abs(h.ax.following_error_count());
      if (fe > max_fe) max_fe = fe;
      if (h.status().flags & fl::LIMIT_RECOVERY) recovery_seen = true;
    }
    if (std::getenv("R10_PRINT")) std::printf("push %u ms: max |FE| %d\n", push_ms, max_fe);
    CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h));
    CHECK(!h.ax.supervisor().feedback_uncertain() && (h.status().flags & SUP_FLAGS) == 0);
    CHECK(std::abs(h.ax.actual_count()) <= 4);
    if (push_ms == 120u) CHECK(max_fe > 200 && max_fe < 300 && recovery_seen);  // 上限を越えたが戻った
    if (push_ms == 60u) CHECK(max_fe < 200);
  }
}

// 過負荷（LOAD_LIMITED）: 荷重で 15 count/s しか進めない（確かな動きはあるので予算は戻る）まま飽和 1 s → 出力 0（ラッチしない）。
// 2 s 休んだらもう一度出す。荷重が無くなれば目標に着き、フラグは消える
inline void test_r10_load_limited_after_1s_saturation_then_retry() {
  Harness h;
  r10_active(h, 0, 200);
  h.plant_max_vel = 15.0;
  uint64_t t_limited = 0;
  const uint64_t t0 = h.now;
  for (int i = 0; i < 200 && !h.ax.supervisor().load_limited(); ++i) drive_count_nonzero(h, 10, 150);
  t_limited = h.now - t0;
  CHECK(h.ax.supervisor().load_limited() && h.ax.pwm_permille() == 0 && (h.status().flags & fl::LOAD_LIMITED) != 0);
  CHECK(t_limited >= 1000 * MS && t_limited <= 1150 * MS);
  CHECK(!h.ax.supervisor().feedback_uncertain());
  // 休みの間は 0
  int n = drive_count_nonzero(h, 1900, 150);
  CHECK(n == 0 && h.ax.supervisor().load_limited());
  // 2 s で再び出す
  drive_count_nonzero(h, 150, 150);
  CHECK(!h.ax.supervisor().load_limited() && h.ax.pwm_permille() > 0);
  // 荷重が無くなる
  h.plant_max_vel = 1e9;
  drive_count_nonzero(h, 1500, 150);
  CHECK(std::abs(h.ax.actual_count() - 150) <= 4 && (h.status().flags & SUP_FLAGS) == 0);
  CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h));
}

// 静止目標で誤差 10 のまま進まない（r6〜r9 は 2 s で ENCODER_FAULT）・軌道が動いているのに止まったまま（走行中の無パルス）:
// どちらも ENCODER_FAULT にしない
inline void test_r10_old_stall_scenarios_never_latch_encoder_fault() {
  Harness h;
  r10_active(h, 0, 100000, 2, 20, false);  // 手で動かすカウント（動かない）
  h.raw -= 10;
  h.drive(3000, h.ax.target_count());
  CHECK(h.ax.state() == st::ACTIVE && no_fault_events(h));
  Harness g;
  r10_active(g, 0, 100000, 6, 20, false);
  g.pos_max_vel = 600;
  const int32_t s0 = g.ax.actual_count();
  for (int i = 0; i < 100; ++i) g.drive(10, s0 + 5000);
  CHECK(g.ax.state() == st::ACTIVE && no_fault_events(g) && g.ax.supervisor().feedback_uncertain());
  CHECK(g.ax.pwm_permille() == 0);
  // 押さえられた軸に追従誤差の上限を掛ければ FOLLOWING_ERROR（カウントが動かないので猶予なし）。保護は残る
  Harness k;
  r10_active(k, 0, 400, 6, 20, false);
  k.pos_max_vel = 600;
  const int32_t k0 = k.ax.actual_count();
  for (int i = 0; i < 100 && k.ax.state() != st::FAULT; ++i) k.drive(10, k0 + 5000);
  CHECK(k.ax.state() == st::FAULT && k.ax.fault_code() == fc::FOLLOWING_ERROR);
}

// FEEDBACK_UNCERTAIN の解除は DISARMED に戻すときだけ（DISARM の完了か FAULT_RESET）。確かな動きでも自動では解除しない
inline void test_r10_feedback_uncertain_requires_explicit_recovery() {
  Harness h;
  r10_active(h, 0, 100000);
  h.held = true;
  drive_count_nonzero(h, 600, 150);
  CHECK(h.ax.supervisor().feedback_uncertain());
  h.held = false;
  h.raw += 5;  // 外から動いた（出力は 0 なので「出力の向きへの確かな動き」ではない）
  h.phys = static_cast<double>(h.raw);
  drive_count_nonzero(h, 500, 150);
  CHECK(h.ax.supervisor().feedback_uncertain() && h.ax.pwm_permille() == 0);
  // HOST_FAULT → FAULT → FAULT_RESET でも解除される
  h.host_fault();
  CHECK(h.ax.state() == st::FAULT && h.ax.supervisor().feedback_uncertain());
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.state() == st::DISARMED && !h.ax.supervisor().feedback_uncertain() && h.ax.referenced());
}

// 端の手前の制動（Issue #46）: 実測速度で止まれない（v² / 2a > ソフトリミットまでの残り）なら反対向きに duty_max。ラッチしない。
// 止まれる速さなら何もしない。動きが止まれば解除
inline void test_r10_end_braking_guard() {
  Harness h;
  r10_active(h, 2400, 100000, 6, 20, false);
  for (int i = 0; i < 12; ++i) {  // 1200 count/s で上へ（停止距離 720 > 残り 300〜240）
    h.raw += 6;
    h.set_position(2400);
    h.step();
  }
  CHECK(h.ax.supervisor().end_braking() && h.ax.pwm_permille() == -200 && (h.status().flags & fl::END_BRAKING) != 0);
  CHECK(h.ax.supervisor().envelope_episode(-1) < 100.0);  // 制動の出力は包絡線に積まない
  h.drive(80, 2400);  // 止まった: v_fast は 40 ms で 0
  CHECK(!h.ax.supervisor().end_braking() && h.ax.pwm_permille() != -200 && h.ax.state() == st::ACTIVE);
  // 止まれる速さ（200 count/s、停止距離 20 < 残り 300）
  Harness g;
  r10_active(g, 2400, 100000, 6, 20, false);
  bool seen = false;
  for (int i = 0; i < 40; ++i) {
    g.raw += 1;
    g.set_position(2400);
    g.step();
    if (g.ax.supervisor().end_braking()) seen = true;
  }
  CHECK(!seen && no_fault_events(g) && no_fault_events(h));
  // 従来の経路（監視なし）には無い
  Harness k;
  r10_active(k, 2400, 100000, 6, 20, false);
  k.ax.tuning.supervisor = false;
  for (int i = 0; i < 12; ++i) {
    k.raw += 6;
    k.set_position(2400);
    k.step();
  }
  CHECK(!k.ax.supervisor_info().enabled && (k.status().flags & fl::END_BRAKING) == 0 && k.ax.pwm_permille() > -200);
}

// SOFT_LIMIT（Issue #46）: 少し越えただけで目標が内側・外へ動き続けていないならラッチせず内側へ戻す（外向きの出力は出さない）。
// ラッチ: 機械端（ソフトリミット + 50）から 20 以内 / 外向きに 200 ms 動き続けた / 2 s 外に居続けた
inline void test_r10_soft_limit_recovers_inward_or_latches_near_end() {
  {  // 少し越えて止まる → 戻す
    Harness h;
    r10_active(h, 2690, 100000, 6, 20, false);
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
  {  // 機械端から 20 以内（2730 = 2750 − 20）
    Harness h;
    r10_active(h, 2690, 100000, 6, 20, false);
    h.raw += 40;
    h.drive(10, 2690);
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT);
  }
  {  // 外へ 50 count/s で動き続ける → 外に出てから 200 ms 前後でラッチ（機械端の手前）
    Harness h;
    r10_active(h, 2698, 100000, 6, 20, false);
    int i = 0;
    for (; i < 400 && h.ax.state() != st::FAULT; ++i) {
      if ((i % 4) == 0) h.raw += 1;
      h.set_position(2698);
      h.step();
    }
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT);
    CHECK(h.ax.actual_count() >= 2708 && h.ax.actual_count() < 2730);
  }
  {  // 外で動けないまま 2 s
    Harness h;
    r10_active(h, 2690, 100000, 6, 20, false);
    h.raw += 20;
    const uint64_t t0 = h.now;
    h.drive(1900, 2690);
    CHECK(h.ax.state() == st::ACTIVE);
    h.drive(200, 2690);
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT && h.now - t0 <= 2100 * MS);
  }
  {  // 従来の経路（監視なし）は即ラッチ（変えていない）
    Harness h;
    r10_active(h, 2690, 100000, 6, 20, false);
    h.ax.tuning.supervisor = false;
    h.raw += 20;
    h.drive(10, 2690);
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT);
  }
}

// FOLLOWING_ERROR（Issue #46）: 上限（200）そのものは変えない。越えてもエンコーダが生きていて |誤差| が減っていればラッチしない。
// 止まった（100 ms カウント変化なし）・増え続ける（越えてから 200 ms）・上限 × 1.5 以上ならラッチ
inline void test_r10_following_error_waits_only_while_recovering() {
  {  // 5 count/tick で −250 まで押され、1 count/tick で戻る（越えている時間 ≈ 300 ms）
    Harness h;
    r10_active(h, 0, 200, 6, 20, false);
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
  {  // −250 で止まる → カウントが 100 ms 動かなければラッチ
    Harness h;
    r10_active(h, 0, 200, 6, 20, false);
    for (int i = 0; i < 50; ++i) { h.raw -= 5; h.set_position(0); h.step(); }
    const uint64_t t0 = h.now;
    for (int i = 0; i < 60 && h.ax.state() != st::FAULT; ++i) h.drive(5, 0);
    CHECK(h.ax.fault_code() == fc::FOLLOWING_ERROR && h.now - t0 <= 160 * MS);
  }
  {  // 越えた後も 1 count/tick で増え続ける → 越えてから 200 ms でラッチ
    Harness h;
    r10_active(h, 0, 200, 6, 20, false);
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
  {  // 上限 × 1.5 以上 → 猶予なし
    Harness h;
    r10_active(h, 0, 200, 6, 20, false);
    h.raw -= 301;
    h.drive(5, 0);
    CHECK(h.ax.fault_code() == fc::FOLLOWING_ERROR);
  }
}

// STATUS の flags（bit 7〜10）。監視が OFF（従来の経路）では立たない。flags の変化で即時 STATUS が出る
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
    if (s[i].flags & fl::FEEDBACK_UNCERTAIN) { immediate = true; break; }
  }
  CHECK(immediate);
  Harness k;  // 従来の経路
  Harness::accepted(k.config());
  CHECK((k.status().flags & SUP_FLAGS) == 0 && !k.ax.supervisor_info().enabled);
}

inline void run_all() {
  test_r10_budget_derivation();
  test_r10_renewal_rules_unit();
  test_r10_recovery_scope_unit();
  test_r10_dead_encoder_mid_stroke_cut_within_budget();
  test_r10_dead_encoder_near_end_budgets();
  test_r10_held_occupant_10s_no_encoder_fault();
  test_r10_external_push_away_does_not_latch();
  test_r10_load_limited_after_1s_saturation_then_retry();
  test_r10_old_stall_scenarios_never_latch_encoder_fault();
  test_r10_feedback_uncertain_requires_explicit_recovery();
  test_r10_end_braking_guard();
  test_r10_soft_limit_recovers_inward_or_latches_near_end();
  test_r10_following_error_waits_only_while_recovering();
  test_r10_status_flags_bits();
}

}  // namespace v1r10_test
