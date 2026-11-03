// v1_tx_pump.h のホストテスト（Issue #15 M7）。test_host.cpp から include する（CHECK マクロは test_host.cpp のもの）。
// Serial の代わりに「リングの空き」「書いたフレーム」を持つ偽ポートで、ブロックしない送信の規則を確認する。
#pragma once

#include <cstring>
#include <deque>
#include <vector>

#include "v1_codec.h"
#include "v1_tx_pump.h"

namespace v1txpump_test {

struct Frame {
  uint16_t len;
  uint8_t type;
  uint32_t enqueued_ms;
  uint8_t data[8];
};

struct FakePort {
  size_t free_bytes = 1024;
  int available_calls = 0;
  std::vector<uint8_t> written_types;  // data[0] に type を入れておく
  size_t available_for_write() {
    ++available_calls;
    return free_bytes;
  }
  size_t write(const uint8_t* d, size_t n) {
    written_types.push_back(d[0]);
    if (free_bytes >= n) free_bytes -= n;  // 入ったぶんだけ空きが減る
    return n;
  }
};

struct Sim {
  std::deque<Frame> q;
  FakePort port;
  v1::TxPump<Frame> pump;
  uint32_t now = 1000;
  void push(uint8_t type, uint16_t len = 50, int32_t enqueue_offset_ms = 0) {
    Frame f{};
    f.len = len;
    f.type = type;
    f.enqueued_ms = static_cast<uint32_t>(static_cast<int32_t>(now) + enqueue_offset_ms);
    f.data[0] = type;
    q.push_back(f);
  }
  void poll(int max_frames = 4) {
    auto pop = [this](Frame* out) {
      if (q.empty()) return false;
      *out = q.front();
      q.pop_front();
      return true;
    };
    pump.run(pop, port, now, max_frames);
  }
  int count(uint8_t type) const {
    int n = 0;
    for (uint8_t t : port.written_types) n += (t == type) ? 1 : 0;
    return n;
  }
};

inline void test_age_is_signed_and_clamped() {
  CHECK(v1::tx_age_ms(1000, 1000) == 0);
  CHECK(v1::tx_age_ms(1200, 1000) == 200);
  CHECK(v1::tx_age_ms(1000, 1001) == 0);     // 別コアが run() の途中で積んだ（now より新しい）
  CHECK(v1::tx_age_ms(1000, 1500) == 0);
  CHECK(v1::tx_age_ms(5, 0xFFFFFFF0u) == 21);  // millis の巻き戻りをまたぐ
}

inline void test_decide_table() {
  v1::TxPolicy p;
  p.status_stale_ms = 100;
  p.hold_write_through_ms = 100;
  using A = v1::TxAction;
  // 空きがあれば書く
  CHECK(v1::tx_decide(false, 50, 50, 0, 0, p) == A::Write);
  CHECK(v1::tx_decide(true, 50, 1000, 99, 0, p) == A::Write);
  CHECK(v1::tx_decide(false, 50, 1000, 5000, 0, p) == A::Write);  // STATUS 以外は古くても送る
  // 空きが足りない: STATUS は捨てる、それ以外は保留
  CHECK(v1::tx_decide(true, 50, 49, 0, 0, p) == A::DropStale);
  CHECK(v1::tx_decide(false, 50, 49, 0, 0, p) == A::Defer);
  CHECK(v1::tx_decide(false, 50, 0, 0, 99, p) == A::Defer);
  // 保留が続いたら空きが無くても書く（捨てない）
  CHECK(v1::tx_decide(false, 50, 0, 0, 100, p) == A::WriteThrough);
  CHECK(v1::tx_decide(false, 50, 49, 10000, 100, p) == A::WriteThrough);
  // 古い STATUS は空きがあっても送らない
  CHECK(v1::tx_decide(true, 50, 1000, 100, 0, p) == A::DropStale);
}

// 通常: 空きがあれば順に書く（1 poll に最大 4 本）。保留なし
inline void test_pump_writes_in_order_when_space() {
  Sim s;
  s.push(v1::msg::ACK);
  s.push(v1::msg::STATUS);
  s.push(v1::msg::LIMITS);
  s.push(v1::msg::STATUS);
  s.push(v1::msg::STATUS);
  s.poll();
  CHECK(s.port.written_types.size() == 4);
  CHECK(s.port.written_types[0] == v1::msg::ACK && s.port.written_types[1] == v1::msg::STATUS &&
        s.port.written_types[2] == v1::msg::LIMITS && s.port.written_types[3] == v1::msg::STATUS);
  CHECK(s.q.size() == 1 && !s.pump.holding());
  s.poll();
  CHECK(s.port.written_types.size() == 5 && s.q.empty());
  CHECK(s.pump.stats.deferred == 0 && s.pump.stats.stale_dropped == 0 && s.pump.stats.written_through == 0);
}

// 11-03 未明の実機の再現: START の ACK と遷移後の STATUS が同じ poll で積まれ、ACK の enqueued_ms が run() で取った now より
// 1 ms 新しい（制御タスクが別コアで run() の途中に積んだ）。r2 は age が巻き戻って ACK を期限切れで捨てた。ACK は必ず出る
inline void test_ack_enqueued_after_poll_timestamp_is_not_dropped() {
  Sim s;
  s.push(v1::msg::STATUS, 50, 0);  // 定期 STATUS
  s.push(v1::msg::ACK, 50, +1);    // START の ACK（now + 1 ms）
  s.push(v1::msg::STATUS, 50, +1); // 遷移後の STATUS
  s.poll();
  CHECK(s.port.written_types.size() == 3);
  CHECK(s.port.written_types[1] == v1::msg::ACK);
  CHECK(s.pump.stats.stale_dropped == 0);
  // 空きが一瞬 0 でも、ACK は保留して次の poll で出る（捨てない）。STATUS は捨ててよい
  Sim t;
  t.port.free_bytes = 0;
  t.push(v1::msg::ACK, 50, +1);
  t.push(v1::msg::STATUS, 50, +1);
  t.poll();
  CHECK(t.port.written_types.empty() && t.pump.holding() && t.pump.held()->type == v1::msg::ACK);
  t.now += 1;
  t.port.free_bytes = 1024;
  t.poll();
  CHECK(t.port.written_types.size() == 2 && t.port.written_types[0] == v1::msg::ACK);
  CHECK(t.pump.stats.stale_dropped == 0);
}

// 100 Hz の SET_POSITION に応じた STATUS が流れ続ける中で、空きが小さい poll が 1 回挟まっても ACK は落ちない
inline void test_ack_survives_transient_small_space_with_status_traffic() {
  Sim s;
  s.port.free_bytes = 1024;
  for (int i = 0; i < 20; ++i) {  // 200 ms 分の定期 STATUS（20 ms ごと）
    s.push(v1::msg::STATUS);
    s.poll();
    s.now += 20;
    s.port.free_bytes = 1024;  // ホストが読んでいる
  }
  // ACK + 遷移後 STATUS が積まれた poll で、空きは 30 byte しかない
  s.port.free_bytes = 30;
  s.push(v1::msg::ACK, 50, +1);
  s.push(v1::msg::STATUS, 50, +1);
  s.poll();
  CHECK(s.pump.holding() && s.pump.held()->type == v1::msg::ACK);
  CHECK(s.pump.stats.deferred == 1);
  // 次の poll（1 ms 後）には空きが戻る → ACK、STATUS の順
  s.now += 1;
  s.port.free_bytes = 1024;
  s.poll();
  CHECK(s.count(v1::msg::ACK) == 1);
  CHECK(s.port.written_types.back() == v1::msg::STATUS);
  CHECK(s.pump.stats.stale_dropped == 0 && s.pump.stats.written_through == 0);
}

// 背圧（空き無し）: ACK は保留して順序を保ち、後ろは触らない。空きができたら保留から順に出る
inline void test_pump_defers_ack_and_keeps_order_under_backpressure() {
  Sim s;
  s.port.free_bytes = 10;
  s.push(v1::msg::ACK);
  s.push(v1::msg::LIMITS);
  s.poll();
  CHECK(s.port.written_types.empty());
  CHECK(s.pump.holding() && s.pump.held()->type == v1::msg::ACK);
  CHECK(s.q.size() == 1);  // LIMITS はキューに残る
  CHECK(s.pump.stats.deferred == 1);
  s.now += 5;
  s.poll();
  CHECK(s.pump.stats.deferred == 2 && s.port.written_types.empty());
  // 空きが戻った
  s.port.free_bytes = 1024;
  s.now += 5;
  s.poll();
  CHECK(s.port.written_types.size() == 2);
  CHECK(s.port.written_types[0] == v1::msg::ACK && s.port.written_types[1] == v1::msg::LIMITS);
  CHECK(!s.pump.holding() && s.q.empty());
  CHECK(s.pump.stats.written_through == 0);
}

// 背圧の間の STATUS は捨てる（次の周期のものが新しい）。ACK → STATUS の順序は崩れない
inline void test_pump_drops_status_under_backpressure_but_not_ack() {
  Sim s;
  s.port.free_bytes = 10;
  s.push(v1::msg::STATUS);
  s.push(v1::msg::STATUS);
  s.push(v1::msg::ACK);
  s.push(v1::msg::STATUS);
  s.poll();
  // STATUS 2 本は捨て、ACK で保留（後ろの STATUS は残る）
  CHECK(s.pump.stats.stale_dropped == 2);
  CHECK(s.pump.holding() && s.pump.held()->type == v1::msg::ACK);
  CHECK(s.q.size() == 1 && s.q.front().type == v1::msg::STATUS);
  s.port.free_bytes = 1024;
  s.poll();
  CHECK(s.port.written_types.size() == 2 && s.port.written_types[0] == v1::msg::ACK &&
        s.port.written_types[1] == v1::msg::STATUS);
}

// PMIC_REPORT は空きが無ければ捨て、同じ poll で後ろの ACK を通常どおり保留する（PMIC を抱えて ACK を止めない。Issue #68）
inline void test_pump_drops_pmic_without_space_and_continues_to_ack() {
  Sim s;
  s.port.free_bytes = 10;
  s.push(v1::msg::PMIC_REPORT, 100);
  s.push(v1::msg::ACK);  // 既定 50 byte。空き 10 には入らないので保留
  s.poll();
  CHECK(s.pump.stats.stale_dropped == 1);
  CHECK(s.pump.holding() && s.pump.held()->type == v1::msg::ACK);
  CHECK(s.q.empty());
  s.port.free_bytes = 1024;
  s.poll();
  CHECK(s.port.written_types.size() == 1 && s.port.written_types[0] == v1::msg::ACK);
  CHECK(s.pump.stats.written_through == 0);
}

// 保留が hold_write_through_ms（100 ms）続いたら空きが無くても書く（捨てない。availableForWrite が 0 を返し続ける状態の退路）。
// 後続もそれぞれ保留 → 書く、でキューは進む
inline void test_pump_writes_through_after_long_hold_never_drops_ack() {
  Sim s;
  s.port.free_bytes = 0;
  s.push(v1::msg::ACK);
  s.push(v1::msg::CAL_RESULT);
  s.poll();
  CHECK(s.pump.holding());
  s.now += V1_TX_HOLD_WRITE_THROUGH_MS - 1;
  s.poll();
  CHECK(s.pump.holding() && s.port.written_types.empty());
  s.now += 1;
  s.poll();  // ACK を空きを見ずに書く。CAL_RESULT は新たに保留
  CHECK(s.port.written_types.size() == 1 && s.port.written_types[0] == v1::msg::ACK);
  CHECK(s.pump.stats.written_through == 1);
  CHECK(s.pump.holding() && s.pump.held()->type == v1::msg::CAL_RESULT);
  s.now += V1_TX_HOLD_WRITE_THROUGH_MS;
  s.poll();
  CHECK(s.port.written_types.size() == 2 && s.port.written_types[1] == v1::msg::CAL_RESULT);
  CHECK(s.pump.stats.written_through == 2);
  CHECK(!s.pump.holding());
}

// STATUS 以外は積まれてから古くなっても捨てない（期限は STATUS だけ）
inline void test_non_status_is_never_expired() {
  Sim s;
  s.push(v1::msg::ACK, 50, -5000);  // 5 s 前に積まれた
  s.push(v1::msg::STATUS, 50, -5000);
  s.poll();
  CHECK(s.port.written_types.size() == 1 && s.port.written_types[0] == v1::msg::ACK);
  CHECK(s.pump.stats.stale_dropped == 1);
}

// 詰まりが解けたとき、積まれて 100 ms 以上経った STATUS は送らず、新しい STATUS だけ送る（最新で置き換える）
inline void test_pump_skips_stale_status_after_backlog() {
  Sim s;
  s.port.free_bytes = 10;
  for (int i = 0; i < 6; ++i) {
    s.push(v1::msg::STATUS);
    s.now += 20;
  }
  // いま now = 1120。先頭の STATUS は 120 ms 前、末尾は 20 ms 前。空きが戻る
  s.port.free_bytes = 1024;
  s.poll(8);
  // 120・100 ms 前の 2 本は古い → 捨てる。80・60・40・20 ms 前の 4 本は送る
  CHECK(s.pump.stats.stale_dropped == 2);
  CHECK(s.port.written_types.size() == 4);
  CHECK(s.q.empty());
}

// 空きが無いときは（保留が 100 ms 続くまで）書かない
inline void test_pump_does_not_write_without_space_before_hold_elapses() {
  Sim s;
  s.port.free_bytes = 0;
  s.push(v1::msg::ACK);
  s.push(v1::msg::STATUS);
  s.poll();
  CHECK(s.port.written_types.empty());
  CHECK(s.pump.holding() && s.pump.held()->type == v1::msg::ACK);
  CHECK(s.pump.stats.deferred == 1 && s.pump.stats.stale_dropped == 0);
  CHECK(s.q.size() == 1);
}

// 空きがフレーム長ちょうどなら書く、1 byte 足りなければ保留
inline void test_pump_space_boundary() {
  Sim s;
  s.port.free_bytes = 50;
  s.push(v1::msg::ACK, 50);
  s.push(v1::msg::ACK, 1);
  s.poll();
  CHECK(s.port.written_types.size() == 1);
  CHECK(s.pump.holding());  // 2 本目は空き 0 で保留
  CHECK(s.port.free_bytes == 0);
}

inline void run_all() {
  test_age_is_signed_and_clamped();
  test_decide_table();
  test_pump_writes_in_order_when_space();
  test_ack_enqueued_after_poll_timestamp_is_not_dropped();
  test_ack_survives_transient_small_space_with_status_traffic();
  test_pump_defers_ack_and_keeps_order_under_backpressure();
  test_pump_drops_status_under_backpressure_but_not_ack();
  test_pump_drops_pmic_without_space_and_continues_to_ack();
  test_pump_writes_through_after_long_hold_never_drops_ack();
  test_non_status_is_never_expired();
  test_pump_skips_stale_status_after_backlog();
  test_pump_does_not_write_without_space_before_hold_elapses();
  test_pump_space_boundary();
}

}  // namespace v1txpump_test
