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
  void push(uint8_t type, uint16_t len = 50) {
    Frame f{};
    f.len = len;
    f.type = type;
    f.enqueued_ms = now;
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
};

inline void test_decide_table() {
  v1::TxPolicy p;
  p.status_stale_ms = 100;
  p.defer_max_ms = 200;
  using A = v1::TxAction;
  // 空きがあれば書く
  CHECK(v1::tx_decide(false, 50, 50, 0, p) == A::Write);
  CHECK(v1::tx_decide(true, 50, 1000, 99, p) == A::Write);
  CHECK(v1::tx_decide(false, 50, 1000, 199, p) == A::Write);
  // 空きが足りない: STATUS は捨てる、それ以外は保留（接続状態は見ない）
  CHECK(v1::tx_decide(true, 50, 49, 0, p) == A::DropStale);
  CHECK(v1::tx_decide(false, 50, 49, 0, p) == A::Defer);
  CHECK(v1::tx_decide(false, 50, 0, 199, p) == A::Defer);
  // 期限切れは空きがあっても送らない（積まれてからの滞在時間）
  CHECK(v1::tx_decide(true, 50, 1000, 100, p) == A::DropStale);
  CHECK(v1::tx_decide(false, 50, 1000, 200, p) == A::DropAged);
  CHECK(v1::tx_decide(false, 50, 0, 200, p) == A::DropAged);
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
  CHECK(s.pump.stats.deferred == 0 && s.pump.stats.stale_dropped == 0 && s.pump.stats.aged_dropped == 0);
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
  CHECK(s.pump.stats.aged_dropped == 0);
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

// 保留した ACK は積まれてから defer_max_ms で捨てる（PC の ACK 期限より十分前。詰まったままのリンクで古い ACK を溜めない）
inline void test_pump_drops_aged_deferred_frame() {
  Sim s;
  s.port.free_bytes = 10;
  s.push(v1::msg::ACK);
  s.now += 50;  // 積まれてから 50 ms 後に初めて poll（描画で loop が遅れた）
  s.poll();
  CHECK(s.pump.holding());
  s.now += V1_TX_DEFER_MAX_MS - 51;  // 積まれてから 199 ms
  s.poll();
  CHECK(s.pump.holding() && s.pump.stats.aged_dropped == 0);
  s.now += 1;  // 200 ms
  s.poll();
  CHECK(!s.pump.holding() && s.pump.stats.aged_dropped == 1);
  CHECK(s.port.written_types.empty());
}

// 期限は積まれてからの滞在時間: 詰まりの間に積まれた ACK 群は、空きが戻っても期限を過ぎたものは送らず、
// 後続が 1 本ずつ新しい猶予をもらうこともない（硬い期限）
inline void test_pump_expiry_is_total_residence_not_per_successor() {
  Sim s;
  s.port.free_bytes = 10;
  s.push(v1::msg::ACK);        // t = 1000
  s.now += 100;
  s.push(v1::msg::LIMITS);     // t = 1100
  s.now += 100;
  s.push(v1::msg::CAL_RESULT); // t = 1200
  s.poll();                    // t = 1200: ACK（200 ms）は期限切れで捨て、LIMITS（100 ms）を保留
  CHECK(s.pump.stats.aged_dropped == 1);
  CHECK(s.pump.holding() && s.pump.held()->type == v1::msg::LIMITS);
  s.now += 100;                // t = 1300: LIMITS が 200 ms → 捨て、CAL_RESULT（100 ms）を保留
  s.poll();
  CHECK(s.pump.stats.aged_dropped == 2);
  CHECK(s.pump.holding() && s.pump.held()->type == v1::msg::CAL_RESULT);
  // 空きが戻っても、期限を過ぎていれば送らない
  s.now += 100;                // t = 1400: CAL_RESULT が 200 ms
  s.port.free_bytes = 1024;
  const int calls_before = s.port.available_calls;
  s.poll();
  CHECK(s.pump.stats.aged_dropped == 3);
  CHECK(s.port.written_types.empty() && !s.pump.holding());
  CHECK(s.port.available_calls == calls_before);  // 期限切れの判定では空きを見ない
}

// 期限内なら、保留の後に空きが戻れば送る
inline void test_pump_sends_deferred_frame_within_deadline() {
  Sim s;
  s.port.free_bytes = 10;
  s.push(v1::msg::ACK);
  s.now += 150;
  s.poll();
  CHECK(s.pump.holding());
  s.now += 40;  // 190 ms
  s.port.free_bytes = 1024;
  s.poll();
  CHECK(s.port.written_types.size() == 1 && s.port.written_types[0] == v1::msg::ACK);
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

// 空きが無ければ接続状態に関わらず書かない（HWCDC::write が中で接続を見直して待つ経路に入り得るため）
inline void test_pump_never_writes_without_space() {
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
  test_decide_table();
  test_pump_writes_in_order_when_space();
  test_pump_defers_ack_and_keeps_order_under_backpressure();
  test_pump_drops_status_under_backpressure_but_not_ack();
  test_pump_drops_aged_deferred_frame();
  test_pump_expiry_is_total_residence_not_per_successor();
  test_pump_sends_deferred_frame_within_deadline();
  test_pump_skips_stale_status_after_backlog();
  test_pump_never_writes_without_space();
  test_pump_space_boundary();
}

}  // namespace v1txpump_test
