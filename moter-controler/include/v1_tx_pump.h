#pragma once

// USB プロトコル v1 の送信側の「ブロックしない書き込み」（Issue #15 M7）。Serial・FreeRTOS を持たない純粋なロジック
//（ホストテストの対象）。v1_link.cpp の loop() 側 v1_link_poll() が Serial の adapter と送信キューを渡して回す。
//
// 背景: HWCDC::write は TX リング（既定 256 byte）に空きが無いと tx timeout（main.cpp で 20 ms）× 最大 20 回まで待つ。
// loop() が描画（pushSprite 数十 ms）と NVS 保存も抱えているので、PC が読んでいない間に write で固まると SET_POSITION の
// 受信まで止まり、START が「期限内の SET_POSITION なし」で拒否される（11-01）。FreeRTOS タスク化は 2 回失敗している
//（v1_link.cpp）ので、書く前に availableForWrite() を見て入らなければ次の poll に回す。
//
// 規則（tx_decide）:
//  1. STATUS で、積まれてから status_stale_ms 以上経っていれば送らない（DropStale）。20 ms ごとに新しい STATUS が来るので、
//     詰まりが解けたときに古い STATUS を順に流さず、新しいものだけを送る（「保留した STATUS は最新で置き換える」）。
//  2. リングの空きがフレーム長以上なら書く（Write。xRingbufferSend は待たずに入る）。
//  3. 空きが無く、CDC が未接続なら書く（WriteUnconnected）。HWCDC は未接続時にリングの古い内容を置き換える方針で
//     待たずに返り、再接続のための IN_EMPTY 割り込みも立て直す。従来と同じ経路を保つ（Issue #11 の条件を変えない）。
//  4. 空きが無く接続中（PC が読んでいない = 背圧）: STATUS は捨てる（DropStale。次の周期のものが新しい）。
//     それ以外（ACK・FAULT_EVENT・HELLO・LIMITS・CAL_RESULT・AXIS_SELECT）は保留して順序を保つ（Defer。この poll はここで終わる）。
//     保留が defer_max_ms を超えたら捨てる（DropAged。PC の ACK 期限 1000 ms より十分短く、STATUS 途絶の判定 200 ms と同程度。
//     ここまで待たされたリンクは PC 側でも途絶として扱われる）。
// 保留は先頭の 1 フレームだけ（それより後ろはキューに残る）。ACK → STATUS の順序は崩れない（STATUS を捨てるだけ）。

#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "v1_codec.h"  // msg::STATUS

namespace v1 {

enum class TxAction : uint8_t { Write, WriteUnconnected, DropStale, Defer, DropAged };

struct TxPolicy {
  uint32_t status_stale_ms = V1_TX_STATUS_STALE_MS;
  uint32_t defer_max_ms = V1_TX_DEFER_MAX_MS;
};

struct TxPumpStats {
  uint32_t written = 0;        // Serial に渡したフレーム数（未接続時の置き換え書き込みを含む）
  uint32_t deferred = 0;       // 空き不足で次の poll に回した回数（poll ごとに 1）
  uint32_t stale_dropped = 0;  // 古い・入らない STATUS を捨てた数
  uint32_t aged_dropped = 0;   // 保留が defer_max_ms を超えて捨てた STATUS 以外の数
};

// age_ms = 積まれてからの時間、waited_ms = 保留し始めてからの時間（保留していなければ 0）
inline TxAction tx_decide(bool is_status, size_t len, size_t free_bytes, bool connected, uint32_t age_ms,
                          uint32_t waited_ms, const TxPolicy& p) {
  if (is_status && age_ms >= p.status_stale_ms) return TxAction::DropStale;
  if (free_bytes >= len) return TxAction::Write;
  if (!connected) return TxAction::WriteUnconnected;
  if (is_status) return TxAction::DropStale;
  if (waited_ms >= p.defer_max_ms) return TxAction::DropAged;
  return TxAction::Defer;
}

// Frame は len / data / type / enqueued_ms を持つ（V1TxFrame）。
// Pop は bool(Frame*)（キューの先頭を取り出す。無ければ false）。
// Port は size_t available_for_write()、bool connected()、size_t write(const uint8_t*, size_t)。
// connected() は空きが無いときだけ呼ぶ（HWCDC::operator bool は未接続時に割り込みの再設定を伴うため、必要なときだけ）。
template <class Frame>
class TxPump {
 public:
  TxPolicy policy;
  TxPumpStats stats;

  bool holding() const { return has_held_; }
  const Frame* held() const { return has_held_ ? &held_ : nullptr; }

  // 1 回の poll。最大 max_frames 本。note_tx(requested, written) は書くたびに呼ぶ（usb_diag の tx! 用。nullptr 可）。
  template <class Pop, class Port>
  void run(Pop pop, Port& port, uint32_t now_ms, int max_frames, void (*note_tx)(size_t, size_t) = nullptr) {
    for (int i = 0; i < max_frames; ++i) {
      if (!has_held_) {
        if (!pop(&held_)) return;
        has_held_ = true;
        held_since_ms_ = now_ms;
        waited_ = false;
      }
      const bool is_status = held_.type == msg::STATUS;
      const uint32_t age = now_ms - held_.enqueued_ms;
      const uint32_t waited = waited_ ? now_ms - held_since_ms_ : 0;
      size_t free_bytes = 0;
      bool connected = true;
      if (!(is_status && age >= policy.status_stale_ms)) {
        free_bytes = port.available_for_write();
        if (free_bytes < held_.len) connected = port.connected();
      }
      const TxAction a = tx_decide(is_status, held_.len, free_bytes, connected, age, waited, policy);
      switch (a) {
        case TxAction::Write:
        case TxAction::WriteUnconnected: {
          const size_t written = port.write(held_.data, held_.len);
          if (note_tx) note_tx(held_.len, written);
          stats.written += 1;
          has_held_ = false;
          break;
        }
        case TxAction::DropStale:
          stats.stale_dropped += 1;
          has_held_ = false;
          break;
        case TxAction::DropAged:
          stats.aged_dropped += 1;
          has_held_ = false;
          break;
        case TxAction::Defer:
          if (!waited_) {
            waited_ = true;
            held_since_ms_ = now_ms;
          }
          stats.deferred += 1;
          return;  // 順序を保つ: 後ろは次の poll
      }
    }
  }

 private:
  Frame held_{};
  bool has_held_ = false;
  bool waited_ = false;        // 一度でも Defer した
  uint32_t held_since_ms_ = 0; // 最初に Defer した時刻
};

}  // namespace v1
