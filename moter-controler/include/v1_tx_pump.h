#pragma once

// USB プロトコル v1 の送信側の「ブロックしない書き込み」（Issue #15 M7）。Serial・FreeRTOS を持たない純粋なロジック
//（ホストテストの対象）。v1_link.cpp の loop() 側 v1_link_poll() が Serial の adapter と送信キューを渡して回す。
//
// 背景: HWCDC::write は TX リングに空きが無いと tx timeout × 最大 20 回まで待つ。loop() が描画（pushSprite 数十 ms）と
// NVS 保存も抱えているので、PC が読んでいない間に write で固まると SET_POSITION の受信まで止まり、START が
// 「期限内の SET_POSITION なし」で拒否される（11-01）。FreeRTOS タスク化は 2 回失敗している（v1_link.cpp）ので、
// 書く前に availableForWrite() を見て、入らなければ書かずに次の poll に回す。
//
// 規則（tx_decide）。age = 積まれてからの時間、held = 空き不足で保留し始めてからの時間:
//  STATUS（置き換えてよいフレーム。20 ms ごとに新しいものが来る）
//   - age ≥ status_stale_ms なら送らない（DropStale）。詰まりが解けたときに古い STATUS を順に流さず、新しいものだけ送る。
//   - 空きがフレーム長以上なら書く（Write）。無ければ捨てる（DropStale）。
//  PMIC_REPORT（診断。約 1 Hz。Issue #68）
//   - 空きがフレーム長以上なら書く（古くても書く。スナップショットを 100 ms で捨てない）。
//   - 無ければ捨てる（DropStale）。保留しない。後ろの ACK をこの 1 本で止めない。
//  それ以外（ACK・FAULT_EVENT・HELLO・LIMITS・CAL_RESULT・AXIS_SELECT。**ここでは決して捨てない**。順序を保つ）
//   - 空きがフレーム長以上なら書く（Write。RINGBUF_TYPE_BYTEBUF の空きは byte 単位なので xRingbufferSend は待たずに入る。
//     ISR はリングから取り出すだけなので、見てから書くまでに空きが減ることはない）。
//   - 無ければ保留して順序を保つ（Defer。この poll はここで終わる。後ろはキューに残る）。
//   - 保留が hold_write_through_ms 続いたら空きが無くても書く（WriteThrough）。availableForWrite() が何かの状態で 0 を返し続けても
//     送信が完全に止まらないための退路で、r1（初版）の「未接続なら書く」に相当する。HWCDC::write の待ちは tx timeout × 20 で
//     有界（v1 では main.cpp が tx timeout を 1 ms にするので最大 20 ms ≈ 旧来の 1 回分）。未接続ならリングの古い内容を置き換えて
//     即返る。一度 WriteThrough に入ったら空きが戻るまで後続もそれぞれ hold_write_through_ms ずつ待つので、詰まったリンクでも
//     キューは少なくとも 1 本 / hold_write_through_ms で進む（溜まった ACK は PC 側の期限で無視される）。
//
// 時刻の注意: now は run() の入口で 1 回取り、フレームの enqueued_ms は別コアの制御タスクが積むときの millis()。run() の途中で
// 積まれたフレームは enqueued_ms が now より新しいことがある（11-03 未明の実機: START の ACK がこれで age が巻き戻って
// 期限切れ扱いになり届かなかった）。差は符号付きで取り、負なら 0 とみなす（tx_age_ms）。

#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "v1_codec.h"  // msg::STATUS

namespace v1 {

enum class TxAction : uint8_t { Write, WriteThrough, DropStale, Defer };

struct TxPolicy {
  uint32_t status_stale_ms = V1_TX_STATUS_STALE_MS;
  uint32_t hold_write_through_ms = V1_TX_HOLD_WRITE_THROUGH_MS;
};

struct TxPumpStats {
  uint32_t written = 0;          // Serial に渡したフレーム数（written_through を含む）
  uint32_t deferred = 0;         // 空き不足で次の poll に回した回数（poll ごとに 1）
  uint32_t stale_dropped = 0;    // 古い・入らない STATUS を捨てた数
  uint32_t written_through = 0;  // 保留が hold_write_through_ms 続いて空きを見ずに書いた数
};

// now より新しい enqueued（別コアが run() の途中で積んだ）は 0 扱い。uint32 の巻き戻りで巨大な age にしない。
inline uint32_t tx_age_ms(uint32_t now_ms, uint32_t enqueued_ms) {
  const int32_t d = static_cast<int32_t>(now_ms - enqueued_ms);
  return d < 0 ? 0 : static_cast<uint32_t>(d);
}

inline TxAction tx_decide(bool is_status, size_t len, size_t free_bytes, uint32_t age_ms, uint32_t held_ms,
                          const TxPolicy& p) {
  if (is_status) {
    if (age_ms >= p.status_stale_ms) return TxAction::DropStale;
    return free_bytes >= len ? TxAction::Write : TxAction::DropStale;
  }
  if (free_bytes >= len) return TxAction::Write;
  if (held_ms >= p.hold_write_through_ms) return TxAction::WriteThrough;
  return TxAction::Defer;
}

// Frame は len / data / type / enqueued_ms を持つ（V1TxFrame）。
// Pop は bool(Frame*)（キューの先頭を取り出す。無ければ false）。
// Port は size_t available_for_write()、size_t write(const uint8_t*, size_t)。
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
        waited_ = false;
      }
      const bool is_status = held_.type == msg::STATUS;
      const bool is_pmic = held_.type == msg::PMIC_REPORT;
      const uint32_t age = tx_age_ms(now_ms, held_.enqueued_ms);
      const uint32_t held_ms = waited_ ? tx_age_ms(now_ms, held_since_ms_) : 0;
      size_t free_bytes = 0;
      if (is_pmic || !(is_status && age >= policy.status_stale_ms)) free_bytes = port.available_for_write();
      // PMIC は空きが無ければ捨てて次へ。保留すると後ろの ACK が止まる。
      TxAction a = is_pmic
                       ? (free_bytes >= held_.len ? TxAction::Write : TxAction::DropStale)
                       : tx_decide(is_status, held_.len, free_bytes, age, held_ms, policy);
      switch (a) {
        case TxAction::Write:
        case TxAction::WriteThrough: {
          const size_t written = port.write(held_.data, held_.len);
          if (note_tx) note_tx(held_.len, written);
          stats.written += 1;
          if (a == TxAction::WriteThrough) stats.written_through += 1;
          has_held_ = false;
          break;
        }
        case TxAction::DropStale:
          stats.stale_dropped += 1;
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
  bool waited_ = false;         // 保留中（一度でも Defer した）
  uint32_t held_since_ms_ = 0;  // 最初に Defer した時刻
};

}  // namespace v1
