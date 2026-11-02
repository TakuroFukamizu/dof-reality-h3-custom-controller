#include "v1_link.h"

#if MC_PROTO_V1

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <atomic>
#include <string.h>

#include "usb_diag.h"
#include "v1_axis.h"  // v1_device_id_from_mac
#include "v1_tx_pump.h"

void v1_link_poll();

namespace {

constexpr UBaseType_t kRxDepth = 16;
constexpr UBaseType_t kTxDepth = 32;
constexpr int kRxBytesPerPoll = 2048;  // 1 周期で読む上限
constexpr int kTxFramesPerPoll = 4;    // 1 周期で書く上限。書く前に TX リングの空きを見るので write で待たない（v1_tx_pump.h。
                                       // 旧: 空きが無いと HWCDC::write は tx timeout 20 ms × 最大 20 回 = 400 ms 待ち得た）
constexpr uint32_t kLinkPeriodMs = 1;  // リンクタスクの周期

QueueHandle_t g_rx = nullptr;
QueueHandle_t g_tx = nullptr;
v1::FrameSplitter g_split;
uint32_t g_oversize_seen = 0;
std::atomic<uint32_t> g_rx_dropped{0};
std::atomic<uint32_t> g_tx_dropped{0};
std::atomic<uint32_t> g_tx_deferred{0};
std::atomic<uint32_t> g_tx_stale_dropped{0};
uint64_t g_device_id = 0;
uint32_t g_boot_id = 0;
v1::TxPump<V1TxFrame> g_pump;  // loop() だけが触る

// Serial（HWCDC）を v1_tx_pump.h の Port にする。空きが無ければ書かない（接続状態は見ない。v1_tx_pump.h の注意）
struct SerialPort {
  size_t available_for_write() {
    const int n = Serial.availableForWrite();
    return n > 0 ? static_cast<size_t>(n) : 0;
  }
  size_t write(const uint8_t* data, size_t len) { return Serial.write(data, len); }
};

bool pop_tx(V1TxFrame* out) { return xQueueReceive(g_tx, out, 0) == pdTRUE; }

void link_task(void*) {
  for (;;) {
    v1_link_poll();
    vTaskDelay(pdMS_TO_TICKS(kLinkPeriodMs));
  }
}

}  // namespace

bool v1_link_begin() {
  g_rx = xQueueCreate(kRxDepth, sizeof(V1RxFrame));
  g_tx = xQueueCreate(kTxDepth, sizeof(V1TxFrame));
  // eFuse MAC を表記順（先頭オクテットが上位）に詰める。ESP.getEfuseMac() は逆順（先頭オクテットが最下位）なので使わない。
  // status の device_id は 5131 + MAC の 12 桁、画面の `dev XXXX` は MAC の末尾 2 オクテットになる。
  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  esp_efuse_mac_get_default(mac);
  g_device_id = v1_device_id_from_mac(mac);
  g_boot_id = esp_random();
  if (g_boot_id == 0) g_boot_id = 1;
  if (g_rx == nullptr || g_tx == nullptr) return false;
  // 専用タスクは使わない（loop() から v1_link_poll() を呼ぶ）。2026-11-01 の実機: Core 0 のタスクから書くと
  // 状態変化で ACK と STATUS を続けて送る瞬間に PC 側で CRC 不一致、Core 1 のタスクにすると HOLDING に入って
  // 約 3 s で 2 台同時に USB が無応答（HELLO にも答えない）になった。原因未特定のため、動作実績のある
  // loop() 方式に戻し、受信の遅れは描画の間引き（100 ms に 1 回）とサーバー側の期限（valid_for 100 ms）で吸収する。
  // 送信は書く前に空きを見てブロックしない（Issue #15 M7、v1_tx_pump.h）。
  (void)link_task;
  return true;
}

void v1_link_poll() {
  if (g_rx == nullptr || g_tx == nullptr) return;

  // 受信: バイト → COBS 区切り → 受信キュー
  int budget = kRxBytesPerPoll;
  while (budget-- > 0 && Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    usb_diag_note_rx(1);
    if (g_split.push(static_cast<uint8_t>(c))) {
      V1RxFrame f;
      f.rx_us = static_cast<uint64_t>(esp_timer_get_time());  // Issue #45: 制御タスクが取り出すまでの待ちを再生の時刻に入れない
      f.len = static_cast<uint16_t>(g_split.len);
      memcpy(f.data, g_split.buf, g_split.len);
      if (xQueueSend(g_rx, &f, 0) != pdTRUE) {
        g_rx_dropped.fetch_add(1);  // 受信キュー満杯。応答しないフレームとして数える
      }
    }
  }
  if (g_split.dropped != g_oversize_seen) {
    g_rx_dropped.fetch_add(g_split.dropped - g_oversize_seen);
    g_oversize_seen = g_split.dropped;
  }

  // 送信: 送信キュー → Serial（順序を保つため 1 本のキューだけ）。空きが無ければ保留／STATUS は捨てる（v1_tx_pump.h）
  // STATUS 以外は捨てない（保留 → 空きが戻るか V1_TX_HOLD_WRITE_THROUGH_MS で書く）。now は run() の入口で 1 回取るが、
  // 制御タスク（別コア）が run() の途中で積んだフレームは enqueued_ms が新しいことがあり、pump 側が符号付きで吸収する。
  SerialPort port;
  const v1::TxPumpStats before = g_pump.stats;
  g_pump.run(pop_tx, port, millis(), kTxFramesPerPoll, usb_diag_note_tx);
  const v1::TxPumpStats& after = g_pump.stats;
  if (after.deferred != before.deferred) g_tx_deferred.fetch_add(after.deferred - before.deferred);
  // old = 捨てた STATUS + 空きを見ずに書いた（WriteThrough）数
  const uint32_t old_now = (after.stale_dropped - before.stale_dropped) + (after.written_through - before.written_through);
  if (old_now) g_tx_stale_dropped.fetch_add(old_now);
}

bool v1_link_rx_pop(V1RxFrame* out) {
  if (g_rx == nullptr) return false;
  return xQueueReceive(g_rx, out, 0) == pdTRUE;
}

bool v1_link_tx_push(const v1::McuMsg& m) {
  if (g_tx == nullptr) return false;
  V1TxFrame t;
  const size_t n = v1::encode_mcu_wire(m, t.data, sizeof(t.data));
  if (n == 0) return true;  // 符号化できない（起こらない）。捨てて前へ進む
  t.len = static_cast<uint16_t>(n);
  t.type = m.type;
  t.enqueued_ms = millis();
  if (xQueueSend(g_tx, &t, 0) != pdTRUE) {
    g_tx_dropped.fetch_add(1);
    return false;
  }
  return true;
}

uint32_t v1_link_take_rx_dropped() { return g_rx_dropped.exchange(0); }
uint32_t v1_link_tx_dropped() { return g_tx_dropped.load(); }
uint32_t v1_link_tx_deferred() { return g_tx_deferred.load(); }
uint32_t v1_link_tx_stale_dropped() { return g_tx_stale_dropped.load(); }
uint64_t v1_link_device_id() { return g_device_id; }
uint32_t v1_link_boot_id() { return g_boot_id; }

#endif  // MC_PROTO_V1
