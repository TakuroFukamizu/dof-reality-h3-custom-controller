#include "v1_link.h"

#if MC_PROTO_V1

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_mac.h>
#include <esp_system.h>

#include <atomic>
#include <string.h>

#include "v1_axis.h"  // v1_device_id_from_mac

namespace {

constexpr UBaseType_t kRxDepth = 16;
constexpr UBaseType_t kTxDepth = 32;
constexpr int kRxBytesPerPoll = 2048;  // 1 回の loop() で読む上限（画面描画を飢えさせない）
constexpr int kTxFramesPerPoll = 4;    // 1 回の loop() で書く上限（1 フレームの write は最大 20 ms 待つ）

QueueHandle_t g_rx = nullptr;
QueueHandle_t g_tx = nullptr;
v1::FrameSplitter g_split;
uint32_t g_oversize_seen = 0;
std::atomic<uint32_t> g_rx_dropped{0};
std::atomic<uint32_t> g_tx_dropped{0};
uint64_t g_device_id = 0;
uint32_t g_boot_id = 0;

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
  return g_rx != nullptr && g_tx != nullptr;
}

void v1_link_poll() {
  if (g_rx == nullptr || g_tx == nullptr) return;

  // 受信: バイト → COBS 区切り → 受信キュー
  int budget = kRxBytesPerPoll;
  while (budget-- > 0 && Serial.available() > 0) {
    const int c = Serial.read();
    if (c < 0) break;
    if (g_split.push(static_cast<uint8_t>(c))) {
      V1RxFrame f;
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

  // 送信: 送信キュー → Serial（順序を保つため 1 本のキューだけ）
  V1TxFrame t;
  for (int i = 0; i < kTxFramesPerPoll; ++i) {
    if (xQueueReceive(g_tx, &t, 0) != pdTRUE) break;
    Serial.write(t.data, t.len);
  }
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
  if (xQueueSend(g_tx, &t, 0) != pdTRUE) {
    g_tx_dropped.fetch_add(1);
    return false;
  }
  return true;
}

uint32_t v1_link_take_rx_dropped() { return g_rx_dropped.exchange(0); }
uint32_t v1_link_tx_dropped() { return g_tx_dropped.load(); }
uint64_t v1_link_device_id() { return g_device_id; }
uint32_t v1_link_boot_id() { return g_boot_id; }

#endif  // MC_PROTO_V1
