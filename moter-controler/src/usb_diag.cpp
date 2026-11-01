#include "usb_diag.h"

#include <Arduino.h>
#include <HWCDC.h>
#include <soc/usb_serial_jtag_struct.h>

#include <atomic>
#include <stdio.h>

// bus reset の回数は HWCDC の esp_event 経由でしか取れない。登録すると arduino_hw_cdc_events タスク（優先度 5）が
// 立ち、RX / TX のたびに ISR から esp_event_isr_post_to が走る（キュー 5、溢れは捨てる）。件 #5/#6 はタスク構成を
// 変えた試作で起きたので、この計器自体が条件を変える可能性を残す。A/B 比較のため -DMC_USB_DIAG_BUSRESET=0 で外せる。
// Serial.onEvent は void で、イベントループの生成失敗は分からない（CORE_DEBUG_LEVEL=0 の v1 では log_e も出ない）。
// br が 0 のままなのは「bus reset なし」のほかに「数え始められなかった」もあり得る。
// イベントタスクは制御タスク（Core 1、優先度 5）と同じ優先度でコア固定なし。制御周期への影響は v1 画面の
// 「最大間隔 us」（tick gap）で見える。
#ifndef MC_USB_DIAG_BUSRESET
#define MC_USB_DIAG_BUSRESET 1
#endif

namespace {

// int_raw / int_ena のビット位置（usb_serial_jtag_struct.h）
constexpr uint32_t kBitSof = 1u << 1;
constexpr uint32_t kBitPidErr = 1u << 4;
constexpr uint32_t kBitCrc5Err = 1u << 5;
constexpr uint32_t kBitCrc16Err = 1u << 6;
constexpr uint32_t kBitStuffErr = 1u << 7;
constexpr uint32_t kBitInToken = 1u << 8;
constexpr uint32_t kErrMask = kBitPidErr | kBitCrc5Err | kBitCrc16Err | kBitStuffErr;

std::atomic<uint32_t> g_bus_resets{0};
std::atomic<uint32_t> g_rx_bytes{0};
std::atomic<uint32_t> g_tx_short{0};
std::atomic<uint32_t> g_last_rx_ms{0};
std::atomic<bool> g_rx_seen{false};
uint32_t g_bus_errors = 0;
uint16_t g_last_frame = 0;
bool g_have_frame = false;

// HWCDC の ISR から esp_event 経由で届く。登録すると RX / TX ごとにもイベントが積まれる（キュー 5、溢れは捨てられる）
// が、ここでは BUS_RESET だけ数える。
void onCdcEvent(void*, esp_event_base_t, int32_t id, void*) {
  if (id == ARDUINO_HW_CDC_BUS_RESET_EVENT) {
    g_bus_resets.fetch_add(1);
  }
}

}  // namespace

void usb_diag_begin() {
  g_last_frame = USB_SERIAL_JTAG.fram_num.sof_frame_index;
  g_have_frame = true;
#if MC_USB_DIAG_BUSRESET
  Serial.onEvent(ARDUINO_HW_CDC_BUS_RESET_EVENT, onCdcEvent);
#endif
}

void usb_diag_note_rx(size_t n) {
  if (n == 0) return;
  g_rx_bytes.fetch_add(static_cast<uint32_t>(n));
  g_last_rx_ms.store(millis());
  g_rx_seen.store(true);
}

// 注意: HWCDC::write は isCDC_Connected() が false のとき TX リングを捨てて「全部書けた」と返す（drop policy）。
// ここで数えられるのは mutex 待ちの timeout（0 を返す）とリング満杯の timeout（途中まで）だけ。接続判定 false での
// 破棄は tx! に出ないので、TX 側の判定は if（TX FIFO の空き）と「rx は増えるのにホストに届かない」で行う。
void usb_diag_note_tx(size_t requested, size_t written) {
  if (written < requested) {
    g_tx_short.fetch_add(1);
  }
}

uint32_t usb_diag_bus_resets() { return g_bus_resets.load(); }

// 読んで消す IN token / エラーのビットは「前回サンプルからの有無」なので、USBDIAG コマンドで呼んだ直後の定期サンプルは
// 一瞬 in0 になり得る（橙が 1 回だけ点く）。
void usb_diag_sample(UsbDiag* out) {
  const uint32_t raw = USB_SERIAL_JTAG.int_raw.val;
  const uint32_t ena = USB_SERIAL_JTAG.int_ena.val;
  const uint16_t frame = USB_SERIAL_JTAG.fram_num.sof_frame_index;
  // HWCDC が使わないビット（エラー 4 種と IN token）はこちらで消して、次のサンプルまでの有無を見る。
  USB_SERIAL_JTAG.int_clr.val = (raw & (kErrMask | kBitInToken));
  if (raw & kErrMask) {
    ++g_bus_errors;
  }
  out->plugged = HWCDC::isPlugged();
  out->sof_alive = g_have_frame && frame != g_last_frame;
  g_last_frame = frame;
  g_have_frame = true;
  out->in_token = (raw & kBitInToken) != 0;
  out->out_data_avail = USB_SERIAL_JTAG.ep1_conf.serial_out_ep_data_avail != 0;
  out->in_data_free = USB_SERIAL_JTAG.ep1_conf.serial_in_ep_data_free != 0;
  out->sof_frame = frame;
  out->int_raw = static_cast<uint16_t>(raw & 0x0FFF & ~kBitSof);  // SOF は毎 ms 立って消えるので位相で揺れる
  out->int_ena = static_cast<uint16_t>(ena & 0x0FFF);
  out->bus_resets = g_bus_resets.load();
  out->bus_errors = g_bus_errors;
  out->rx_bytes = g_rx_bytes.load();
  out->rx_age_ms = g_rx_seen.load() ? millis() - g_last_rx_ms.load() : UINT32_MAX;
  out->tx_short = g_tx_short.load();
}

void usb_diag_format(const UsbDiag& d, char* buf, size_t n) {
  // 例: "sof1 in1 r000 e20c oa0 if1 br0 er0 tx!0 2s rx1234"
  // 6x8 フォントで x=8 から描くので 52 字まで。rx を最後に置き、桁が伸びて切れるのは rx だけにする。
  // isPlugged() は sof とほぼ同じ情報なので行には出さない（構造体には残す）。
  //   sof: SOF が進んだ  in: IN token が来た  r/e: int_raw / int_ena（hex、SOF bit 除く）
  //   oa: OUT FIFO にデータ残り  if: TX FIFO に空き  br: bus reset 回数  er: バスエラー回数
  //   tx!: 送信の取りこぼし回数  <age>: 最終受信からの時間（10 s 以降は段階表示。描き直しを減らす）
  //   rx: 受信バイト数（100000 以上は k）
  char rx[12];
  if (d.rx_bytes < 100000) {
    snprintf(rx, sizeof(rx), "%lu", static_cast<unsigned long>(d.rx_bytes));
  } else {
    snprintf(rx, sizeof(rx), "%luk", static_cast<unsigned long>(d.rx_bytes / 1000));
  }
  char age[8];
  const uint32_t ms = d.rx_age_ms;
  if (ms == UINT32_MAX) {
    snprintf(age, sizeof(age), "-");
  } else if (ms < 10000) {
    snprintf(age, sizeof(age), "%lus", static_cast<unsigned long>(ms / 1000));
  } else if (ms < 30000) {
    snprintf(age, sizeof(age), "10s+");
  } else if (ms < 60000) {
    snprintf(age, sizeof(age), "30s+");
  } else if (ms < 300000) {
    snprintf(age, sizeof(age), "1m+");
  } else {
    snprintf(age, sizeof(age), "5m+");
  }
  snprintf(buf, n, "sof%d in%d r%03x e%03x oa%d if%d br%lu er%lu tx!%lu %s rx%s", d.sof_alive ? 1 : 0,
           d.in_token ? 1 : 0, static_cast<unsigned>(d.int_raw), static_cast<unsigned>(d.int_ena),
           d.out_data_avail ? 1 : 0, d.in_data_free ? 1 : 0, static_cast<unsigned long>(d.bus_resets),
           static_cast<unsigned long>(d.bus_errors), static_cast<unsigned long>(d.tx_short), age, rx);
}
