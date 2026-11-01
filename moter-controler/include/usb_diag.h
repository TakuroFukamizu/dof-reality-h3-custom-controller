#pragma once

// Issue #11（USB に列挙されたまま CDC が無応答になる）の切り分け用。
// ESP32-S3 の USB-Serial/JTAG 周辺のレジスタと HWCDC の状態を、ボードをリセットせずに画面で見るための窓。
// 読むだけで状態を変えないものだけを使う（HWCDC::isConnected() / availableForWrite() は IN_EMPTY 割り込みの
// 再有効化や mutex 待ちを伴うので使わない）。エラー系の raw ビットと IN token ビットは HWCDC が使わないので、
// 読んだ後にこちらで消す。

#include <stddef.h>
#include <stdint.h>

struct UsbDiag {
  bool plugged;          // HWCDC::isPlugged()（tick hook の SOF 判定。読むだけ）
  bool sof_alive;        // 前回サンプルから SOF フレーム番号が進んだ（バスが生きている）
  bool in_token;         // 前回サンプルから IN endpoint 1 への IN token が来た（ホストがこのアドレスを読みに来ている）
  bool out_data_avail;   // OUT FIFO にデータが残っている（ISR が取っていない）
  bool in_data_free;     // TX FIFO に空きがある（ホストが読み出した）
  uint16_t sof_frame;    // 直近の SOF フレーム番号（11 bit）
  uint16_t int_raw;      // USB_SERIAL_JTAG.int_raw（下位 12 bit）
  uint16_t int_ena;      // USB_SERIAL_JTAG.int_ena（下位 12 bit）
  uint32_t bus_resets;   // HWCDC の BUS_RESET イベント回数（起動から）
  uint32_t bus_errors;   // pid / crc5 / crc16 / stuff error の raw ビットが立っていたサンプル数の合計
  uint32_t rx_bytes;     // アプリが Serial.read() で受け取ったバイト数
  uint32_t rx_age_ms;    // 最後に受信してからの時間（未受信なら UINT32_MAX）
  uint32_t tx_bytes;     // Serial.write が受け付けたバイト数
  uint32_t tx_short;     // Serial.write が要求より少なく返した回数（timeout で捨てた）
};

void usb_diag_begin();                          // Serial.begin() の後に 1 回
void usb_diag_note_rx(size_t n);                // 受信したバイト数を足す
void usb_diag_note_tx(size_t requested, size_t written);
void usb_diag_sample(UsbDiag* out);             // レジスタを読む。描画周期（≤10 Hz）で呼ぶ
void usb_diag_format(const UsbDiag& d, char* buf, size_t n);  // 1 行（ASCII、50 字以内）
uint32_t usb_diag_bus_resets();                 // イベント回数だけ（ログ用）
