#pragma once

// USB プロトコル v1 のリンク層（Arduino 側）。
// loop()（Core 0）が Serial のバイトを COBS の区切りで 1 フレームずつ受信キューへ入れ、送信キューから
// フレームを取り出して Serial.write する。制御タスク（Core 1）は受信キューから取り出し、送信キューへ積む。
// 書き込みは loop() 側だけで行う（PC が読んでいないとき制御タスクが止まらないように）。

#include <stddef.h>
#include <stdint.h>

#include "v1_codec.h"

struct V1RxFrame {
  uint16_t len;                       // COBS 符号化のまま（区切りなし）
  uint8_t data[v1::MAX_ENCODED_LEN];
};

struct V1TxFrame {
  uint16_t len;                       // COBS 符号化 + 区切り 0x00
  uint8_t data[v1::MAX_WIRE_LEN];
};

bool v1_link_begin();                        // キュー生成、device_id / boot_id の決定
void v1_link_poll();                         // loop() から毎回
bool v1_link_rx_pop(V1RxFrame* out);         // 制御タスク。無ければ false
bool v1_link_tx_push(const v1::McuMsg& m);   // 制御タスク。満杯なら false（呼び出し側が保留 / 破棄を決める）
uint32_t v1_link_take_rx_dropped();          // メッセージ層より下で捨てた受信フレーム数（前回呼び出しから）
uint32_t v1_link_tx_dropped();               // 送信キュー満杯で push に失敗した回数（累計、表示用）
uint64_t v1_link_device_id();                // eFuse MAC 48 bit、上位 16 bit は 0x5131
uint32_t v1_link_boot_id();                  // esp_random()
