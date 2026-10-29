#pragma once

// Issue #1（DIN 12V と USB の同時接続で CoreS3 が電源断する）の調査用。
// AXP2101 の状態を 1 秒ごとにシリアルへ出す。env:m5stack-cores3-pmiclog（MC_PMIC_LOG=1）でだけ有効。
// PMIC には、ADC のダイ温度チャネルを有効にする 1 ビット以外は書き込まない。IRQ 状態もクリアしない。

// M5.begin() の直後に 1 回呼ぶ。起動時のレジスタと、前回の最終値（RTC メモリ）を取っておく。
void pmic_log_boot();

// loop() から毎回呼ぶ。1 秒ごとに PMIC 行、10 秒ごとに PMICBOOT 行を出す。
void pmic_log_tick();
