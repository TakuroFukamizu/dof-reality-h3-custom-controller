#pragma once

// Issue #1（DIN 12V と USB の同時接続で CoreS3 が電源断する）の調査用。
// AXP2101 の状態を 1 秒ごとにシリアルへ出す。env:m5stack-cores3-pmiclog（MC_PMIC_LOG=1）でだけ有効。
// PMIC への書き込みは 2 つだけ: ADC のダイ温度チャネルを有効にする 1 ビットと、POR の目印（データバッファ REG04〜07）。
// IRQ 状態はクリアしない。
// Issue #68（Base15 + USB 給電中に CoreS3 が DCDC 低電圧で切れたまま）の調査で、M5.begin() より前の値も読む。

// M5.begin() の前に 1 回呼ぶ。内部 I2C（G12 SDA / G11 SCL）を自分で開いて AXP2101 を読み、閉じる。
// M5Unified / M5GFX が REG23 などを書き換える前の値を取っておき、pmic_log_boot() 以降に PMICPRE 行で出す。
void pmic_log_preboot();

// M5.begin() の直後に 1 回呼ぶ。起動時のレジスタと、前回の最終値（RTC メモリ）を取っておく。
void pmic_log_boot();

// loop() から毎回呼ぶ。1 秒ごとに PMIC 行、10 秒ごとに PMICBOOT 行と PMICPRE 行を出す。
void pmic_log_tick();
