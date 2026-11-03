#pragma once

#include <stdint.h>

// AXP2101 の読み出し（Issue #1 / #68）。
//
// レジスタを読む処理と、テキスト行を出す処理は分かれている。
//   env:m5stack-cores3-pmiclog（MC_PMIC_LOG）は従来どおりシリアルへテキストを出す。
//     PMIC / PMICPRE / PMICBOOT の並びと書式は変えない。
//   env:m5stack-cores3-v1（MC_PROTO_V1）は常に読む（MC_PMIC_LOG は不要）。
//     結果は RAM のスナップショットに残す。USB CDC は v1 のバイナリ専用なので、Serial へ文字は出さない。
//     1 秒ごとの読みは loop()（UI）からだけ行う。制御タスク（5 ms）からは I2C を触らない。
//
// PMIC への書き込みは 2 つだけ: ADC のダイ温度チャネルを有効にする 1 ビットと、POR の目印（データバッファ REG04〜07）。
// IRQ 状態はクリアしない。

// M5.begin() より前に読むレジスタ（PMICPRE 行の r00 … i4a と同じ順）。
constexpr uint8_t kPmicPreRegs[] = {0x00, 0x01, 0x14, 0x20, 0x21, 0x23, 0x80, 0x82,
                                    0x83, 0x84, 0x85, 0x86, 0x90, 0x48, 0x49, 0x4A};
constexpr int kPmicPreRegCount = 16;
static_assert(sizeof(kPmicPreRegs) == kPmicPreRegCount, "pre regs");

// 起動時の基準値（PMICBOOT 行の reg= と同じ順）。
constexpr uint8_t kPmicBaselineRegs[] = {0x10, 0x15, 0x16, 0x18, 0x19, 0x22, 0x23, 0x24,
                                         0x27, 0x30, 0x50, 0x62, 0x64, 0x68};
constexpr int kPmicBaselineRegCount = 14;
static_assert(sizeof(kPmicBaselineRegs) == kPmicBaselineRegCount, "baseline regs");

// POR の目印。データバッファ REG04〜07。前回の起動で "H368" を書く。
constexpr uint8_t kPmicMark[4] = {0x48, 0x33, 0x36, 0x38};

// M5.begin() より前の読み（PMICPRE）。done が false のときはまだ呼んでいない。
struct PmicPreSnapshot {
  bool done;
  uint8_t ok;       // 読めた数（目印 4 byte を 1 回と数える）
  uint8_t tried;    // 読もうとした数。I2C が開けなければ 0
  uint32_t t_ms;    // 読んだときの millis()
  uint64_t t_us;    // 読んだときの esp_timer（v1 の mcu_time）
  bool mark_ok;
  uint8_t mark[4];
  uint8_t regs[kPmicPreRegCount];
  bool regs_ok[kPmicPreRegCount];
};

// M5.begin() の直後の読み（PMICBOOT）。valid が false のときはまだ呼んでいない。
struct PmicBootSnapshot {
  bool valid;
  int reset_reason;
  bool rtc_ok;
  uint32_t last_up;
  uint16_t last_vbus;
  uint16_t last_vsys;
  uint16_t last_vbat;
  uint8_t last_s0;
  uint8_t last_s1;
  uint8_t on;  // REG20
  uint8_t off;  // REG21（前回の電源断要因）
  uint8_t irq48;
  uint8_t irq49;
  uint8_t irq4a;
  bool on_ok;
  bool off_ok;
  bool irq48_ok;
  bool irq49_ok;
  bool irq4a_ok;
  uint8_t baseline[kPmicBaselineRegCount];
  bool baseline_ok[kPmicBaselineRegCount];
  uint8_t read_ok;
  uint8_t read_tried;
  uint64_t t_us;
};

// 1 秒ごとの読み（PMIC 行）。seq は読むたびに 1 増える（0 はまだ無い）。
struct PmicPeriodic {
  bool valid;
  uint32_t seq;
  uint32_t t_ms;
  uint64_t t_us;
  uint32_t up_s;
  uint16_t vbus;
  uint16_t vsys;
  uint16_t vbat;
  uint16_t ts;
  int tdie_deci;  // 0.1 ℃。読めなかったときは計算しない（0）
  bool vbus_ok;
  bool vsys_ok;
  bool vbat_ok;
  bool ts_ok;
  bool tdie_ok;
  uint8_t s0;
  uint8_t s1;
  uint8_t i48;
  uint8_t i49;
  uint8_t i4a;
  uint8_t r18;
  uint8_t r68;
  uint8_t r14;
  uint8_t r23;
  bool s0_ok;
  bool s1_ok;
  bool i48_ok;
  bool i49_ok;
  bool i4a_ok;
  bool r18_ok;
  bool r68_ok;
  bool r14_ok;
  bool r23_ok;
  uint8_t read_ok;
  uint8_t read_tried;
};

// M5.begin() の前に 1 回呼ぶ。内部 I2C（G12 SDA / G11 SCL）を自分で開いて AXP2101 を読み、閉じる。
// M5Unified / M5GFX が REG23 などを書き換える前の値をスナップショットに残す。
// テキスト（PMICPRE 行）は pmic_log_boot() 以降、MC_PMIC_LOG のときだけ出す。
void pmic_log_preboot();

// M5.begin() の直後に 1 回呼ぶ。起動時のレジスタと、前回の最終値（RTC メモリ）をスナップショットに残す。
void pmic_log_boot();

// loop() から毎回呼ぶ。1 秒ごとにレジスタを読む。
// MC_PMIC_LOG では加えて PMIC 行を出し、10 秒ごとに PMICBOOT 行と PMICPRE 行を出す。
// MC_PROTO_V1 では文字を出さない。I2C が失敗したらその回の残りは読まず、失敗回数だけ数える
// （バスが死んでいるときに 1 トランザクションの待ちで打ち切る）。
void pmic_log_tick();

// スナップショットのコピー。制御タスクから呼んでよい（I2C はしない）。準備前は false。
bool pmic_copy_pre(PmicPreSnapshot* out);
bool pmic_copy_boot(PmicBootSnapshot* out);
bool pmic_copy_periodic(PmicPeriodic* out);

// 起動以降の I2C 失敗回数（飽和しない uint32。実トランザクションの失敗だけ。打ち切って読まなかった分は含まない）。
uint32_t pmic_i2c_error_count();
