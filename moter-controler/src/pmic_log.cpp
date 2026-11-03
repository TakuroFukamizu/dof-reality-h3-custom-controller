#include "pmic_log.h"

#include <M5Unified.h>
#include <esp_attr.h>
#include <esp_system.h>

#include <cstring>

// 出力する行（値は整数。電圧は mV、温度は 0.1 ℃）
//   PMIC up=<s> vbus= vsys= vbat= ts= tdie= s0= s1= i48= i49= i4a= r18= r68= | rst= rtc= on= off= r14= r23=
//   PMICPRE ok=<n>/<m> t_ms= por= mark= r00= r01= r14= r20= r21= r23= r80= r82= r83= r84= r85= r86= r90= i48= i49= i4a=
//   PMICBOOT rst= rtc= last_up= last_vbus= last_vsys= last_vbat= last_s0= last_s1= on= off= i48= i49= i4a= reg=<基準レジスタ>
//
// 読み方（AXP2101 datasheet）
//   s0  = REG00: b5 VBUS good, b3 battery present, b1 thermal regulation, b0 input current limit
//   s1  = REG01: b4 system power on, b3 VINDPM, b2:0 charge status
//   on  = 起動時の REG20（起動要因）: b2 VBUS insert, b0 POWERON key
//   off = 起動時の REG21（前回の電源断要因、POR でだけ消える）:
//         b7 die OT, b6 DCDC OV, b5 DCDC UV, b4 VBUS OV, b3 VSYS UV, b1 software, b0 key
//   i49 = REG49: b7/b6 VBUS insert/remove, b5/b4 battery insert/remove
//   i4a = REG4A: b7 watchdog, b6 LDO OC, b5 BATFET OC, b2 die OT level 1
//   ts  = TS ピンの ADC 生値。CoreS3 では BUS_OUT（M-Bus 5V）の分圧で、おおよそ BUS_OUT の mV になる
//   rtc = 1 なら前回の最終値が RTC メモリに残っていた（ESP32 だけのリセット）。0 なら電源ごと落ちた
//   r14/r23 = PMIC 行では毎回読み直した REG14（最小システム電圧）と REG23（DCDC の低電圧・過電圧で切る機能）
//
// PMICPRE（Issue #68）: M5.begin() より前、M5Unified / M5GFX が PMIC を設定する前に読んだ値。
//   ok    = 読めたレジスタ数 / 読もうとした数。0 なら内部 I2C が開けなかった
//   t_ms  = 読んだときの millis()
//   mark  = データバッファ REG04〜07 の値。前回の起動でここに目印 "H368"（48333638）を書く。
//   por   = 1 なら目印が無かった = AXP2101 が POR から起動した（初回の書き込み直後も 1）。
//           0 なら POR していない（電源キーや VBUS 挿入での OFF→ON、または ESP32 だけのリセット）
//   r23   = M5Unified が b0〜b4（DCDC1〜5 の低電圧で切る）を切る前の値。POR 直後は既定値が見える
//   r80   = DCDC の ON/OFF、r82〜r86 = DCDC1〜5 の設定電圧、r90 = LDO の ON/OFF
//   r14   = SWcharge 版データシートでは b2:0 が最小システム電圧（3.2〜3.9 V、既定 3.7 V）。
//           linear charger 版とビット配置が違うので、書き換えには使わない

namespace {

constexpr uint32_t kRtcMagic = 0x504D4943;  // "PMIC"

RTC_NOINIT_ATTR uint32_t rtc_magic;
RTC_NOINIT_ATTR uint32_t rtc_uptime_s;
RTC_NOINIT_ATTR uint16_t rtc_vbus;
RTC_NOINIT_ATTR uint16_t rtc_vsys;
RTC_NOINIT_ATTR uint16_t rtc_vbat;
RTC_NOINIT_ATTR uint8_t rtc_s0;
RTC_NOINIT_ATTR uint8_t rtc_s1;

struct BootInfo {
  int reset_reason;
  bool rtc_ok;
  uint32_t last_up;
  uint16_t last_vbus;
  uint16_t last_vsys;
  uint16_t last_vbat;
  uint8_t last_s0;
  uint8_t last_s1;
  uint8_t on;
  uint8_t off;
  uint8_t irq48;
  uint8_t irq49;
  uint8_t irq4a;
};

// 起動時の基準値として読むレジスタ。EFUSE 依存の値（0x22, 0x24 など）を記録する。
constexpr uint8_t kBaselineRegs[] = {0x10, 0x15, 0x16, 0x18, 0x19, 0x22, 0x23, 0x24,
                                     0x27, 0x30, 0x50, 0x62, 0x64, 0x68};
uint8_t baseline[sizeof(kBaselineRegs)];

BootInfo boot = {};
bool booted = false;
uint32_t next_line_ms = 0;
uint32_t next_boot_line_ms = 0;

uint8_t reg8(uint8_t reg) { return M5.Power.Axp2101.readRegister8(reg); }

// ---- M5.begin() より前の読み出し（Issue #68） ----
constexpr uint8_t kAxpAddr = 0x34;
constexpr uint32_t kPreI2cFreq = 400000;
constexpr uint8_t kMark[4] = {0x48, 0x33, 0x36, 0x38};  // "H368"
constexpr uint8_t kPreRegs[] = {0x00, 0x01, 0x14, 0x20, 0x21, 0x23, 0x80, 0x82,
                                0x83, 0x84, 0x85, 0x86, 0x90, 0x48, 0x49, 0x4A};
constexpr const char* kPreNames[] = {"r00", "r01", "r14", "r20", "r21", "r23", "r80", "r82",
                                     "r83", "r84", "r85", "r86", "r90", "i48", "i49", "i4a"};
static_assert(sizeof(kPreRegs) == sizeof(kPreNames) / sizeof(kPreNames[0]), "names");

struct PreInfo {
  bool done;
  uint8_t ok;
  uint8_t tried;
  uint32_t t_ms;
  bool mark_ok;
  uint8_t mark[4];
  uint8_t regs[sizeof(kPreRegs)];
  bool regs_ok[sizeof(kPreRegs)];
};
PreInfo pre = {};

void printPreLine() {
  if (!pre.done) {
    return;
  }
  // 目印を読めなかったときは por を判定しない（-1）
  const int por = pre.mark_ok ? (memcmp(pre.mark, kMark, sizeof(kMark)) == 0 ? 0 : 1) : -1;
  Serial.printf("PMICPRE ok=%u/%u t_ms=%lu por=%d mark=%02X%02X%02X%02X", pre.ok, pre.tried,
                static_cast<unsigned long>(pre.t_ms), por, pre.mark[0], pre.mark[1], pre.mark[2], pre.mark[3]);
  for (size_t i = 0; i < sizeof(kPreRegs); ++i) {
    if (pre.regs_ok[i]) {
      Serial.printf(" %s=%02X", kPreNames[i], pre.regs[i]);
    } else {
      Serial.printf(" %s=--", kPreNames[i]);
    }
  }
  Serial.print("\n");
}

// ADC は 14 ビット。getVBUSVoltage() は VBUS good が落ちると 0 を返すので、生値を自分で読む。
uint16_t adc14(uint8_t reg) {
  uint8_t buf[2] = {0, 0};
  M5.Power.Axp2101.readRegister(reg, buf, 2);
  return static_cast<uint16_t>(((buf[0] & 0x3F) << 8) | buf[1]);
}

// ダイ温度（0.1 ℃）。datasheet の式 22 + (7274 - raw) / 20 [℃]。
int tdie_deci() {
  const int raw = adc14(0x3C);
  return 220 + (7274 - raw) / 2;
}

void printBootLine() {
  Serial.printf("PMICBOOT rst=%d rtc=%d last_up=%lu last_vbus=%u last_vsys=%u last_vbat=%u last_s0=%02X last_s1=%02X "
                "on=%02X off=%02X i48=%02X i49=%02X i4a=%02X reg=",
                boot.reset_reason, boot.rtc_ok ? 1 : 0, static_cast<unsigned long>(boot.last_up), boot.last_vbus,
                boot.last_vsys, boot.last_vbat, boot.last_s0, boot.last_s1, boot.on, boot.off, boot.irq48,
                boot.irq49, boot.irq4a);
  for (size_t i = 0; i < sizeof(kBaselineRegs); ++i) {
    Serial.printf("%s%02X:%02X", i == 0 ? "" : ",", kBaselineRegs[i], baseline[i]);
  }
  Serial.print("\n");
}

}  // namespace

void pmic_log_preboot() {
  pre = {};
  pre.done = true;
  pre.t_ms = millis();
  // CoreS3 の内部 I2C（AXP2101 / AW9523 など）。M5.begin() も同じポート・ピンで開き直す。
  if (!m5::In_I2C.begin(I2C_NUM_1, GPIO_NUM_12, GPIO_NUM_11)) {
    return;
  }
  pre.tried = sizeof(kPreRegs) + 1;
  pre.mark_ok = m5::In_I2C.readRegister(kAxpAddr, 0x04, pre.mark, sizeof(pre.mark), kPreI2cFreq);
  if (pre.mark_ok) {
    ++pre.ok;
  }
  for (size_t i = 0; i < sizeof(kPreRegs); ++i) {
    pre.regs_ok[i] = m5::In_I2C.readRegister(kAxpAddr, kPreRegs[i], &pre.regs[i], 1, kPreI2cFreq);
    if (pre.regs_ok[i]) {
      ++pre.ok;
    }
  }
  // 次の起動で「POR したか」を見分ける目印。データバッファは POR でだけ消える。
  if (pre.mark_ok) {
    m5::In_I2C.writeRegister(kAxpAddr, 0x04, kMark, sizeof(kMark), kPreI2cFreq);
  }
  m5::In_I2C.release();
}

void pmic_log_boot() {
  boot.reset_reason = static_cast<int>(esp_reset_reason());
  boot.rtc_ok = rtc_magic == kRtcMagic;
  if (boot.rtc_ok) {
    boot.last_up = rtc_uptime_s;
    boot.last_vbus = rtc_vbus;
    boot.last_vsys = rtc_vsys;
    boot.last_vbat = rtc_vbat;
    boot.last_s0 = rtc_s0;
    boot.last_s1 = rtc_s1;
  }
  boot.on = reg8(0x20);
  boot.off = reg8(0x21);
  boot.irq48 = reg8(0x48);
  boot.irq49 = reg8(0x49);
  boot.irq4a = reg8(0x4A);
  for (size_t i = 0; i < sizeof(kBaselineRegs); ++i) {
    baseline[i] = reg8(kBaselineRegs[i]);
  }
  // M5Unified は REG30 = 0x0F でダイ温度（b4）を有効にしないので、そのビットだけ立てる。
  M5.Power.Axp2101.bitOn(0x30, 0x10);
#if MC_PMIC_NOBATT
  // Issue #1 の検証。DIN Base のバッテリーを外して使うときの設定を試す。
  // 外したバッテリー端子を DIN Base の充電 IC が約 4.3 V に保ち、AXP2101 がそれを電池とみなして
  // 充電・過電圧保護・BATFET 開閉を繰り返していた。
  // REG18 b1 = 0: 電池への充電を止める（M5Unified の setBatteryCharge）。System Reset で 1 に戻る。
  // REG68 b0 = 0: 電池の検出を止める。datasheet 6.7.3.5 によると、止めると「常に電池あり」とみなす。POR で 1 に戻る。
  // M5.begin() が REG68 = 0x01 を書くので、begin() の後に書く。
  M5.Power.setBatteryCharge(false);
  M5.Power.Axp2101.bitOff(0x68, 0x01);
#endif

  rtc_magic = kRtcMagic;
  rtc_uptime_s = 0;
  booted = true;
  printPreLine();
}

void pmic_log_tick() {
  if (!booted) {
    return;
  }
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_boot_line_ms) >= 0) {
    next_boot_line_ms = now + 10000;
    printBootLine();
    printPreLine();
  }
  if (static_cast<int32_t>(now - next_line_ms) < 0) {
    return;
  }
  next_line_ms = now + 1000;

  const uint16_t vbus = adc14(0x38);
  const uint16_t vsys = adc14(0x3A);
  const uint16_t vbat = adc14(0x34);
  const uint16_t ts = adc14(0x36);
  const int tdie = tdie_deci();
  const uint8_t s0 = reg8(0x00);
  const uint8_t s1 = reg8(0x01);
  const uint8_t i48 = reg8(0x48);
  const uint8_t i49 = reg8(0x49);
  const uint8_t i4a = reg8(0x4A);
  const uint32_t up = now / 1000;

  rtc_uptime_s = up;
  rtc_vbus = vbus;
  rtc_vsys = vsys;
  rtc_vbat = vbat;
  rtc_s0 = s0;
  rtc_s1 = s1;

  const uint8_t r18 = reg8(0x18);
  const uint8_t r68 = reg8(0x68);
  const uint8_t r14 = reg8(0x14);
  const uint8_t r23 = reg8(0x23);
  // on / off は起動時に読んだ値の再表示。r14 / r23 は毎回読み直す（後から書き換わっていないかを見る）
  Serial.printf("PMIC up=%lu vbus=%u vsys=%u vbat=%u ts=%u tdie=%d s0=%02X s1=%02X i48=%02X i49=%02X i4a=%02X "
                "r18=%02X r68=%02X | rst=%d rtc=%d on=%02X off=%02X r14=%02X r23=%02X\n",
                static_cast<unsigned long>(up), vbus, vsys, vbat, ts, tdie, s0, s1, i48, i49, i4a, r18, r68,
                boot.reset_reason, boot.rtc_ok ? 1 : 0, boot.on, boot.off, r14, r23);
}
