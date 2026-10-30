#include "pmic_log.h"

#include <M5Unified.h>
#include <esp_attr.h>
#include <esp_system.h>

// 出力する行（値は整数。電圧は mV、温度は 0.1 ℃）
//   PMIC up=<s> vbus= vsys= vbat= ts= tdie= s0= s1= i48= i49= i4a= r18= r68= | rst= rtc= on= off=
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
}

void pmic_log_tick() {
  if (!booted) {
    return;
  }
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_boot_line_ms) >= 0) {
    next_boot_line_ms = now + 10000;
    printBootLine();
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
  Serial.printf("PMIC up=%lu vbus=%u vsys=%u vbat=%u ts=%u tdie=%d s0=%02X s1=%02X i48=%02X i49=%02X i4a=%02X "
                "r18=%02X r68=%02X | rst=%d rtc=%d on=%02X off=%02X\n",
                static_cast<unsigned long>(up), vbus, vsys, vbat, ts, tdie, s0, s1, i48, i49, i4a, r18, r68,
                boot.reset_reason, boot.rtc_ok ? 1 : 0, boot.on, boot.off);
}
