#include "pmic_log.h"

#include <M5Unified.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include <atomic>
#include <cstring>

// テキスト行（MC_PMIC_LOG だけ。値は整数。電圧は mV、温度は 0.1 ℃）
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
//
// v1（MC_PROTO_V1）では上の行は出さない。スナップショットを RAM に残し、PMIC_REPORT で送る。
// 1 秒読みで I2C が失敗したら、その回の残りはバスを触らない（M5GFX の I2C タイムアウトは最大約 13 ms。
// 失敗のたびに待つと UI ループと USB の受信が数十 ms 止まる）。

namespace {

constexpr uint32_t kRtcMagic = 0x504D4943;  // "PMIC"

RTC_NOINIT_ATTR uint32_t rtc_magic;
RTC_NOINIT_ATTR uint32_t rtc_uptime_s;
RTC_NOINIT_ATTR uint16_t rtc_vbus;
RTC_NOINIT_ATTR uint16_t rtc_vsys;
RTC_NOINIT_ATTR uint16_t rtc_vbat;
RTC_NOINIT_ATTR uint8_t rtc_s0;
RTC_NOINIT_ATTR uint8_t rtc_s1;

constexpr const char* kPreNames[] = {"r00", "r01", "r14", "r20", "r21", "r23", "r80", "r82",
                                     "r83", "r84", "r85", "r86", "r90", "i48", "i49", "i4a"};
static_assert(sizeof(kPreNames) / sizeof(kPreNames[0]) == kPmicPreRegCount, "names");

constexpr uint8_t kAxpAddr = 0x34;
constexpr uint32_t kPreI2cFreq = 400000;

PmicPreSnapshot pre = {};
PmicBootSnapshot boot = {};
bool booted = false;
uint32_t next_line_ms = 0;
uint32_t next_boot_line_ms = 0;

// 1 秒読みは loop() が書き、制御タスクが読む。スロットを 2 つにして、公開中の側は書き換えない。
PmicPeriodic periodic_slots[2] = {};
uint32_t periodic_pub = 0;
uint32_t periodic_seq = 0;
portMUX_TYPE pmic_mux = portMUX_INITIALIZER_UNLOCKED;
std::atomic<uint32_t> i2c_errors{0};

// この回の読みでバスが死んだ。v1 では残りを読まない。pmiclog では従来どおり全部読む（行を変えない）。
bool bus_dead = false;

void note_i2c_err() {
  i2c_errors.fetch_add(1, std::memory_order_relaxed);
#if MC_PROTO_V1
  bus_dead = true;
#endif
}

void publish_periodic(const PmicPeriodic& p) {
  const uint32_t next = periodic_pub ^ 1u;
  periodic_slots[next] = p;
  portENTER_CRITICAL(&pmic_mux);
  periodic_pub = next;
  portEXIT_CRITICAL(&pmic_mux);
}

// ADC は 14 ビット。getVBUSVoltage() は VBUS good が落ちると 0 を返すので、生値を自分で読む。
// 失敗したときは 0。AXP2101 の VBAT / VBUS / VSYS は 1 mV/LSB。
bool read_adc(uint8_t reg, uint16_t* out) {
  uint8_t buf[2] = {0, 0};
  if (bus_dead) {
    *out = 0;
    return false;
  }
  const bool ok = M5.Power.Axp2101.readRegister(reg, buf, 2);
  *out = static_cast<uint16_t>(((buf[0] & 0x3F) << 8) | buf[1]);
  if (!ok) {
    *out = 0;
    note_i2c_err();
    return false;
  }
  return true;
}

bool read8(uint8_t reg, uint8_t* out) {
  uint8_t b = 0;
  if (bus_dead) {
    *out = 0;
    return false;
  }
  const bool ok = M5.Power.Axp2101.readRegister(reg, &b, 1);
  if (!ok) {
    *out = 0;
    note_i2c_err();
    return false;
  }
  *out = b;
  return true;
}

// ダイ温度（0.1 ℃）。datasheet の式 22 + (7274 - raw) / 20 [℃]。raw を読めなかったときは ok = false。
bool read_tdie(int* out) {
  uint16_t raw = 0;
  if (!read_adc(0x3C, &raw)) {
    *out = 0;
    return false;
  }
  *out = 220 + (7274 - static_cast<int>(raw)) / 2;
  return true;
}

#if !MC_PROTO_V1
void printPreLine() {
  if (!pre.done) {
    return;
  }
  // 目印を読めなかったときは por を判定しない（-1）
  const int por = pre.mark_ok ? (memcmp(pre.mark, kPmicMark, sizeof(kPmicMark)) == 0 ? 0 : 1) : -1;
  Serial.printf("PMICPRE ok=%u/%u t_ms=%lu por=%d mark=%02X%02X%02X%02X", pre.ok, pre.tried,
                static_cast<unsigned long>(pre.t_ms), por, pre.mark[0], pre.mark[1], pre.mark[2], pre.mark[3]);
  for (size_t i = 0; i < kPmicPreRegCount; ++i) {
    if (pre.regs_ok[i]) {
      Serial.printf(" %s=%02X", kPreNames[i], pre.regs[i]);
    } else {
      Serial.printf(" %s=--", kPreNames[i]);
    }
  }
  Serial.print("\n");
}

void printBootLine() {
  Serial.printf("PMICBOOT rst=%d rtc=%d last_up=%lu last_vbus=%u last_vsys=%u last_vbat=%u last_s0=%02X last_s1=%02X "
                "on=%02X off=%02X i48=%02X i49=%02X i4a=%02X reg=",
                boot.reset_reason, boot.rtc_ok ? 1 : 0, static_cast<unsigned long>(boot.last_up), boot.last_vbus,
                boot.last_vsys, boot.last_vbat, boot.last_s0, boot.last_s1, boot.on, boot.off, boot.irq48,
                boot.irq49, boot.irq4a);
  for (size_t i = 0; i < kPmicBaselineRegCount; ++i) {
    Serial.printf("%s%02X:%02X", i == 0 ? "" : ",", kPmicBaselineRegs[i], boot.baseline[i]);
  }
  Serial.print("\n");
}
#endif

}  // namespace

void pmic_log_preboot() {
  pre = {};
  pre.done = true;
  pre.t_ms = millis();
  pre.t_us = static_cast<uint64_t>(esp_timer_get_time());
  // CoreS3 の内部 I2C（AXP2101 / AW9523 など）。M5.begin() も同じポート・ピンで開き直す。
  if (!m5::In_I2C.begin(I2C_NUM_1, GPIO_NUM_12, GPIO_NUM_11)) {
    return;
  }
  bus_dead = false;
  pre.tried = kPmicPreRegCount + 1;
  pre.mark_ok = m5::In_I2C.readRegister(kAxpAddr, 0x04, pre.mark, sizeof(pre.mark), kPreI2cFreq);
  if (pre.mark_ok) {
    ++pre.ok;
  } else {
    note_i2c_err();
    memset(pre.mark, 0, sizeof(pre.mark));
  }
  for (size_t i = 0; i < kPmicPreRegCount; ++i) {
    if (bus_dead) {
      pre.regs_ok[i] = false;
      pre.regs[i] = 0;
      continue;
    }
    pre.regs_ok[i] = m5::In_I2C.readRegister(kAxpAddr, kPmicPreRegs[i], &pre.regs[i], 1, kPreI2cFreq);
    if (pre.regs_ok[i]) {
      ++pre.ok;
    } else {
      pre.regs[i] = 0;
      note_i2c_err();
    }
  }
  // 次の起動で「POR したか」を見分ける目印。データバッファは POR でだけ消える。
  // 目印を読めているときだけ書く（従来と同じ）。v1 でバスが死んでいても、ここまで来ていれば書く。
  if (pre.mark_ok) {
    m5::In_I2C.writeRegister(kAxpAddr, 0x04, kPmicMark, sizeof(kPmicMark), kPreI2cFreq);
  }
  m5::In_I2C.release();
  bus_dead = false;
}

void pmic_log_boot() {
  bus_dead = false;
  boot = {};
  boot.valid = true;
  boot.t_us = static_cast<uint64_t>(esp_timer_get_time());
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
  boot.read_tried = 5 + kPmicBaselineRegCount;
  boot.on_ok = read8(0x20, &boot.on);
  boot.off_ok = read8(0x21, &boot.off);
  boot.irq48_ok = read8(0x48, &boot.irq48);
  boot.irq49_ok = read8(0x49, &boot.irq49);
  boot.irq4a_ok = read8(0x4A, &boot.irq4a);
  boot.read_ok = static_cast<uint8_t>(boot.on_ok + boot.off_ok + boot.irq48_ok + boot.irq49_ok + boot.irq4a_ok);
  for (size_t i = 0; i < kPmicBaselineRegCount; ++i) {
    boot.baseline_ok[i] = read8(kPmicBaselineRegs[i], &boot.baseline[i]);
    if (boot.baseline_ok[i] && boot.read_ok < 255) ++boot.read_ok;
  }
  // M5Unified は REG30 = 0x0F でダイ温度（b4）を有効にしないので、そのビットだけ立てる。
  // バスが死んでいたら書かない（書こうとしてまた待たない）。
  if (!bus_dead) {
    M5.Power.Axp2101.bitOn(0x30, 0x10);
  }
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
  bus_dead = false;
#if !MC_PROTO_V1
  printPreLine();
#endif
}

void pmic_log_tick() {
  if (!booted) {
    return;
  }
  const uint32_t now = millis();
#if !MC_PROTO_V1
  if (static_cast<int32_t>(now - next_boot_line_ms) >= 0) {
    next_boot_line_ms = now + 10000;
    printBootLine();
    printPreLine();
  }
#endif
  if (static_cast<int32_t>(now - next_line_ms) < 0) {
    return;
  }
  next_line_ms = now + 1000;
  bus_dead = false;

  PmicPeriodic p = {};
  p.valid = true;
  p.t_ms = now;
  p.t_us = static_cast<uint64_t>(esp_timer_get_time());
  p.up_s = now / 1000;
  p.read_tried = 14;
  p.vbus_ok = read_adc(0x38, &p.vbus);
  p.vsys_ok = read_adc(0x3A, &p.vsys);
  p.vbat_ok = read_adc(0x34, &p.vbat);
  p.ts_ok = read_adc(0x36, &p.ts);
  p.tdie_ok = read_tdie(&p.tdie_deci);
  p.s0_ok = read8(0x00, &p.s0);
  p.s1_ok = read8(0x01, &p.s1);
  p.i48_ok = read8(0x48, &p.i48);
  p.i49_ok = read8(0x49, &p.i49);
  p.i4a_ok = read8(0x4A, &p.i4a);
  p.r18_ok = read8(0x18, &p.r18);
  p.r68_ok = read8(0x68, &p.r68);
  p.r14_ok = read8(0x14, &p.r14);
  p.r23_ok = read8(0x23, &p.r23);
  const int oks = p.vbus_ok + p.vsys_ok + p.vbat_ok + p.ts_ok + p.tdie_ok + p.s0_ok + p.s1_ok + p.i48_ok +
                  p.i49_ok + p.i4a_ok + p.r18_ok + p.r68_ok + p.r14_ok + p.r23_ok;
  p.read_ok = static_cast<uint8_t>(oks);

  // 次回の起動が RTC メモリを見る（ESP32 だけのリセットか、電源ごと落ちたか）。
  // 読めた値だけ残す。失敗した電圧で前回値を潰さない。
  if (p.vbus_ok) rtc_vbus = p.vbus;
  if (p.vsys_ok) rtc_vsys = p.vsys;
  if (p.vbat_ok) rtc_vbat = p.vbat;
  if (p.s0_ok) rtc_s0 = p.s0;
  if (p.s1_ok) rtc_s1 = p.s1;
  rtc_uptime_s = p.up_s;
  rtc_magic = kRtcMagic;

  periodic_seq += 1;
  p.seq = periodic_seq;
  publish_periodic(p);
  bus_dead = false;

#if !MC_PROTO_V1
  // on / off は起動時に読んだ値の再表示。r14 / r23 は毎回読み直す（後から書き換わっていないかを見る）
  Serial.printf("PMIC up=%lu vbus=%u vsys=%u vbat=%u ts=%u tdie=%d s0=%02X s1=%02X i48=%02X i49=%02X i4a=%02X "
                "r18=%02X r68=%02X | rst=%d rtc=%d on=%02X off=%02X r14=%02X r23=%02X\n",
                static_cast<unsigned long>(p.up_s), p.vbus, p.vsys, p.vbat, p.ts, p.tdie_deci, p.s0, p.s1, p.i48,
                p.i49, p.i4a, p.r18, p.r68, boot.reset_reason, boot.rtc_ok ? 1 : 0, boot.on, boot.off, p.r14, p.r23);
#endif
}

bool pmic_copy_pre(PmicPreSnapshot* out) {
  if (out == nullptr || !pre.done) return false;
  portENTER_CRITICAL(&pmic_mux);
  *out = pre;
  portEXIT_CRITICAL(&pmic_mux);
  return true;
}

bool pmic_copy_boot(PmicBootSnapshot* out) {
  if (out == nullptr || !boot.valid) return false;
  portENTER_CRITICAL(&pmic_mux);
  *out = boot;
  portEXIT_CRITICAL(&pmic_mux);
  return true;
}

bool pmic_copy_periodic(PmicPeriodic* out) {
  if (out == nullptr) return false;
  portENTER_CRITICAL(&pmic_mux);
  *out = periodic_slots[periodic_pub];
  portEXIT_CRITICAL(&pmic_mux);
  return out->valid;
}

uint32_t pmic_i2c_error_count() { return i2c_errors.load(std::memory_order_relaxed); }
