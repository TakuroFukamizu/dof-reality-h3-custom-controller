# Ways to isolate why the CoreS3 + DIN Base powers off when both 5 V sources are connected

Scope: practical, cheap-first diagnostics for Issue #1 (CoreS3 SE + DIN Base; 12 V on the DC jack → DIN 5 V on M-Bus, plus USB-C from a Mac; no battery; GND shared with the 24 V motor system). Every claim has a source. Local primary evidence (datasheet text, M5Unified source in this repo, this Mac's unified log) is labelled "local".

Primary sources used repeatedly:
- AXP2101 datasheet V1.4 (X-Powers), as distributed in XPowersLib: https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf (text extracted locally with pdftotext; section numbers are quoted below)
- XPowersLib register map: https://github.com/lewisxhe/XPowersLib/blob/master/src/REG/AXP2101Constants.h and the driver https://github.com/lewisxhe/XPowersLib/blob/master/src/XPowersAXP2101.hpp
- M5Unified as resolved by this project: `moter-controler/.pio/libdeps/m5stack-cores3/M5Unified` (library.json `"version": "0.2.26"`), files `src/utility/Power_Class.inl` and `src/utility/power/AXP2101_Class.inl/.hpp` (upstream: https://github.com/m5stack/M5Unified)

---

## Q1. Telling apart (a) AXP2101 power-off, (b) ESP32-S3 brownout reset or crash, (c) the Mac disabling the port: what each looks like and what to capture

### Takeaway
What we have already seen (screen dark, USB gone, stays off until **both** sources are removed for ~20 s) fits (a) best. An AXP2101 in its OFF state stays off while any input keeps it above UVLO, and only a POR clears it. A brownout or crash (b) self-resets and re-enumerates within seconds. A Mac-side port shutdown (c) does not turn off the CoreS3 screen while 12 V is present. The deciding evidence is AXP2101 **REG 0x21 (PWROFF status)**. It is reset **only by POR**, so the usual recovery (pull both cables for 20 s) erases it. Next time, recover **without a POR**: leave the cables in, hold the power key, and read REG 0x20/0x21 on that boot.

### Cited Findings
**What the AXP2101 does (datasheet V1.4, local text)**
- POR: "AXP2101 is powered from the higher voltage between VBUS and BAT. When VBUS voltage(VVBUS) is higher than VVBUS_UVLOZ or BAT voltage … All registers are reset to the default value." (§6.5.1). VVBUS_UVLOZ = 3.75 V typ.; VBUS over-voltage rising threshold VVBUS_OV = 7 V; operating range 3.9–5.5 V (electrical table) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- OFF state: "PMU has power off and power on status. When at off state, all voltage outputs are turned off except RTCLDO." (§6.5.4) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- Power-on sources with PWRON configured as a key: (1) POK held longer than ONLEVEL, (2) "VBUS low go high" (configurable), (3) VBAT low go high, (4) IRQ pin low >16 ms, (5) battery charged to normal (§6.5.4.2) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- Power-off sources (§6.5.4.3): POK long press; writing 1 to REG10H[0]; "VSYSGOOD high go low. When VSYS<VOFF or VBUS>7V, AXP2101 will be powered off. The default of VOFF is 2.6V which can be configured by REG24H[2:0]"; DCDC output 15 % below its setting (REG23H[4:0]); DCDC output far above its setting (REG23H[5]); die temperature above warning level 2 (145 °C) (REG22H[2]) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- **REG 0x21 PWROFF status** (all bits RO, Reset = **POR**): b7 die over-temperature, b6 DCDC over-voltage, b5 DCDC under-voltage, b4 VBUS over-voltage, b3 VSYS under-voltage, b2 POWERON always low (EN mode), b1 software power-off, b0 POWERON pulled down for off level (key) (§6.13.2.18) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf); the same bit order is used by `getPowerOffSource()` in [XPowersAXP2101.hpp](https://github.com/lewisxhe/XPowersLib/blob/master/src/XPowersAXP2101.hpp)
- **REG 0x20 PWRON status** (Reset = System Reset): b5 EN mode, b4 battery insert, b3 battery >3.3 V when charged, **b2 VBUS insert and good**, b1 IRQ pin pulled low, **b0 POWERON key** (§6.13.2.17) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- REG 0x22 PWROFF_EN: b2 die OT level-2 power-off enable (default 1), b1 PWRON>OFFLEVEL power-off enable (EFUSE), b0 power-off vs restart after a key power-off (EFUSE) (§6.13.2.19). REG 0x23: b5 DCDC 120 %/130 % OVP turn-off (default 1), b4..b0 DCDC5..1 85 % UVP turn-off (default 1) (§6.13.2.20). REG 0x24 b2:0 = VOFF 2.6–3.3 V (EFUSE) (§6.13.2.21) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- REG 0x00 PMU status1: **b5 VBUS good**, b4 BATFET, b3 battery present, b2 battery active mode, **b1 thermal regulation**, **b0 in current limit** (§6.13.2.1). REG 0x01 PMU status2: b6:5 battery current direction, **b4 system power on/off**, **b3 VINDPM active**, b2:0 charge status (§6.13.2.2) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- REG 0x15 VINDPM = 3.88 V + N×0.08 V, default 0110b = 4.36 V. REG 0x16 input current limit: 000=100 mA, 001=500, 010=900, 011=1000, 100=1500 (default), 101=2000 mA (§6.13.2.11–12). §6.5.3.2: "When VBUS voltage reaches VINDPM, the charge current will decrease automatically until the current is zero. If ISYS is over the input power supply capability, VSYS will drop. If VBAT is above VSYS, PMU will enter the supplement mode." — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- IRQ status (RW1C, write 1 to clear): REG 0x48 (battery/gauge), **REG 0x49** b7 VBUS insert, b6 VBUS remove, b5/b4 battery insert/remove, b3–b0 power-key events (POR reset for b7–b4), **REG 0x4A** b7 watchdog expire, b6 LDO over-current, b5 BATFET over-current, b2 die over-temperature level 1, b1 charger safety timer (§6.13.2.43–45) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- DATA_BUFFER REG 0x04–0x07: 4 RW bytes, Reset = POR (§6.13.2.3–6). They survive an ESP32 reset and a PMIC soft-off, but not a PMIC POR — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- REG 0x19 TWSI watchdog: 1/2/4/8/16/32/64/128 s, enabled only by REG18[0] (default 0 = disabled) (§6.13.2.14–15) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- ADC: 14-bit, VBUS LSB 1 mV, up to 8.192 V (§6.x ADC table); VBUS 0x38/0x39, VSYS 0x3A/0x3B, TDIE 0x3C/0x3D — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)

**What M5Unified 0.2.26 does to the AXP2101 on CoreS3/CoreS3 SE (local source)**
- In `Power_Class.inl` `begin()` for `board_M5StackCoreS3`/`board_M5StackCoreS3SE`: `Axp2101.bitOff(0x23, 0x1F)` ("DCDC1-5 UVP power-off disable"), so **DCDC under-voltage power-off is disabled but DCDC OVP (REG23 b5) stays enabled**. It also writes `0x27=0x00` (comment: "PowerKey Hold=1sec / PowerOff=4sec"), `0x10=0x30`, `0x30=0x0F` (ADC enable), `0x50|=0x10` (TS pin as external input), and `setChargeVoltage(4200)`. It sets the AW9523 "SY7088 BOOST_EN" bit — [M5Unified](https://github.com/m5stack/M5Unified) (local: `moter-controler/.pio/libdeps/m5stack-cores3/M5Unified/src/utility/Power_Class.inl` ~L409–440)
- `AXP2101_Class::begin()` only reads REG 0x03 (chip ID 0x4A). Nothing in `M5.begin()` clears IRQ status or touches REG 0x20/0x21, so firmware can read them after `M5.begin()` — [M5Unified](https://github.com/m5stack/M5Unified) (local `AXP2101_Class.inl` L31–43; grep for `clearIRQStatuses` in Power_Class.inl found nothing)
- Accessors: `M5.Power.Axp2101` is a public `AXP2101_Class` member and inherits `readRegister8(reg)` from `I2C_Device`. Also available: `isVBUS()` (REG00 b5), `getVBUSVoltage()` (REG 0x38, **returns 0.0 when REG00 b5 is clear**, so a 0 does not mean 0 V), `getAPSVoltage()` (VSYS, 0x3A), `getInternalTemperature()` (`22 + (7274 - reg16(0x3C))/20`), `getIRQStatuses()`, `clearIRQStatuses()`, `isVbusInsertIrq()`/`isVbusRemoveIrq()`/`isBatDieOverTemperatureIrq()`/`isLdoOverCurrentIrq()` — [M5Unified](https://github.com/m5stack/M5Unified) (local `AXP2101_Class.hpp` L119–189, `.inl` L202–300)
- The project firmware uses `cfg.output_power = false` and `M5.Power.setUsbOutput(false)` (local: `moter-controler/src/main.cpp` L778, L790). On CoreS3, `setUsbOutput` drives AW9523 bit `_core_s3_usb_en = 0b00100000` ("USB OTG EN") (local `Power_Class.inl` L901, L1426–1430) — [M5Unified](https://github.com/m5stack/M5Unified)

**ESP32-S3 side (ESP-IDF docs + local framework)**
- `esp_reset_reason()` values: ESP_RST_POWERON "Reset due to power-on event", ESP_RST_BROWNOUT "Brownout reset (software or hardware)", ESP_RST_PANIC, ESP_RST_INT_WDT, ESP_RST_TASK_WDT, ESP_RST_WDT, ESP_RST_SW, ESP_RST_DEEPSLEEP, ESP_RST_EXT, ESP_RST_USB "Reset by USB peripheral", ESP_RST_JTAG, ESP_RST_PWR_GLITCH, ESP_RST_CPU_LOCKUP, ESP_RST_UNKNOWN — [ESP-IDF Misc System API (ESP32-S3)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/misc_system_api.html)
- Build used here: `platform = espressif32@6.12.0`, `framework = arduino` (local `moter-controler/platformio.ini`). Installed Arduino core package version 3.20017 (= Arduino-ESP32 2.0.17), whose ESP32-S3 sdkconfig has `CONFIG_ESP32S3_BROWNOUT_DET_LVL 7`, `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y`, `CONFIG_ESP_CONSOLE_UART_DEFAULT 1`, and **`CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG 1`**. So the system console (panic/brownout text) goes to UART0 and is also mirrored to USB-Serial-JTAG (local `~/.platformio/packages/framework-arduinoespressif32/tools/sdk/esp32s3/qio_qspi/include/sdkconfig.h`).
- Brownout handling with `CONFIG_ESP_BROWNOUT_USE_INTR` (local ESP-IDF `components/esp_hw_support/power_supply/brownout.c`, newer IDF than the one in Arduino 2.0.17, same design): the BOD interrupt handler stalls the other core, calls `esp_reset_reason_set_hint(ESP_RST_BROWNOUT)`, prints "Brownout detector was triggered", flushes UART FIFOs, then calls `esp_rom_software_reset_system()`. That is a **software system reset**, not a power-on reset — [ESP-IDF source](https://github.com/espressif/esp-idf/blob/master/components/esp_hw_support/power_supply/brownout.c)
- The ESP32-S3 brownout Kconfig labels level 7 as "2.44V" (6 = 2.56 V … 1 = 3.30 V). It warns "there may be some variation of brownout voltage level between each ESP3-S3 chip" and "The voltage levels here are estimates" (local IDF `components/esp_hw_support/power_supply/port/esp32s3/Kconfig.power`; similar wording in [Kconfig mirror](https://codex.ac6.fr/Zephyr/xref/hal_espressif-latest/components/esp_system/port/soc/esp32/Kconfig.system), [esp32.com thread](https://esp32.com/viewtopic.php?p=130361)).
- USB-Serial/JTAG: in deep sleep "the USB Serial/JTAG device appears disconnected from the host/PC", and after light sleep the host may report the device as disconnected — [ESP-IDF USJ console](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html)

### Inferences
- **Signature table** (built from the findings above; the timings are reasoning, not measurements):

| Cause | Screen | Mac USB | DIN red LED | Recovery | Evidence on the next boot |
|---|---|---|---|---|---|
| (a) AXP2101 OFF (protection or soft-off) | dark, stays dark | detach, no re-enumeration | stays on (DIN side unaffected) | key press should restart it if VBUS is still "good"; otherwise only a POR (both sources removed until the input falls below 3.75 V) | REG21 ≠ 0 names the cause **if no POR happened**; ESP reason POWERON; RTC_NOINIT lost |
| (a′) AXP2101 POR (its input dropped below UVLO) | dark | detach | may flicker | auto power-on depends on the "VBUS low→high" EFUSE option | REG21 = 0, REG20 b2 set, ESP POWERON |
| (b) ESP32 brownout | brief blank, then reboots | detach then re-enumerates within a few seconds | — | automatic | ESP_RST_BROWNOUT (ring buffer in RTC memory survives if RTC stayed powered; unverified for S3 brownout) |
| (b′) panic or WDT | reboot (or hang if the panic handler stalls) | detach/re-enumerate | — | automatic | ESP_RST_PANIC / *_WDT; core dump if enabled |
| (b″) firmware hang with rails up | frozen image, backlight on | may stay enumerated with transfer errors | — | reset button | nothing; the screen stays lit, which rules out "dark" |
| (c) Mac disables the port or cuts VBUS | **stays on** when 12 V is present (DIN feeds the board) | detach; macOS log shows a port event | — | replug the USB cable | ESP keeps running; uptime counter keeps going |

- "Unplugging USB alone does not recover; both must be removed for ~20 s" fits a PMIC in its OFF state kept alive by DIN 5 V. The 20 s is probably bulk capacitance on a battery-less board decaying below UVLO and forcing a POR (inference, not measured). A pure (b) or (c) would not need this.
- **Most informative next step (costs nothing):** after the next failure, leave both cables in and hold the CoreS3 power key 1–2 s (M5Unified sets 0x27=0x00, i.e. ~1 s ON level per its comment). If it boots, REG 0x21 still holds the cause because no POR happened. If it does not boot while 5 V is present, the PMIC likely sees the input as not "good" (REG00 b5 path: VBUS < VACOV and must hold above VVBUSMIN under a ~30 mA test load, §6.5.3.1), which points upstream at the 5 V source. (We do not know whether the user has tried the key; Issue #1 does not say.)
- **How to read REG 0x21 for this board:** b3 (VSYS UV) → a sagging or fighting 5 V input (no battery means no supplement mode, §6.5.3.2); b4 (VBUS >7 V) → a spike on the input, e.g. from motor or relay switching; b6 (DCDC OV) → something back-driving a PMIC rail, e.g. the externally powered 5 V encoder/level-shifter side pushing current into 3.3 V. This is plausible only because OVP is still enabled. b5 (DCDC UV) should not occur because M5Unified disabled it. b7 → die over-temperature. b1 → software power-off (a firmware bug calling `M5.Power.powerOff()`).
- **Truth table combining ESP and PMIC evidence** (needs an RTC_NOINIT magic word and a boot counter):
  - RTC data intact + ESP_RST_BROWNOUT → the 3.3 V rail sagged; the ESP32 is the victim.
  - RTC data intact + PANIC/WDT → firmware.
  - RTC data lost + ESP_RST_POWERON + REG21 ≠ 0 → the PMIC shut the rails; the bit says why.
  - RTC data lost + POWERON + REG21 = 0 + REG20 b2 = 1 → the PMIC itself went through POR (its input fell below 3.75 V), or REG21 was cleared by a POR during recovery.
- **Firmware to add (sketch, not compiled):**

```cpp
// --- at top level ---
#include <esp_system.h>
#include <Preferences.h>
RTC_NOINIT_ATTR uint32_t rtc_magic, rtc_boot, rtc_uptime_s;
RTC_NOINIT_ATTR uint16_t rtc_vbus_mv, rtc_vsys_mv; RTC_NOINIT_ATTR int16_t rtc_tdie;
RTC_NOINIT_ATTR uint8_t rtc_s0, rtc_s1;

void logPowerOnBoot() {               // call right after M5.begin()
  auto& a = M5.Power.Axp2101;
  uint8_t on  = a.readRegister8(0x20), off = a.readRegister8(0x21);
  uint8_t i48 = a.readRegister8(0x48), i49 = a.readRegister8(0x49), i4a = a.readRegister8(0x4A);
  uint8_t s0 = a.readRegister8(0x00), s1 = a.readRegister8(0x01);
  uint8_t b[4]; for (int i=0;i<4;i++) b[i] = a.readRegister8(0x04+i);   // PMIC DATA_BUFFER (survives PMIC soft-off, not POR)
  bool rtc_ok = (rtc_magic == 0xC0DE5EED);
  Serial.printf("RST=%d PWRON=%02X PWROFF=%02X IRQ=%02X/%02X/%02X S0=%02X S1=%02X buf=%02X%02X%02X%02X rtc_ok=%d last_up=%us vbus=%u vsys=%u tdie=%d\n",
    (int)esp_reset_reason(), on, off, i48, i49, i4a, s0, s1, b[0],b[1],b[2],b[3],
    rtc_ok, rtc_ok?rtc_uptime_s:0, rtc_vbus_mv, rtc_vsys_mv, rtc_tdie);
  // also persist this line to NVS (Preferences) and draw it on the LCD, because USB may not be attached
  a.writeRegister8(0x48, i48); a.writeRegister8(0x49, i49); a.writeRegister8(0x4A, i4a); // RW1C clear
  if (!rtc_ok) { rtc_magic = 0xC0DE5EED; rtc_boot = 0; } rtc_boot++;
}

void samplePower() {                  // every 1 s from loop()
  auto& a = M5.Power.Axp2101;
  rtc_uptime_s = millis()/1000;
  rtc_vbus_mv = (uint16_t)(a.getVBUSVoltage()*1000);   // 0 if REG00 b5 (VBUS good) == 0
  rtc_vsys_mv = (uint16_t)(a.getAPSVoltage()*1000);
  rtc_tdie    = (int16_t)a.getInternalTemperature();
  rtc_s0 = a.readRegister8(0x00); rtc_s1 = a.readRegister8(0x01);
  a.writeRegister8(0x04, rtc_uptime_s & 0xFF); a.writeRegister8(0x05, (rtc_uptime_s>>8)&0xFF);  // uptime in PMIC buffer
  a.writeRegister8(0x06, rtc_s0); a.writeRegister8(0x07, rtc_s1);
  Serial.printf("t=%u vbus=%u vsys=%u tdie=%d S0=%02X S1=%02X R15=%02X R16=%02X\n", rtc_uptime_s,
    rtc_vbus_mv, rtc_vsys_mv, rtc_tdie, rtc_s0, rtc_s1, a.readRegister8(0x15), a.readRegister8(0x16));
  // every 10 s: Preferences.putUInt("up", rtc_uptime_s) etc. (NVS is the only store that survives a full POR)
}
```
  Fields to watch as the ~85 s point approaches: REG00 b0 (input current limit), b1 (thermal regulation), b5 (VBUS good); REG01 b3 (VINDPM) and b4; VBUS and VSYS trends; TDIE. Rising TDIE or VINDPM/current-limit flags before the drop would point to an overloaded or fighting 5 V input.
- The panic/brownout text is mirrored to USB-Serial-JTAG (secondary console). But the USJ link is exactly what fails at the moment of interest, and a 3.3 V sag or rail loss stops the PHY, so do not count on seeing "Brownout detector was triggered" on the Mac. Use RTC/NVS/PMIC-buffer logging and the LCD instead.
- Because the interrupt-mode brownout ends in a software system reset, RTC slow memory (RTC_NOINIT_ATTR) should survive. So "RTC intact + ESP_RST_BROWNOUT" is the expected brownout signature. If the supply keeps falling until the hardware/POR level, the result is POWERON with RTC data lost, the same as a rail loss.
- In the snippet, `Preferences` usage is only described in a comment. Minimal form: `Preferences p; p.begin("pwr", false); p.putUInt("up", rtc_uptime_s); p.putUChar("s0", rtc_s0); p.end();` every 10 s, and on boot read the same keys before overwriting. NVS has wear levelling; 6 writes/min during tests is fine (inference).
- `writeRegister8(reg, data)` is a public `I2C_Device` method in M5Unified (local `I2C_Class.hpp` L168–171), so the snippet's PMIC writes compile against the same object as `readRegister8`.

### Gaps
- No PMIC timer matches ~85 s: the TWSI watchdog is disabled by default and steps 64/128 s; charger safety timers are not documented at this scale. **No source explains the 85 s periodicity**; leave it to measurement.
- REG 0x22 b1/b0, REG 0x24 VOFF, and whether "VBUS low→high" auto power-on is enabled are EFUSE (factory) values. Their CoreS3 values are not in any doc found. **Read them on the device** (`readRegister8(0x22)`, `0x24`) and record them.
- VACOV (the "good source" upper limit in §6.5.3.1) is not specified numerically in the extracted text.
- RTC_NOINIT retention after an interrupt-mode brownout follows from the software-reset path in `brownout.c` but was not tested on hardware. The hardware-BOD path (`USE_INTR=n`) and the exact Arduino 2.0.17 (IDF 4.4) handler were not checked. The PMIC DATA_BUFFER and NVS logging are the fallback.
- Not verified: the M5Unified comment that 0x27=0x00 means a 1 s key ON level. The datasheet REG 27 bit fields were not extracted.

---

## Q2. Does the ESP32-S3 USB-Serial-JTAG enumerate through a VBUS-cut ("data-only") connection while powered from DIN 5 V? How does that compare with an ADuM3160/4160 isolator?

### Takeaway
Probably yes. The ESP32-S3 USJ PHY turns on its D+ pull-up at power-up regardless of VBUS (no VBUS sense input). The Mac drives its own VBUS and detects attach through CC (Rd on the CoreS3) and the D+ pull-up. So a VBUS-blocking USB-C adapter that passes CC, D+/D−, and GND should enumerate at 12 Mbps. This is inferred, not confirmed by an official Espressif statement. A VBUS cut removes the **second 5 V source but keeps the ground connection**. An ADuM3160-class isolator removes the **ground connection but usually adds its own isolated 5 V**. They test different hypotheses, so run both.

### Cited Findings
- "the USB D+ interface has a default pull-up resistor, which keeps the USB D+ pin at a high level" and "the pull-up resistor on the USB D+ pin is present at power-up" — [esp-iot-solution USB-Serial-JTAG](https://docs.espressif.com/projects/esp-iot-solution/en/latest/usb/usb_overview/usb_serial_jtag.html)
- The ESP-IDF USB device docs say self-powered devices using the USB-OTG stack "must also connect VBUS through voltage divider or comparator" for VBUS monitoring (search summary of [ESP-IDF v5.0 USB device driver](https://docs.espressif.com/projects/esp-idf/en/v5.0/esp32s3/api-reference/peripherals/usb_device.html); the current page has moved to [esp-usb docs](https://docs.espressif.com/projects/esp-usb/en/latest/esp32s3/usb_device.html)). That requirement applies to the OTG/TinyUSB stack, not the fixed-function USJ.
- A user (not Espressif staff) on esp32.com says the ESP32-S2/S3 has no hardware VBUS monitoring and that software controls the D+/D− pull-ups through PHY registers ([esp32.com "ESP32-S3 USB ID pin"](https://esp32.com/viewtopic.php?p=91784); seen only as a search snippet because the thread was behind a bot challenge). **Unverified.**
- The CoreS3 SE schematic shows 5.1 kΩ/1 % on CC1 and CC2, i.e. passive Rd sink resistors (local: pdftotext of `Sch_M5_CoreS3_SE_v1.0.pdf` in the shared scratchpad; M5Stack docs).
- This Mac enumerates the CoreS3 as `0x303a/1001/0101 (USB JTAG/serial debug unit / 1) at 12 Mbps` on `usb-drd2-port-hs@02100000`, a direct Mac Type-C port, not the dock (local unified log, 2026-10-25 21:51:39).
- ADuM3160: "Full/Low Speed USB Digital Isolator", 12 Mbps max, 2.5 kV rms, transparent upstream-facing isolator that detects data direction, 3.0–5.5 V supply on each side able to run directly from VBUS — [ADI ADuM3160 product page](https://www.analog.com/en/products/adum3160.html) (via search summary; direct fetch timed out). LCSC lists "Integrated isolated power supply" ([LCSC](https://lcsc.com/product-detail/Digital-Isolators_Analog-Devices-ADUM3160BRWZ_C108501.html)), which **conflicts** with ADI's data-only description.
- A typical ADuM3160 board (wagiminator) powers the isolated side from an HLK-1D0505A 1 W isolated DC-DC: unregulated 4.7–5.3 V, 200 mA max, needs ≥20 mA minimum load (5.1 V Zener). "Hardwired for USB2.0 full speed (12Mbps) support only"; signal isolation 2.5 kV, power 1.5 kV — [wagiminator/ADuM3160-USB-Isolator](https://github.com/wagiminator/ADuM3160-USB-Isolator)

### Inferences
- The USJ is full-speed (the Mac log confirms 12 Mbps), so ADuM3160/4160 isolators (FS/LS only) are compatible.
- **VBUS-cut test (removes "two 5 V sources", keeps GND):** use a USB-C VBUS-blocker or a C-to-C cable or adapter with VBUS cut that still passes CC, D+/D−, and GND. If enumeration works and the ~85 s shutdown disappears with 12 V + USB data, the cause is the 5 V interaction (back-feed/fight, or AXP2101 input handling of two sources). If it still fails, the shared 5 V is not needed for the failure; look at DIN 5 V/DC-DC, noise, or ground. A VBUS cut also stops any back-feed into the Mac port.
- **Isolator test (removes ground loop, adds an isolated 5 V):** an ADuM3160 board feeds its own isolated ~5 V (≤200 mA) into the CoreS3 USB VBUS, so two sources are still present. To separate the variables, cut or disable the isolator's downstream VBUS output if the board allows it, or chain isolator + VBUS-blocker (data-only and galvanically isolated). Check that the isolator's downstream side still gets its VDD2 from the isolated DC-DC, not from the CoreS3 VBUS pin.
- Caveat: if the CoreS3 back-drives 5 V onto its own USB-C VBUS pin (through the USB OTG switch/boost, even though `setUsbOutput(false)` should prevent it), a VBUS-blocker hides that. Measure the CoreS3-side VBUS pin with the blocker in place (see Q4).
- Caveat: a data-only path violates the USB rule that a device must not pull up D+ without VBUS. The Mac does not check this in practice because it sources VBUS on its own side (inferred).

### Gaps
- No official Espressif statement found that USJ enumerates without VBUS at the connector. Treat it as "very likely, verify on the bench" (a 1-minute test).
- The ADuM3160 datasheet was not fetched directly, so its power-pin requirements (VBUS1/VBUS2, VDD1/VDD2) need confirming in the PDF. ADuM4160 (5 kV version) was not looked up.
- No source checked whether specific commercial USB-C VBUS-blocker adapters pass CC correctly. A blocker that also blocks CC will make the Mac see no attach.

---

## Q3. Pulling macOS unified logs for USB events around a timestamp

### Takeaway
On this Mac the USB stack logs under `process == "kernel"` with sender `IOUSBHostFamily` (USB device lifecycle) and `IOAccessoryManager` (Type-C port/CC). A CoreS3 disconnect appears as `AppleUSBHostPort::terminateDevice: destroying 0x303a/1001/0101 (USB JTAG/serial debug unit): hardware connection lost`. Because the CoreS3 has passive Rd on CC, a **device-side power loss should show `terminateDevice` without a `[Port-USB-C@N: CC] active: NO` line**. A CC detach next to it means a physical unplug or a connector/GND/port-level event. No "over-current" string appeared in 3 days of logs.

### Cited Findings (local observations on this Mac, 2026-10-26; raw dump at `/private/tmp/claude-501/-Users-takuro-src-dof-reality-h3-custom-controller/7a3afa4b-3db4-470f-9967-aa2f9085ab28/scratchpad/maclog_usb_3d.txt` and `…/maclog_usb_3d_nospam.txt`)
- **zsh gotcha:** in zsh, `log` is a builtin (`log: too many arguments`), so call **`/usr/bin/log`** explicitly (local).
- Device lifecycle strings actually seen (IOUSBHostFamily, kernel):
  - `usb-drd2-port-hs@02100000: AppleUSBHostPort::terminateDevice: destroying 0x303a/1001/0101 (USB JTAG/serial debug unit): hardware connection lost` (2026-10-25 21:51:36.311 and 22:49:57.990)
  - `AppleUSBHostPort::enumerateDeviceComplete_block_invoke: enumerated 0x303a/1001/0101 (USB JTAG/serial debug unit / 1) at 12 Mbps` (21:51:39.768)
  - `AppleUSBHostPort::cableChangeOccurred: powering off` / `powering on` on `usb-drd2-port-hs/ss`
  - Hub-level example of a real port-disable token: `AppleUSB20HubPort::resetAndCreateDevice: failed to address device, disabling port` (another device, 2026-10-26 01:08:34)
- Type-C/CC strings (IOAccessoryManager): `IOPortTransportState::setActive(): [Port-USB-C@3: CC] active: NO (transportType: 1 [CC])` came just before both CoreS3 terminate events (21:51:36.241 → terminate 21:51:36.311; 22:49:57.922 → terminate 22:49:57.990). The 21:51 detach was followed by `CC] active: YES` 2.3 s later, which looks like a physical replug or reflash. `Port-USB-C::configureAccessoryPowerInternal … set PowerModeOff?` lines appear at sleep/wake (`_sleeping=1`) and are not obviously tied to the device.
- **Log spam:** `AppleUSBIORequest::complete: device 1 (USB JTAG/serial debug unit@02100000) endpoint 0x82: status 0xe00002ed (transaction error): 0 bytes transferred` was logged ~20,000 lines/min (likely rate-capped) from the start of retained logs (2026-10-25 20:43:06) until the 21:51:36 detach: 1,359,623 lines out of 1,364,832 kernel-USB lines in 3 days. This flooding pushed older entries out, which is why retention starts at 20:43. Endpoint 0x82 is a bulk IN endpoint (CDC data) on a device that stayed attached but was not answering correctly.
- No `overcurrent` / `over-current` string matched in 3 days (local grep). The exact macOS over-current message text is therefore **unverified**.
- General `log show` usage (predicates on process/subsystem/eventMessage, `--last`, combining with AND) — [Scalefusion guide](https://blog.scalefusion.com/macos-logging-and-mac-system-logs/), [flaviocopes](https://flaviocopes.com/courses/macos-internals-troubleshooting/query-unified-logs-narrowly/)

### Inferences
- **Commands to use** (tested locally except the `stream` form):
```sh
# live capture during a soak test (start before powering the rig; Ctrl-C after the event)
/usr/bin/log stream --style compact --predicate 'process == "kernel" AND (senderImagePath CONTAINS[c] "IOUSBHostFamily" OR senderImagePath CONTAINS[c] "IOAccessoryManager" OR senderImagePath CONTAINS[c] "AppleUSB") AND NOT (eventMessage CONTAINS "AppleUSBIORequest::complete")' | tee ~/usb_$(date +%Y%m%d_%H%M%S).log

# after the fact, a window around a known time
/usr/bin/log show --style compact --start "2026-10-26 14:00:00" --end "2026-10-26 14:10:00" \
  --predicate 'process == "kernel" AND (eventMessage CONTAINS[c] "usb" OR senderImagePath CONTAINS[c] "IOAccessoryManager") AND NOT (eventMessage CONTAINS "AppleUSBIORequest::complete")'

# lifecycle-only summary
/usr/bin/log show --last 1d --style compact --predicate 'process == "kernel"' \
 | grep -E 'terminateDevice|enumerateDeviceComplete|CC\] active|cableChangeOccurred|disabling port|over-?current'
```
- **How to read it at the failure time:** `terminateDevice … hardware connection lost` with **no** `CC] active: NO` → the CoreS3's D+ pull-up disappeared (ESP32/PHY unpowered or reset): device-side, (a) or (b). `CC] active: NO` first → connector/GND or Mac-port-level event, or someone unplugged the cable. A port fault on the Mac should show extra port/power lines with no CoreS3 power loss (screen still on).
- The 21:51 and 22:49 events may be the user unplugging or reflashing, not Issue #1 failures. Match them against the user's own notes before drawing conclusions. The 20:43–21:51 transaction-error storm matches a state where the CoreS3 was attached but its USJ was not servicing IN transfers (hung firmware, a half-powered PHY, or signal-integrity/ground-noise trouble). Ask the user what the device was doing then.
- Run the serial monitor with host timestamps so firmware lines line up with the Mac log, e.g. `~/.platformio/penv/bin/pio device monitor -b 115200 --filter time --filter log2file` (per project memory, the PATH `pio` is broken). Note that `time`/`log2file` are standard PlatformIO monitor filters; this was not re-verified in this session.

### Gaps
- The macOS over-current/port-power-fault message text was not found in docs or local logs. Apple does not document IOUSBHostFamily log strings.
- Whether Apple Silicon Type-C ports log a VBUS fault (e.g. back-feed detected) under IOAccessoryManager is unknown. Capture with `log stream` during a failure to find out.

---

## Q4. Multimeter and USB-meter methods: which rail is lower, back-feed direction, DC jack current, DC-DC temperature

### Takeaway
Two cheap instruments answer most of the "5 V fight" questions: a bidirectional inline USB-C meter (e.g. POWER-Z KM003C) between the Mac and the CoreS3, and a multimeter on DIN 5 V (M-Bus 5V–GND) with min/max. Negative or reverse IBUS while 12 V is present means the board pushes current into the Mac (back-feed). VBUS and DIN 5 V readings show which source is higher. A sagging DIN 5 V that falls toward VINDPM (4.36 V default) as the 85 s point approaches points at the DC-DC or input path.

### Cited Findings
- KM003C does bidirectional current detection "by selecting different current detection paths based on whether the device being tested is in a charging or discharging state" — [ChargerLAB](https://www.chargerlab.com/bi-directional-current-detection-the-cool-feature-of-chargerlab-power-z-km003c). The Linux POWERZ driver (tested on KM003C) says "the sign of the channel curr1_input (label IBUS) indicates the direction" — [kernel.org POWERZ](https://docs.kernel.org/hwmon/powerz.html). The KM003C can also show D+/D− and CC1/CC2 — [ChargerLAB functions](https://www.chargerlab.com/inventory-of-10-commonly-used-functions-of-power-z-km003c)
- AXP2101 thresholds to compare readings against: VBUS operating 3.9–5.5 V, UVLO 3.75 V, OVP 7 V, VINDPM default 4.36 V, input current limit default 1.5 A — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- Firmware can log VBUS (`getVBUSVoltage`, 1 mV LSB) and VSYS (`getAPSVoltage`) once per second as an on-board "meter" faster than most DMM min/max modes — [M5Unified](https://github.com/m5stack/M5Unified) (local `AXP2101_Class.inl` L247–300)

### Inferences
- **Measurements, cheapest first:**
  1. **DIN 5 V with a DMM in min/max mode** (M-Bus 5V pin to GND, 12 V only, then 12 V + USB): note steady value, minimum over the 85 s, and whether it moves when USB is plugged. Brightness wavering on the red LED suggests DIN 5 V or 12 V is moving. Measure 12 V at the DC jack too.
  2. **Inline USB-C meter at the Mac end:** USB only → normal sink current (CoreS3 draws from the Mac). Then add 12 V. If IBUS heads toward 0 or flips sign, DIN 5 V is above Mac VBUS and the board feeds the Mac port. Note the VBUS value at the meter in both states. If the meter shows CC voltages, check that CC stays at the Rd level throughout.
  3. **Back-feed path check without the Mac:** 12 V only, USB-C unplugged. Measure the CoreS3 USB-C VBUS pin (via a C breakout or the meter's VBUS readout with no host) to GND. Any voltage near 5 V means DIN 5 V reaches the USB connector despite `setUsbOutput(false)`.
  4. **DC jack current:** DMM in series with the 12 V lead (A range) at idle and during the 85 s. A steady climb points at a thermal or over-current issue. Optionally use a bench supply with a current display.
  5. **Temperature:** finger or IR thermometer on the DIN Base buck regulator IC and inductor, and on the CoreS3 back (AXP2101 area), at 0 s and 80 s. Cross-check with logged AXP2101 TDIE and REG00 b1 (thermal regulation).
- A DMM min/max samples slowly (typically hundreds of ms; general knowledge, unsourced). It will miss µs dips caused by motor/relay switching. For those, use the AXP2101 REG21/IRQ latches as event detectors, or borrow an oscilloscope.

### Gaps
- The KM003C sign convention (which sign means sink vs source) is not documented in the sources found. Calibrate with a known USB-only case first.
- The DIN Base DC-DC part number, current rating, and thermal limits are not covered here (another researcher's scope).

---

## Q5. Ground-loop / motor-noise tests

### Takeaway
Change one ground-related variable at a time while 12 V + USB are both present: (1) motor 24 V supply and relays fully disconnected, including the shared GND wire; (2) the Mac running on battery with its charger unplugged, if it is a laptop, or through an isolator otherwise; (3) a USB isolator (ADuM3160 class) between the Mac and the CoreS3. The spontaneous ~85 s shutdown happens "with no external event", so noise from motor switching is a weaker fit for the timed failure. It remains a strong candidate for the earlier event-triggered shutdowns (24 V or relay switching, USB plug-in).

### Cited Findings
- An ADuM3160 isolator breaks the USB ground path (2.5 kV signal isolation) while passing full-speed data; the typical board's isolated DC-DC is limited to 200 mA — [wagiminator](https://github.com/wagiminator/ADuM3160-USB-Isolator), [ADI ADuM3160](https://www.analog.com/en/products/adum3160.html)
- The AXP2101 latches the input-side events a GND/noise transient would cause: REG21 b4 (VBUS >7 V) and REG49 b7/b6 (VBUS insert/remove IRQ, set when VBUS crosses UVLO) — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf)
- This Mac's log shows a long run of USB transaction errors on the CoreS3 bulk-IN endpoint while it stayed attached (Q3). That pattern can come from signal-integrity or ground-noise problems or from a hung device (local observation; cause unknown).

### Inferences
- **Order:** (i) disconnect everything except DIN Base + CoreS3 + 12 V + Mac USB (motor driver MD13S, relays, encoders, and level shifter all unplugged, shared GND removed). If the 85 s failure still happens, the external system is not required and the ground-loop hypothesis drops for this symptom. (ii) If it disappears, reconnect pieces one at a time: GND only, then 5 V encoders and level shifter (back-drive into 3.3 V → REG21 b6), then 24 V supply with motor idle, then relays switching. (iii) For the event-triggered failures, toggle relays or 24 V while logging REG21/REG49/REG4A, and run with the isolator or the VBUS-blocker.
- Laptop on battery removes the Mac's mains-earth path. If the Mac is a desktop (the logs show an Anker Thunderbolt dock, so the type is unknown), use the isolator instead.

### Gaps
- No measurements yet of the GND potential difference between the Mac USB shield/GND and the motor system GND. A DMM in AC and DC volts between them, with everything powered and USB unplugged, is a cheap check (inference).

---

## Q6. Test plan, cheapest first (each step tied to the hypotheses it confirms or rules out)

### Takeaway
Order: (0) after the next failure, recover with the power key instead of a POR and read REG 0x20/0x21 → (1) firmware logging (reset reason, REG20/21/IRQ, 1 Hz VBUS/VSYS/TDIE/status, NVS + PMIC-buffer persistence, LCD uptime) → (2) 12 V-only soak for ≥10 min → (3) 12 V + USB through a VBUS-cut adapter → (4) inline USB-C meter and DMM on DIN 5 V/12 V → (5) external system disconnected → (6) USB isolator → (7) Mac log captured live in every run.

### Cited Findings
- REG21 is POR-reset, the key is a power-on source from OFF, and VBUS UVLO is 3.75 V, so recovering without a POR keeps the evidence — [AXP2101 DS](https://github.com/lewisxhe/XPowersLib/blob/master/datasheet/AXP2101_Datasheet_V1.4_en.pdf) §6.5.1, §6.5.4.2, §6.13.2.18
- `esp_reset_reason()` distinguishes POWERON / BROWNOUT / PANIC / WDT — [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/misc_system_api.html)
- macOS `terminateDevice` vs `CC] active: NO` lines distinguish device-side power loss from a connector or port event (local log; Q3)

### Inferences
| # | Test (cost) | Method | Confirms / rules out |
|---|---|---|---|
| 0 | Key-press recovery (free) | After a failure, keep both cables in, hold the power key 1–2 s; boot firmware prints REG20/21/49/4A on the LCD | Boots + REG21 bit set → **PMIC protection**, bit gives the cause (b3 VSYS UV = sag/fight; b4 VBUS OV = spike; b6 DCDC OV = back-drive; b7 OT; b1 soft-off = firmware). Does not boot with 5 V present → input not "good" or latched upstream (DIN DC-DC / source fight) |
| 1 | Logging firmware (free) | Q1 snippet; 1 Hz serial + NVS every 10 s + PMIC DATA_BUFFER; uptime on the LCD | Last values before death: VBUS/VSYS trend, VINDPM (REG01 b3), current limit (REG00 b0), thermal regulation (REG00 b1), TDIE. ESP_RST_BROWNOUT with RTC intact → **ESP brownout**; PANIC/WDT → **firmware**; POWERON + REG21 → **PMIC** |
| 2 | 12 V-only soak (free) | ≥10–15 min with the same load, no USB | Fails → **DIN DC-DC thermal/over-current or battery-less AXP2101 behavior on DIN 5 V alone**; two-source and Mac-port hypotheses ruled out. Passes → the second source or USB GND is involved |
| 3 | 12 V + USB via VBUS-cut adapter (cheap part) | Data-only C path; confirm enumeration (12 Mbps) | Passes → **5 V back-feed/fight or AXP2101 dual-source handling** (and Mac-port VBUS shutdown). Fails → the problem is not the second 5 V; suspect GND loop/noise or the DIN supply |
| 4 | Inline USB-C meter + DMM (cheap) | IBUS sign and VBUS with 12 V on/off; DIN 5 V and 12 V min/max; CoreS3 VBUS pin with USB unplugged; DC jack current; IC temperatures | Reverse IBUS or VBUS pin live with USB unplugged → **back-feed** confirmed. DIN 5 V sagging toward 4.36 V or rising current/temperature → **DC-DC thermal/OC** |
| 5 | External system disconnected (free) | Only CoreS3 + DIN + 12 V + USB; then add pieces back | Failure persists → **ground loop / motor noise / encoder back-drive** ruled out for the 85 s symptom; reappears with one piece → that piece (REG21 b6 if it is a 5 V back-drive) |
| 6 | USB isolator (≈ low cost) | ADuM3160 board between the Mac and the CoreS3 (ideally with its isolated 5 V disabled, or combined with the VBUS-blocker) | Passes when 3 fails → **ground loop via the Mac**. Combined with 3 it separates "two sources" from "ground" |
| 7 | Mac log (free, every run) | `/usr/bin/log stream …` from Q3 | `terminateDevice` without CC detach → device-side; extra port/power-fault lines with the CoreS3 still lit → **Mac port shutdown** |

- The "plug USB while 12 V is present → shutdown" trigger can be reproduced in seconds. Run it first with the step-1 firmware and step-0 key recovery; it may give a REG21 reading faster than the 85 s soak.
- Also record the EFUSE-defined registers once (0x22, 0x24, 0x25, 0x27, 0x15, 0x16, 0x18, 0x19, 0x68) as a baseline, so the plan does not rest on datasheet defaults.

### Gaps
- No oscilloscope is assumed. Fast transients (µs) on 5 V/3.3 V during relay or motor switching cannot be measured directly with the plan above; the PMIC latches (REG21, REG49) are the only proxy.
- The CoreS3 internal power-path topology (how M-Bus 5V and USB VBUS combine into the AXP2101 VBUS pin, diode/switch parts) is outside this note. The interpretation of tests 3–4 depends on it.
