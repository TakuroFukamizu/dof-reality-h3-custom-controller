# Prior cases resembling the CoreS3 SE + DIN Base dual-source (12V base + USB, no battery) shutdown

Scope: past reports (forums, GitHub issues, blogs) that look like Issue #1. That issue: with 12V on the DIN Base and USB-C to a Mac both connected and no battery, the CoreS3 SE goes dark about 85 s after boot with no event, disappears from the Mac, and recovers only after both sources are removed for about 20 s. Each case below is rated **High / Medium / Low / Analogy-only** for how closely it matches.

Firmware context, checked locally so the cases can be rated:
- `encoder-tester/platformio.ini` pins `m5stack/M5Unified@0.2.26` and `M5GFX@0.2.33`.
- `moter-controler/platformio.ini` uses `M5Unified@^0.2.5`. That resolves to 0.2.26 in the `m5stack-cores3` env and 0.2.28 in the `m5stack-cores3-usbpower` env.
- So builds made from the current libdeps contain the M5Unified 0.2.24 fix (PR #380/#381). **Caveat:** `^0.2.5` is a floating pin, and `moter-controler/.pio/libdeps/m5stack-cores3/M5Unified/library.properties` is dated 2026-10-13, close to the 0.2.26 release (PR #400, 2026-10-13). Any Issue #1 failure observed on a board flashed from a build made before about 2026-09-29/10-13 may have run M5Unified < 0.2.24, where the DCDC-UVP latch was still armed. The flashed build at each failure time needs confirming, for example by printing the M5Unified version and reading REG23 at boot. For CoreS3/CoreS3 SE, `Power_Class.inl` runs `Axp2101.bitOff(0x23, 0x1F); // DCDC1-5 UVP power-off disable` right after `Axp2101.begin()`.
- The same file says: "TS senses the 5V bus (no battery thermistor)". It then sets the TS pin to "external input, does not affect the charger" (`bitOn(0x50, 0x10)`).
- Local file: `/Users/takuro/src/dof-reality-h3-custom-controller/moter-controler/.pio/libdeps/m5stack-cores3/M5Unified/src/utility/Power_Class.inl`, lines 408–436.
- Neither project's `src/` writes PMIC registers directly. `grep` for `Axp2101`, `writeRegister`, `0x18`, `0x19` and `Power.` in `moter-controler/src` and `encoder-tester/src` finds only `cfg.output_power` and `M5.Power.setUsbOutput(false)`, in `moter-controler/src/main.cpp:776-790` and `encoder-tester/src/main.cpp:34-47`. The `*-usbpower` envs instead use `output_power=true` with no 12V.

## Q1. M5Stack forum / GitHub: CoreS3 or Core2 shutting off or rebooting with USB and base/DIN/M-Bus power both present; base-related shutdowns; battery-less operation

### Takeaway
I found no report of a CoreS3 + DIN Base shutdown in English, Japanese or Chinese. The closest reports fall into three groups:
- AXP2101 latch-off on a battery-less CoreS3, triggered by inrush when the 5V bus output is enabled. This is already fixed in the user's M5Unified version.
- Shutdowns or power cycling when the firmware flips the M-Bus 5V from input to output. The user's firmware already disables this.
- One Japanese blog: with USB and an AC adapter both on a CoreS3, the red charge LED flickers and operation is unstable.

None of them matches the ~85 s delayed onset.

### Cited Findings

**Case A: M5Unified PR #381 / Release 0.2.24 (PR #380). CoreS3 SE / CoreS3 family / Core2 v1.1. Match: Medium on mechanism, Low on symptoms. Ruled out for builds from the current libdeps (0.2.26/0.2.28) only; the flashed version at failure time is unconfirmed.**
- Dates: PR #381 opened and merged 2026-09-30. Release 0.2.24 (PR #380) merged into master 2026-09-29. Author ainyan03, merged by lovyan03. — [PR #381](https://github.com/m5stack/M5Unified/pull/381); [PR #380](https://github.com/m5stack/M5Unified/pull/380)
- Symptom: a battery-less CoreS3 SE on a weak, hub-routed USB supply powered off right after `M5.begin()`. "the backlight flashed once and the board stayed silent." — [PR #381](https://github.com/m5stack/M5Unified/pull/381)
- Root cause (confirmed by register read): REG21 = 0x20, i.e. DCDC under-voltage power-off. `setExtOutput(true)` connects the empty 5V bus (BUS_OUT) to the running boost through BUS_OUT_EN. The inrush "pulls a DCDC output below 85 % of its setting, and the AXP2101 latches itself off until the power key is pressed." On a hub-powered case the author also saw REG21 = 0x08 (VSYS under-voltage). — [PR #381](https://github.com/m5stack/M5Unified/pull/381)
- Fix (confirmed, merged): disable REG23[4:0] (DCDC1–5 UVP power-off) in `begin()`, so "a dip then resets the ESP32 through its brown-out detector instead of latching the PMIC off". DCDC OVP and VSYS UVLO (VOFF) stay enabled. The fix also pulse-precharges BUS_OUT. — [PR #381](https://github.com/m5stack/M5Unified/pull/381); [PR #380](https://github.com/m5stack/M5Unified/pull/380)
- Recovery: the latch clears on the **power key**. No power removal is needed. — [PR #381](https://github.com/m5stack/M5Unified/pull/381)
- The PR does not mention external 5V from M-Bus or a base. — [PR #380](https://github.com/m5stack/M5Unified/pull/380)

**Case B: M5CoreS3 Issue #72, "CoreS3 Requires Battery for Grove/MBUS Power When USB Powered". Match: Low (no shutdown), but relevant to the battery-less power-path logic.**
- Opened 2026-05-21 by amaschas, closed 2026-09-28. — [Issue #72](https://github.com/m5stack/M5CoreS3/issues/72)
- M5Unified refuses to enable the 5V bus output on USB power with no battery. The guard in `Power_Class.cpp` is `bool cancel = (enable && !Axp2101.getBatState() && Axp2101.getTSVoltage() > 2.0f && Axp2101.isVBUS());`. — [Issue #72](https://github.com/m5stack/M5CoreS3/issues/72)
- M5Stack (Ocean-lhy, 2026-08-25): "the hardware does not provide a reliable way for the firmware to determine which connector is the active power source". The guard is intentional, and "A side effect is that USB-C-only operation without a battery is also blocked." — [Issue #72](https://github.com/m5stack/M5CoreS3/issues/72)
- ainyan03 (2026-09-03) found that `setExtOutput(false)` cut BUS_OUT_EN before the boost stopped, so 5V stayed on. Fixed in M5Unified PR #352, shipped in 0.2.22. — [Issue #72](https://github.com/m5stack/M5CoreS3/issues/72)
- Also in the thread (2026-07-19): a CoreS3 Lite powering an LLM module from CoreS3 USB only "blinked to black". It worked when the LLM module had its own USB power. — [Issue #72](https://github.com/m5stack/M5CoreS3/issues/72)

**Case C: M5Stack forum topic 7288. CoreS3 / CoreS3 SE with a LAN (PoE) module or RS232 Module 13.2 feeding 9–24V. Match: Medium (base/module-fed 5V into the CoreS3, periodic power loss), with the cause fixed in firmware.**
- No dates on the page. Symptom: the backlight flashes every 3–5 s with PoE, the LAN module's DC jack, or the RS232 module's DC jack. It reproduced on two LAN modules and both CoreS3 models. USB and battery presence are not stated. — [Forum 7288](https://community.m5stack.com/topic/7288/power-not-working-between-cores3-or-cores3-se-lan-module-with-poe-or-rs232-module)
- Moderator felmue: the behavior matches VBUS (5V coming in from the module) being switched to output, which is "equivalent to power off", then back to input, in a repeating cycle. He reproduced it on the factory firmware (CoreS3 UserDemo v0.6) but not on UIFlow2 v2.2.1. — [Forum 7288](https://community.m5stack.com/topic/7288/power-not-working-between-cores3-or-cores3-se-lan-module-with-poe-or-rs232-module)
- Resolution (confirmed by the original poster): flashing an Arduino example firmware fixed it. The cause was firmware power-direction handling, not hardware. — [Forum 7288](https://community.m5stack.com/topic/7288/power-not-working-between-cores3-or-cores3-se-lan-module-with-poe-or-rs232-module)

**Case D: logikara.blog, CoreS3 usage article (published 2024-02-12, updated 2025-08-20). CoreS3 with battery. Match: Medium on the LED-waver symptom only. No shutdown is reported.**
- Quote: "USBとACアダプターの両方を繋いだ場合は、USBの時の動作が基本ですが、充電中の赤LEDが点滅したり動作が安定しません". The author recommends not connecting both. — [logikara](https://logikara.blog/core-s3-init/)
- The same article says no charging happens on the AC adapter. When charging from an AC adapter, `setExtOutput` should be `false`. — [logikara](https://logikara.blog/core-s3-init/)

**Case E: arcana-mfg blog (2025-02-07, updated 2025-11-01). CoreS3 SE + Base AAA (dry cells, no LiPo). Match: Low (fails at boot), Analogy for battery-less CoreS3 SE with base power.**
- "電源起動したタイミングで一瞬LCDのバックライトが点灯して落ちています。" The author suspects inrush when external output is enabled: "外部への供給をしたタイミングでラッシュ電流が結構流れているようです。" Changing AXP2101 settings did not help, and the author switched to an M5Stack Basic. (Speculation by the author, not confirmed.) — [arcana-mfg](https://arcana-mfg.com/blog/2025/02/07/m5-stack-cores3-se-x-base-aaa%E3%81%A7%E5%9B%B0%E3%81%A3%E3%81%9F%E8%A9%B1/)

**Case F: Core2 (AXP192), M-Bus 5V input vs output mode. Match: Analogy only. The user's firmware already sets `output_power=false`.**
- By default, `M5.begin()` switches M-Bus 5V from input to output. That is why "M5Core2 runs from USB but keeps restarting when powered by 5 V on M-BUS". Fix: use `kMBusModeInput`. — Summarized from [Forum 3701](https://community.m5stack.com/topic/3701/m5stack-does-not-start-with-5v-pin-of-m-bus/1) and [gwendesign Core2 notes](https://www.gwendesign.ch/kb/m5stack/m5core2/) via search snippet. I did not fetch the full text.
- Forum 4910 (M5Stack Tough, AXP192, with battery): felmue found that "switching on the internal 5 V booster while power comes in via 5 V pin on M-bus immediately shuts the system down". His guess: "the AXP192 doesn't like the 'circle'". He gave no fix: "Not sure if there is a way to solve this w/o hardware modification." — [Forum 4910](https://community.m5stack.com/topic/4910/technical-questions-about-the-charging-circuit-of-core-2-basic-core/6)
- Forum 8254 (Core2 v1.3, AXP192 + SY7088): with EXTEN on and USB removed, a self-sustained boost→VBUS→AXP loop drew ~450 mA from the battery. Claims: "SY7088 output is directly connected to VBUS" and "The supposed N_VBUSEN mechanism does NOT break the electrical path". Fix: an interlock, "VBUS must be set to OFF when EXTEN is Enable". felmue could not reproduce it on his unit. — [Forum 8254](https://community.m5stack.com/topic/8254/m5stack-core2-v1.3-ouroboros-like-power-loop)

**Case G: Core2 power-mode thread (topic 6853, AXP2101/AXP192, 5V output).** Surfaced in search but not read. — [Forum 6853](https://community.m5stack.com/topic/6853/core2-mbus-power-mode-5v-output-axp2101-axp192)

### Inferences
- The best-documented CoreS3 latch-off mechanism (DCDC UVP, REG21 bit 5) is disabled in builds made from M5Unified 0.2.26/0.2.28 (`bitOff(0x23, 0x1F)`). If a failure is confirmed on such a build and the PMIC is latching off, it must be a different power-off source:
  - VSYS < VOFF (REG21 bit 3)
  - VBUS > 7 V (bit 4)
  - die over-temperature (bit 7)
  - software power-off (bit 1)
  - PWRON long-press (bit 0)
  - DCDC OVP (bit 6)

  Alternatively, it is not a PMIC latch at all (for example, an ESP32 hang with the backlight off). See the [AXP2101 datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf) §6.5.4.3 and REG21.
- Every M-Bus input/output-flip case (C, F) depends on firmware switching BUS_OUT to output. The user runs `cfg.output_power=false` and `setUsbOutput(false)`, so those cases fit only if some code path still toggles BUS_OUT_EN or USB_OTG_EN. That is worth confirming by logging the AW9523 port bits during the ~85 s window.
- All the prior cases fail at `begin()` or within seconds. None shows a delay of tens of seconds with no event, so the 85 s timing is unexplained by known cases.
- The diagnostic method used in PR #381 transfers directly. After the next failure, try the **power key first**, before removing power. If the board comes back, read REG20 (power-on source) and REG21 (power-off source) before anything else. Per the datasheet, REG21 is POR-reset, so removing all power wipes the evidence. The user's report does not say whether the power key was tried during the failed state.

### Gaps
- No DIN Base-specific shutdown report was found in EN/JA/ZH searches. M5Stack docs describe the DIN Base only as 9–24V DC or a 500mAh battery with a hard power switch ([DIN Base docs](https://docs.m5stack.com/en/base/DIN%20BASE)).
- Forum threads 4910, 7288 and 8254 show no post dates on the page.
- No source found explains how the CoreS3 power path behaves when M-Bus 5V input and USB VBUS are both present: which one feeds the AXP2101 VBUS, and whether there is reverse-current blocking between them.

## Q2. AXP2101/AXP192 battery-less problems on other boards (T-Beam, Core2 v1.1, StickC Plus2, etc.): shutdowns, charge-LED flicker, battery-detect oscillation, register fixes

### Takeaway
I found no confirmed case where a battery-less AXP2101 board shuts down after tens of seconds and is fixed by a battery-detect, charger or TS register change. The datasheet and libraries do document the relevant knobs: reg68H[0] battery detect, TS-pin mode, CHGLED mode, and the watchdog. M5Unified already configures TS for CoreS3.

### Cited Findings
- AXP2101 battery detection is enabled by default and can be disabled through reg68H[0]. "If the function is disabled, PMU considers that battery is always present." The result is in reg00H[3]. — [AXP2101 datasheet V1.0](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf) (§6.7.3.5, extracted locally)
- XPowersLib's AXP2101 charge example calls `disableTSPinMeasure()` with the comment that the TS detection must be disabled "when the board has no battery temperature sensing, to avoid abnormal charging". — [XPowersLib example](https://github.com/lewisxhe/XPowersLib/blob/master/examples/AXP2101_Charge_Example/AXP2101_Charge_Example.ino) (via search snippet)
- M5Unified 0.2.26 on CoreS3: the TS pin senses the 5V bus, not a thermistor. Its power-on default "comes from the EFUSE and differs between chips". Left as a temperature input, "the charger treats the bus voltage as a cold battery and never charges". Fix applied: `bitOn(0x50, 0x10)`, TS as external input. The same file notes one unit read charge-voltage register 0x64 = 0x00 (reserved), so it forces 4200 mV. — local `Power_Class.inl` (path above)
- M5Unified's Core2 v1.1 AXP2101 init writes `0x68, 0x01` (battery detection enabled) and `0x69, 0x13` (CHGLED). CoreS3 uses `0x69, 0x11`. — local `Power_Class.inl` lines 860–864 and 421
- CHGLED: in REG69H mode Type B, "No battery or charge finished, and power supply by VBUS" is a steady state. 1 Hz blink means charging and 4 Hz means alarm, so a blinking LED with no battery suggests the PMU thinks it is charging or in an alarm. I found no source confirming battery removal alone causes flicker. — [AXP2101 datasheet (aw-ol mirror)](https://bbs.aw-ol.com/assets/uploads/files/1660616603344-15714b00-4eed-4559-8ade-19a51f064af3-axp2101_datasheet_v1.0_en.pdf) (via search summary)
- LilyGO T-Beam Supreme (AXP2101): the board powers on only from USB, not from battery, and does not seem to recognize or charge the battery. A responder suggested a power-enable GPIO. Unresolved. — [LilyGO community post 1272](https://community.lilygo.cc/post/1272); [topic 726](https://community.lilygo.cc/topic/726/t-beam-supreme-only-powers-on-with-usb)
- T-Beam v1.2 (AXP2101) did not work with an older AXP implementation in GXAirCom, which needed a library update. (Title only; not read.) — [GXAirCom #137](https://github.com/gereic/GXAirCom/issues/137)
- Meshtastic T-Beam "turns off even when powered by USB": after a while the device powers off, and replugging USB or pressing the power button recovers it. A reply suggests a power bank's low-current auto-off. — [Meshtastic forum 11199](https://meshtastic.discourse.group/t/t-beam-turns-off-even-when-powered-by-usb/11199) (snippet only; the site was unreachable by DNS during fetch). Match: Low (power-bank source).
- Meshtastic firmware #5781 (T-Beam v1.2, AXP2101, with battery): after low-battery deep sleep, replugging USB does not wake the board because AXP interrupts are ignored in firmware. — [meshtastic/firmware #5781](https://github.com/meshtastic/firmware/issues/5781) (snippet). Match: Low.
- AXP192 (Meshtastic forum): if battery and USB voltage drop low enough, the chip shuts down and restarts only on a button press. — [Meshtastic AXP192 thread 3335](https://meshtastic.discourse.group/t/axp192-power-management-ic-issues/3335) (snippet)

### Inferences
- M5Unified does not disable battery detection (reg68H) on CoreS3. With no battery, the PMU may cycle between battery detected and absent, or try to charge into an open BAT pin. That is a plausible source of the wavering red LED on the DIN Base. But no prior case ties it to a shutdown, so treat it as a hypothesis to test (read reg00H[3] and the charger status over the 85 s), not as a known fix.
- On CoreS3 the TS pin reads the 5V bus voltage, so TS voltage could reveal whether bus 5V sags during the 85 s window. M5Unified exposes `getTSVoltage()` (see the Issue #72 code line).

### Gaps
- No StickC Plus2 or Core2 v1.1 report of battery-less delayed shutdown was found.
- No source shows reg68H[0] = 0 (battery detection off) fixing a shutdown or LED flicker on any board.
- The LilyGO and Meshtastic threads were read only as snippets (one site was unreachable).

## Q3. PMIC latches off and recovers only after removing all power for seconds; delays of tens of seconds to minutes

### Takeaway
I found no prior case with an ~85 s delay or "all power off for ~20 s" recovery. The datasheet suggests why unplugging USB alone may not recover the board: one power-on trigger is a VBUS **low→high** edge. If 12V-derived 5V keeps VBUS high, that edge never happens, and the power key or full power removal is needed. Of the datasheet timers, the AXP2101 watchdog (64 s default, 128 s max) is close in magnitude to 85 s, but it is disabled by default and I found no evidence M5Unified enables it.

### Cited Findings
- AXP2101 power-on sources (PWRON mode):
  1. POK press longer than ONLEVEL
  2. "VBUS low go high"
  3. "VBAT low go high"
  4. IRQ low for more than 16 ms
  5. battery charged to normal

  Items 2–5 are "configured by customization". REG20 records which source fired. — [AXP2101 datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf) §6.5.4.2
- AXP2101 power-off sources:
  1. POK held longer than OFFLEVEL
  2. software write REG10H[0]
  3. "When VSYS<VOFF or VBUS>7V, AXP2101 will be powered off" (VOFF default 2.6 V, REG24H[2:0])
  4. DCDC 15% low (REG23H[4:0])
  5. DCDC over-voltage (REG23H[5])
  6. die temperature over 145 °C (REG22H[2])

  REG21 bits record the cause, and they are "POR"-reset. — same datasheet §6.5.4.3, REG21/REG22
- Watchdog: REG18[0] enables it (default 0 = disabled). REG19[2:0] sets the timeout (1–128 s, default 110b = 64 s). REG19[5:4] = 11 means "IRQ, System Reset, DCDC/LDO PWROFF & PWRON". — same datasheet REG18/REG19
- In M5Unified 0.2.26, the only REG18 write I found in the AXP2101 class touches bit 1 (charge enable): `writeRegister8(0x18, (val & 0xFD) | (enable << 1))`. A `0x18, 0x0E` line in the Core2 v1.1 init is commented out. — local `AXP2101_Class.inl` line 101–103, `Power_Class.inl` line 867
- PR #381: the latch clears on the power key. The bus charge "remained", which let the power key start the board. — [PR #381](https://github.com/m5stack/M5Unified/pull/381)

### Inferences
- If the CoreS3 is in AXP2101 OFF state while DIN Base 5V keeps the AXP2101 VBUS (or BUS) node high, unplugging USB produces no VBUS low→high edge, so there is no auto power-on. Removing both sources long enough for VSYS/VBUS to discharge (~20 s) causes a POR, and the next VBUS rise powers it on.

  This is consistent with the user's recovery behavior, but **not confirmed**: it depends on how M-Bus 5V reaches the AXP2101 on the CoreS3, which no source I found describes. Untested.
- The **power key** test during the failed state is the most discriminating experiment.
  - If it wakes the board, the PMIC was in OFF state, and REG20/REG21 can then be read to name the cause.
  - If it does not, suspect a deeper lock-up: AW9523 or boost state, a DIN Base converter in hiccup/latch, or ESP32 + backlight off with the PMIC still on.
- An 85 s delay suggests something thermal or slowly accumulating: a converter or PMIC heating, an RC charging, or a charger timer. Die over-temperature (145 °C) seems unlikely at hobby currents, but it can be identified from REG21 bit 7. No source supports a specific timer, so this is speculation.

### Gaps
- No prior case reports a delay of tens of seconds or recovery only after full power removal on an AXP2101 board.
- AXP2101 "customization" (EFUSE) defaults for VBUS-edge power-on on the CoreS3 are not published in any source I found.
- Neither M5Unified 0.2.26 (as far as grep shows) nor the project's own `src/` enables the AXP2101 watchdog (REG18[0]). Its power-on default is 0 per the datasheet, but EFUSE customization is not published, so reading REG18/REG19 at runtime is still the definitive check.

## Q4. Mac (Apple Silicon / USB-C) port over-current or back-feed protection: does macOS disable the port, and does that look like the device "disappearing"?

### Takeaway
I found no Apple documentation or credible report about a USB device back-feeding VBUS into a Mac port and macOS disabling it. The only documented macOS behavior is the "USB Accessories Disabled" notification on over-current, which is visible to the user and is cleared by unplugging. If the Mac showed no such notification, an over-current port shutdown is less likely, but that is unconfirmed.

### Cited Findings
- "USB Accessories Disabled" appears when connected devices "need more power than what the Mac's USB ports can provide". Advice: use a self-powered device or a powered hub, and unplug other devices. These are third-party repair sites, so low authority. — [iBoysoft](https://iboysoft.com/howto/usb-accessories-disabled-mac.html); [Apple Community 256073187](https://discussions.apple.com/thread/256073187); [Apple Community 255429318](https://discussions.apple.com/thread/255429318)
- An Apple Community user reported the warning cleared after unplugging the adapter for at least 5 minutes. (Anecdotal; from search summary.) — [Apple Community 255429318](https://discussions.apple.com/thread/255429318)
- Apple Silicon USB-C ports or docks sometimes stop communicating until the cable is replugged or the dock is power-cycled. There is no stated root cause and no mention of back-feed. — [Apple Community 256195769](https://discussions.apple.com/thread/256195769); [Apple Community 251481838](https://discussions.apple.com/thread/251481838)

### Inferences
- In this case the CoreS3 **screen** also goes dark, and the board does not recover when USB is unplugged. So a Mac-side port shutdown alone cannot explain the failure. A Mac port fault would leave the CoreS3 running on DIN Base power with its screen on. The Mac "losing the device" is therefore more likely a consequence of the CoreS3 powering off than a cause.
- Back-feed is still worth checking for safety. The ESP32-S3 USB PHY being unpowered, or 5V from the DIN Base reaching the Mac's VBUS, would make the CoreS3 a source on the Mac port. That can be checked with a USB-C power meter showing current direction.

### Gaps
- I found no Apple-published behavior for reverse current on VBUS into an Apple Silicon port, and no EE forum measurement of it.
- I found no report of a USB device that "disappears" because macOS cut VBUS without showing a notification.

## Q5. Ground-loop / motor-noise USB disconnects and ESP32 resets with motor drivers sharing GND with a USB-connected MCU

### Takeaway
General ESP32 troubleshooting sources list motor/relay switching as a common reset trigger, through voltage dips, inductive noise and ground bounce. The usual mitigations are an isolated or separate motor supply, flyback suppression, a good USB cable and a USB isolator. I found no specific case with an M5Stack and an MD13S-class driver. This fits the earlier "shutdown when the 24V/relays switched" events better than the event-free 85 s failure.

### Cited Findings
- Troubleshooting guide: "ESP32 resets when relay or motor switches", with likely causes "Voltage dip, inductive noise, ground bounce, or load wiring issue". Fixes listed: separate load power, flyback protection and decoupling, a better USB cable, and avoiding weak hubs. Check for `rst:0xf (BROWNOUT_RST)` in serial output. — [pcbvault](https://pcbvault.com/electronics-troubleshooting/esp32-keeps-resetting.html); [espboards external power guide](https://www.espboards.dev/troubleshooting/issues/power/esp32-external-power-guide/) (via search summary)
- motorbrainz (ESP32 + L298) ties the 12V, USB and JTAG grounds together and advises that external 12V come from an isolated supply. — [rtyle/motorbrainz](https://github.com/rtyle/motorbrainz) (via search summary)
- ESP32-S3 random USB disconnects are reported in arduino-esp32 #10000 and on the esp32.com forum. These were not tied to motors in what I read. — [arduino-esp32 #10000](https://github.com/espressif/arduino-esp32/issues/10000); [esp32.com t=33867](https://www.esp32.com/viewtopic.php?t=33867)

### Inferences
- With USB (Mac ground) and the DIN Base (12V adapter ground and external 24V system ground) both connected, there are two ground paths. A ground loop through the Mac is plausible during 24V or relay switching transients. That matches the earlier event-triggered shutdowns, but it does not explain an event-free shutdown after 85 s.
- A transient that pushes VBUS above 7 V would trigger an AXP2101 power-off (REG21 bit 4) per the [datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf). This is one more reason to read REG21 after an event-triggered failure. (Inference.)

### Gaps
- No report found of an M5Stack, MD13S or similar 24V driver setup with a USB-connected CoreS3 where the PMIC latched off on switching transients.
- I did not survey USB isolator (e.g. ADuM3160/4160) case studies with ESP32-S3 native USB, so I gathered no source on whether they are compatible.
