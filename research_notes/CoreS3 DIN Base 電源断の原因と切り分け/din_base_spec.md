# M5Stack DIN Base (M132) — power circuit, specifications, and behavior with the battery removed

Conventions: **[Verified]** means read directly from a primary source (M5Stack schematic PDF or docs page, or the chip datasheet). **[Inference]** means my reasoning from those sources. Schematic-derived values (resistor values, net names) cite the schematic PDF they come from. I looked at all schematics as rendered images and checked them against their text layer.

Core-side naming trap: the DIN Base net `BUS_5V` (the buck output on M-Bus) is the net called **`BUS_OUT`** in the CoreS3 schematic. The CoreS3's own net `BUS_5V` is something else: the output of its internal SY7088 boost converter. This note always says "DIN BUS_5V" or "core BUS_OUT" to keep them apart.

---

## Q1. Which product is it? Input range, 5 V current rating, DC-DC IC, and is the buck synchronous (can it sink or be back-driven)?

### Takeaway
The base is the M5Stack "Base DIN", SKU **M132** (schematic `SCH_DinBase_V1.1`). The CoreS3 docs page links this same schematic for its bundled DinBase. The input is DC 9–24 V into a Silergy **SY8303AIC** synchronous buck set to about **5.1 V**. There is no published 5 V current rating. Because the SY8303 is synchronous, its high-side FET's body diode gives a path from 5 V back to the input when the DC jack is absent. Whether it actively sinks current while running is not documented; it has a low-quiescent light-load mode, which suggests it does not sink much.

### Cited Findings
- **[Verified]** The product page title is "Base DIN" with SKU **M132**. Input is "DC 9 to 24V" through a 5.5 × 2.1 mm jack, centre positive. The DC-DC buck chip is **SY8303AIC** and the lithium charge chip is **TP4057**. The battery is 500 mAh. The power switch "can completely cut off external power and battery." Its PinMap covers Core, Core2 and CoreS3. — [M5Stack docs: DIN BASE](https://docs.m5stack.com/en/base/DIN%20BASE)
- **[Verified]** The docs page gives **no 5 V output current rating** and no protection or warning notes. — [M5Stack docs: DIN BASE](https://docs.m5stack.com/en/base/DIN%20BASE)
- **[Verified]** The CoreS3 page says the kit "ships with a DinBase" and that it "can be powered by an external DC 12V (9 ~ 24V) supply or an internal 500mAh battery." Its schematic section links the same file, `559/SCH_DinBase_V1.1.pdf`. — [M5Stack docs: CoreS3](https://docs.m5stack.com/en/core/CoreS3)
- **[Verified]** Power chain from the schematic: DC jack DC1 (pin 1 = `PWR`) → **D3 DSS34** Schottky → slide switch S1 (DPDT, pins 5→4) → net **`HPWR`** → SY8303AIC VIN. EN is tied to VIN, so the buck is always on whenever HPWR is present. HPWR also has a **D4 SMF30CA** 30 V TVS to GND and C2 10 µF. — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)
- **[Verified]** Buck output: LX → L1 10 µH → **`BUS_5V`**, with C3 and C5 at 10 µF each and **D1 "TVS 5V"** to GND. The feedback divider is R1 120 kΩ (top) and R3 16 kΩ (bottom), and R2 200 kΩ goes from FS to GND. — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)
- **[Verified]** SY8303A datasheet formulas: VOUT = 0.6 × (1 + R1/R2), with VREF 0.591–0.609 V, and Fsw(kHz) = 10⁵ / RFS(kΩ). — [SY8303A datasheet Rev 0.9A (chipdip copy)](https://static.chipdip.ru/lib/484/DOC043484845.pdf)
  - **[Inference, arithmetic]** VOUT = 0.6 × (1 + 120/16) = **5.10 V**, or about 5.02–5.18 V given the reference tolerance alone. Fsw = 10⁵/200 ≈ **500 kHz** (the datasheet gives 425–575 kHz at RFS = 200 k).
- **[Verified]** The SY8303A is a "synchronous step-down DC/DC converter capable of delivering **3A**". It runs from 4.5–40 V with integrated top and bottom switches (110/70 mΩ) and peak current control. Its features include "ultra low quiescent operating to achieve high efficiency under light load", with IQ of 19 µA. — [SY8303A datasheet](https://static.chipdip.ru/lib/484/DOC043484845.pdf)
- **[Verified]** The LCSC listing for the exact board part, **SY8303AIC (C97054)**, gives 3 A, 4.5–40 V, a synchronous design, a "light load efficient mode", cycle-by-cycle current limit, short-circuit protection, and thermal shutdown with auto-recovery. — [LCSC C97054](https://www.lcsc.com/product-detail/C97054.html)
- **[Verified]** The D3 Schottky (DSS34) on the DC input blocks current from flowing back out of the jack. — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)

### Inferences
- The only datasheet I found is for **SY8303A** (ordering code SY8303AAIC). The board uses **SY8303AIC**. LCSC lists the same headline specs for both, so I treat them as electrically equivalent, but that is not confirmed.
- Back-drive with no DC input: when 12 V is absent and something drives DIN BUS_5V (for example USB 5 V passed out through the core), current can flow BUS_5V → L1 → LX → **high-side FET body diode** → HPWR. HPWR would then sit at about 4.3–4.6 V. D3 blocks it from reaching the jack, and on CoreS3 HPWR goes nowhere else (see Q2). This is how any synchronous buck behaves; the datasheet does not discuss it.
- Back-drive with 12 V present: the "ultra-low IQ / light-load" wording points to pulse-skipping/PFM with diode emulation. In that mode the converter stops switching and does **not** sink current when the external 5 V is above its 5.1 V setpoint. So the buck most likely neither fights a slightly higher USB 5 V nor sinks from it.
- Output current: the board has no rating. Its parts (SY8303 at 3 A, DSS34 Schottky at 3 A) suggest **≤3 A** at 5 V, minus thermal derating in TSOT23-8. A CoreS3 load is far below that.

### Gaps
- No official 5 V output current rating from M5Stack.
- No full SY8303AIC (non-A) datasheet found. The light-load mode (PFM or forced-PWM) and behavior when the output is back-driven are not stated in the SY8303A sheet I read.
- **[Verified]** The CoreS3 **SE** package does *not* include a DinBase. Its contents are "1 × CoreS3-SE" and a hex key, and it has no battery. — [M5Stack docs: CoreS3 SE](https://docs.m5stack.com/en/core/M5CoreS3%20SE). The user's DIN Base was therefore bought separately (M132) or taken from a CoreS3 kit. M5Stack publishes only schematic V1.1 for both, and I could not verify which PCB revision the user's unit is.

---

## Q2. Is there reverse-current or back-feed protection between DIN 5 V and the M-Bus 5 V that the core may also drive?

### Takeaway
**There is none on the DIN Base.** DIN BUS_5V is the raw buck output, wired straight to M-Bus pin 27 (core pin 28, `BUS_OUT`) and to the 5 V pins of the PORT.B/PORT.C Grove connectors. The only parts on it are filter capacitors and a 5 V TVS. On the CoreS3 side, the firmware settings `output_power=false` and `setUsbOutput(false)` put **two ME1502 load switches in the ON state at once**: BUS_OUT→VBUS and VUSB→VBUS. That ties DIN 5 V and the Mac's USB VBUS together at the AXP2101 VBUS pin with no diode-OR.

### Cited Findings
- **[Verified]** On the DIN schematic, the M-Bus connector J3 has pin 27 "5V-27" = `BUS_5V`, pin 29 "VBAT-29" = `BUS_BAT`, and pins 26/28/30 = `HPWR`, the raw DC input after D3 and S1 at about 11.5 V with 12 V in. Grove J1/J2 pin "5V" = `BUS_5V`. **No diode, MOSFET or load switch** sits between the SY8303 output and these pins. — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)
- **[Verified]** On the CoreS3 **SE** schematic (page 5, "M5.BUS" connector BUS1), pin 28 = **`BUS_OUT`**, pin 30 = **`VBAT`**, and **pins 25, 27 and 29 are drawn with no-connect X marks**. Pin 26 = `I2S_DATI`. Grove Port A (J3) pin 3 is also `BUS_OUT`. — [Sch_M5_CoreS3_SE_v1.0.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0.pdf). In the text layer of the plain CoreS3 schematic, pin 28 is also `BUS_OUT`, pin 30 is `VBAT`, and the ME1502 / `BUS_OUT_EN` / `USB_OTG_EN` nets are identical. The NC markers are graphics, so I confirmed them visually only on the SE sheet. — [Sch_M5_CoreS3_v1.0.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/490/Sch_M5_CoreS3_v1.0.pdf)
- **[Verified]** The CoreS3 docs pin table labels pins 25/27/29 "HVIN (Base DIN)", pin 28 "5V" and pin 30 "BAT". — [M5Stack docs: CoreS3](https://docs.m5stack.com/en/core/CoreS3). The table describes what the base presents on those pins. It is consistent with the schematic, which shows the core itself leaves them unconnected.
- **[Verified]** The DIN schematic labels its bus with Core (Basic/ESP32) GPIO names (e.g. "8-G23/MOSI", "17-G22/IIC_SCL"), but the DIN docs PinMap says the same connector serves Core, Core2 and CoreS3. It is one universal base, not a CoreS3-specific variant. — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf); [M5Stack docs: DIN BASE](https://docs.m5stack.com/en/base/DIN%20BASE)
- **[Verified]** CoreS3 power-path page:
  - U14 ME1502**A**M5G (EN active-high, `BUS_OUT_EN`) connects the internal boost `BUS_5V` to `BUS_OUT`.
  - U19 ME1502**C**M5G (EN active-low, `BUS_OUT_EN`) connects `BUS_OUT` to AXP2101 `VBUS`.
  - U17 ME1502AM5G (EN-high, `USB_OTG_EN`) connects `BUS_OUT` to `VUSB`.
  - U18 ME1502CM5G (EN-low, `USB_OTG_EN`) connects `VUSB` to `VBUS`.
  - The RSET resistors are 60 k on U14/U17 and 30 k on U18/U19. `BUS_OUT_EN` and `USB_OTG_EN` come from the AW9523B (P0_1 and P0_5).
  — [Sch_M5_CoreS3_v1.0.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/490/Sch_M5_CoreS3_v1.0.pdf)
- **[Verified]** ME1502 absolute maximum ratings: VIN 6 V, **VOUT −0.3 V to VIN**. — [ME1502 datasheet](https://static.chipdip.ru/lib/087/DOC043087044.pdf)
- **[Verified]** ME1502 is a 70 mΩ high-side current-limited load switch with Ioc = 60 kΩ/Rset (2.0 A typ, 1.7–2.3 A at Rset 30 k). Short-circuit current is 1.2 A. Over-temperature shutdown is at 155 °C with 20 °C hysteresis and **auto-restart cycling**. UVLO is 2.2 V. VIN is 2.7–5.5 V, 6 V absolute maximum. The datasheet says "**无衬底二极管，芯片关断时可防止反向电流**": no body diode, so it blocks reverse current **only when switched off**. The output is pulled down when the switch is off. — [MICRONE ME1502 datasheet V02 (chipdip copy)](https://static.chipdip.ru/lib/087/DOC043087044.pdf)

### Inferences
- Pin mapping between the two connectors: the numbers are mirrored odd↔even across the mated pair. DIN 27 (5V) ↔ core 28 (`BUS_OUT`), DIN 29 (VBAT) ↔ core 30 (`VBAT`), and DIN 26/28/30 (`HPWR`) ↔ core 25/27/29 (NC). The signal pins support this: DIN 25 "G34/IIS_IN" ↔ core 26 `I2S_DATI`, DIN 1 "G35" ↔ core 2 `BUS_ADC1`. So the DIN Base's high-voltage `HPWR` reaches the connector but CoreS3 **does not use it**. Neither schematic states this mapping; it is my cross-reading of the two.
- With `output_power=false` (BUS_OUT_EN low), U14 is off and U19 is on. With `setUsbOutput(false)` (USB_OTG_EN low), U17 is off and U18 is on. **DIN BUS_5V (about 5.10 V) and the Mac's VBUS (about 5.0–5.2 V) are therefore paralleled through about 140 mΩ** of two ON load switches. Whichever source is higher supplies the load and may push current into the other. The ME1502 blocks reverse current only when off, and its current limit is specified only for VIN→VOUT, so reverse current through an ON switch has **no documented limit**.
  - If the DIN side is higher: current flows DIN → U19 → VBUS → U18 (reverse) → the Mac's VBUS. Macs normally block or limit reverse current, but this is back-feeding into the host.
  - If the Mac is higher: the Mac powers the CoreS3 **and the DIN Base's TP4057 charger**, which can take up to 500 mA (see Q3). Since the SY8303 most likely does not sink current, it just idles in that case.
- In this both-ON configuration, whichever source is higher drives the *other* switch's VOUT above its own VIN by the IR drop. That briefly exceeds the stated absolute maximum "VOUT ≤ VIN". The overshoot is marginal (tens of mV at a few hundred mA through 70 mΩ) and **unlikely to be destructive**, but it means the DC+USB configuration runs one ME1502 outside its datasheet limits.
- Because CoreS3 does not use DIN `HPWR` (core pins 25/27/29 are NC), the body-diode back-drive path in Q1 only charges the DIN input capacitors, about 20 µF. It is harmless but explains why the DIN input node stays partly charged from USB alone.
- Whether the core-side configuration really leaves U18 and U19 both on depends on the firmware and M5Unified behavior. That belongs to the core-side research, not this note.

### Gaps
- ME1502 behavior when reverse current flows through an ON switch (whether the current limit or OTP applies) is not specified.
- I did not find the exact output voltage of the Mac's USB port, so it is unknown which source wins.

---

## Q3. How is the 500 mAh battery wired, what does the red LED mean, and what happens with the battery removed?

### Takeaway
The DIN Base has **its own charger (TP4057, 500 mA CC / 4.24 V CV)**. Its output `BUS_BAT` connects **directly** to M-Bus VBAT, which the CoreS3 wires to the AXP2101 battery terminal `VBAT`. The charger is powered only while the DC jack has voltage, through a P-MOSFET gated by the jack voltage. The red LED is the TP4057 **CHRG (charging)** indicator, not a power LED. With the battery removed and 12 V present, the TP4057 still drives `BUS_BAT`, and therefore the CoreS3 PMIC's battery pin, with its charger output. Its datasheets describe the no-battery state as **CHRG blinking (about 20 Hz with 1 µF on BAT) and STDBY on**. That matches the reported red-LED flicker or wavering.

### Cited Findings
- **[Verified]** Battery connector P1 (`SMT_HDR_2x1.25mm`) `BAT+` → S1 second pole (pins 2→1) → **`BUS_BAT`** → J3 pin 29 VBAT. When S1 is off, both the DC path and the battery path are disconnected. — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)
- **[Verified]** U2 **TP4057**: VCC = `VCC_5V`, BAT pin = **`BUS_BAT` directly, not through S1**, PROG = R8 **2 kΩ**. The BAT node has C6 100 nF and C7 10 µF, plus **R6 100 kΩ from `VCC_5V` to `BUS_BAT`**. — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)
- **[Verified]** The charger supply is gated by the DC jack. `VCC_5V` = `BUS_5V` through **Q1 SI2301** (P-MOS; source on BUS_5V, R12 100 k gate pull-up). Q1's gate is pulled low by **Q2 DTC114YUA**, whose base is driven from **`PWR`** (the DC-jack side of D3, before S1) through R9 100 k. So the charger and both LEDs run only while DC is present on the jack, and USB 5 V alone does not power them. — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)
- **[Verified]** LED D2 is a "1615RG" red/green bicolor part. **Red = TP4057 CHRG** (pin 1, R11 2 kΩ to VCC_5V). **Green = STDBY** (pin 5, R7 4.7 kΩ). — [SCH_DinBase_V1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)
- **[Verified]** CoreS3 net `VBAT` on M5.BUS pin 30 is the same net name as the AXP2101 `VBAT` pin (U1 pin 35) in the CoreS3 schematic. — [Sch_M5_CoreS3_v1.0.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/490/Sch_M5_CoreS3_v1.0.pdf)
- **[Verified]** TP4057 (AOSSEMI-branded datasheet) electrical data:
  - Float voltage 4.198–4.282 V (typ 4.24 V).
  - CC current 465–535 mA at RPROG = 2 k.
  - Trickle charge at 50 mA below 2.9 V.
  - Termination at C/10 after PROG < 100 mV for 1.8 ms.
  - Auto-recharge when BAT < 4.1 V, with a 0.8–4 ms recharge delay.
  - VCC UVLO 3.8 V. Thermal regulation reduces charge current above 135 °C.
  — [TP4057 datasheet (AOSSEMI, ickey copy)](https://www.ickey.cn/static-pf/datasheet/b4/69/4fd6/b4694fd6c0e2ac77bd63bcba19d56ef2.pdf)
- **[Verified]** The same TP4057 datasheet's status table: charging = CHRG on, STDBY off. Charge done = CHRG off, STDBY on. "**BAT端连接1μF电容，无电池: CHRG 闪烁（频率约20Hz）, STDBY 亮**", meaning with no battery and 1 µF on BAT, CHRG flashes at about 20 Hz and STDBY stays on. It also states "当电池没有接到充电器时，CHRG闪烁表示没有安装电池": a flashing CHRG means no battery is installed. — [TP4057 datasheet (AOSSEMI)](https://www.ickey.cn/static-pf/datasheet/b4/69/4fd6/b4694fd6c0e2ac77bd63bcba19d56ef2.pdf)

### Inferences
- **The DIN Base, not the CoreS3, charges the battery.** With 12 V present, two chargers sit on the same `VBAT` node: the TP4057 on the DIN Base and the AXP2101's own charger, which runs from VBUS when enabled. With no battery, that node is only about 10 µF + 100 nF, plus whatever the AXP2101 draws or pushes. Its voltage will therefore cycle on the charge/terminate/recharge thresholds (4.24 V CV, 4.1 V recharge), and both PMIC and charger will repeatedly see a "battery" appear and disappear.
- **The red LED is a charge-state LED, and its flicker is the documented no-battery behavior, not a fault indicator.** A *steady* red LED with wavering brightness would mean the TP4057 is spending most of its time in "charging" mode, i.e. something (most plausibly the AXP2101) is continuously drawing current from `BUS_BAT`. With the 10 µF on this board, the CHRG duty cycle and blink rate will differ from the datasheet's 1 µF case.
- **A quick check that needs no scope: is the green LED on?** The two LED colors share one package (D2 1615RG). Per the datasheet, the no-battery state is **red blinking AND green on**. The issue only mentions the red LED.
  - Green on + red flicker → the textbook TP4057 no-battery oscillation.
  - Green off + red steady or wavering → the TP4057 is held in CC/CV charging, meaning something (most plausibly the AXP2101) is continuously drawing from `BUS_BAT`.
- **R6 (100 kΩ from VCC_5V to BUS_BAT)** pulls the battery node toward 5 V whenever the TP4057 is not charging and nothing loads it. The VBAT pin could then float above 4.24 V with the battery removed. The schematic does not say why R6 is there.
- **Red LED still on after the CoreS3 goes dark** tells us `VCC_5V`, and therefore DIN `BUS_5V`, is still present. Q1 passes VCC_5V only from BUS_5V, so the SY8303 buck had **not** shut down when the core died. The power loss is on the core side (AXP2101 or ME1502 switches), not a DIN converter dropout.
- Why removing USB alone does not recover: while 12 V is present, the DIN Base keeps the core PMIC's **VBUS** up (DIN 5 V → U19) **and** its **VBAT** up (TP4057 → BUS_BAT). The AXP2101 never sees a full power cycle until 12 V is also removed. This fits the "remove both for about 20 s" observation; the 20 s itself is presumably core-side capacitor/PMIC reset time, not something on the DIN Base.

### Gaps
- No M5Stack documentation describes the DIN Base LED meanings or no-battery behavior. Everything above comes from the schematic plus the TP4057 vendor datasheets.
- The TP4057 vendor on the DIN Base is unknown (TP4057 is second-sourced). The datasheet I read is the AOSSEMI version.
- No forum or GitHub reports were found about DIN Base no-battery flicker or CoreS3 shutdowns with DinBase. Two web searches returned only product pages.

---

## Q4. Does M5Stack publish notes or warnings about using DC input and USB together, or about battery removal?

### Takeaway
**No.** Neither the DIN Base nor the CoreS3 docs page has any warning about DC and USB simultaneous use, battery removal, or back-feed. I found no community or GitHub reports either.

### Cited Findings
- **[Verified]** The DIN Base docs list only input polarity and the power switch ("completely cut off external power and battery"). There are no warnings. — [M5Stack docs: DIN BASE](https://docs.m5stack.com/en/base/DIN%20BASE)
- **[Verified]** The CoreS3 docs describe "an AXP2101 PMU and four power-path control circuits" and the AXP2101 + AW9523B direction control (`BUS_OUT_EN` / `USB_OTG_EN`, explained only in a figure). They give no guidance on DC + USB interaction. They point to a "CoreS3 Power Manager Example". — [M5Stack docs: CoreS3](https://docs.m5stack.com/en/core/CoreS3)
- **[Verified]** Searches for M5Stack community and forum posts about CoreS3 + DinBase + 12 V + USB shutdowns or battery removal returned only shop and spec pages (e.g. [Mouser M132 page](https://www.mouser.com/es/new/m5stack/m5stack-m132-dinbase-interface-module/), [The Pi Hut CoreS3](https://thepihut.com/products/m5stack-cores3-esp32s3-lot-development-kit)).

### Inferences
- The board is evidently designed with the battery installed. In that case the TP4057 sees a real load and the VBAT node is held by the cell. Running with the cell removed while 12 V is present is an undocumented operating mode.

### Gaps
- I did not search M5Stack's Chinese-language forum or the M5Unified GitHub issues exhaustively; the search budget was spent on primary schematics. A targeted `M5Unified issues DinBase` / `CoreS3 BUS_OUT_EN USB` search could still turn something up.
- The M5Unified copy in the repo (`/Users/takuro/src/dof-reality-h3-custom-controller/encoder-tester/.pio/libdeps/m5stack-cores3/M5Unified/src/utility/Power_Class.hpp`, library.properties says 0.2.26) has comments about CoreS3 `setExtOutput` transitions and no-battery TS handling. Its comment style (some Japanese) suggests it may be a modified copy, so I do not cite it as upstream M5Unified behavior.

---

## Q5. Could thermal limits or over-current protection in the converter (or nearby parts) trip after tens of seconds?

### Takeaway
Unlikely on the DIN Base. The SY8303 is rated 3 A with a 4–6.8 A peak limit and recovers automatically from thermal shutdown. Nothing on the DIN Base latches or uses a timer. The parts that *can* cycle thermally on a time scale of seconds are the **TP4057** (fold-back at 135 °C if something continuously draws charge current) and the core-side **ME1502** switches (OTP at 155 °C with auto-restart). Neither latches the system off by itself.

### Cited Findings
- **[Verified]** SY8303A: top-FET current limit 4–6.8 A, cycle-by-cycle. Thermal shutdown 150 °C with 15 °C hysteresis and auto-recovery. θJA 60.2 °C/W (TSOT23-8). 1 ms soft-start. Input UVLO ≤4.35 V with 0.3 V hysteresis. — [SY8303A datasheet](https://static.chipdip.ru/lib/484/DOC043484845.pdf); auto-recovery and short-circuit protection are also listed for SY8303AIC at [LCSC C97054](https://www.lcsc.com/product-detail/C97054.html)
- **[Verified]** TP4057: thermal feedback reduces charge current above 135 °C. The SOT23-6 BAT current absolute maximum is 500 mA. — [TP4057 datasheet (AOSSEMI)](https://www.ickey.cn/static-pf/datasheet/b4/69/4fd6/b4694fd6c0e2ac77bd63bcba19d56ef2.pdf)
- **[Verified]** ME1502: constant-current limiting, then OTP at 155 °C → off → restart at about 135 °C, "如此循环" (cycling repeatedly). θJA 210 °C/W in SOT23-5. — [ME1502 datasheet](https://static.chipdip.ru/lib/087/DOC043087044.pdf)

### Inferences
- SY8303 dissipation at 12 V → 5.1 V and well under 1 A is a few hundred mW, a temperature rise of a few tens of °C. That is far from OTP, and even an OTP event would auto-recover and blank the LED (VCC_5V would drop). The red LED stayed lit at shutdown, which argues against a DIN converter trip.
- TP4057 heat: if the AXP2101 (or the system through it) keeps drawing from `BUS_BAT`, the TP4057 dissipates about (5.1 V − V_BAT) × I. For example, (5.1 − 3.7) × 0.5 A ≈ 0.7 W in SOT23-6, enough to reach thermal fold-back within tens of seconds. That would make the red LED's brightness or duty waver and let `VBAT` droop. **This is a hypothesis worth measuring:** probe M-Bus VBAT and feel or IR-check the TP4057 during the ~85 s window. It is not documented anywhere.
- The ~85 s delay is not explained by any DIN Base part. The DIN Base has no timers, and its only protections auto-recover.

### Gaps
- No thermal data for the DIN Base PCB itself.
- No measured waveforms of `BUS_BAT`, `BUS_5V` or VBUS during the failure. This is the most useful next measurement: DIN `BUS_BAT` (M-Bus pin 29/30) and `BUS_5V` against GND with an oscilloscope while 12 V and USB are both connected.
