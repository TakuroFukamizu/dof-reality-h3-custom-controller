# CoreS3 / CoreS3 SE の電源経路（M-Bus 5V・USB VBUS・バッテリーの調停）と遅延ラッチオフの候補

調査日: 2026-10-26。一次資料は回路図 PDF/PNG、データシート、ライブラリのソースに限る。
対象ファームの M5Unified は **0.2.26**（`moter-controler/.pio/libdeps/m5stack-cores3/M5Unified/.piopm`）。`platformio.ini` には `m5stack/M5Unified@^0.2.5` とあるが、これは下限の指定にすぎず、実際に解決されたのは 0.2.26 である。
CoreS3 関連の電源コードは、upstream の commit `368ad60`（0.2.28、2026-10-25）と行番号まで一致する（diff の最初の差分は `Power_Class.inl` L2086 の timerSleep）。以下の GitHub リンクはこの commit に固定する。M5GFX は 0.2.33 がインストールされている（同じ libdeps 内）。

略記
- SE-pN = CoreS3 SE 回路図 v1.0 の N ページ（`https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_0N.png`）
- AXP-DS = X-Powers AXP2101 Datasheet V1.0 英語版（M5Stack がホストしているもの）
- PC = `M5Unified/src/utility/Power_Class.inl`

---

## Q1. M-Bus 5V は CoreS3 にどう入るか。USB VBUS と短絡・逆流し得るか。CoreS3 と SE の違い

### Takeaway
M-Bus の 5V ピン（コア側ネット `BUS_OUT`）と USB-C の VBUS（ネット `VUSB`）は、どちらも EN 負論理の負荷スイッチ ME1502CM5G（U19、U18）を通って AXP2101 の VBUS ピンに直結している。ダイオード OR も調停回路もない。
`BUS_OUT_EN=0` かつ `USB_OTG_EN=0` は「USB 入力 + BUS 入力」と公式に図示されている状態であり、AW9523 のリセット既定値でもある。この状態では 2 つの 5V 源が約 70 mΩ×2 の FET 越しに並列につながり、双方向に電流が流れ得る。CoreS3 と SE の電源部は同一である。

### Cited Findings
**ネットとトポロジ（回路図）**
- SE-p6 に負荷スイッチが 4 個ある。
  - U14 ME1502AM5G（EN 高有効）: VIN=`BUS_5V`、VOUT=`BUS_OUT`、EN=`BUS_OUT_EN`、RSET=R17 60K（"Boost" 経路）
  - U17 ME1502AM5G（EN 高有効）: VIN=`BUS_OUT`、VOUT=`VUSB`、EN=`USB_OTG_EN`、RSET=R44 60K（"OTG" 経路）
  - U19 ME1502CM5G（EN 低有効）: VIN=`BUS_OUT`、VOUT=`VBUS`、EN=`BUS_OUT_EN`、RSET=R46 30K（"PMU" 経路）
  - U18 ME1502CM5G（EN 低有効）: VIN=`VUSB`、VOUT=`VBUS`、EN=`USB_OTG_EN`、RSET=R45 30K
  - バルクコンデンサは `AXP_PS` と `BUS_OUT` にそれぞれ 220 µF（C124、C125）。
  - — [SE-p6](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_06.png)
- 無印 CoreS3 の p6 も部品・ネット・抵抗値が同一（目視と pdftotext のネット照合で確認。p1 と p5 の `BUS_OUT`/`R29`/`R30`/`TS`/`VBAT`/`AXP_PG` も同一）— [CoreS3 p6](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/490/Sch_M5_CoreS3_v1.0_page_06.png)、[CoreS3 回路図 PDF](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/490/Sch_M5_CoreS3_v1.0.pdf)、[SE 回路図 PDF](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0.pdf)
- 前段の AXP2101 側（SE-p1）: U1 AXP2101 の pin37 VBUS がネット `VBUS` に接続。pin33 BAT はネット `VBAT`。VSYS/VIN1-4 はネット `AXP_PS`。DCDC1（LX1/FB1）が `VDD_3V3` を出す。TS（pin31）は `BUS_OUT` を R29 10K / R30 10K で分圧した点につながり、TS = BUS_OUT/2。PWRON（pin30）= `PWR_KEY`（S2 の電源ボタン）、PWROK（pin29）= `AXP_PG`（S1 も接続）。J5 は VBAT 用の 2 ピンヘッダ — [SE-p1](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_01.png)
- SY7088 昇圧（U3）: IN=`AXP_PS`、OUT=`BUS_5V`、EN=`BOOST_EN`（R14 10K でプルダウン）。USB-C J1 の VCC はネット `VUSB` — [SE-p2](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_02.png)
- M-Bus コネクタ BUS1: pin28 = `BUS_OUT`、pin30 = `VBAT`、pin6 = `AXP_PG`、pin25/27/29 = NC。Grove J3（Port A）の pin3 も `BUS_OUT`。
  AW9523B（U10, 0x58）の割り当ては P0_1=`BUS_OUT_EN`、P0_5=`USB_OTG_EN`、P1_7=`BOOST_EN`。AD0/AD1 は GND、RSTN（pin23）は `AXP_PG` — [SE-p5](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_05.png)
- 補足: M-Bus の "5V" はコア側では `BUS_OUT` である。`BUS_5V` は SY7088 昇圧の出力だけを指すネット名で、M-Bus の 5V ではない（SE-p2、SE-p5、SE-p6 より）。

**公式の電源方向図**
- docs の図（"电源.png"）は 4 状態を示す。
  - `BUS_OUT_EN=0, USB_OTG_EN=0` = "USB INPUT / BUS INPUT"（両方から AXP2101 へ）
  - `1,0` = "USB INPUT / BUS OUTPUT"
  - `0,1` = "USB OUTPUT / BUS INPUT"
  - `1,1` = "USB OUTPUT / BUS OUTPUT"
  - 両入力を同時に使うことへの注意書きはページ上に見当たらない。
  - — [電源方向図](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/core/CoreS3/%E7%94%B5%E6%BA%90.png)、[CoreS3 SE docs](https://docs.m5stack.com/en/core/M5CoreS3%20SE)
- 同ページの記述:
  - "uses the AXP2101 power management chip together with the AW9523B IO expansion chip to control the direction of power input and output"
  - 比較表では CoreS3-SE の Battery Capacity が ❌、CoreS3 が 500 mAh。CoreS3 の Back Cover は "Base DIN"。
  - — [CoreS3 SE docs](https://docs.m5stack.com/en/core/M5CoreS3%20SE)

**AW9523 の既定値**
- AW9523B の Table 1: AD1=GND / AD0=GND のとき、パワーオン時の P1_7..P0_0 はすべて 0。Config_Port の既定値は 0（出力）。RSTN は "low reset; it has an internal 100 kΩ pull-low resistor" — [AW9523B datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/core/CoreS3/AW9523B-EN.pdf)
- したがって AW9523 がリセットされた直後は `BUS_OUT_EN=0`、`USB_OTG_EN=0`、`BOOST_EN=0` になる。U19 と U18 が ON、U14 と U17 が OFF、昇圧は停止。これは SE-p5 と Table 1 からの直接の帰結である。

**ME1502 の特性（Microne の中国語データシート V02。chipdip がホストする二次ホストのコピー）**
- 「70mΩ導通電阻」。RSET の式は「Ioc=60K/Rset」で、全温度範囲の誤差は ±15%。
  - Rset=30K のとき限流門限 1.7/2.0/2.3 A（min/typ/max）、短絡電流 1.2 A。
  - よって U18 と U19 は約 2 A、U14 と U17（60K）は約 1 A。
- 「无衬底二极管，芯片关断时可防止反向电流」（基板ダイオードがなく、**オフ時に**逆電流を防ぐ）。オン時の逆電流阻止については記載がない。
- 絶対最大定格は VOUT「-0.3 to VIN」。
- 過温保護 155 ℃、ヒステリシス 20 ℃。限流状態が続くと「功率管将被关断…结温降至…后恢复启动，如此循环」（自動復帰を繰り返す。ラッチはしない）。
- UVLO は 2.2 V（立ち上がり、typ）。θJA は 210 ℃/W（SOT23-5）。
- オフ時は VOUT に下拉（放電）機能がある。
- — [ME1502 datasheet（chipdip ミラー）](https://static.chipdip.ru/lib/087/DOC043087044.pdf)

**メンテナ自身が逆流経路の存在を書いている**
- PC L988-991（`_core_s3_output_locked`）:「BUS_OUT_EN=0 は BUS 入力側スイッチを ON にする操作なので、BUS_OUT が 5V のまま切り替えると USB VBUS が BUS_OUT へ流れ込み続け、以後 BUS_OUT_EN=0 でも 5V が残る」 — [Power_Class.inl#L988-L991](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L988-L991)
- PC L1036-1037:「A precharged BUS_OUT left with BUS_OUT_EN = 0 is pulled up from VBUS」 — [Power_Class.inl#L1036-L1037](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L1036-L1037)
- M5GFX 0.2.33 の `M5GFX.cpp` L1788 にも同じ注意がある:「a precharged BUS_OUT left with BUS_OUT_EN off latches to VBUS and blocks later enables」 — ローカル `moter-controler/.pio/libdeps/m5stack-cores3/M5GFX/src/M5GFX.cpp` L1779-1790、[M5GFX repo](https://github.com/m5stack/M5GFX/blob/master/src/M5GFX.cpp)

**DIN Base 側（コアのバッテリー端子に関わる部分だけ）**
- DIN Base V1.1:
  - DC ジャック → DSS34 → スイッチ S1 → `HPWR` → SY8303 降圧 → `BUS_5V` → M5-Bus の "5V-27"。
  - `BUS_BAT` → M5-Bus の "VBAT-29"。
  - TP4057 充電 IC（VCC は `VCC_5V`、BAT は `BUS_BAT`、PROG は R8 2K）が `BUS_BAT` に出力する。
  - `VCC_5V` は `BUS_5V` を SI2301 で切り替えたもので、`PWR`（DC 入力あり）のときに ON。
  - 1615RG の 2 色 LED（D2）が TP4057 の CHRG と STDBY に接続されている。
  - FB 分圧は R1 120K / R3 16K（R2 200K の役割は図からは確定できない）。
  - — [DIN Base 回路図 V1.1](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/559/SCH_DinBase_V1.1.pdf)

### Inferences
- **構造的な結論**: `BUS_OUT_EN=0` かつ `USB_OTG_EN=0` のとき、DIN Base の 5V（`BUS_OUT`）と Mac の VBUS（`VUSB`）は U19 → VBUS ノード → U18 の経路で直結する。ON 抵抗の合計は約 140 mΩ。電圧が高い側の源が自分側のスイッチを順方向に、相手側のスイッチを**逆方向**に通って、低い側の源へ電流を押し込む。
  - ME1502 の逆流阻止は OFF 時だけの機能である。M5Unified のコメントも ON 時の逆流を経験的に認めている。
  - 限流は電流検出が VIN→VOUT 向きに働くと考えられ、順方向側の約 2 A の制限しか効かない可能性が高い（逆方向の限流の有無はデータシートに記載がない → Gaps）。
- この直結は**ファームが動く前から**成立している。AW9523 の既定値がすべて 0 なので、ESP32 が起動していなくても、AXP2101 が OFF でも、両方の源は VBUS で並列になる。
  - 例外は「AW9523 の電源（VDD_3V3 = AXP2101 DCDC1）が落ちている間、EN ピンがどう見えるか」で、EN 線には外付けのプルダウンがない。
  - AW9523 の I/O は無電源ではほぼ 0 V に引かれると考えられ、負論理の U18/U19 は ON 側に倒れる可能性が高い。これは推定である。
- `BUS_5V`（昇圧出力）と `BUS_OUT`（M-Bus）は U14 で隔てられた別ネットである。`output_power=false` で昇圧を止めても、M-Bus と USB の分離は**起きない**。分離できる組み合わせは次のとおりだが、どれも今回の用途には使えない。
  - `BUS_OUT_EN=1`: U19 を OFF にできるが、同時に U14 が ON になり、BUS_OUT に昇圧の出力（または停止中の SY7088 の出力ノード）を接続してしまう。
  - `USB_OTG_EN=1`: U18 を OFF にできるが、同時に U17 が ON になり、BUS_OUT から Mac の VBUS に給電する方向になる。
  - 結論として、**バッテリーなしで DIN 給電と USB 接続を同時に使うと、どの設定でも 2 系統の 5V が何らかの形でぶつかる**（Q2 の cancel ロジックも参照）。
- CoreS3 の 500 mAh バッテリーは、標準の背面カバーである DIN Base 側に載っている（docs の比較表より推定）。DIN Base の `BUS_BAT` は M-Bus の VBAT を経由して AXP2101 の BAT ピンに直結する（SE-p1 と SE-p5 の `VBAT`）。
  - 標準構成では AXP2101 にバッテリーが見えている。VBUS が落ち込んでも BATFET の supplement mode が VSYS を支える。
  - **今回はバッテリーを外しているので、この緩衝がない。**
  - さらに 12V 投入時は、DIN Base の TP4057 が無負荷（10 µF + 100 nF）の `BUS_BAT` を駆動し続ける。AXP2101 の BAT ピンには「バッテリーのように見えるが容量のない 4.2V 源」がつながる。AXP2101 も自前の充電器で同じノードを充電しようとするので、充電器が 2 台並列になる。
  - DIN Base の赤 LED の揺らぎには 2 つの説明が成り立つ。
    - (a) バッテリーなしで TP4057 の CHRG/STDBY が発振している（TP4057 のデータシートでは未確認）。
    - (b) D2 のアノード側の電源は R7/R11 を介した `VCC_5V`（= `BUS_5V` を Q1 で切り替えたもの = M-Bus 5V = コアの `BUS_OUT`）なので、**点滅ではなく明るさがゆっくり揺れるなら、競合している 5V レール自体が 85 s の間に動いている直接の証拠**になり得る。
    - 最も判別力のある計測は、85 s の間の `BUS_OUT` の電圧（テスターかオシロ、または `getTSVoltage()×2` のログ）である。
- **UVLO による「VBUS への張り付き」の仕組み**: U19 の VIN は `BUS_OUT` である。ME1502 は VIN が UVLO（2.2 V typ、立ち上がり）未満の間、EN に関係なく OFF になる。
  - そのため空の BUS_OUT は VBUS から切り離されたままだが、UVLO を一度越えると U19 が ON になり、VBUS 側から逆に給電されて BUS_OUT が保持される。
  - M5Unified の「passes about 1.7 V it is pulled up from VBUS even with BUS_OUT_EN = 0」「latches to VBUS」は、この挙動を指していると読める（推定）。
  - USB のみの構成では TS ≈ 0 V なので setExtOutput(true) は cancel されず、BUS_OUT_EN=1（U19 OFF）にできた。DIN を接続した構成では TS ≈ 2.5 V なので cancel される。
- 12V だけのとき（`VUSB` なし）は U18 の VIN が 0 V で、ME1502 は UVLO により OFF、逆電流も阻止される。USB だけで `output_power=true` のときは U19 が OFF（BUS_OUT_EN=1）。どちらも源は 1 系統だけで、競合しない。この点は「USB のみで 15 分以上問題なし、12V のみで起動 OK」という観察と整合する。

### Gaps
- ME1502 の ON 時の逆方向電流の扱い（逆方向の限流や過温保護が働くか）はデータシートに記載がない。英語版や Microne 公式サイト上の原本は入手できず、chipdip のミラー（V02）のみ確認した。
- `BUS_OUT_EN`/`USB_OTG_EN` に外付けのプルダウンがない。AW9523 の無電源時や、DCDC1 が落ちた後の EN の実電位、ME1502 の EN に内部プルがあるかは未確認。実機で測れば確定できる。
- SY8303（DIN Base）の出力が逆に押し込まれたとき、シンク能力や OVP がどう振る舞うかは調べていない（範囲外）。Mac 側ポートの逆流保護も不明。
- DIN Base の FB 抵抗の上下の対応（R1 と R2 のどちらが上側か）が図から確定できず、5V の設定値は推定できていない。
- SY7088 のシャットダウン時の負荷切断（EN=0 のとき OUT=`BUS_5V` と IN=`AXP_PS` が切り離されるか）は一次資料で確認できていない。検索結果の要約では、Preliminary 版のアプリケーションノート（Rev 0.1A）に「load disconnect during shutdown」と「EN 内部 1 MΩ プルダウン」の記載があるとされる。ただし PDF（[LCSC](https://datasheet.lcsc.com/lcsc/2108131730_Silergy-Corp-SY7088DGC_C178240.pdf)、[ミラー](https://git.bertfpv.com/englebert/M5CoreTX/raw/branch/master/docs/SY7088-Silergy.pdf)）は取得できず、未検証。これが成り立たない場合、`BUS_OUT_EN=1, BOOST_EN=0` のときに DIN の 5V が U14 → `BUS_5V` → L5 → `AXP_PS`（VSYS）へ、AXP の入力段を迂回して流れ込む経路になり得る。

---

## Q2. M5Unified が CoreS3 に対して行うこと（`output_power` の true/false、`setUsbOutput(false)`、`setExtOutput()`）

### Takeaway
`M5.begin()` の処理順は次のとおり。
1. M5GFX のボード検出: AW9523 を初期化し、AXP の UVP 電源断を無効化する。
2. `Power.begin()`: BOOST_EN を 1 にし、AXP2101 を初期設定する。
3. `Power.setExtOutput(cfg.output_power)` を呼ぶ。

`output_power=false` は「BUS_OUT_EN=0（U19 は ON のまま）、BOOST_EN=0」を確定させるだけで、DIN 入力を選択する操作ではない。`setUsbOutput(false)` は USB_OTG_EN=0（U18 は ON のまま）にする。結果として両入力が並列になる既定状態がそのまま残る。
`output_power=true` にしても、バッテリーなしで BUS に外部 5V が来ていれば、ライブラリが有効化を取り消す（`ESP_LOGW` を出すだけ）。

### Cited Findings
**呼び出し順**
- `M5Unified::_begin()` は `Power.begin(); Power.setExtOutput(cfg.output_power);` の順に呼ぶ。`output_power` の既定値は true — [M5Unified.inl#L2322-L2323](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/M5Unified.inl#L2322-L2323)、[M5Unified.hpp#L133](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/M5Unified.hpp#L133)
- 本プロジェクトの呼び出し: `cfg.output_power = false;` のあと `M5.begin(cfg)`、続いて `M5.Power.setUsbOutput(false);`（`MC_USB_POWER` 版は `output_power = true` で、setUsbOutput は呼ばない） — `/Users/takuro/src/dof-reality-h3-custom-controller/moter-controler/src/main.cpp` L773-791

**M5GFX のボード検出（`M5.begin` → Display の初期化時、0.2.33）**。ローカル `.../M5GFX/src/M5GFX.cpp` L1739-1845:
- AW9523（0x58）への書き込み:
  - `0x02 |= 0b00000101`（P0_0 TOUCH_RST、P0_2 AW_RST）
  - `0x03 |= 0b00000011`。SPI のプルアップが効いていない（GPIO35/36/37 の読み値が 0）ときだけ `0b10000011`（BOOST_EN を含む）
  - `0x04=0b00011000`、`0x05=0b00001100`、`0x11=0b00010000`（P0 をプッシュプルに）、`0x12=0xFF`、`0x13=0xFF`
- AXP2101 への書き込み: `0x23` の bit0-4 を OFF（コメントは「A DCDC dip below 85% … makes the AXP2101 power itself off, and it stays off; let the SoC brown out and restart instead. OVP (bit 5) stays enabled.」）
- BUS_OUT_EN をプリチャージ付きで ON にするのは、SPI のプルアップがない場合（result==0）だけ。CoreS3 には SPI 線に R18/R19/R21 の 10K プルアップがある（SE-p5）ので、通常この分岐には入らない。
- その後、AXP2101 に `0x90=0xBF`、`0x94=28`（ALDO3 3.3V）、`0x95=28`（ALDO4 3.3V）を書く。
- — ローカル M5GFX.cpp L1767-1845、[M5GFX repo](https://github.com/m5stack/M5GFX/blob/master/src/M5GFX.cpp)

**`Power.begin()` の CoreS3/SE 分岐**（PC L409-440）
- AW9523: `0x03` の bit7 を ON（コメント「SY7088 BOOST_EN」）— [Power_Class.inl#L412](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L412)
- AXP2101:
  - `0x23` の bit0-4 を OFF（DCDC1-5 の 85% UVP による電源断を無効化。bit5 の DCDC OVP は有効のまま）— [L415](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L415)
  - `0x90=0xBF`、`0x92=13`（ALDO1 1.8V）、`0x93=28`、`0x94=28`、`0x95=28`（ALDO2-4 3.3V）
  - `0x27=0x00`（コメント「PowerKey Hold=1sec / PowerOff=4sec」。AXP-DS では IRQLEVEL=1 s、OFFLEVEL=4 s、ONLEVEL=128 ms）
  - `0x69=0x11`（CHGLED）
  - `0x10=0x30`（bit5 off-discharge を有効、bit3「PWROK pull low to restart」を無効、bit2「PWRON 16s」を無効）
  - `0x30=0x0F`（ADC を有効）
  - — [L416-427](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L416-L427)
  - `setChargeVoltage(4200)`（0x64）。続いて `0x50` の bit4 を ON（TS を外部固定入力として扱い、充電に影響させない）。コメントは「TS senses the 5V bus (no battery thermistor)」— [L428-435](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L428-L435)
  - コメント「The power key is wired to the AXP2101 PWRON only, and the AXP2101 IRQ pin is shared with the RTC INT, so neither reaches the ESP32」— [L436-439](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L436-L439)
- CoreS3 の分岐では、Core2 v1.1 用の `0x12=0x00`（BATFET 無効）と `0x68=0x01` は**実行されない**（それらは L856-881 の `else if (_pmic == axp2101)` にあり、CoreS3 は L745 以降の Axp192 判定に入らないため）— [L856-881](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L856-L881)

**`setExtOutput(enable)` → `_core_s3_set_ext_output()`**（PC L1066-1107, L1197-1205）
- 定数: `_core_s3_bus_en = 0b00000010`（P0_1）、`_core_s3_usb_en = 0b00100000`（P0_5）、`port1_bitmask_boost = 0b10000000`（P1_7）— [L900-901](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L900-L901)
- **false の場合**（`_core_s3_output_locked(BUS_EN, false)`、L967-1059）:
  - `buf[0]=orig & ~BUS_EN`。BUS_EN が 0 になるので、BOOST_EN も 0 にする。
  - 元々 BUS_EN=1 だった場合は、先に BOOST_EN=0 を書いて 200 ms 待ち（BUS_OUT を放電）、それから BUS_EN=0 にする。
  - 元々 0 だった場合は P0、P1 の順に 1 バイトずつ書く。
  - 結果は `BUS_OUT_EN=0`、`BOOST_EN=0`。
  - — [L967-1059](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L967-L1059)
- **true の場合**: 有効化の前に `_core_s3_ext_output_unsafe()` で判定する。
  - 判定内容: `reg00 & 0x08`（バッテリーあり）または `!(reg00 & 0x20)`（VBUS が good でない）なら安全。AW9523 P0 の BUS_EN が既に 1 なら安全。それ以外は TS ADC（0x36-0x37、0.5 mV/LSB）が `raw > 4000`（2.0 V 超）なら危険とする。
  - 危険なら 20 ms 間隔で最大 1 s 再判定し、それでも危険なら `false` を返す。`ESP_LOGW("Power","setExtPower(true) is canceled.")` を出す。
  - コメント「無バッテリーで BUS_OUT (TS で検出) に外部 5V が来ている間は、有効化すると自身の給電を断つので取り消す」
  - — [L1062-1107](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L1062-L1107)、[L1197-1205](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L1197-L1205)
  - 有効化するときは、BOOST_EN=1 を書いたあと、パルス幅を徐々に広げるプリチャージ（8 回、i×16 µs）を行い、最後に BUS_EN=1 を書いて読み戻しで確認する — [L1028-1049](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L1028-L1049)
- **`setUsbOutput(false)`** は `_core_s3_output(_core_s3_usb_en, false)`。P0_5=0 にし、BUS_EN が 0 なら BOOST_EN=0 も書く — [L1414-1433](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L1414-L1433)

**M5Unified が CoreS3 で触らない AXP2101 レジスタ（AXP-DS の既定値のまま）**
- 0x14 VSYS DPM = 4.7 V（110b）
- 0x15 VINDPM = 4.36 V（0110b）
- 0x16 IINLIM = 1500 mA（100b）
- 0x18: 充電を有効（bit1=1）、ゲージを有効、**ウォッチドッグは無効（bit0=0）**
- 0x19 WDT タイマー = 64 s（未使用）
- 0x22: DIE 過温 Level2 による電源断を有効（bit2=1）。bit1/bit0 は EFUSE
- 0x24 VOFF: EFUSE（2.6-3.3 V）
- 0x68 バッテリー検出: 有効
- 0x12/0x13 DIE 過温 Level1 = 125 ℃
- — [AXP2101 datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf) §6.13.2.10-6.13.2.21、6.13.2.65
- 確認方法: M5Unified には 0x14、0x15、0x16、0x18、0x19、0x22、0x24 を書くコードがない（CoreS3 分岐を grep で確認）— [Power_Class.inl](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl)

### Inferences
- 本ファーム（`output_power=false` に加えて `setUsbOutput(false)`）の最終状態は `BUS_OUT_EN=0, USB_OTG_EN=0, BOOST_EN=0` で、AW9523 のリセット既定値と同じである。ソフトウェアは並列状態を「選んだ」のではなく、「変えなかった」にすぎない。
  - `Power.begin()` の L412 で BOOST_EN が一瞬 1 になる。この間 U14 は OFF なので、BUS_5V は M-Bus に出ず、影響は小さいと見る。
- `output_power=true` にしても、12V（DIN の 5V）と USB の両方がある構成では、`_core_s3_ext_output_unsafe` が cancel する。
  - TS = BUS_OUT/2 ≒ 2.5 V > 2.0 V で、VBUS は good、バッテリーなし（reg00 bit3=0）という条件がそろうため。
  - ただし DIN Base の TP4057 が VBAT を 4.2V 付近に保っていて、AXP2101 が「バッテリーあり」と判定していれば、cancel されずに BUS_OUT_EN=1 になる。その場合は U14（昇圧）と DIN の 5V が BUS_OUT 上でぶつかり、U19 が OFF になる。
  - どちらに転んでも、2 源の競合は解消しない。
- AW9523 は RSTN=`AXP_PG` で、電源は VDD_3V3（DCDC1）から取っている。このため ESP32 だけがリセットされた場合（ソフトリセットやブラウンアウト）は、AW9523 の出力状態が保持される。PMIC が落ちて PWROK が Low になると、全ビットが 0 に戻る。

### Gaps
- M5GFX の検出シーケンスについては、upstream の commit 固定の行番号を取っていない（ローカルの 0.2.33 の行番号で記載）。
- 実機の reg00 bit3（バッテリーあり/なし）が、DIN Base の TP4057 が駆動している状態でどちらを示すかは未確認。`_core_s3_ext_output_unsafe` の分岐を左右する。

---

## Q3. AXP2101 の保護機能と電源断の条件、約 85 s の遅延の候補、再投入の条件、電源断理由のレジスタ

### Takeaway
AXP2101 の電源断要因（REG21 で区別できる）は次の 8 種類である。
- PWRON キー長押し
- ソフト電源断
- EN 端子 Low
- VSYS < VOFF
- VBUS > 7 V
- DCDC UVP
- DCDC OVP
- ダイ過温（Level2、145 ℃）

M5Unified が DCDC UVP を無効化しており、ウォッチドッグも無効なので、残る候補は「VSYS 低下」「VBUS 過電圧」「ダイ過温」「DCDC OVP」になる。
電源断後の再起動条件は「PWRON キー（ONLEVEL 128 ms）」または「VBUS の Low→High」である。BUS 側から VBUS が供給され続ける限り、エッジが来ず、OFF のままになる。この挙動は「USB を抜くだけでは戻らず、両方抜くと戻る」という症状を説明できる。
REG21 は POR でのみクリアされるので、「両方抜いて 20 s」の復帰では消える。電源ボタンで復帰させれば読める。

### Cited Findings
**入力の閾値と既定値**
- VBUS: 動作範囲 3.9-5.5 V、UVLO 3.75 V、**過電圧の立ち上がり閾値 7 V**
- VINDPM の範囲 3.88-5.08 V（既定 4.36 V）、IINLIM 100-2000 mA
- DCDC1-4 の UVP は 85%、OVP は 130%
- — AXP-DS Table 5-5 / §5.5 [AXP2101 datasheet](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/AXP2101_Datasheet_V1.0_en.pdf)

**POR と起動**
- 「AXP2101 is powered from the higher voltage between VBUS and BAT. When VBUS voltage is higher than VVBUS_UVLOZ or BAT voltage is higher than VBAT_UVLOZ … All registers are reset to the default value.」 — AXP-DS §6.5.1（同上）
- VINDPM 到達時:「the charge current will decrease automatically until the current is zero. If ISYS is over the input power supply capability, VSYS will drop. If VBAT is above VSYS, PMU will enter the supplement mode.」 — AXP-DS §6.5.3.2

**電源投入の要因**（PWRON モード時）
1. POK を ONLEVEL 以上押す
2. VBUS low go high
3. VBAT low go high
4. IRQ Low 16 ms 以上
5. 充電で Vbat > 3.3V

2-5 は「configured by customization」。
— AXP-DS §6.5.4.2

**電源断の要因**
1. POK を OFFLEVEL 以上押す（REG22[1]）
2. REG10H[0]=1
3. 「VSYSGOOD high go low. When VSYS<VOFF or VBUS>7V, AXP2101 will be powered off. The default of VOFF is 2.6V … REG24H[2:0]」
4. DCDC が設定値より 15% 低い（REG23[4:0]）
5. DCDC 過電圧（REG23[5]）
6. ダイ温度が warning level2（145 ℃）を超える（REG22[2]）

— AXP-DS §6.5.4.3

**リセットの種類**
- System reset（「registers will be reset when PMU is powered on … all voltage outputs are turned off except RTCLDO and VREF」）の契機:
  - PWROK を Low にする（REG10[3]）
  - REG10[1]
  - ウォッチドッグのタイムアウト（REG18[0]、REG19[5:4]）
- Power on reset:「all voltage outputs are turned off including RTCLDO and VREF」
- — AXP-DS §6.5.4.5

**状態レジスタ**
- REG00: bit5 VBUS good、bit4 BATFET、bit3「Battery present」、bit1「thermal regulation」、bit0「in current limit state」
- REG01: bit4「System is power on」、bit3「VINDPM」、bit2:0 充電状態
- — AXP-DS §6.13.2.1-2
- **REG20 PWRON status**（Reset 列は「System Reset」）:
  - bit5 EN モード
  - bit4 Battery Insert
  - bit3 Battery > 3.3V when charged
  - **bit2 Vbus Insert and Good**
  - bit1 IRQ
  - bit0 POWERON key
- **REG21 PWROFF status**（Reset 列は**「POR」**）:
  - bit7 ダイ過温
  - bit6 DCDC 過電圧
  - bit5 DCDC 低電圧
  - **bit4 VBUS 過電圧**
  - **bit3 Vsys 低電圧**
  - bit2 EN Low
  - bit1 ソフト
  - bit0 POWERON キー
- 注意: REG21 は POR でしかクリアされないので、**前回の POR 以降に起きたすべての PWROFF 要因が OR で積み上がる**と読むべきである（例: 以前のボタン長押しによる電源断の bit0 が残る）。複数のビットが立っていても、すべてが今回の要因とは限らない（データシートの Reset 列が「POR」であることからの推定）。
- — AXP-DS §6.13.2.17-18
- REG22 PWROFF_EN: bit2 ダイ過温 Level2 による電源断（既定 1）、bit1/bit0 は EFUSE — AXP-DS §6.13.2.19
- REG23: bit5 DCDC OVP による電源断（既定 1）、bit4-0 DCDC5-1 の 85% UVP による電源断（既定 1） — AXP-DS §6.13.2.20
- REG24: 「Battery Voltage for POWEROFF」2.6-3.3 V、0.1 V 刻み、既定は EFUSE — AXP-DS §6.13.2.21

**ウォッチドッグと過温**
- REG18 bit0「Watchdog Module enable」の既定は 0。REG19 bit2:0 のタイマー既定は 110b = 64 s。REG19[5:4] で「IRQ only / System Reset / PWROK 1s / DCDC/LDO PWROFF & PWRON」を選ぶ — AXP-DS §6.13.2.14-15
- ダイ過温 Level1 は 115/125/135 ℃（既定 125 ℃）で IRQ。Level2 は 145 ℃で電源断。ADC でダイ温度を測れる（ch4） — AXP-DS §6.13.2.8-9、§6.5.4.3、§6.10

**バッテリーと TS**
- バッテリー検出:「As long as an AC adapter is present and usable, battery detection will be enabled … can be disabled through reg68H[0]. If the function is disabled, PMU considers that battery is always present.」 — AXP-DS §6.7.3.5
- REG50 bit4:「1: TS pin is the external fixed input and doesn't affect the charger」（既定は EFUSE） — AXP-DS §6.13.2.46
- ゲージの low battery shutdown threshold（REG1A[3:0]）は存在するが、§6.11 には IRQ のことしか書かれておらず、§6.5.4.3 の電源断要因にも入っていない — AXP-DS §6.11、§6.13.2.16

**ライブラリでの扱い**
- XPowersLib には次がある。
  - `getPowerOnSource()` / `getPowerOffSource()`（REG 0x20/0x21 を生値のまま enum にキャストしている。ビットフィールドを enum の番号として返すので、値の意味づけは誤りやすい）
  - 個別のビット関数 `isOverTemperatureOffSource()`、`isDcOverVoltageOffSource()`、`isDcUnderVoltageOffSource()`、`isVbusOverVoltageOffSource()`、`isVsysUnderVoltageOffSource()`、`isPwronAlwaysLowOffSource()`、`isSwConfigOffSource()`、`isPwrSourcePullDown()`
  - — [XPowersAXP2101.hpp L151-171, L760-823](https://github.com/lewisxhe/XPowersLib/blob/d6997586e68f65afd51baa775903df930db39821/src/XPowersAXP2101.hpp)、定数 `XPOWERS_AXP2101_PWRON_STATUS (0x20)` / `PWROFF_STATUS (0x21)`（[AXP2101Constants.h L32-33](https://github.com/lewisxhe/XPowersLib/blob/d6997586e68f65afd51baa775903df930db39821/src/REG/AXP2101Constants.h#L32-L33)）
- M5Unified には 0x20/0x21 を読む API がない（ソース全体を grep して該当なし）。ただし `M5.Power.Axp2101` は public メンバーで（Power_Class.hpp L245）、`readRegister(0x21, &v, 1)` や `readRegister8(0x21)` で直接読める。
  - `getVBUSVoltage()`、`getTSVoltage()`、`getAPSVoltage()`（VSYS）、`getInternalTemperature()` も用意されている。
  - — [AXP2101_Class.hpp](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/power/AXP2101_Class.hpp)、[Power_Class.hpp](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.hpp)
- M5Unified の Core2 v1.1 分岐にあるコメント「A DCDC dip below 85% … makes the AXP2101 power itself off, and it stays off until the power key」（メンテナが「一度落ちたら電源キーまで OFF のまま」と実機で認識している） — [Power_Class.inl#L870-L873](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L870-L873)

### Inferences
**ラッチオフと復帰の仕組み（推定。症状とはすべて整合する）**
- AXP2101 がどの要因で PWROFF しても、その後の流れは同じになる。
  1. DCDC1 が止まり、VDD_3V3 が落ちる。ESP32、LCD、AW9523 が停止し、USB デバイスが消える。
  2. PWROK が Low になり、AW9523 がリセットされる（EN 線は 0 または無電源）。
  3. U19 が ON のままなので、DIN Base の 5V が VBUS を保ち続け、AXP2101 本体は通電したまま「OFF 状態」にとどまる。
  4. 自動の再投入要因は「VBUS Low→High」だが、VBUS が落ちないのでエッジが来ない。
- USB を抜いても VBUS は BUS 側から供給され続けるので、変化がない。12V と USB の両方を抜いて約 20 s 待つと、VBUS（と BAT ノード）が UVLO を下回って POR になり、再接続で VBUS の挿入エッジ（REG20 bit2）が発生して起動する。
- 「約 20 s」は、BUS_OUT の 220 µF（C125）、AXP_PS の 220 µF（C124）、VBUS 周辺の 22 µF、DIN Base 側の容量が放電する時間と考えて矛盾はない。ただしこの時定数は未測定。
- **判別テスト**: 停止後に側面の電源ボタン（S2 → PWR_KEY → AXP の PWRON。M5Unified の 0x27=0x00 により ONLEVEL=128 ms）を押す。
  - これで起動するなら「PMIC のソフトな OFF 状態」で確定する。そのまま次の起動時に REG21 を読めば、電源断の要因がわかる（REG21 は POR でしか消えない）。
  - ボタンでも起動しないなら、別の仕組み（AXP の POR すら起きない固着など）を疑う。

**電源断要因の絞り込み**
| 要因 | 状態 | 85 s の遅延との整合 |
|---|---|---|
| DCDC 85% UVP（REG21 bit5） | M5GFX と M5Unified が 0x23 bit0-4 を OFF にしているので**除外** | — |
| TWSI ウォッチドッグ | REG18[0]=0 が既定で、M5Unified も有効化しない。**除外**（既定タイマーは 64 s だが無効） | — |
| ソフト電源断 / キー | ファームから `powerOff()` を呼んでいなければ除外 | — |
| **ダイ過温 Level2 145 ℃（bit7）** | REG22[2]=1 が既定で、触られていない | **秒〜分単位の時定数を持つ唯一の内部要因**。ただし、2 源間の循環電流は U18→VBUS→U19 を流れ、AXP の中を通らない。AXP が発熱する源は明確でない（VBUS→VSYS の経路とチャージャーの損失くらい） |
| **VSYS < VOFF（bit3）** | VOFF は EFUSE（2.6-3.3 V）。バッテリーがないので supplement による緩衝がない | 判定そのものは即時。ただし VBUS 側の何かが 85 s 後に崩れた（DIN の降圧が逆に押されて保護動作した、Mac ポートの保護が落ちた、ME1502 が過温サイクルに入ったなど）結果としてなら起こり得る |
| **VBUS > 7 V（bit4）** | 常に有効 | 遅延の説明にはならない。ただし「12V 投入中に USB を挿したとき」「24V やリレーを切り替えたとき」の過去の事例とは整合する。ケーブルのホットプラグによるリンギングや、GND の跳ね上がりで VUSB/BUS_OUT が瞬間的に 7 V を超えれば、U18/U19（ON）を通じて AXP の VBUS に届く |
| DCDC OVP 130%（bit6） | 有効 | 入力側の過渡で DCDC が跳ねた場合にあり得る。根拠は弱い |

- ME1502 の過温保護（155 ℃、20 ℃ヒステリシスで自動復帰）はラッチしない。循環電流が約 2 A の限流に達して過温サイクルに入ると、VBUS ノードと `BUS_OUT`（M-Bus 5V。DIN 側では `BUS_5V`/`VCC_5V`）がゆっくり振動する可能性がある。これは「DIN の赤 LED が揺らぐ」現象と整合し得るが、同じ現象は TP4057 のバッテリーなし発振でも説明できる。
- 約 85 s の遅延を直接生む仕組みは、一次資料の中には見つからなかった。熱的な蓄積（ME1502、AXP のダイ、DIN の SY8303、Mac ポートのいずれか）が最有力だが、未確定である。

**推奨する計測**（REG21 は POR で消える前提で設計する）
- 1 秒ごとにシリアルへ出す:
  - `M5.Power.Axp2101.readRegister8(0x00)` / `0x01`（VBUS good、バッテリーあり、限流中、VINDPM 中）
  - `getVBUSVoltage()`、`getAPSVoltage()`（VSYS）、`getTSVoltage()`（≒BUS_OUT/2）、`getBatteryVoltage()`（VBAT ノード）、`getInternalTemperature()`
  - 最優先は `getTSVoltage()×2`（≒`BUS_OUT`）の推移。DIN の LED の揺らぎがレールの変動によるものかを判別できる。
  - これで 85 s の間に何が単調に変化するか（温度か、電圧か）がわかる。シリアルは停止の直前まで出る。
- 起動時に 0x20 と 0x21 を記録する。電源ボタンで復帰させたときだけ意味がある。

### Gaps
- M5Stack の EFUSE の中身（VOFF=REG24、REG22[1:0]、TS の既定、VBUS 挿入による起動の有効可否）は不明。VBUS 挿入による起動が有効であること自体は、「USB を挿すと起動する」という日常の挙動からの推定である。
- AXP2101 データシートには、REG21 が「System Reset」と「POR」のどちらの範囲で保持されるかの記載以上に、PWROFF 後に VBUS が残っているときの挙動（何秒で再試行するか、など）の記述はない。英語版 V1.0 にも見当たらなかった。
- 中国語版データシートやエラッタは参照していない。
- 約 85 s という時間を直接説明する一次資料は見つからなかった。

---

## Q4. ESP32-S3 のブラウンアウト検出（BOD）で説明できるか。PMIC の電源断との見え方の違い

### Takeaway
BOD は「チップをリセットして再起動する」仕組みで、ラッチしない。画面が暗いまま、USB が消えたまま、電源を完全に抜かないと戻らない、という症状は BOD ではなく、AXP2101 の PWROFF（PMIC の OFF 状態）の特徴である。

### Cited Findings
- ESP32-S3 の BOD:「it will reset the chip in order to prevent unintended behaviour」。既定のレベルは `ESP_BROWNOUT_DET_LVL_SEL_7` = 「2.44V」（レベル 1 = 3.30V まで選べる） — ESP-IDF `components/esp_hw_support/power_supply/port/esp32s3/Kconfig.power`（ローカル `~/.platformio/packages/framework-espidf/...`）、[ESP-IDF Kconfig reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/kconfig-reference.html)
- 本環境の Arduino 用 sdkconfig も `CONFIG_ESP_BROWNOUT_DET=y`、`CONFIG_ESP_BROWNOUT_DET_LVL=7` — `~/.platformio/packages/framework-arduinoespressif32-libs/esp32s3/sdkconfig` L2013-2021（注: moter-controler は `platform = espressif32@6.12.0`。どの libs が実際にリンクされるかは未確認）
- BOD のハンドラは `esp_reset_reason_set_hint(ESP_RST_BROWNOUT)` を設定し、「Brownout detector was triggered」をログに出してから `esp_rom_software_reset_system()` を呼ぶ — ESP-IDF `components/esp_hw_support/power_supply/brownout.c` L63-80（ローカル）
- ESP32-S3 の 3.3V は AXP2101 の DCDC1 から来ている（SE-p1 の LX1/FB1 → `VDD_3V3`）。M5Unified と M5GFX は、DCDC が落ち込んだ場合に AXP を落とさず「let the SoC brown out and restart instead」となるよう、0x23 の UVP を無効化している — [SE-p1](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/498/Sch_M5_CoreS3_SE_v1.0_page_01.png)、[Power_Class.inl#L870-L873](https://github.com/m5stack/M5Unified/blob/368ad60807161625aa65baad37a4a3af2a7eccea/src/utility/Power_Class.inl#L870-L873)

### Inferences
| 観察点 | ESP32 の BOD | AXP2101 の PWROFF |
|---|---|---|
| 画面 | 一瞬消えて、再起動で起動画面に戻る | 消えたまま |
| Mac の USB | 一度消えて再列挙される（数秒以内） | 消えたまま |
| 次の起動での `esp_reset_reason()` | `ESP_RST_BROWNOUT` | POR 後は `ESP_RST_POWERON` |
| AW9523 の状態 | 保持（RSTN=AXP_PG は High のまま） | リセット |
| 復帰の手段 | 不要（自動） | 電源ボタン、または VBUS の Low→High（両方の電源を抜く） |
| REG21 | 変化なし | 要因ビットがセットされる（POR まで保持） |

- 観察された症状（暗転、USB 消失、USB を抜くだけでは戻らず、両方抜いて約 20 s で復帰）は、右の列とすべて一致し、左の列とは一致しない。
- ただし BOD が先に起き、続いて VSYS < VOFF で PMIC が落ちる、という連鎖は否定できない。VBUS の崩れがゆっくりなら、DCDC1 → ESP の BOD → VSYS < VOFF の順に進み得る。
- 区別するには、起動ごとに `esp_reset_reason()` と REG20/REG21 を記録し、停止前の数十秒の `getVBUSVoltage()` と `getAPSVoltage()` の推移を見る。

### Gaps
- 本プロジェクトのビルド（espressif32@6.12.0 / Arduino 2.x 系）で、BOD レベルの実効値が 7 であることを sdkconfig で確認していない（ローカルで見たのは別の libs パッケージの sdkconfig）。
- 停止時に BOD のログ（「Brownout detector was triggered」）がシリアルに出ていたかどうかは、ユーザーの観察記録にない。
