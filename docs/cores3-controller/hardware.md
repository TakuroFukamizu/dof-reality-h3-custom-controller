# ハードウェア

## CoreS3 と Base15

| 構成 | 外部 I/O | 備考 |
| --- | --- | --- |
| CoreS3 SE + Base15 産業用プロト基板モジュール V1.1（K025-B） | Grove Port.B（手配線）、XH 8pin、XH 2pin | **現在の実装は CoreS3 SE（K128-SE）**。`platformio.ini` の board は `m5stack-cores3`、PSRAM 8MB Quad。**Base15 は 2026-10-31 時点で未製作。配線・実機確認はこれから** |
| （旧）CoreS3 SE + DIN Base | Grove Port.A / B / C | 2026-10-25 までの試験構成。[進捗](progress-and-open-issues.md) の試験結果はこの構成で取得 |

- ESP32-S3: 240MHz デュアルコア、USB CDC、2 インチ 320×240 タッチディスプレイ。GPIO は 3.3V、**5V 耐圧ではない**。
- CoreS3 SE 本体にバッテリーは無い。Base15 にもバッテリー・充電回路は無い（verified: [Sch_BASE_v1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/539/Sch_BASE_v1.1.pdf)、[CoreS3 SE 製品比較表](https://docs.m5stack.com/en/core/M5CoreS3%20SE)）。**この構成では CoreS3 を動かし続けるバッテリーが存在しない**。

### DIN Base から Base15 へ変更した理由

1. Grove コネクタで直結できる相手が MD13S しかない。Protection Board も Grove コネクタだが、+12V と +5V が共存するなどピン配置が特殊で直結できない。
2. Feedback Sensor、ブレーキ解除リレー、主電源リレーなど、Grove 以外で接続する I/O が複数ある。
3. Bottom 内に回路（リレー用 MOSFET、Feedback Sensor 用レベル変換）を収めるには表面実装部品より空間が必要で、バッテリーを外すことになる。

結果としてバッテリーと追加 Grove 端子が不要になり、DIN Base を採用し続ける合理性が低くなった。Base15 には XH 8pin と XH 2pin のソケットを増設し、Base15 内のプロト領域にリレー用 MOSFET とエンコーダ用レベル変換を実装する。Grove Port.B は MD13S 用に DIN Base と同じピン配置で Base15 に実装する。

### Base15 V1.1 の調査結果

出典: [製品ページ](https://docs.m5stack.com/en/base/Base15_v1.1)、[回路図 Sch_BASE_v1.1.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/539/Sch_BASE_v1.1.pdf)、[CoreS3 SE 製品ページ（M-Bus 表）](https://docs.m5stack.com/en/core/M5CoreS3%20SE)、[Stack Screw Length](https://docs.m5stack.com/en/guide/basics/stack_screw_length)。Switch Science の製品ページ（products/8661）は取得できなかった（HTTP 429）ため、M5Stack 公式資料を一次資料にしている。

確認済み（verified）:

- 電源: DC ジャック（PWR3.5）→ PPTC ヒューズ → MP1584EN 降圧（入力 9〜24V → 5V、連続 2A）→ M-Bus 28 番（5V）に直結。CoreS3 SE へはこの M-Bus 5V 経由で給電する（DIN Base と同じ方式）。
- 生の入力電圧 IN+（9〜24V）が M-Bus 25 / 27 / 29 番（HPWR）と P1 ヘッダ（12V+ / 12V−）のパッドに出ている。CoreS3 SE 側ではこの 3 ピンは NC だが、**Base15 基板上は活線パッド**。XH コネクタ・MOSFET・レベル変換の配線時に短絡させない。
- M-Bus ヘッダ J1 は 30 ピン全てが受動的なパススルーで、各ピンの外側に 1 対 1 のはんだパッドがある。プロト領域から配線できる。
- **基板のシルクと回路図のピン名は初代 Core（ESP32）の GPIO 番号**で書かれている。CoreS3 SE で使うときは「位置」で読み替える（後述の表）。
- Grove は「customizable」2 ポート（J3 UART 型 / J4 I²C 型）。回路図上は隣接する 4 ピンパッド P3 / P4 に繋がるだけで、**バスの GPIO・5V・GND には配線されていない**。HY2.0-4P コネクタ 2 個は同梱・未実装。Port.B 相当にするには GND・5V（28 番）・G9（10 番）・G8（4 番）を手配線する。
- 寸法: 仕様表 54 × 54 × 28 mm、本文「本体厚 15 mm（バスヘッダ除く）」、寸法図（[K025-model-base15.pdf](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1002/K025-model-base15.pdf)）は全高 21.5 mm・取付穴 44 mm ピッチと、高さが資料間で不一致。Stack Screw Length 表では Base15 = 12.0 mm 相当として計算されている。
- Stack Screw Length 表に「Core2/CoreS3 + Module + Base15」の組合せが載っている。

未確認（unverified）:

- CoreS3 **SE** との互換性の明示的記載は公式ページに見つからなかった（Stack Compatibility 表は JS 描画で取得不可）。CoreS3 と同一のバス配列なので実質互換と考えているが、一次資料での裏取りは未完。
- プロト領域の穴数（公式記載なし。写真目視で 100 穴強、2.54 mm ピッチ）と、本体 15 mm 内の部品高さクリアランス。MOSFET・レベル変換・XH コネクタが収まるかは実物で確認する。
- MP1584 出力 +5V と M-Bus 5V の間に逆流防止ダイオードは回路図上無い。USB 給電（書き込み時）と Base15 DC 給電を同時に行った場合の挙動は未検証。DIN Base 時代の Issue #1（12V と USB 同時接続での電源断）と同じ系統の確認が必要。

### M-Bus の位置とシルク表記の対応

CoreS3 SE の M-Bus 表（偶数が右列）と Base15 回路図 J1 のピン名を位置で突き合わせたもの。はんだ付けは **位置** で行い、シルクの数字をそのまま GPIO 番号と読まない。

| 信号（CoreS3 SE） | M-Bus 位置 | Base15 シルク / 回路図名 | 用途 |
| --- | --- | --- | --- |
| G10（ADC1_CH9） | 2 | 35 | Protection Board VIOUT（分圧後） |
| G8 | 4 | 36 | MD13S DIR |
| G9 | 10 | 26 | MD13S PWM |
| 3V3 | 12 | 3.3V | レベル変換の LV 電源 |
| G18 | 15 | 16 | エンコーダ B |
| G17 | 16 | 17 | エンコーダ A |
| G2 | 19 | 2 | 主電源リレー IN |
| G1 | 20 | 5 | （旧ブレーキ。Base15 では未使用） |
| G13（I2S_DOUT） | 23 | 15 | ブレーキ解除リレー（MOSFET ゲート） |
| 5V | 28 | 5V | Port.B 赤、XH 8pin #7 |
| GND | 1 / 3 / 5 | GND | GND |

使ってはならないパッド: シルク「13」= G7、「1」= G43（TXD0）、「18」= G36（SCK）、「SDA / SCL」= G12 / G11（AXP2101 など CoreS3 内部 I²C）。HPWR（25 / 27 / 29）は生の 9〜24V。

## GPIO 割当（現状）

`moter-controler/include/config.h` に基づく。2026-10-31 に Base15 向けへ変更した。

| 接続経路 | GPIO | 用途 | 状態 |
| --- | --- | --- | --- |
| XH 2pin #2 | **G2** | 主電源リレー制御。Base15 内は GPIO 直結（駆動段は Base15 外。注 3） | DIN Base 構成（Port.A 黄）で動作確認済み。Base15 経由は未確認 |
| XH 8pin #5 → Base15 内 MOSFET | **G13** | ブレーキ解除リレー制御 | **旧 G1 から変更。未配線・未確認（コードのみ変更）** |
| Grove Port.B 黄 | **G9** | MD13S PWM | DIN Base 構成で動作確認済み。Base15 の手配線 Port.B は未確認 |
| Grove Port.B 白 | **G8** | MD13S DIR | 同上 |
| XH 8pin #2 → Base15 内レベル変換 | **G17** | エンコーダ A（センサー白） | DIN Base 構成（Port.C 黄）で計数確認済み。Base15 経由は未確認 |
| XH 8pin #3 → Base15 内レベル変換 | **G18** | エンコーダ B（センサー緑） | 同上 |
| XH 8pin #6 → Base15 内分圧 | **G10**（ADC） | Protection Board VIOUT | 未配線。`PIN_VIOUT` の定数のみ。ADC 読み取りコードは無い |
| M-Bus 12 番 | 3V3 | レベル変換の LV 電源 | Base15 内配線 |
| M-Bus 28 番 | 5V | Port.B 赤（MD13S には接続しない）、XH 8pin #7 予備 | |

### XH 8pin ソケット

Feedback Sensor エンコーダ、ブレーキ解除リレー、Protection Board を接続する。M5Stack 5V 出力 / 3.3V 出力は今後の拡張用。

| # | 接続先 | 備考 | CoreS3 SE 接続先 |
| --- | --- | --- | --- |
| 1 | GND | | GND |
| 2 | Feedback Sensor エンコーダ A（センサー白） | Base15 内でレベル変換へ接続 | G17 |
| 3 | Feedback Sensor エンコーダ B（センサー緑） | Base15 内でレベル変換へ接続 | G18（注 1） |
| 4 | Feedback Sensor 電源 5V | レベル変換の HV 用。既存電源基板の 5V | N/A |
| 5 | ブレーキ解除リレー制御（リレーモジュールの IN） | Base15 内でリレー駆動用 MOSFET へ接続 | G13 |
| 6 | Protection Board VIOUT | Base15 内で分圧抵抗へ接続 | G10 |
| 7 | M5Stack 5V 出力 | 予備 | 5V（M-Bus 28 番） |
| 8 | M5Stack 3.3V 出力 | 予備 | 3.3V（M-Bus 12 番） |

- 注 1: 原案では #3 も G17 と記載されていたが、A と同じピンにはできないため G18 とし、2026-10-31 に G18 で確定した。
- #4（センサー電源 5V、既存電源基板由来）と #7（CoreS3 / Base15 の M-Bus 5V）は **別の 5V 系統**。Base15 内で結ばない。

### XH 2pin ソケット

主電源リレー制御を接続する。3 系統の CoreS3 のうち、実際に接続するのは 1 系統のみの想定。

| # | 接続先 | 備考 | CoreS3 SE 接続先 |
| --- | --- | --- | --- |
| 1 | GND | | GND |
| 2 | 主電源リレー制御（リレーモジュールの IN） | 電源基板側に駆動素子 Q1（原案では MOSFET。種別は未確認）があるので、Base15 内は GPIO 直結 | G2（注 2） |

- 注 2: 原案に CoreS3 側ピンの記載が無く、現行の G2（旧 Port.A 黄）を維持。2026-10-31 に G2 で確定した。
- 注 3: XH 2pin #2 の先は、原案では「リレーモジュールの IN」と「電源基板側に MOSFET がある」の両方が書かれている。現行の試験構成は 1ch リレーモジュールの IN に入れている。最終的にリレーモジュールを介すか、電源基板の DC_STOP1 緑（R1 15kΩ → Q1）へ直結するかは要確認。どちらでも Base15 内は GPIO 直結。

### 注意点

- Port.B の赤（5V）は MD13S に接続しない。GND は共通にする。
- G2 は M5Unified が外部 I²C（Ex_I2C: SCL = G1 / SDA = G2）に使いうるため、外部機器探索（`external_display_value`、`external_speaker_value`、`external_imu`、`external_rtc`）を無効化し、起動時に I²C の High でリレーが入らないようにしている。G1 が空いても I²C は SCL / SDA 両方が必要なので、このガードは緩めない。
- `external_speaker_value = 0` は Module Display / RCA 検出時に M5Unified が M-Bus 23 番 = G13 を I²S DOUT として取るのを防ぐ意味も持つ。
- **G13 の内部用途**: CoreS3 SE 内部では I²S データ出力（ESP32-S3 → 内蔵スピーカーアンプ AW88298 の DATAI）で、M-Bus 23 番にも出ている（verified: [CoreS3 SE M-Bus 表](https://docs.m5stack.com/en/core/M5CoreS3%20SE)、M5Unified `pin_data_out = GPIO_NUM_13`）。`internal_spk = false`・`internal_mic = false` を維持すれば `M5.begin()` は G13 / G14 に触れず、I²S ドライバも初期化されない。AW88298 は電源投入既定のスタンバイ（PWDN=1, AMPPD=1, I2SEN=0）のまま。DATAI は IOVDD = 3.3V（ESP32 と同じレール）の CMOS 入力なので、G13 を 3.3V GPIO 出力にして問題ない。DATAI の入力リーク電流はデータシートに記載が無く未確認。
- **G13 のブート時レベル**: GPIO13 はストラップピンではなく（ストラップは GPIO0 / 3 / 45 / 46）、リセット中は入力無効・リセット後は内部プル無しで浮く。GPIO1 / 2 も同じなので、ブート中の挙動は旧 G1 と同等。リセット中の浮きでブレーキ解除リレーが入らないよう、リレー IN がオフ側に保たれるプル抵抗を外付けする。非反転段なら G13 側のゲートにプルダウン。ローサイド反転段ではゲートプルダウンが IN を HIGH に倒すので向きが逆になる。回路構成が決まってから確定する。ファームは主電源用 `RELAY_ON_LEVEL` とブレーキ用 `BRAKE_ON_LEVEL` を分けてあり、反転段ならブレーキ側だけを 0 にする。ROM ブートローダーが GPIO13 を一時的に駆動しないかは一次資料で確認できておらず、実機で起動中の波形を確認する。
- 表示の自動検出に失敗して board_unknown になった場合のみ、M5Unified が StampS3 / Capsule 判定で G13 を Low → High にマイクロ秒パルスして復元する経路がある。CoreS3 SE と正常に検出されれば実行されない。ただし M5GFX の表示自動検出そのものは `cfg` では止められない。CoreS3 の判定（G11 / G12 の内部 I²C で AXP2101 と AW9523 を見る）に失敗すると他機種の探索に進み、その中に G13 を SPI 出力（PaperColor / PaperS3 判定）、G2 を I²C 出力（PaperColor 判定、AirQ の `_pin_reset`）として駆動する経路がある。NVS に保存済みのボード種別はこのフォールバックを防がない。内部 I²C が正常なら到達しないが、この経路に対する保護は外付けのプル抵抗だけなので省略しない。
- GPIO9 は ESP32-S3 ではストラップピンではないが、リセット後は内部プル無しで浮く。PWM 入力には外部プルダウンを入れ、リレー駆動入力にもオンレベルと逆のプル抵抗（HIGH オンならプルダウン）を外付けする。
- G10 は ADC1_CH9（Wi-Fi と競合する ADC2 ではない）、CoreS3 内部未使用、M-Bus 2 番（BUS_ADC1）に直結。Base15 では 2 番位置（シルク「35」）のパッドから引き出す。はんだ付け後、電源 OFF で M-Bus 2 番への導通を確認する。
- GPIO17 / 18 は ADC2（CH6 / CH7、Wi-Fi と競合）なので VIOUT には使わない。VIOUT をエンコーダのピンに割り当てることはしない。
- 1 軸あたり PWM / DIR / エンコーダ ×2 / ブレーキ / 主電源 / VIOUT の 7 信号が必要。Grove 直結できるのは MD13S だけなので、残りは Base15 の XH ソケット経由で M-Bus から引き出す。

## MD13S との接続

Grove（HY2.0-4P）の線色は 黒 = GND、赤 = 5V、黄 = IO1、白 = IO2。M5Unified の `port_b_pin1` は「5V に近い端子」で、色では白側になる。コードの IO1 / IO2 は公式表（黄 / 白）に合わせている。Grove ケーブルを MD13S の 4 線コネクタ用に加工する。

| MD13S | CoreS3 側 |
| --- | --- |
| GND | Port.B 黒 |
| NC | — |
| PWM | Port.B 黄 G9 |
| DIR | Port.B 白 G8 |

Base15 の Grove 2 ポート（J3 / J4）はバスに未配線なので、どちらかを Port.B として GND・5V（M-Bus 28 番）・G9（10 番、黄）・G8（4 番、白）に手配線する。J3 / J4 のどちらを使うかは未定。赤（5V）は MD13S 側では使わない。

PWM 設定: 20kHz（MD13S の上限）、10bit、sign-magnitude。速度 0 で PWM Low（両出力 Low）。反転時は PWM を落とし 5ms 待ってから DIR を変える。`DIR_POSITIVE_LEVEL = 0`（正転 = DIR Low = MA High）。

## リレー（主電源・ブレーキ）

- GPIO はコイルを直接駆動しない。主電源は `RELAY_ON_LEVEL = 1`（G2 HIGH でオン）、ブレーキは `BRAKE_ON_LEVEL = 1`（G13 HIGH で解除指令）で、別定数。リセット中に浮いてオンしないよう、リレー IN がオフ側に保たれるプル抵抗を外付けする（主電源は G2 にプルダウン。ブレーキは MOSFET の回路構成で向きが決まる）。
- 主電源リレー: G2 → XH 2pin #2 → リレーモジュールの IN（現行試験構成）または既存電源基板の DC_STOP1 緑 → R1 15kΩ → Q1 に直結（原案の記述。注 3、要確認）。Base15 内は GPIO 直結で、駆動段は Base15 の外。緑線は 3.3V ロジックで、元の STM32 が直接駆動していた実績がある。最終構成では 3 軸共通のインターロックに置き換える（[安全設計](safety.md)）。
- ブレーキ解除リレー: G13 → Base15 内のリレー駆動用 MOSFET → XH 8pin #5 → リレーモジュールの IN（またはリレーコイル）→ 接点で 24V を開閉 → Brake 2pin。24V はその軸の Protection Board の 24V 入力側から取る。接点はブレーキのコイル負荷を開閉するので、フライバックダイオードや TVS などの保護を定格確認後に追加する。MOSFET の型番、ゲートプルダウン値、XH 8pin #5 に出す信号（MOSFET のドレイン出力か、リレーモジュールの IN に合わせた論理）は未定。
- リレーの接点定格が DC24V のブレーキ電流を開閉できることを確認する。ブレーキの消費電流は未測定。

## エンコーダ用レベルシフタ

Feedback Sensor の白・緑は 5V 系なので、CoreS3 へ入れる前に 3.3V へ変換する。レベル変換は Base15 内に実装し、センサーは XH 8pin #2 / #3 / #4 / #1 で接続する。

### 使用部品: 秋月電子 AE-LCNV4-MOSFET（BSS138、販売コード 113837）

HV 側・LV 側それぞれに 10kΩ プルアップを持つ双方向レベル変換。OE 配線は不要。

| AE-LCNV4-MOSFET | 接続先 |
| --- | --- |
| HV | XH 8pin #4（センサーへ供給する +5V） |
| LV | CoreS3 M-Bus 12 番 3V3 |
| GND | XH 8pin #1、CoreS3 GND、センサー黒、センサー用 5V 電源の GND |
| HV1 | XH 8pin #2（センサー白） |
| LV1 | G17（M-Bus 16 番） |
| HV2 | XH 8pin #3（センサー緑） |
| LV2 | G18（M-Bus 15 番） |
| HV3 / LV3 / HV4 / LV4 | 未使用 |

```text
XH #2 センサー白 ── HV1 [AE-LCNV4-MOSFET] LV1 ── G17（M-Bus 16）
XH #3 センサー緑 ── HV2                     LV2 ── G18（M-Bus 15）
XH #4 センサー +5V ─ HV
M-Bus 12 3V3 ──────── LV
XH #1 センサー GND ── GND ─────────────────── CoreS3 GND
```

- M-Bus の 5V（28 番）を LV に入れない。GPIO を HIGH にして電源代わりにしない。
- センサー用 5V は既存の電源基板 5V を継続使用し、XH 8pin #4 から HV に入れる。M-Bus 5V（#7）とは結ばない。
- G17 / G18 は内部プルアップ・プルダウンなしの入力にする。プルアップはシフタ側にある。
- HV–GND、LV–GND に 0.1µF を基板近くに追加。信号線（HV1/2、LV1/2）にはコンデンサを付けない（10kΩ プルアップで立ち上がりが遅くなる）。
- 期待値: 変換前 白 約 0.1V / 緑 約 5V → 変換後 LV1 Low 0V 付近 / LV2 High 約 3.3V。LV に 5V 近くが出たら CoreS3 に接続せず配線を再確認する。
- レベル変換は絶縁ではない。GND 回り込み対策とは別。
- 低速で読めても最高速度でパルスを取りこぼさないことの保証にはならない。波形・計数の検証が必要（DIN Base 構成では 25% で約 3000 count/s まで計数欠けなし。Base15 経由では未確認）。
- センサー断線時もプルアップで HIGH になるため、HIGH 表示だけで接続正常とは判定しない。

### 代替候補

| 部品 | 特徴 | 状況 |
| --- | --- | --- |
| 秋月 AE-TXS0104E-BO（TXS0104E） | 4ch 双方向、プッシュプル／オープンドレイン両対応。OE を 3.3V 側で HIGH にする必要あり（基板上に OE プルダウン 10kΩ） | 購入済み、交換評価用に保留 |
| Adafruit TXB0104（千石電商） | 4ch、プッシュプル向き。オープンドレインや強いプルアップには不向き | 候補 |
| 固定方向の 5V 耐圧入力バッファ（74LVC 系） | 自動方向判定の波形乱れを避けたい場合 | 候補 |

BSS138 で低速計数ができても速度を上げるとカウントが合わない場合は、原因を波形・ノイズ・ソフト設定に分けて調べる。TXS0104E への交換で必ず解決するとは扱わない。

## Protection Board VIOUT の分圧（予定）

VIOUT は ACS71240 のアナログ出力で、5V 版なら 3.3V を超える。デジタル用レベルシフタは使えず、抵抗分圧で ADC の範囲に収める。分圧抵抗は Base15 内に実装し、入力は XH 8pin #6、出力は G10（M-Bus 2 番）。

| 構成 | 分圧比 | VIOUT 5V 時 |
| --- | --- | --- |
| 上 10kΩ / 下 10kΩ（推奨） | 0.5 | 2.5V |
| 上 20kΩ / 下 30kΩ | 0.6 | 3.0V |

```text
XH #6 VIOUT ── 10kΩ ──┬── G10（ADC、M-Bus 2 番）
                      10kΩ
                       │
                      GND（Protection Board の GND と CoreS3 の GND を共通）
```

- ESP32-S3 Arduino の `ADC_11db` では測定範囲上限が約 3.1V。10k/10k なら上限に余裕がある。ソフトの分圧比も合わせる。
- ACS71240 の出力負荷条件（10kΩ 以上）と、ADC のサンプリング回路が要求する入力インピーダンスに注意する。ノイズが大きければバッファやフィルタを追加。
- 分圧だけでは電源投入順序による逆給電や過渡電圧を防げない。必要なら入力保護を追加。
- Protection Board の +24V は GPIO に接続しない。+5V と GND は電源系統として配線する。
- ファームウェアは `PIN_VIOUT = 10` の定数のみで、ADC 読み取りは未実装。

## 配線・施工の方針

- ジャンパーワイヤの仮配線を本番に使わない。圧着済みケーブル、ロック付きコネクタ（XH、HY2.0-4P）、ストレインリリーフを優先する。
- HY2.0-4P（Grove）と XH は小信号用。モーター電流やブレーキ電流を流さない。ブレーキの 24V 開閉はリレー接点側で行い、Base15 内には入れない。
- 信号配線はモーターの電力線やリレーの負荷線と束ねない。レベルシフタは Base15 内（CoreS3 直下）に置き、LV1/LV2 から M-Bus までを短くする。
- センサー・レベルシフタの細い GND 線をモーター電流の戻り経路にしない。
- Base15 にはバッテリーが無く、Base15 の DC 入力が失われると CoreS3 は即座に停止する。バッテリーで駆動許可を保持する経路は存在しないが、主電源（24V 側）が失われたことを検知できる構成にする方針は変わらない。
- Base15 内の HPWR パッド（生の 9〜24V）の近くに信号配線を通さない。

## 部品リスト（1 軸分 ＋ 予備）

| 部品 | 数量目安 | 用途 | 状況 |
| --- | --- | --- | --- |
| M5Stack CoreS3 SE | 3 + 予備 | 制御 | 1 台で試験中 |
| Base15 産業用プロト基板モジュール V1.1（K025-B） | 3 + 予備 | Bottom。電源、XH ソケット、MOSFET、レベル変換を収容 | 未購入・未製作 |
| XH 8pin ソケット（基板用）＋ ハウジング・コンタクト | 3 + 予備 | エンコーダ・ブレーキ・VIOUT | |
| XH 2pin ソケット（基板用）＋ ハウジング・コンタクト | 3 + 予備 | 主電源リレー制御 | |
| HY2.0-4P → バラ線ケーブル | 3 + 予備 | MD13S 接続（Port.B） | |
| リレー駆動用 MOSFET（型番未定）＋ プル抵抗（位置・向きは回路構成で確定） | 3 + 予備 | Base15 内でブレーキ解除リレーを駆動 | 型番未定 |
| 3.3V 入力対応 1ch リレーモジュール | 3 + 予備（主電源用も。注 3） | ブレーキ 24V 開閉、主電源リレー駆動 | 1 軸分準備済み |
| AE-LCNV4-MOSFET | 3 + 予備 | エンコーダ 5V→3.3V | 購入済み、使用中 |
| AE-TXS0104E-BO | 評価用 | 代替レベルシフタ | 購入済み |
| 10kΩ 抵抗 | 各 10 本 | VIOUT 分圧、プル抵抗 | |
| 0.1µF セラミックコンデンサ | 数個 | レベルシフタ電源 | |
| DC 負荷向け TVS・フライバックダイオード | 定格確認後 | ブレーキ接点保護 | |
| 絶縁クリップ付き測定線 | 数本 | 安全な電圧測定 | |
| 予備 USB-C ケーブル | 2〜3 本 | CoreS3 通信・給電 | |

M-Bus の GPIO 引き出しは Base15 のプロト領域とパッドで行うため、DIN Base 時代に検討した「M-BUS GPIO 引き出し部品」は不要になった。
