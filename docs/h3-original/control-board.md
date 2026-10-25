# オリジナル制御基板

DOF Reality 独自の制御基板。STM32F401 ボードをソケットに差し込む構造で、各モーターの PWM/DIR 出力、エンコーダ入力、電流監視、ブレーキリレー、主電源リレー制御、OLED 表示、PC との USB 通信を 1 枚で統括していた。
4 軸分（H4 向け拡張）が実装されており、H3 では 3 軸分を使用。

## マイコンボード

| 項目 | 値 |
| --- | --- |
| MCU | STMicroelectronics **STM32F401CCU6**（刻印で確認。当初の F411CEU6 推定は訂正） |
| CPU | Arm Cortex-M4F、最大 84MHz |
| Flash / SRAM | 256KB / 64KB |
| 動作電圧 | 1.7〜3.6V。GPIO は 3.3V ロジック |
| パッケージ | UFQFPN48 |
| ボード | Black Pill 系互換ボード。`PWR` LED、`KEY` ボタン、左下と右上に `5V` 端子 |

## コネクタ・部品と接続先（導通確認済み）

| 名称 | 接続先 | 信号 | 備考 |
| --- | --- | --- | --- |
| DC_STOP1 | 電源基板 | 黒 GND / 赤 12V / 緑 主電源リレー制御（PB10）/ 青 5V | [電源基板](power-board.md) |
| PB_1 / PB_2 / PB_3 | Protection Board #1〜3 | GND / +24V / VIOUT / +5V | 「PB」は Protection Board の意 |
| PB_4 | NC | | 拡張用 |
| CYT_1 / CYT_2 / CYT_3 | MD13S #1〜3 | GND / NC / PWM / DIR | 「CYT」は Cytron の意 |
| CYT_4 | NC | | 拡張用 |
| 8pin コネクタ ×4 | 各モーター | 上側 6pin = Feedback Sensor、下側 2pin = Brake | [モーター](motor-and-feedback-sensor.md)、[ブレーキ](brake.md) |
| K1〜K4 | 各モーターの Brake 2pin | NO 接点でブレーキ 24V を開閉 | コイルは並列、Q1 で一括駆動 |
| U5 | Feedback Sensor の予備 pin（抵抗経由）×4 | I²C → PB8/PB9 | ADS1015。今回のモーターでは入力 NC |
| U6 | PB_1〜4 の VIOUT ×4 | I²C → PB8/PB9 | ADS1015 |
| U3 | 3 端子部品 | 不明（レギュレータの可能性） | 型番未読 |
| SCREEN1 | OLED/LCD | 表示 | 故障時は無表示 |
| USB1 | PC | USB CDC | 故障時は COM 未認識 |
| SW1 | 操作入力と推定 | | 確度低 |
| D3 / D4 | LED | 故障時 D3 点灯、D4 消灯 | 機能未特定 |

当初「PB_n はエンコーダ入力」「K1〜K4 は軸別に制御」と推定していたが、いずれも導通確認で訂正した。

## U5 / U6 = TI ADS1015

刻印 `21 BRPI`。`BRPI` は Texas Instruments ADS1015 のマーキング。

| 項目 | 値 |
| --- | --- |
| 分解能 / 入力 | 12bit、4ch（AIN0〜3） |
| 通信 | I²C。STM32 の PB8 = I2C1_SCL、PB9 = I2C1_SDA に接続 |
| 最大サンプリング | 3,300 SPS |
| 電源 | 2.0〜5.5V（基板上の動作電圧は未確認） |
| パッケージ | VSSOP-10 |

```text
Protection Board #1〜4 ── VIOUT ── U6 AIN0〜3 ─┐
                                              ├── I²C（PB8/PB9）── STM32
Feedback Sensor 予備 pin #1〜4 ── 抵抗 ── U5 ─┘
```

つまり元の基板は、**4 軸分のモーター電流を ADS1015 経由で監視**していた。U5 は旧型モーターのアナログ位置センサー用と考えられるが、今回の Hk/Hr 系モーターでは該当ピンがケーブル側 NC で未使用。

## エンコーダ入力ピン（実測）

Feedback Sensor の白・緑は STM32 に直結。各軸が独立したタイマーの CH1/CH2 に対応しており、ハードウェアのエンコーダモードで計数していたと判断できる。

| モーター | 白 | 緑 | タイマー |
| --- | --- | --- | --- |
| #1 | PA9（TIM1_CH2） | PA8（TIM1_CH1） | TIM1 |
| #2 | PB3（TIM2_CH2） | PA15（TIM2_CH1） | TIM2 |
| #3 | PB5（TIM3_CH2） | PB4（TIM3_CH1） | TIM3 |
| #4 | PB7（TIM4_CH2） | PB6（TIM4_CH1） | TIM4 |

## ブレーキリレー K1〜K4 の駆動

- 4 個のコイルは並列に接続され、制御基板上の **Q1 でまとめて駆動**される。STM32 の **1 本の GPIO**（ピン番号未確認）から全軸一括で ON/OFF。
- 接点は **NO**。無通電時に導通なし。
- 各リレーの接点は、対応する軸の Protection Board の 24V **入力側**から取った 24V を、その軸の Brake 2pin へ開閉する。
- 駆動指令は共通、給電経路は軸ごと。

## 使用している STM32 のピン（判明分）

| ピン | 機能 |
| --- | --- |
| PB10 | 主電源リレー制御（DC_STOP1 緑） |
| PB8 / PB9 | I²C（U5・U6 ADS1015） |
| PA8 / PA9 | モーター #1 エンコーダ |
| PA15 / PB3 | モーター #2 エンコーダ |
| PB4 / PB5 | モーター #3 エンコーダ |
| PB6 / PB7 | モーター #4 エンコーダ |
| 不明 | CYT_1〜4 の PWM / DIR（計 8 本） |
| 不明 | ブレーキリレー Q1 の駆動 |
| 不明 | SCREEN1、SW1、D3 / D4 |

## GND 構成

Protection Board の制御側 GND と、Protection Board 上の MD13S 向け電源 GND は完全に導通する。したがって **STM32 の GND とモーター電源の GND は電気的に共通**で、制御系とモーター系は絶縁されていない。
別電源で給電していても、モーターの大電流やノイズが制御配線・USB へ回り込む経路になりうる。新設計でも GND の経路を明示して設計する必要がある（[安全設計](../cores3-controller/safety.md)）。

## 故障時の状態

| 項目 | 状態 |
| --- | --- |
| STM32 ボードの LED | 点灯したまま変化なし |
| D3 / D4 | D3 点灯、D4 消灯 |
| LCD | 無表示 |
| USB | Windows で抜き差ししてもデバイスの変化なし |
| 3.3V | ほぼ正常 |
| 5V（STM32 ボードの 5V 端子） | **2.9V**。電源基板の 5V 出力は 5.0V で、経路で 2.1V 降下。GND 間の電位差は 0.9mV |

5V 供給経路（コネクタ・パターン・保護素子・U3 周辺）の高抵抗化、または STM32 ボード側の過負荷が疑われるが、原因分析は保留した（[故障診断の経緯](troubleshooting-log.md)）。
