# moter-controler

M5Stack CoreS3 SE で MD13S・主電源リレー・ブレーキリレーを操作し、フィードバックセンサーを読んで、系統ごとの原点復帰・定位置移動を行う。設計は `docs/superpowers/specs/2026-10-25-moter-controler-homing-design.md`、実機記録は [docs/acceptance_homing.md](docs/acceptance_homing.md)、PC から 3 系統を操作した記録は [docs/multi_axis_20261031.md](docs/multi_axis_20261031.md)（スクリプトは `tools/multi_axis/`）。

## ビルド

PATH の `pio` は使わない。

```sh
cd moter-controler
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3            # Bottom（Base15）の DC ジャックから給電する本番設定
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-usbpower   # USB のみ給電（開発用。Bottom の DC は接続しない。DIN Base 時代の Issue #1 回避用）
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-usbpower -t upload --upload-port /dev/cu.usbmodem2101
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-v1            # USB プロトコル v1（h3-control-server 接続。下の節）
tests/run_tests.sh   # ホストテスト（状態機械とプロファイル、v1 のコーデックと状態機械）
```

## 配線（Base15）

Bottom は Base15 産業用プロト基板モジュール V1.1（M5Stack K025-B）。DIN Base は使わない。Base15 に XH 8pin と XH 2pin のソケットを増設し、リレー駆動の MOSFET とエンコーダ用のレベル変換を Base15 内に収める。バッテリーは無い。給電は Base15 の DC ジャック（9〜24V → MP1584 → BUS 5V）。Base15 での電源経路は未検証。詳細は [../encoder-tester/docs/wiring.md](../encoder-tester/docs/wiring.md)。

| 信号 | 定数 | GPIO | 接続 |
| --- | --- | --- | --- |
| MD13S PWM | `PIN_MD13S_PWM` | G9 | Grove Port.B 黄（Base15 上。DIN Base と同じ配置） |
| MD13S DIR | `PIN_MD13S_DIR` | G8 | Grove Port.B 白 |
| エンコーダ A（センサー白） | `PIN_ENC_A` | G17 | XH 8pin #2 → Base15 内レベル変換 |
| エンコーダ B（センサー緑） | `PIN_ENC_B` | G18 | XH 8pin #3 → Base15 内レベル変換 |
| ブレーキ解除リレー IN | `PIN_MOTOR_BRAKE` | G13 | XH 8pin #5 → Base15 内 MOSFET（非反転。2026-10-31 に G13 HIGH で解除を実機確認）。GPIO は HIGH で解除指令（`BRAKE_ON_LEVEL = 1`）。回路を反転段に変えるときは `BRAKE_ON_LEVEL` だけを 0 にする。主電源の `RELAY_ON_LEVEL` は触らない |
| Protection Board VIOUT | `PIN_VIOUT` | G10 | XH 8pin #6 → Base15 内で分圧。ADC 読み取りは未実装 |
| 主電源リレー IN | `PIN_POWER_RELAY` | G2 | XH 2pin #2。GPIO 直結（駆動素子 Q1 は電源基板側。種別は未確認）。HIGH で給電（`RELAY_ON_LEVEL = 1`） |

G13 は CoreS3 SE 内部で I2S_DOUT（スピーカーアンプ AW88298）と共用。`internal_spk = false` と `external_speaker_value = 0` を保つ（`src/main.cpp` の `setup()`）。リセット中の浮きでブレーキ解除リレーが入らないよう、G13 側（MOSFET ゲート）にプルダウンを外付けする（非反転段で確認済み）。

## 端当ての退避

下限へ・上限へ・校正で、開始から 400 カウント未満しか動かずに止まったときは、失敗にせず反対方向へ 600 カウント退避してから改めて端へ向かう（シリアルの理由欄に `retreat`）。既に端に居る状態から始めても到達できる。退避でも動かなければ「失敗:stall」「失敗:slow」。

## 手動速度の守り

手動速度（±1 / ±10、`S`）の出力中、|速度| が 7 % 以上のとき、カウントが 200 ms 変わらなければ「無パルス停止」、|速度| が 10 % 以上で 200 ms 窓の速度が 150 count/s 未満なら「遅すぎ停止」として出力を 0 にし、速度指令を 0 に戻す（画面に通知、シリアルに `WARN manual_stall` / `WARN manual_slow`）。センサーのケーブルが抜けた状態で動かし続けないための守り。7 % に達してから 300 ms は猶予。7 % 未満は機構が動かないことがある（7 % で動くことを実測）ので監視しない。速度の符号が変わったら（反転）計測をやり直す（反転をまたぐ窓では正味の変化が小さく、遅すぎに見えるため。2026-10-31 に +9 → −15 で誤停止）。

## 画面

上から: 系統 `#n`（タップで切替。電源 OFF 時のみ）と状態 / 電源・ブレーキ（解除/作動） / センサー行（原点基準のカウント、count/s、A/B、原点・校正の有無） / 速度 −10 −1 [n] +1 +10 / 自動移動（下限へ・上限へ・中立へ・校正。#3 は右端へ・左端へ） / 停止・非常停止 / フッター（USB 診断行、DIR、ピンのレベル、行程・中立・符号）。

USB 診断行（v1 env では tick 行の下。GitHub Issue #11「列挙されたまま CDC が無応答」の切り分け用。ボードをリセットせずに読む）:
`sof1 in1 pl1 r002 e20c oa0 if1 br0 er0 tx!0 rx1234 2s`。`sof` = 500 ms の間に SOF フレーム番号が進んだ（バスが生きている）、
`in` = その間に IN endpoint 1 への IN token が来た（ホストがこのアドレスを読みに来ている）、`pl` = HWCDC::isPlugged()、
`r`/`e` = USB_SERIAL_JTAG の int_raw / int_ena（hex。bit2 = OUT 受信、bit3 = IN empty、bit9 = bus reset）、
`oa` = OUT FIFO にデータが残っている（ISR が取っていない）、`if` = TX FIFO に空き、`br` = bus reset 回数、
`er` = PID/CRC/stuff エラーの検出回数、`tx!` = Serial.write が途中で諦めた回数（HWCDC は接続判定 false のとき捨てても「全部書けた」と返すので、その破棄は数えられない。TX 側は `if` と「`rx` は増えるのにホストに届かない」で見る）、`rx` = 受信バイト数と最終受信からの秒。
`br` は HWCDC の esp_event 経由で数える（イベントタスクが 1 本増え、RX/TX のたびに ISR から post する）。計器自体が条件を変えていないか比べるため `-DMC_USB_DIAG_BUSRESET=0` で外せる。
最初の書き込みでは、ポーラを動かして緑（`sof1 in1`）、Mac 側でポートを閉じて橙（`in0`）になることを確認してから判定表を信じる（IN token ビットが NAK 応答の IN token でも立つことは未検証）。
色: 緑 = sof と in が両方ある、橙 = SOF はあるが IN token が来ない（ポート未オープンか、ホストとデバイスのアドレス不一致）または OUT FIFO に残り、赤 = SOF が止まっている（バス停止 / suspend / PHY）。

## シリアル（115200）

`R 0|1` `B 0|1` `S -100..100` `X` `E` `?` `HELP` に加えて:

| コマンド | 動作 |
| --- | --- |
| `M 1\|2\|3` | 系統の選択（電源 OFF 時のみ）。NVS に保存し、原点は無効になる |
| `HOME` | P 端（速度 + で当たる端）へ 10 % で当て、原点 0 にする |
| `UPPER` | N 端へ。原点が必要。到達位置が実測ストローク |
| `NEUTRAL` | 中立へ比例制御。原点と校正が必要 |
| `CAL` | HOME → UPPER → 保存 → NEUTRAL |
| `UNCAL` | 選択中の系統の校正を消す（電源 OFF 時のみ） |
| `USBDIAG` | USB-Serial/JTAG の状態 1 行（画面最下行と同じ。Issue #11 の切り分け用） |

「中立へ」の比例制御は Duty 下限 `NEUTRAL_DUTY_MIN`（7 %）から始め、猶予後に無パルスなら `HOME_DUTY`（10 %）へ一気に、遅すぎなら 1 % ずつ下限を上げる（重力負荷の持ち上げ側で 8 % 台では動かない #2 の対策。10 % でも遅ければ失敗）。到達帯は入口 `NEUTRAL_BAND`（±10）、Dwell 中は出口 `NEUTRAL_BAND_EXIT`（±20）で判定し、帯の縁で数カウント揺れても 200 ms の Dwell を数え続ける（#3 が 3077〜3080 で往復して収束しなかった対策）。

`?` は `STAT` 行と `ENC` 行を出す。自動移動中は 500 ms ごとに `ENC`、状態が変わると `AUTO <kind> <phase> <reason> pos=`。校正完了で `AUTO saved ...`。要求が拒否されると `ERR auto_<reason>`。

## 座標と保存値

- P 端 = 原点 0。N 端へ向かって増える。`ENCODER_SIGN=+1` で速度 − のときカウント増。系統ごとに校正時に符号を判定して保存する。
- NVS `h3`: `motor`、`m{n}_stroke`、`m{n}_neutral`、`m{n}_cal`、`m{n}_sign`。
- 2026-10-25 時点: #3 校正済み（stroke 6160、neutral 3080、sign +1）。#1・#2 は未校正。
- 2026-10-26 のレビュー修正（未書き込み・実機未確認）: 自動移動中の速度操作は中止（PWM 0）に統一、中立の再始動時に速度窓を取り直す、PCNT リミット跨ぎの読み直し、入力の鮮度監視、校正中は保存ストロークを信じない。

## USB プロトコル v1（h3-control-server 接続）

PC 側の [h3-control-server](../control-server/) と USB でつなぐためのファームウェア。仕様は [docs/control-server/protocol.md](../docs/control-server/protocol.md)、載せ方は [docs/superpowers/specs/2026-10-31-moter-controler-v1-protocol-design.md](../docs/superpowers/specs/2026-10-31-moter-controler-v1-protocol-design.md)。3 台とも同じファームで、担当する系統は画面で選ぶ（NVS の `motor`）。

| env | 用途 |
| --- | --- |
| `m5stack-cores3-v1` | Base15 の DC ジャックから給電する本番設定 + v1（`-DMC_PROTO_V1=1`） |
| `m5stack-cores3-v1-usbpower` | USB のみ給電 + v1（机上確認用。Bottom の DC は接続しない） |

書き込みは手動で行う（下はコマンド例。実機が別の作業で使用中のときは実行しない）:

```sh
cd moter-controler
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-v1
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-v1 -t upload --upload-port /dev/cu.usbmodemXXXX   # 例。ポートは SER= で個体を確認してから
```

構成:

- `include/v1_codec.h` — COBS、CRC-16/CCITT-FALSE、フレーム、protocol.md §4 の全メッセージの LE エンコード・デコード（純粋、ヘッダのみ）。
- `include/v1_axis.h` — 1 軸の状態機械 `V1Axis`（DISARMED → ARMING → HOLDING ⇄ ACTIVE、STOPPING、DISARMING、FAULT）。`control-server/src/sim_axis.rs` と同じ入出力・同じ応答を目標にしている。位置制御は速度フィードフォワード＋比例（`v1_duty_command`: `V1_FF_COUNT_S_PER_PERCENT`・`V1_KP_PERCENT_PER_100`・不感帯 `V1_DEADBAND` / `V1_DEADBAND_OUT` のヒステリシス・`V1_DUTY_MIN`〜`V1_DUTY_MAX`。[docs/multi_axis_20261031.md](docs/multi_axis_20261031.md) の 3 系統運転の実測から置いた仮値）。無パルスの `ENCODER_FAULT` は、ブレーキ解除指令中に目標（軌道）が動いている間だけ判定する（目標が止まっているときの静止や DISARMING 中の残り誤差では出さない。代わりに、エンコーダが死んだまま目標が止まっている間は検出できず、目標が動いてから 200 ms で出る。FAULT_RESET を拒む「残る原因」は画面の非常停止のラッチだけ）。Duty の向きは系統プロファイルの `sign` から決める（`actual_count` には符号を掛けない）。
- `src/v1_link.cpp` — `Serial` のバイト受信 → COBS 区切り → 受信キュー、送信キュー → `Serial.write`（loop() 側だけが書く）。`device_id` は `0x5131` + eFuse MAC を表記順に並べた 48 bit（`v1_device_id_from_mac`。例 `1C:DB:D4:BA:8D:58` → `51311cdbd4ba8d58`。画面の `dev XXXX` は MAC の末尾 2 オクテット）、`boot_id` は `esp_random()`。
- `src/control_task.cpp` — `MC_PROTO_V1` では 5 ms タスクが `V1Axis` を回し、主電源 G2（電源担当軸のみ）・ブレーキ G13・PWM を直接書く。
- 時間定数は `config.h` の `POWER_SETTLE_MS` / `BRAKE_RELEASE_WAIT_MS` / `BRAKE_APPLY_WAIT_MS`（各 300 ms、仮値）と、校正用の `CAL_POWER_SETTLE_MS`（5000 ms、仮値）。

校正（v1、protocol.md §1.2）: PC の CALIBRATE（`0x0C`、mode 1）で MCU が **DISARMED のときだけ**（CONFIG・位置基準は不要）`CALIBRATING`（8）に入り、従来 env の `CAL` と同じ端当て状態機械（`auto_move.h` の `AutoMachine`、`AutoKind::Cal`: P 端当て → 退避 → N 端当て → 中立。無パルス・遅すぎ・移動量・時間の守り付き）を自分で回す。手順は 主電源 ON 指令（電源担当軸だけ）→ `CAL_POWER_SETTLE_MS` → ブレーキ解除給電 → `BRAKE_RELEASE_WAIT_MS` → 端当て → 終了手順（ブレーキ給電 OFF → `BRAKE_APPLY_WAIT_MS` → PWM 0 → 主電源 OFF 指令）→ CAL_RESULT（`0x87`）を 1 回 → STATUS（DISARMED）。成功すると **P 端 = 0 の位置基準を MCU 自身が立て**（`POSITION_REFERENCED`。PC は SET_REFERENCE を送らない）、行程・中立・符号を系統プロファイルとして NVS に保存する（保存は loop() 側。画面に「校正を保存」）。CAL_RESULT の `n_end_count` / `neutral_count` は `actual_count` と同じ座標（P 端 = 0、符号は掛けない。符号 −1 の系統では負）。失敗（`stall` / `slow` / `travel` / `timeout`）・中止（CALIBRATE mode 0 または DISARM、reason `stop`）では基準を消し、保存せず、FAULT にもしない。HOST_FAULT・画面の非常停止は校正中も即 FAULT（CAL_RESULT は `CANCELLED` で FAULT_EVENT の前に 1 回）。校正中は ARM_PREPARE / START / CONFIG / SET_REFERENCE が `BAD_STATE`、SET_POSITION は捨てる（数えない）、系統のタップは効かない。主電源を持たない軸は電源担当軸と**同時に**校正を指示する（24V が無いと端当てが無パルスで失敗する。PC 側がまとめて送る）。

画面（v1）: 系統 `#n`（タップで 1 → 2 → 3。**DISARMED のときだけ**。切り替えると CONFIG・位置基準が消え AXIS_SELECT を送る）/ 状態名（FAULT は赤で異常コード名、CALIBRATING は段階 P 端へ / N 端へ / 中立へ）/ 電源・ブレーキの指令と Duty / `actual_count`・目標・追従誤差・速度 / CONFIG・基準・電源担当の有無 / リンク（PC から最後に受けてからの経過 ms、`rx_error`、`stale`、送信キュー満杯）/ 校正（校正中は段階と理由、終わった後は最後の CAL_RESULT の結果・理由・N 端・中立・sign）/ tick 数と最大間隔 / **非常停止**（いつでも。押すと `FAULT(LOCAL_ESTOP = 12)`、ラッチ中はボタンが「非常停止 解除」になる。解除してから PC が FAULT_RESET を送る）。

v1 で無効になるもの: テキストコマンド（`R` `S` `HOME` …）と `STAT` / `ENC` / `AUTO` 行、IDF のログ出力と Arduino core の `log_e()`（v1 env は `CORE_DEBUG_LEVEL=0`。COBS の区切り 0x00 と混ざるため）、手動の電源・ブレーキ・速度・自動移動のボタン（校正は PC からの CALIBRATE で行う。上の項）。

ホストテスト `tests/run_tests.sh` は v1 のテスト（protocol.md §7 の全ベクタの encode / decode、状態機械の各規則、模擬機構での校正の成功・失敗・中止・FAULT）も走らせる。

## 安全

自動移動は、主電源 ON（settle 後）・ブレーキ解除・手動速度 0 のときだけ始まり、STOP・非常停止・電源 OFF・ブレーキ作動・速度操作で即中止。無パルス 200 ms、速度 150 count/s 未満が 200 ms（片相断線の検出）、移動量 12000、時間 45 s で止まる。UI ループからの入力更新が 200 ms 途絶えると制御タスクが出力を 0 にする。画面と USB の非常停止はソフトのラッチで、物理の非常停止ではない。電流監視は無い。
