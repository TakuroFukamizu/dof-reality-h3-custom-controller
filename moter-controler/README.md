# moter-controler

M5Stack CoreS3 SE で MD13S・主電源リレー・ブレーキリレーを操作し、フィードバックセンサーを読んで、系統ごとの原点復帰・定位置移動を行う。設計は `docs/superpowers/specs/2026-10-25-moter-controler-homing-design.md`、実機記録は [docs/acceptance_homing.md](docs/acceptance_homing.md)。

## ビルド

PATH の `pio` は使わない。

```sh
cd moter-controler
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3            # Bottom（Base15）の DC ジャックから給電する本番設定
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-usbpower   # USB のみ給電（開発用。Bottom の DC は接続しない。DIN Base 時代の Issue #1 回避用）
/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-usbpower -t upload --upload-port /dev/cu.usbmodem2101
tests/run_tests.sh   # ホストテスト（状態機械とプロファイル）
```

## 配線（Base15）

Bottom は Base15 産業用プロト基板モジュール V1.1（M5Stack K025-B）。DIN Base は使わない。Base15 に XH 8pin と XH 2pin のソケットを増設し、リレー駆動の MOSFET とエンコーダ用のレベル変換を Base15 内に収める。バッテリーは無い。給電は Base15 の DC ジャック（9〜24V → MP1584 → BUS 5V）。Base15 での電源経路は未検証。詳細は [../encoder-tester/docs/wiring.md](../encoder-tester/docs/wiring.md)。

| 信号 | 定数 | GPIO | 接続 |
| --- | --- | --- | --- |
| MD13S PWM | `PIN_MD13S_PWM` | G9 | Grove Port.B 黄（Base15 上。DIN Base と同じ配置） |
| MD13S DIR | `PIN_MD13S_DIR` | G8 | Grove Port.B 白 |
| エンコーダ A（センサー白） | `PIN_ENC_A` | G17 | XH 8pin #2 → Base15 内レベル変換 |
| エンコーダ B（センサー緑） | `PIN_ENC_B` | G18 | XH 8pin #3 → Base15 内レベル変換 |
| ブレーキ解除リレー IN | `PIN_MOTOR_BRAKE` | G13 | XH 8pin #5 → Base15 内 MOSFET（回路構成は未定）。GPIO は HIGH で解除指令（`BRAKE_ON_LEVEL = 1`）。MOSFET が反転段なら `BRAKE_ON_LEVEL` だけを 0 にする。主電源の `RELAY_ON_LEVEL` は触らない |
| Protection Board VIOUT | `PIN_VIOUT` | G10 | XH 8pin #6 → Base15 内で分圧。ADC 読み取りは未実装 |
| 主電源リレー IN | `PIN_POWER_RELAY` | G2 | XH 2pin #2。GPIO 直結（駆動素子 Q1 は電源基板側。種別は未確認）。HIGH で給電（`RELAY_ON_LEVEL = 1`） |

G13 は CoreS3 SE 内部で I2S_DOUT（スピーカーアンプ AW88298）と共用。`internal_spk = false` と `external_speaker_value = 0` を保つ（`src/main.cpp` の `setup()`）。リセット中の浮きでブレーキ解除リレーが入らないよう、リレー IN がオフ側に保たれるプル抵抗を外付けする（非反転段なら G13 側にプルダウン。向きは MOSFET の回路構成が決まってから確定）。

## 手動速度の守り

手動速度（±1 / ±10、`S`）の出力中、|速度| が 7 % 以上のとき、カウントが 200 ms 変わらなければ「無パルス停止」、|速度| が 10 % 以上で 200 ms 窓の速度が 150 count/s 未満なら「遅すぎ停止」として出力を 0 にし、速度指令を 0 に戻す（画面に通知、シリアルに `WARN manual_stall` / `WARN manual_slow`）。センサーのケーブルが抜けた状態で動かし続けないための守り。7 % に達してから 300 ms は猶予。7 % 未満は機構が動かないことがある（7 % で動くことを実測）ので監視しない。

## 画面

上から: 系統 `#n`（タップで切替。電源 OFF 時のみ）と状態 / 電源・ブレーキ（解除/作動） / センサー行（原点基準のカウント、count/s、A/B、原点・校正の有無） / 速度 −10 −1 [n] +1 +10 / 自動移動（下限へ・上限へ・中立へ・校正。#3 は右端へ・左端へ） / 停止・非常停止 / フッター（DIR、ピンのレベル、行程・中立・符号）。

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

`?` は `STAT` 行と `ENC` 行を出す。自動移動中は 500 ms ごとに `ENC`、状態が変わると `AUTO <kind> <phase> <reason> pos=`。校正完了で `AUTO saved ...`。要求が拒否されると `ERR auto_<reason>`。

## 座標と保存値

- P 端 = 原点 0。N 端へ向かって増える。`ENCODER_SIGN=+1` で速度 − のときカウント増。系統ごとに校正時に符号を判定して保存する。
- NVS `h3`: `motor`、`m{n}_stroke`、`m{n}_neutral`、`m{n}_cal`、`m{n}_sign`。
- 2026-10-25 時点: #3 校正済み（stroke 6160、neutral 3080、sign +1）。#1・#2 は未校正。
- 2026-10-26 のレビュー修正（未書き込み・実機未確認）: 自動移動中の速度操作は中止（PWM 0）に統一、中立の再始動時に速度窓を取り直す、PCNT リミット跨ぎの読み直し、入力の鮮度監視、校正中は保存ストロークを信じない。

## 安全

自動移動は、主電源 ON（settle 後）・ブレーキ解除・手動速度 0 のときだけ始まり、STOP・非常停止・電源 OFF・ブレーキ作動・速度操作で即中止。無パルス 200 ms、速度 150 count/s 未満が 200 ms（片相断線の検出）、移動量 12000、時間 45 s で止まる。UI ループからの入力更新が 200 ms 途絶えると制御タスクが出力を 0 にする。画面と USB の非常停止はソフトのラッチで、物理の非常停止ではない。電流監視は無い。
