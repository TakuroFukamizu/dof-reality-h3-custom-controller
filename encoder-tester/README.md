# encoder-tester

M5Stack CoreS3 SE で、DOF Reality H3 の 1 軸を有限時間だけ動かし、フィードバックセンサーのカウントを同時に記録する。分解能は、外部の角度か回転数を別に入れたときだけ計算する。PWM とカウントだけでは PPR にしない。

既存の `moter-controler` とは別フォルダ。あちらは変更しない。

## いまの状態

分けて読む。

| 項目 | 状態 |
| --- | --- |
| ソースとホストテスト | このフォルダにある。ホストテストは 282 件成功 |
| ファームウェアのビルド | pioarduino 55.03.312 / Arduino 3.3.12 で成功。`m5stack-cores3`（Bottom の DC から給電する設定）と `m5stack-cores3-usbpower`（USB のみ）の 2 環境 |
| 実機への書き込み | 2026-10-25 に `m5stack-cores3-usbpower` を書き込み済み |
| 実機での駆動 | 2026-10-25 に 10 % / 15 % で正逆駆動。再現性 1 % 以内（DIN Base + Grove 配線。ブレーキは当時 G1） |
| カウントの確認 | 直交 A/B の 4 逓倍計数を確認 |
| 分解能 | **モーター軸 400 count/rev（4 逓倍、100 PPR 相当）**。減速比とシート角度への換算は未測定。詳細は [docs/results_20261025.md](docs/results_20261025.md) |

電流監視は接続していない。画面の ESTOP と USB の `ESTOP` は、出力指令を落とすソフトのラッチであり、物理の非常停止ではない。G2 は 3 軸共通の AC 給電になり得る。CoreS3 は 1 台だけにする。

2026-10-31 に Bottom を DIN Base から Base15 産業用プロト基板モジュール V1.1 に変更し、ブレーキ解除リレーを G1 から G13 に移した（`include/config.h`、[docs/wiring.md](docs/wiring.md)）。Base15 構成での実機確認はまだ。

## ビルド

PATH 上の `pio` は使わない。この環境ではライブラリ解決に失敗する。次の PlatformIO を使う。

リポジトリの直下から:

```sh
cd encoder-tester
/Users/takuro/.platformio/penv/bin/pio run
```

`encoder-tester` の中から:

```sh
/Users/takuro/.platformio/penv/bin/pio run
```

初回は pioarduino 55.03.312 の取得がある。書き込みは、操作者が明示したときだけ、次の形で行う。

```sh
/Users/takuro/.platformio/penv/bin/pio run -t upload --upload-port PORT
```

`PORT` は `pio device list` で見る。この README を書いた作業では書き込んでいない。

ホストの状態機械テスト:

```sh
encoder-tester/tests/run_tests.sh
```

リポジトリ直下にいるときは `encoder-tester/tests/run_tests.sh`。フォルダの中にいるときは `tests/run_tests.sh`。

## 起動

起動直後は DISARMED。USB を挿し直しても、設定の確認と ARM が無ければ駆動しない。必須の `SET` が欠けると、画面と USB に理由を出して ARM を拒否する。欠ける値を、こちらで安全そうな数にして埋めない。

起動時に、主電源はオフ、ブレーキ解除はオフ、PWM は 0。これは既知の極性（HIGH でリレーが入る）に沿った初期出力であり、センサーで電圧を確認したという意味ではない。

USB シリアルを開くと、CoreS3 は `USB_UART_CHIP_RESET` で再起動し、画面が一度消える。収集ツールも同じことをする。見ているだけのためにポートを開かない。

## 画面

320×240。ページは DRV / ENC / VIS / CFG。下に ESTOP と STOP が常にある。状態名は英語のまま。

- DRV: ARM、押しているあいだの JOG+ / JOG-、POWER ON CMD / BRAKE RELEASE CMD、要求と制限後の Duty、生 PWM。ARM は不足設定があると出さない。
- ENC: 表示カウント、A/B のレベル、ZERO DISPLAY、`CURRENT MONITOR: NOT CONNECTED`。PPR は外部角度まで UNKNOWN。
- VIS: 基準を記録、正ジョグ、逆ジョグ、停止、1回転完了を記録、破棄。自動で 1 回転するモードではない。
- CFG: 不足している SET、ARM を止める理由、ピン、警告。

ZERO DISPLAY は表示の原点だけを変える。安全側の基準カウントは動かない。画面に数値キーは無い。設定は USB の `SET`。

## USB コマンド

1 行 1 コマンド。数値の例は書式であり、推奨する運転値ではない。`HELP` でも同じ一覧が出る。

```text
STATUS
ARM
DISARM
POWER ON
POWER OFF
BRAKE RELEASE
BRAKE APPLY
RUN duty_pct=VALUE direction=positive|negative duration_ms=VALUE
STOP
ESTOP
FAULT RESET
ZERO DISPLAY
MARK START
MARK END
VISUAL CONFIG shaft=output reference=VALUE direction=positive revolutions=VALUE max_drive_ms=VALUE max_travel_counts=VALUE
VISUAL REFERENCE
VISUAL FINISH
VISUAL CANCEL
VISUAL RESULT
CAL ANGLE VALUE
CAL REVS VALUE
STREAM ON
STREAM OFF
HEARTBEAT
AUTHORITY LOCAL
AUTHORITY USB
CONFIG
SET KEY VALUE
```

対応:

- `POWER ON` は、DISARMED のときだけ ARM と同じ準備列に入る。
- `POWER OFF` と `DISARM` と `BRAKE APPLY` は停止の列。ブレーキをすでに解除していれば、PWM 0、停止確認、ブレーキ作動、主電源オフの順。PowerWait の途中なら主電源を落とす。
- `BRAKE RELEASE` は GPIO を直接いじらない。すでに解除指令中なら `OK already`。それ以外は `ERR brake_sequence`。
- `RUN` は READY かつ USB 権限のときだけ。範囲外は拒否し、黙って丸めない。
- 画面のジョグは LOCAL 権限のとき、押しているあいだだけ。運転中の逆方向は停止であり、DIR は反転しない。
- `AUTHORITY USB` は `LEASE_MS` が入っているときだけ。`HEARTBEAT` で期限を延ばす。ケーブルが刺さっているだけでは期限にならない。
- 権限が USB のとき、画面の ARM とジョグは無視する。STOP、ESTOP、FAULT RESET は画面からでも効く。
- 停止は、時間、相対移動量、画面の更新止まり、USB の期限の、先に来たもので始まる。カウント上限だけには頼らない。
- シャフト名がちょうど `encoder` のときだけ、PPR を「その軸の count/rev ÷ 4」と表示する。公称値ではない。他の軸名、または軸名の無い試験では PPR は UNKNOWN。
- 校正結果は制御ゲインに書き戻さない。

Duty の単位は、10 bit フルスケール 1023 に対する百分率。100% は 1023、50% は 511、1% は 10。255 で割らない。正方向は DIR Low。PWM は 20 kHz。

## SET

DISARMED のときだけ受け付ける。`-1` は未設定。下の数値は受け付ける範囲であり、最初に入れる値ではない。

| キー | 範囲 | 単位 | ARM に必要 |
| --- | --- | --- | --- |
| MAX_DUTY_PERCENT | 1..100 | 1023 に対する % | 要 |
| MAX_RUN_MS | 1..3600000 | ms | 要 |
| MAX_TRAVEL_COUNTS | 1..1000000000 | count | 要 |
| POWER_SETTLE_MS | 0..600000 | ms。未測定。出荷済み制御の 100 ms 待ちとは別 | 要 |
| BRAKE_RELEASE_WAIT_MS | 0..600000 | ms | 要 |
| BRAKE_APPLY_WAIT_MS | 0..600000 | ms | 要 |
| STOP_SPEED_THRESHOLD | 0..100000000 | count/s の絶対値 | 要 |
| STOP_CONFIRM_MS | 0..600000 | ms | 要 |
| STOP_TIMEOUT_MS | 1..600000 | ms。STOP_CONFIRM_MS より大きい | 要 |
| READY_HOLD_MS | 1..3600000 | ms | 要 |
| START_GRACE_MS | 0..600000 | ms | 要 |
| MAX_COUNT_PER_S | 1..2000000000 | count/s | 要 |
| UI_TIMEOUT_MS | 1..60000 | ms | 要 |
| LEASE_MS | 1..60000 | ms。heartbeat より長く | 要 |
| JOG_STALE_MS | 1..5000 | ms | 要 |
| JOG_DUTY | 1..100 | 1023 に対する %。これだけでは ARM できない | 不要 |
| ENCODER_SIGN | 1 または -1 | 表示と記録の符号。変えると目視セッションは無効 | 不要 |
| GLITCH_NS | 0..1000000 | ns。0 はフィルタなし。変えると PCNT を初期化し直す | 不要 |

`STOP_TIMEOUT_MS` が確認時間以下なら、その書き込みだけ拒否し、前の値は残る。

## 2026-10-25 の状態

- ボードは `moter-controler` のファームで動いていた。encoder-tester の書き込みは成立していない。SET 値の再決定とコード監査は [docs/set_proposal.md](docs/set_proposal.md)。
- 新しいホストツール `tools/session.py`。`--mode dry` は SET と読み返しと DISARMED 基準だけ（ARM しない）。`--mode idle` は RUN 無しの ARM 1 回。`--mode run` は `--trial 10:+ --trial 10:-` のはしご。`moter-controler` の応答、途中のリセット、読み返し不一致で止まる。
- `tools/sim_firmware.py` は pty 上のプロトコル模擬。`session.py` のオフライン確認用で、実機の代わりではない。

## ログとホストツール

CSV は 50 Hz 前後、画面は 66 ms 前後。列は次のとおり。

```text
seq,t_us,trial_id,axis,state,power_on_cmd,power_gpio_level,brake_release_cmd,brake_gpio_level,requested_command,requested_unit,applied_duty_pct,pwm_raw,pwm_full_scale,pwm_hz,dir_level,count_total_x4,count_trial_x4,velocity_count_s,a_level,b_level,elapsed_ms,stop_reason,log_drop_count
```

`axis` は `1`。GPIO 列はピンのレベルであり、リレーが閉じた証拠ではない。`count_total_x4` は符号付き累積で、ZERO DISPLAY 後の表示値ではない。イベントは `EVT` 行。PWM を切ったあとの惰性は、試験完了の coast で分ける。

依存:

```sh
python3 -m venv .venv
.venv/bin/pip install -r tools/requirements.txt
```

ポート一覧、収集、グラフ。収集はポートを開くので、ボードが再起動する。この作業ではボードに対して実行していない。

```sh
.venv/bin/python tools/capture_drive_encoder.py --list
.venv/bin/python tools/capture_drive_encoder.py --port PORT
.venv/bin/python tools/analyze_trials.py logs/capture_YYYYMMDD_HHMMSS.csv
```

収集の既定は `logs/capture_時刻.csv` と、同じ名前の `.log`。すでにあるパスは上書きしない。数字で始まる行だけを CSV に入れ、`EVT` や `OK` / `ERR` は横のログに入れる。heartbeat は 200 ms。`LEASE_MS` はそれより長くする。`--no-heartbeat` で止める。Ctrl+C は `STOP` を送って終了する。`ESTOP` ではない。PC の STOP が届かなくても、MCU 側の期限で停止の列に入る。

グラフの題は、これが PPR 測定ではない、と書く。PWM と count/s を PPR と呼ばない。

## 配線と試験

ピンと禁止事項は [docs/wiring.md](docs/wiring.md)。実機で見る順番は [docs/test_checklist.md](docs/test_checklist.md)。チェックリストは未実施の一覧である。

手では回さない。目視 1 回転は、目印を人が見て、停止とカウント安定のときに基準と完了を記録する。記録しただけではモーターは動かない。
