# SET 値の再決定とコード監査（2026-10-25）

結果は [results_20261025.md](results_20261025.md)。この値で dry / idle / run / step をすべて実施した。変更したのは step モードの `START_GRACE_MS=100`（150 ms 駆動のため）だけ。

Grok で実施した 2026-10-22 の試験は参考情報とし、SET 値はここで決め直す。
この文書の値は `tools/session.py` の `SETTINGS` と同じ。片方を変えたら両方を変える。

## 2026-10-25 時点の実機状態

- `/dev/cu.usbmodem2101` の CoreS3 は **`moter-controler` のファームウェア**で動いていた（`STAT R=1 B=0 ...` 形式の応答）。encoder-tester ではない。
- 同時刻に、そのタッチ UI が操作されていた（主電源 ON、ブレーキ入切を数回、速度 1→4→0）。操作中のボードへは書き込みも ARM もしない。
- `pio run -t upload` を 3 回試みたが、encoder-tester は起動しなかった（起動バナー無し）。bootloader と partitions は書けた表示が出ており、firmware.bin の途中で出力パイプが閉じて中断した可能性がある。次回の書き込みは出力を切らずに最後まで流す。
- encoder-tester のビルドは成功している（pioarduino 55.03.312、Arduino 3.3.12、`.pio/build/m5stack-cores3/firmware.bin`）。ホストテストは 282 件成功。

## コード監査（引き継ぎ資料 6 節）

| # | 項目 | 読んだ結果 |
| --- | --- | --- |
| 1 | 起動・リセット・FAULT 時の出力 | `setup()` で `M5.begin()` の前後に主電源 OFF、ブレーキ解除 OFF、PWM 0、DIR 正を書く。FAULT は `outputs_off()`、制御タスク停止は `outputs_safe()`。GPIO1/2 の I²C 探索は無効化。外部プルダウンの有無はコードでは分からない |
| 2 | PWM/DIR | G9 PWM、G8 DIR。LEDC 20 kHz 10 bit。`raw = percent × 1023 / 100`（切り捨て）。10% は 102、5% は 51。0% は raw 0。正方向は DIR Low。`moter-controler` の `magnitude × 1023 / 100` と同じ換算 |
| 3 | PCNT の 4 逓倍 | ch A: edge=G17 level=G18、立上り DEC・立下り INC、level Low で INVERSE。ch B: edge=G18 level=G17、立上り INC・立下り DEC、level Low で INVERSE。A 先行 1 周期 (00→10→11→01→00) で各遷移 +1、計 +4。真の 4 逓倍。内部プル無効。glitch filter 既定なし |
| 4 | PCNT 上下限 | ±20000 の watch point と `accum_count` でドライバが累積。制御タスクが 5 ms ごとに読み、uint32 差分で int64 に伸ばす。clear は初期化と GLITCH_NS 変更時だけ。共有変数は critical section |
| 5 | 各値の定義 | `docs/idle_arm_check.md` の記述とコードが一致（trial 開始＝RUNNING 入り周期の GPIO 前カウント、coast＝BRAKE_APPLY 入り時点で確定）。EVT 時刻は周期先頭 |
| 6 | 遷移とリレー順 | tick 後に主電源→ブレーキ→PWM の順で GPIO を書く。停止は PWM 0 → 速度 ≤ 閾値が STOP_CONFIRM_MS 続く → ブレーキ作動 → BRAKE_APPLY_WAIT_MS → 主電源 OFF。停止確認が STOP_TIMEOUT_MS 以内に終わらないと FAULT（全出力同時 OFF） |
| 7 | MCU 側の独立停止 | 時間（run_end_ms）、相対移動（MAX_TRAVEL_COUNTS）、USB リース、UI 更新停止、無パルス（START_GRACE_MS 後に count が開始値と**完全一致**）、過速度（RUNNING 中のみ、FAULT）。PC 無しでも止まる |
| 8 | SET | DISARMED 中のみ。範囲検査。RAM のみで再起動で消える。`CONFIG` で読み返せる。STOP_TIMEOUT_MS ≤ STOP_CONFIRM_MS は拒否 |
| 9 | ポート開放 | `ARDUINO_USB_CDC_ON_BOOT=1`、`USB_MODE=1`（USB-Serial-JTAG）。README は開くと再起動すると書く。`moter-controler` 側は今回の開放で再起動しなかった（バナー無しで応答継続） |
| 10 | 校正モード | `CAL ANGLE` / `CAL REVS`、VISUAL（基準記録・完了記録、ジョグ跨ぎ保持）は実装済み。`shaft=encoder` のときだけ PPR 表示 |

気になる点（修正はしていない）:

- 無パルス停止は `count == run_origin_count` の完全一致。ノイズで ±1 動くと働かない。
- 過速度は FAULT で全出力同時 OFF。段階的停止ではない。
- `a_level` / `b_level` は 5 ms 周期の digitalRead 1 点。波形ではない。
- 電源投入 (G2) 後に CoreS3 が落ちる既知問題（引き継ぎ 3.2）。`session.py` は 2 回目以降の `BOOT` 行を検出して試験を無効にする。

## SET 値

| キー | 値 | 根拠 |
| --- | ---: | --- |
| MAX_DUTY_PERCENT | 25 | セッション中の上限。途中で上げない。はしごは 10→15→20→25。`moter-controler` では 24V 給電・ブレーキ解除で「スムーズに回った」実績があり、ブレーキ掛かったままの「50〜60」は使わない。raw は 102/153/204/255 |
| MAX_RUN_MS | 600 | RUN は 400 ms で始める。100 ms では減速機付きモーターが動き出す前に終わる |
| MAX_TRAVEL_COUNTS | 3000 | 分解能不明なので、最初の試験で count/s が出たら `count/s × 0.4 s × 1.5` 程度に置き直す。travel 停止は通常停止で無害 |
| POWER_SETTLE_MS | 1500 | G2 は AC リレー→別体 24V 電源。前回の 1000 より長く取る。長い分は安全側 |
| BRAKE_RELEASE_WAIT_MS | 300 | 未実測。前回と同じ。短くする根拠が無い |
| BRAKE_APPLY_WAIT_MS | 300 | 同上 |
| STOP_SPEED_THRESHOLD | 20 | 0 だと 100 ms 窓で完全に 0 count/s が 200 ms 続く必要があり、ジッタで `stop_timeout` FAULT になる。20 count/s は窓あたり 2 カウント |
| STOP_CONFIRM_MS | 200 | 前回と同じ |
| STOP_TIMEOUT_MS | 2000 | 惰性が長くても FAULT にしにくい。停止確認は上の閾値で行う |
| READY_HOLD_MS | 3000 | RUN を送らなければ 3 s で通常停止 |
| START_GRACE_MS | 200 | **RUN 400 ms より短い**ので、無パルス停止が実際に働く。前回は 200 > 100 ms で働かなかった |
| MAX_COUNT_PER_S | 1000000 | FAULT 経路。モーター軸側に高分解能エンコーダがあると 10000 は正常運転で超え得る。実測後に締める |
| UI_TIMEOUT_MS | 1000 | 前回と同じ |
| LEASE_MS | 2000 | heartbeat 200 ms の 10 倍 |
| JOG_STALE_MS | 150 | 前回と同じ。USB 権限中は使わない |
| JOG_DUTY | 10 | 画面ジョグ用。今回は使わない |
| ENCODER_SIGN | 1 | 配線表の定義（B Low で A 立上り＝正）。停止後の負カウントだけで反転しない |
| GLITCH_NS | 0 | 最初はフィルタ無し。ブレーキ切替の飛びが再現したら、別セッションで 1000 ns を試す（PCNT 上限は約 12.7 µs） |

## 試験手順（`tools/session.py`）

書き込みが済み、現地の確認が取れてから。1 回のポート開放で 1 セッション。開放ごとに SET を入れ直し、読み返しが一致しないと止まる。

```sh
cd encoder-tester
# 0. 出力を切らずに書き込む。終わったら BOOT state=DISARMED を見る
/Users/takuro/.platformio/penv/bin/pio run -t upload --upload-port /dev/cu.usbmodem2101

# A. ゼロエネルギー。SET + 読み返し + DISARMED 20 s の count/a/b 基準
tools/.venv/bin/python tools/session.py --port /dev/cu.usbmodem2101 --mode dry

# B. PWM 無しの ARM 1 回。リレー切替とカウントを 5 ms TRACE で見る。2 回
tools/.venv/bin/python tools/session.py --port /dev/cu.usbmodem2101 --mode idle

# C. はしご。正逆交互で正味の移動を小さく保つ。拒否・FAULT・リセットで止まる
tools/.venv/bin/python tools/session.py --port /dev/cu.usbmodem2101 --mode run \
  --trial 10:+ --trial 10:- --trial 15:+ --trial 15:- --duration-ms 400
```

C で count が動いたら、その count/s から `--travel` を置き直し、同じ Duty で正逆 3 回ずつ繰り返して再現性と往復の残差を見る。上限 25 まで動かなければ**止めて報告する**。センサーが死んでいるのか動いていないのかは、カメラか目視が無いと区別できない。

## 分解能の出し方（現地の 1 回の測定が必要）

count/rev は、外部で測った角度と対にしたときだけ出す。

1. 出力軸（アームなど目で見える部分）にスマートフォンの傾斜計か分度器を付け、角度 θ0 を読む。
2. `session.py --mode run --trial N:+` を 1 回。終わったら角度 θ1 を読む。
3. `CAL ANGLE |θ1−θ0|` を USB で送る（最後に完了した trial の差分に対して計算される）。返る `cpr` が「観測した軸の count/rev（4 逓倍）」。
4. 逆方向でも行い、バックラッシュと読み誤差を見る。

これをしない限り、分解能は UNKNOWN のまま。PWM と count/s だけから PPR は出さない。

## 開始前に現地で確認すること（引き継ぎ 7 節 段階 A）

- 機構が支持され、可動端に余裕があり、他の軸は動けない。
- 物理の非常停止に手が届く。画面と USB の ESTOP はソフトのラッチ。
- `moter-controler` の操作が終わっていて、ボードへ書き込んでよい。
- positive（DIR Low）で機構がどちらへ動くか、目で見られる。
- 主電源 ON で CoreS3 が落ちる現象（引き継ぎ 3.2）が今日も出るか。出るなら電源経路を先に直す。
