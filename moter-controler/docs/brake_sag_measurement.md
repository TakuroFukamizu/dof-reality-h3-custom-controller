# ブレーキの効き始めと、ブレーキ解除時の沈みの計測（計測専用）

r10 の保護の監視（PR #50、branch `hw-r10-integrated-test`）は、見えない走行の上限を **測っていない仮値** で見積もっている:
受動的な沈み（ブレーキ解除・PWM 0）200 count/s、ブレーキの作動 150 ms、解除待ち 300 ms、保持の包絡線 150 count/s。
これを実測で置き換えるために、ブレーキ指令・PWM・エンコーダ・時刻を揃えて記録する道具を用意した。機能ではなく計測の道具。

- ファーム: `env:m5stack-cores3-v1-diag`（`-DV1_DIAG_BRAKE_TEST=1`）。通常の v1 に DIAG_START / DIAG_READ を足しただけ。
  量産の組み立て（`env:m5stack-cores3-v1`）には一切入らない（受けても未知の種別として rx_error になる。ホストテストで確認）。
  画面の起動異常の行に `DIAG` と出続ける。**計測が終わったら `env:m5stack-cores3-v1` を焼き戻す。**
- 道具: `moter-controler/tools/diag_brake/diag_brake.py`（サーバーを止めて MCU のポートを直接開く）
- 状態機械: `include/v1_diag_brake.h`（純粋）、受理条件・主電源・異常: `include/v1_axis.h`（`V1_DIAG_BRAKE_TEST` の節）
- 記録: **1 ms ごと**（記録中だけ制御タスクが 5 ms の tick の間を 1 ms で刻む。受信・tick・異常の監視は 5 ms のまま）。
  1 点 = t_us（ブレーキ解除指令 = 0）・actual_count・pwm_permille（カウント増が正）・ブレーキ解除指令・主電源指令・VIOUT の ADC 生値（換算なし）。
  RAM に最大 1024 点。終わったら DIAG_RESULT（要約）を 1 回送り、PC が DIAG_READ で 16 点ずつ読む。

## 1. モード

| モード | 流れ | 止める条件（ブレーキ作動 + PWM 0 を同じ周期で指令） | 後の記録 |
| --- | --- | --- | --- |
| `sag` | 主電源 ON（ARMING と同じ 2.5 s 待ち、ブレーキ作動のまま）→ **PWM 0 のままブレーキ解除（t = 0）** | 経過 ≥ `--t-max`（≤ 500 ms）／ \|変位\| ≥ `--d-max`（≤ 200 count）／ 予測位置（いまの速度 × 200 ms 先）か現在位置がソフトリミットの 100 count 内側を越える | 400 ms |
| `engage` | 同じ待ち → ブレーキ解除（t = 0）→ `--delay`（≤ 200 ms、PWM 0）→ `--dir` に `--duty`（100〜250 permille）で走る | 走行の向きの速度（20 ms 窓）≥ `--v-target`（≤ 3000）／ 走行 ≥ `--d-run`（≤ 600 count）／ 走行 300 ms ／ 端の余裕（同上）／ 逆向きに 20 count（荷重に負けた）／ CONFIG の overspeed ／ 待ちの間に 100 count 沈んだ | 500 ms |
| （`POWER_HOLD`） | 道具が電源担当軸（#1）に自動で送る。主電源だけ入れ、ブレーキ作動・PWM 0 のまま。最長 `--power-hold-ms`、PC から 2 s 何も来なければ切る | — | — |

「解除の時刻」は `sag` の t = 0 そのもの（記録の最初の点。`brake_release = 1`）。ブレーキが離れて動き始めるまでの遅れ（解除待ちの実測）は記録から読む（§5）。
上限を超える値は **拒否**（INVALID_PARAM）し、丸めない。

開始の条件（どれかが欠ければ拒否、何も動かない）: DISARMED（ブレーキ作動・PWM 0）・CONFIG 済み・POSITION_REFERENCED・ソフトリミット内・
画面の非常停止がラッチされていない・前の計測の記録が終わっている・開始位置に余裕がある
（`sag` は両側に `--d-max`、`engage` は走る向きに `--d-run` + 300、反対側に 200。ソフトリミットの 100 count 内側から測る）。

打ち切り: 画面の非常停止（LOCAL_ESTOP）・HOST_FAULT・制御周期の逸脱（CONTROL_OVERRUN）は、その周期にブレーキ作動・PWM 0・主電源 OFF（既存の FAULT）。
計測の記録は作動後の分まで続け、DIAG_RESULT(FAULT) で終わる（FAULT のまま。FAULT_RESET は通常どおり）。
`engage` で走らせて 150 ms カウントが 3 以上動かなければ **ENCODER_FAULT をラッチ** する（無パルス）。中止（`DIAG_START(CANCEL)`・DISARM・道具の Ctrl-C）は
その場でブレーキ作動・PWM 0 にし、作動後の記録を経て CANCELLED で終わる。

### 基準（POSITION_REFERENCED）の扱い

エンコーダは計測の間も数え続けるので、**計測そのものでは基準を消さない**（DISARMED でブレーキを外して手で動かしたときと同じ扱い）。
走らせてカウントが動かなかった（無パルス）ときだけ、既存の意味どおり ENCODER_FAULT → FAULT_RESET で基準が消える
（DIAG_RESULT の `referenced_after = 0` で知らせる。サーバーから校正し直す）。LOCAL_ESTOP などそれ以外の FAULT では基準は残る（既存と同じ）。
逆向きに動いた（`REVERSE`）ときも基準は残す（荷重に負けた動きはエンコーダの故障ではない。数えた量は正しい）。

## 2. 安全

- **最初は座席を空にする**（無負荷の全項目が終わるまでバラストも載せない）。人が乗った計測はしない。
- 物理の非常停止（24 V の遮断）を **押せる位置に操作者が付く**。画面の非常停止も効く（最長 5 ms でブレーキ作動・PWM 0）。
- **1 回に 1 軸だけ**。他の軸は DISARMED（ブレーキ作動・PWM 0）のまま。#2・#3 を測るときの #1 は POWER_HOLD（ブレーキ作動・PWM 0・主電源だけ ON）。
- ファームの上限: SAG はブレーキ解除 500 ms・変位 200 count まで、ENGAGE は Duty 25 %・走行 600 count・300 ms まで。
  さらにソフトリミットの 100 count 内側を、いまの速度で 200 ms 先まで見て止める（ブレーキが仮値どおり 150 ms で効く前提の余裕。§5 の結果で見直す）。
- 小さい値から始める（§4 の順）。各段の結果（沈んだ量・止まるまでの滑り）を見て、次の段へ進むかを操作者が決める。
- サーバーと同時に使わない（ポートの取り合い）。道具は 1 回ごとにポートを開いて閉じる。
- 道具が途中で落ちても: ファームは記録が終われば自分で DISARMED（主電源 OFF）に戻る。POWER_HOLD は受信が 2 s 途絶えたら主電源を切る。

## 3. 準備

1. 3 台に `env:m5stack-cores3-v1-diag` を焼く（メインのセッションが操作者の OK の後に行う。各ポートの USB Serial Number を `ioreg` で確かめてから）。
2. サーバー（ふだんの設定 `h3-bringup-01-live-20261102.toml`）を起動し、いつもどおり 3 軸を校正する（**基準は RAM だけ。電源を入れ直した・焼き直したら校正し直す**）。
   サーバーの試験ページで計測する位置（§4）へ動かし、DISARM して **サーバーを止める**。MCU は DISARMED・基準ありのまま残る。
3. 個体とポートの対応を確かめる: `ioreg -p IOUSB -l -w0 | grep -E "USB Serial Number|USB Product Name"`。
   個体の MAC: #1 = `1C:DB:D4:BA:8D:58`、#2 = `68:EE:8F:D8:42:F0`、#3 = `30:ED:A0:0B:20:40`（設定の `device_id` の下 12 桁）。
4. 設定ファイル `h3-bringup-01-live-20261102.toml` は main にあり、このブランチ（hw-r10a-test から分岐）には無い。
   main のチェックアウトから動かすか、`--config` に main のチェックアウトの絶対パスを渡す。
5. 動かさずに読む（道具の動作確認）:

```sh
PY=~/.platformio/penv/bin/python   # pyserial と tomllib が入っている（PATH の python3 には pyserial が無い）
T=moter-controler/tools/diag_brake/diag_brake.py
CFG=control-server/config/h3-bringup-01-live-20261102.toml
$PY $T --port /dev/cu.usbmodem12501 --mac 1C:DB:D4:BA:8D:58 --axis 1 --config $CFG info
```

道具は、開く前に USB Serial Number を `--mac` と照合し（違えば開かない）、HELLO で `device_id`・選択中の系統（`--axis`）・計測専用ファームであること、
STATUS で DISARMED・基準ありを確かめてから、設定の値（ソフトリミット・保護値・電源担当）で CONFIG を送る。
CONFIG の `calibration_hash` は 0 で送るので、後でサーバーを起動すると CONFIG を送り直す（サーバーの通常の動き。問題はない）。

FAULT から戻す: サーバーを HOLDING のまま止めると COMMAND_TIMEOUT（lease 400 ms）で FAULT になり、道具は DISARMED でないので拒否する。
サーバーを起動し直さずに `fault-reset` で解除できる（動かさない。画面の非常停止がラッチ中ならファームが拒否する）。
ENCODER_FAULT の解除は基準を消すので、その後はサーバーから校正し直す。サーバーは **DISARM してから** 止める。

```sh
$PY $T --port /dev/cu.usbmodem12501 --mac 1C:DB:D4:BA:8D:58 --axis 1 --config $CFG fault-reset
```

**電源を入れた後（焼いた後・校正の後も）の最初の 1 回は `sag` にする。** ブレーキ解除で実際に動く（`release_to_motion_ms` が出る）ことを見てから
`engage` に進む。主電源の待ちは ARMING と同じ 2.5 s で、校正（CAL_POWER_SETTLE_MS = 5 s。10/31 に投入 4 s 後は動かなかった記録）より短い。
24 V が上がる前に `engage` が走ると無パルスで ENCODER_FAULT になり、基準を失う（#1 単独のとき。#2・#3 は #1 の POWER_HOLD が先に入っている）。

## 4. 計測の計画

位置: 中立（N0 ≈ 2778 / 2753 / 3103）、中立 +1500（P15）、中立 −1500（M15）。リフト（#1・#2）は全部、ヨー（#3）は N0 だけでよい（重力の荷重が無い）。
位置の移動はサーバーの試験ページで行い、DISARM → サーバー停止 → 計測。ENGAGE を続けると位置がずれるので、5 回ごとに `info` で位置を見て、
余裕が足りなくなったら（道具が INVALID_PARAM で拒否する）反対向きの ENGAGE で戻すか、サーバーで戻す。

各条件 **5 回以上**。荷重は「無負荷 → バラスト」の順で、同じ表をもう一度回す（バラストの質量・載せ方を記録に書く）。

| 段 | モード | パラメータ | 進む条件（無負荷） |
| --- | --- | --- | --- |
| S1 | sag | `--t-max 150 --d-max 60` | 5 回とも d_max に届かない、作動後の滑り ≤ 20 count |
| S2 | sag | `--t-max 300 --d-max 100` | 同上 |
| S3 | sag | `--t-max 500 --d-max 200` | — |
| E1 | engage | `--duty 120 --v-target 300 --d-run 300 --delay 50`、`--dir +1` と `-1` | 作動後の滑り ≤ 60 count |
| E2 | engage | `--duty 150 --v-target 600 --d-run 300 --delay 50`、両向き | 同上 |
| E3 | engage | `--duty 200 --v-target 1000 --d-run 400 --delay 50`、両向き | 同上 |
| E4 | engage | `--duty 250 --v-target 1200 --d-run 500 --delay 50`、両向き（1200 = いまの max_vel） | — |

コマンド（#1 は電源担当なので単独。ラベルは `位置_荷重_段` にする）:

```sh
$PY $T --port /dev/cu.usbmodem12501 --mac 1C:DB:D4:BA:8D:58 --axis 1 --config $CFG --label N0_empty_S1 \
    sag --t-max 150 --d-max 60
$PY $T --port /dev/cu.usbmodem12501 --mac 1C:DB:D4:BA:8D:58 --axis 1 --config $CFG --label N0_empty_E2 \
    engage --dir 1 --duty 150 --v-target 600 --d-run 300 --delay 50
```

#2・#3 は電源担当の #1 も指定する（道具が #1 に POWER_HOLD を送り、終わったら中止を送る。#1 はブレーキ作動・PWM 0 のまま）:

```sh
$PY $T --port /dev/cu.usbmodem12101 --mac 68:EE:8F:D8:42:F0 --axis 2 --config $CFG \
    --power-port /dev/cu.usbmodem12501 --power-mac 1C:DB:D4:BA:8D:58 --label N0_empty_S1 \
    sag --t-max 150 --d-max 60
$PY $T --port /dev/cu.usbmodem1101 --mac 30:ED:A0:0B:20:40 --axis 3 --config $CFG \
    --power-port /dev/cu.usbmodem12501 --power-mac 1C:DB:D4:BA:8D:58 --label N0_empty_E2 \
    engage --dir -1 --duty 150 --v-target 600 --d-run 300 --delay 50
```

（ポート名は差し直しで変わる。毎回 `ioreg` で確かめる。）1 回 = 主電源待ち 2.5 s + 計測 1 s 弱 + 読み出し約 1 s。

出力: `moter-controler/docs/data/brake_sag/<日時>_ax<n>_<mode>_<label>.{csv,json,log}`。
標準出力に 1 行の要約（結果・止めた理由・作動時の変位と速度・作動後の最終変位・静止までの時間・解除中の最悪 100 ms の速さ・記録の間隔の最大・基準）。
終了コード: 0 = OK、2 = FAULT / CANCELLED、1 = 拒否・通信の失敗（何も動いていないか、動いた後に中止を送った）。

## 5. 保守的な境界の出し方（平均ではなく最悪側）

各条件（軸 × 位置 × 荷重 × 向き × 速度）で n ≥ 5。**最大値（と最小値・幅）を n と分解能（1 ms・1 count）付きで書く**。平均は参考。

| 置き換える仮値 | 読む値（JSON） | 境界 |
| --- | --- | --- |
| 受動的な沈み 200 count/s | `host_metrics.released_max_rate_100ms_count_s`（SAG。解除中の最悪の 100 ms 窓） | 全位置・全荷重の最大。さらに解除が長いほど速くなるか（S1→S3）を見る。加速しているなら速度の境界ではなく「t 秒で沈む距離」の最大で書く |
| 解除待ち 300 ms | `host_metrics.release_to_motion_ms`（SAG。解除指令から \|変位\| ≥ 2 まで） | 最大。荷重が軽いと動かないこともある（そのときは「その条件では離れたか分からない」と書き、ENGAGE の `t_drive` 後の動き出しで補う） |
| ブレーキの作動 150 ms | `firmware_result.t_settle_us`（作動指令から 20 ms の間 ±1 count に収まるまで）、`host_metrics.engage_to_half_speed_ms` | 速度ごとの最大。`t_settle` は ±1 count/20 ms（≈ 50 count/s）で判定するので、遅い沈みは早めに「静止」と出うる（SAG では `engage_to_still_ms` と最終変位を併せて見る） |
| 作動中の滑り（見えない走行の予算） | `host_metrics.stop_disp_after_engage`（= final_disp − disp_at_engage）と `v_at_engage_count_s` | 速度に対する上側の包絡線 d ≤ a + b·\|v\|（最悪点を全部上から押さえる直線）。E4 より速い速度へは外挿しない |
| 保持の包絡線 150 count/s | バラストでの SAG の最悪 100 ms 窓、ENGAGE の作動後の最大速度 | バラスト条件の最大 |

まとめ方: 条件ごとに `max / min / n` の表（CSV の要約）を作り、ファームの仮値と並べる。仮値より悪い値が 1 回でも出たら、その値を新しい境界の候補にする
（余裕の掛け方は r10 の側で決める。ここでは測った最悪値を渡す）。

## 6. 既知の制約・未確認

- 主電源の待ち 2.5 s（ARMING と同じ）は校正の 5 s より短い。24 V が上がり切る前に `engage` が走ると無パルス → ENCODER_FAULT（§3 の「最初は sag」）。
- 実機では未実行（2026-11-02 時点。ホストテストと、V1Axis を pty の向こうで動かした模擬で道具の一連を確認しただけ）。
- 1 ms 記録の間隔は実機で未確認（DIAG_RESULT の `max_sample_gap_us` に出る。1000 から大きく外れるなら制御タスクの負荷を疑う）。
- VIOUT の分圧と ADC の換算は未確認（生値を残すだけ。11 dB 減衰 = 約 3.1 V まで）。
- 端の判定は CONFIG のソフトリミット基準（機械端より 300 count 内側）＋ 100 count。先読み 200 ms はブレーキの仮値 150 ms に頼っている。
  S1 / E1 の結果で作動後の滑りが大きければ、上の段に進む前に余裕を見直す。
- ENGAGE の無パルス（150 ms）は ENCODER_FAULT をラッチして基準を消す（既存の意味）。Duty 下限 100 permille は #2 の持ち上げ側が 9 % で動き出した実測から。
- #2・#3 は #1 の POWER_HOLD に頼る（主電源リレーは #1 だけ）。#1 の USB が途切れると 2 s で 24 V が落ちる（計測は中途で止まり、ブレーキは作動側）。
