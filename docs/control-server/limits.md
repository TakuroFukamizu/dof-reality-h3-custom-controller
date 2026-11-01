# 動きの制限値（limits）

状態: **実装済み・模擬軸で試験済み。数値はどれも実機未検証の見積もり。** 設計は [docs/superpowers/specs/2026-11-01-motion-limits-design.md](../superpowers/specs/2026-11-01-motion-limits-design.md)。

座席の動きを制限する値は 2 段だけにしている。「いまどこで何に制限されているか」をこの 2 段で追えるようにするため、それ以外の段は作らない。

```text
ゲーム / 管理者 ─(要求)─▶ PC の現在値（3 軸共通。初期値は [limits]）─▶ clamp(上限) ─▶ 有効値
                                                                  │
   上限 = ファームの絶対上限（LIMITS で申告）と overspeed_count_s / 1.5 と gain 1.0
                                                                  │
   有効値 → 毎フレームの SET_POSITION（速度・加速度）、SET_LIMITS（Duty 上限など。変わったときだけ）
                                                                  │
   ファーム: 受けた値を絶対上限で黙ってクランプ（拒否しない）
```

## 1. 2 段の制限

| 段 | どこにあるか | 変え方 | 内容 |
| --- | --- | --- | --- |
| 1. 絶対上限 | ファーム `moter-controler/include/config.h` の定数 `V1_ABS_*`。PC へは HELLO の後の LIMITS で申告される（status の `axes[].fw_limits.abs_*`） | 焼き直し | Duty 60 %、速度 6000 count/s、加速度 40000 count/s² |
| 2. 現在値 | PC の設定ファイル `[limits]`（初期値）。起動後は 3 軸共通の 1 組（status の `motion_limits`） | 設定ファイル、`WS /v1/motion` の `limits`、`POST /v1/limits` | §2 の表 |

- 段 2 で段 1 を超える要求を受けても拒否はしない。段 1 でクランプして、有効値と理由（`ceiling_clamped`）を返す。gain の `gain_limited` と同じ流儀。
- CONFIG（ARM 時に MCU へ送る）の速度・加速度には段 1 の値を入れる。実際の上限は毎フレームの SET_POSITION が運ぶ有効値で決まる。CONFIG は「もう 1 つの上限」ではない。
- ソフトリミット（`soft_min/max_count`、`decel_zone_count`）と校正 3 点は幾何で、制限値ではない。`[[axis]]` に置いたままで、実行時には変えない。
- `POST /v1/session`（新しいゲームの接続）で現在値は `[limits]` の初期値に戻る。ゲームが上げた値は、別のゲームや再接続に引き継がれない。

## 2. `[limits]` の項目と根拠

`control-server/config/*.toml` の `[limits]`（`schema_version = 2`）。3 軸共通。下の初期値は `example.toml` / `sim.toml` のもの（上げ切った先の目標）。`bringup.toml` は §4 の低い値で置いてある。

### 2.1 実行時に変えられる 4 つ

| 項目 | 初期値 | 上限 | 根拠 / 注意 |
| --- | ---: | --- | --- |
| `max_gain` | 0.5 | 1.0 | 混合式 `m = 512 + gain·511·(roll ± pitch)` で、pitch と roll を同時に振り切っても 0.5 以下なら仮想 0〜1023 で飽和しない。旧 h3game の既定 0.35 は検証値ではなかった。`bringup.toml` は向きの確認用に 0.15 |
| `max_vel_count_s` | 3000 | min(絶対上限 6000, `overspeed_count_s` / 1.5) | 7.5 回転/s。ストローク 5500 count を加減速込みで約 2.3 s。実測の Duty 1 % ≈ 100〜130 count/s（2026-10-31）から、フィードフォワードだけで約 30 % |
| `max_acc_count_s2` | 15000 | 絶対上限 40000 | 0 → 3000 count/s に 0.2 s。旧値 2000 では 1.5 s かかり、30 Hz の目標変化に追いつけなかった |
| `duty_max_percent` | 40 | 絶対上限 60 | 外挿で約 4000〜5000 count/s。max_vel 3000 に対して FF + P の余裕を残す。旧ファームは 20 % 直書き |

### 2.2 初期値だけ（SET_LIMITS でファームへ配る。実行時には変えない）

| 項目 | 初期値 | 旧（ファーム直書き） | 役割 |
| --- | ---: | ---: | --- |
| `duty_min_percent` | 3 | 7 | 位置ループが出す最小の非 0 Duty。7 % は無負荷で約 700 count/s 出るので、それより遅い目標速度は「7 % で動く / 0 で止まる」の往復になり低速が段付きになる（FF + P の下限が限界サイクルを作る）。3 % はこれを小さくする**仮説**。自動移動（端当て・中立）の 7 %（`NEUTRAL_DUTY_MIN`）は別の定数で、変えていない |
| `kp_percent_per_100` | 2 | 2 | 比例ゲイン。誤差 100 count あたり 2 % |
| `ff_count_s_per_percent` | 100 | 100 | フィードフォワード。軌道速度 [count/s] ÷ これ = Duty [%]。実測の下側 |
| `deadband_count` / `deadband_out_count` | 10 / 20 | 10 / 20 | 目標が止まっているときの到達帯（入る側 / 出る側のヒステリシス） |

ファーム側の固定値との関係（Issue #15 H4 / M1 / M2、2026-11-02。いずれも実機未検証）:

- **動き出し Duty と `duty_min`**: 11-02 の実機（kp 6 / duty_min 9）で、自重のかかった軸の動き出しは約 9 %、7〜8 % では動かなかった。位置ループに積分は無いので、`duty_min` が動き出し Duty より低いと、目標が止まっているときの静的誤差（不感帯の外）は詰められない。ファームはこれを **停止ブースト**で補う: 軌道が止まっていて下限 Duty でもカウントが 200 ms（`V1_STALL_BOOST_MS`）動かなければ下限を 1 % ずつ上げ、`duty_max` で頭打ち、不感帯に入る・誤差の符号が反転する・軌道が動き出すと 0 に戻す（protocol.md §5）。`duty_min` を下げる（§4 手順 3）と、静止時の押し当ては下限から動き出し Duty まで 200 ms × 差分だけ遅れる。
- **下限の向き**（`V1_FLOOR_SIGN_GATE`、既定 1）: FF と P の合成が `duty_min` 未満で誤差と逆向きなら 0（惰行）。`duty_min`（9 % ≈ 900 count/s）が `max_vel_count_s`（bringup 600）や center の速度より高いと、軌道速度は原理的に出せず「バースト → 停止」になる。旧動作ではこのとき「反転」も出た（11-02 の正弦波で Duty が常に ±9 % だったのと整合）。反転をなくすのが gate で、バースト自体を減らすのは `duty_min` を下げるか `max_vel` を上げる。
- **`V1_STALL_DUTY`（10 %）と `duty_max`**: 無パルスの ENCODER_FAULT のしきい値は `max(10 %, duty_max × 75 %)`（bringup の duty_max 20 → 15 %、40 → 30 %）。動けない軸がどちらの異常になるかは `kp × following_error_limit_count / 100 + FF` とこのしきい値の大小で決まる: 未満なら FOLLOWING_ERROR（基準が残り、FAULT_RESET → ARM で続けられる）、以上なら ENCODER_FAULT（FAULT_RESET で基準が消え、SET_REFERENCE か校正が要る）。bringup（kp 6、FE 400、duty_max 20）は 24 % ≥ 15 % なので ENCODER_FAULT 側。**FOLLOWING_ERROR 側に置きたいなら `kp × FE / 100 < 0.75 × duty_max` にする**（PR #14 の `kp × FE / 100 ≥ duty_min` と合わせて、`duty_min ≤ kp × FE / 100 < 0.75 × duty_max`）。`duty_max_percent < 10` は受理されるが無パルス判定が決して成立しない（ファームの画面に「無パルス検出なし」）。

### 2.3 保護（CONFIG で配る。実行時には変えない）

| 項目 | 初期値 | 旧 | 根拠 |
| --- | ---: | ---: | --- |
| `following_error_limit_count` | 800 | 400 | 速度を上げると追従遅れも増える。max_vel 3000 で 100 ms 遅れると 300 count |
| `overspeed_count_s` | 9000 | 2000 | 1.5 × 絶対上限 6000。暴走検出として残す。設定の検査で `≥ 1.5 × max_vel_count_s` を要求し、実行時の max_vel も `overspeed / 1.5` で頭打ちにする（bringup.toml の記録: max_vel の 2 倍では過渡で止まった） |

### 2.4 ファームの絶対上限（`config.h`）

| 定数 | 値 | 根拠 |
| --- | ---: | --- |
| `V1_ABS_DUTY_MAX` | 60 % | MD13S と 24 V モーターの余裕側。100 % は使わない |
| `V1_ABS_MAX_VEL_COUNT_S` | 6000 | 15 回転/s。Duty 60 % の外挿（6000〜7800）より下 |
| `V1_ABS_MAX_ACC_COUNT_S2` | 40000 | 0 → 6000 に 0.15 s |

PC は LIMITS を受けるまで同じ値（`protocol::fw_absolute`）を仮の上限に使い、模擬軸もこの値を名乗る。ファームの値を変えたら PC 側の定数も合わせる。

## 3. 実行時に変える

### 3.1 ゲームから（WS `/v1/motion`、トークン不要）

```json
{"type":"limits","protocol":1,"source_session":"...","seq":1252,
 "max_gain":0.5,"max_vel_count_s":3000,"max_acc_count_s2":15000,"duty_max_percent":40}
```

- 4 項目はどれも省略可。書いた項目だけ変わる。`seq` と `source_session` の規則は pose と同じ（`seq` は受理したときだけ消費する）。**鮮度は延長しない**。
- ack: `{"type":"ack","seq":1252,"accepted":true,"effective_gain":0.5,"effective_limits":{...4 項目...},"limits":["ceiling_clamped"],"fresh":false}`。`effective_gain` は新しい gain の上限。`ceiling_clamped` はどれかが上限で切られたとき。
- 拒否: 非有限は `non_finite`、`max_gain < 0`・速度と加速度の 0 以下・`duty_max_percent` の 1〜100 の外は `out_of_range`。競合中（`axis_conflict`）は pose と同じく拒否。
- 新しい gain の上限は、いま追従中の姿勢にもその場で効く（`effective_gain` が下がり `gain_limited` が付く。鮮度は変えない）。Python クライアントは `submit_limits(...)` を pose より先に送る。
- bridge（`H3_BACKEND=server`）はゲームの POST に `"limits": {...}` があればそのまま渡す（`clients/h3game/README.md`）。

### 3.2 管理者から（REST）

```sh
curl -s $URL/v1/limits | python3 -m json.tool                          # 現在値（トークン不要・loopback だけ）
curl -sX POST -H "$AUTH" -d '{"duty_max_percent":30}' $URL/v1/limits   # 書いた項目だけ変える（Bearer、rate limit あり）
```

応答（GET / POST 共通）: `{"ok":true,"effective":{...},"requested":{...},"ceiling":{...},"defaults":{...},"limited":[...]}`。`requested` が要求値、`ceiling` がいまの上限、`effective` がその結果、`defaults` が `[limits]` の初期値、`limited` は上限で切られた項目名。不正な値は 400（`non_finite` / `out_of_range`）。

### 3.3 status で見る

- `motion_limits`: 上の GET と同じ 5 つ。
- `axes[].fw_limits`: その個体が LIMITS で返した有効値（`duty_max_percent` …）と絶対上限（`abs_duty_max_percent`、`abs_max_vel_count_s`、`abs_max_acc_count_s2`）。SET_LIMITS を受ける前は `null`。
- blockers: `firmware_limits_unsupported: axisN`（SET_LIMITS を知らない旧ファーム。焼き直すまで ARM しない）、`limits_rejected: axisN`（SET_LIMITS が拒否された。1 秒後に再送）、`limits_unconfirmed: axisN`（いまの調整値の SET_LIMITS が ACK され、かつ LIMITS が届くまで。ファーム既定の 7〜20 % や前の接続の値で ARM しないための阻害。ACK が来なければ 1 秒の期限切れの後に、ACK は来たが LIMITS が落ちたなら 1 秒後に、新しい request_id で送り直す）。
- events: `limits`（LIMITS の受信）、`ack_rejected`（SET_LIMITS の拒否）。

### 3.4 試験ページ（`GET /`）

「動きの制限」の 4 本のスライダー（上限は status の `motion_limits.ceiling` に追従）→「制限を送る」。WS の `limits` で送るので、姿勢送信（WS）を開始してから使う。「設定の初期値に戻す」は `defaults` を送る。limits の ack は姿勢の ack とは別に表示する。

## 4. 実機で上げる手順

どの値も実機未検証。以下の順で、1 軸ずつ、低い値から。前提は hardware-bringup.md §7 と同じ（座席に人がいない、可動範囲に物も人もない、物理非常停止に手が届く）。

1. **ファームを焼き直す。** SET_LIMITS に対応しないファームは `firmware_limits_unsupported` で ARM できない。3 台とも `env:m5stack-cores3-v1`。
2. **低い値から始める。** `bringup.toml` の `[limits]` は低い値で置いてある（`max_gain 0.15、max_vel 600、max_acc 2000、duty_max 20、duty_min 7、following_error 400、overspeed 2000` ＝ 旧 PR と同じ動き。`example.toml` / `sim.toml` の値が上げ切った先の目標）。ここで §7 の向きの確認と同じ Pose を流し、`following_error_count`・`velocity_count_s`・`pwm_permille` を控える。
3. **滑らかさ: `duty_min_percent` を 7 → 5 → 3。** 低速（gain 0.15、ゆっくりした pitch）で段付きが減るか、逆に停止付近でふらつかないか。ふらつくなら 5 に戻す。
4. **速度・加速度: 試験ページか `POST /v1/limits` で `max_vel` を 600 → 1200 → 2000 → 3000、`max_acc` を 2000 → 5000 → 10000 → 15000。** 実行時の `max_vel` の上限は `overspeed_count_s / 1.5` でも頭打ちになる（bringup.toml の 2000 なら 1333）ので、1200 を超える前に `overspeed_count_s` を上げて再起動する（3000 なら 4500 以上。保護なので実行時には変えない）。 各段で `following_error_count` の最大（800 に近づいたら止める）と `pwm_permille` の最大（Duty 上限 40 % に張り付いたら速度が出ていない）を見る。FOLLOWING_ERROR が出るなら `max_vel` を下げるのが先、恒常的なら `ff_count_s_per_percent` を 100 → 80（同じ速度に多めの Duty）。
5. **Duty 上限: `duty_max_percent` を 20 → 30 → 40。** 速度側で頭打ちになっているときだけ上げる。モーター・MD13S の発熱、リレーの音、ブレーキ引継ぎの沈みを見る。
6. 決めた値を `[limits]` に書いてサーバーを再起動し、§9 の記録に残す。

## 5. 症状と調整項目

| 症状 | 見るもの | 調整 |
| --- | --- | --- |
| 動きが小さい | `effective_gain`、`limits` の `gain_limited` | `max_gain`（ゲームの gain 要求がそれ以下なら、ゲーム側の gain） |
| 追いつかない・遅い | SET_POSITION の `max_vel` / `max_acc`（status の `motion_limits.effective`）、`following_error_count` | `max_vel_count_s`、`max_acc_count_s2` |
| 低速が段付き・かくつく | `pwm_permille` が 0 と ±下限の間を往復 | `duty_min_percent` を下げる。残るなら `deadband_*`、Kp。下限で目標から離れる向きに走るなら `V1_FLOOR_SIGN_GATE`（ファーム、既定 1）が効いているか |
| 止まったまま誤差が残る（目標静止） | `following_error_count` が不感帯の外で一定、`pwm_permille` が ±下限 → 200 ms ごとに 10 ずつ増える（停止ブースト） | 動き出し Duty が高い。`duty_min_percent` を動き出し Duty（実測 9 %）に近づける。ブーストが `duty_max` に張り付いても動かないなら機構・電源 |
| 動けない軸が ENCODER_FAULT になり基準が消える | FAULT_EVENT の直前の `pwm_permille` が `max(100, duty_max × 7.5)` 以上 | `kp × following_error_limit / 100 < 0.75 × duty_max` にする（§2.2）か、機構を直す |
| 高速側で頭打ち | `pwm_permille` が `duty_max` に張り付く | `duty_max_percent`（絶対上限 60 まで） |
| 移動中に FOLLOWING_ERROR | `following_error_count` の増え方 | `max_vel` を下げる → `ff` を小さく → それでも出るなら `following_error_limit_count` |
| OVERSPEED | `velocity_count_s` | `overspeed_count_s`（max_vel の 1.5 倍以上を保つ）。暴走なら配線と符号 |

## 6. 変更履歴

- 2026-11-02（Issue #15 H4 / M1 / M2、ファーム。実機未検証）: 停止ブースト（`V1_STALL_BOOST_MS` 200 ms、`V1_STALL_BOOST_MIN_MOVE` 3 count。下限を 1 %/窓で `duty_max` まで）、Duty 下限を目標向きだけに（`V1_FLOOR_SIGN_GATE`、既定 1、`-D` で 0）、無パルスの ENCODER_FAULT のしきい値を `max(V1_STALL_DUTY 10, duty_max × V1_STALL_DUTY_RATIO_PERCENT 75 %)` に。`[limits]` との関係は §2.2: `duty_min ≤ kp × following_error_limit / 100 < 0.75 × duty_max` なら動けない軸は FOLLOWING_ERROR（基準が残る）。`duty_max_percent < 10` は受理されるが無パルス判定が無効（ファーム画面に警告）。

- 2026-11-01: `[game].max_gain` と `[[axis]]` の `max_vel_count_s` / `max_acc_count_s2` / `following_error_limit_count` / `overspeed_count_s` を `[limits]` へ（`schema_version` 1 → 2。旧ファイルは読み込みで失敗する）。SET_LIMITS（0x0D）/ LIMITS（0x88）、capabilities bit3、WS `limits`、`GET/POST /v1/limits`、status の `motion_limits` / `axes[].fw_limits`。`calibration_hash` は幾何だけに。
