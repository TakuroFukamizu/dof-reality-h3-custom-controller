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
| `duty_min_percent` | 3 | 7 | **床**: 位置ループが「床の適用状態 ON」のときに出す最小の非 0 Duty（Issue #25 手順 8 c の E、2026-11-02）。ON になるのは |誤差| > `deadband_out_count`（20）で合成（FF + P）が誤差の向きのとき、OFF は |誤差| ≤ min(15（`V1_FLOOR_OFF_COUNT`）, `deadband_out_count` − 1) か向きの不一致（P = 0、つまり `kp` = 0 なら常に OFF で FF だけに床は掛からない）。**追従中の小さい誤差（|誤差| ≤ 15、または 15〜20 で OFF の履歴）では床は掛からず、生の FF + P（permille）がそのまま出る**ので、`duty_min` は「停止から動き出すときの押し当て」と「誤差 20 超の補正の最小値」だけを決める。旧（r3 以前）は常に掛けていたので 7 % ≈ 700 count/s より遅い目標速度が「7 % で動く / 0 で止まる」の往復（段付き）になったが、E ではその往復は原理的に出ない（FF が足りず誤差が 20 を超えた所で床が入り、15 以下に戻れば抜ける）。自動移動（端当て・中立）の 7 %（`NEUTRAL_DUTY_MIN`）は別の定数で、変えていない。**Issue #25 手順 9（2026-11-02）以降**は、追従中の動き出し補助（T、`V1_TRACK_ASSIST`）も同じ床を使う: 軌道が 100 count/s 超で動いているのにカウントが 150 ms 動かなければ |誤差| が 20 以下でも `duty_min` を掛け、3 count 動けば外す。つまり `duty_min` = 「動き出しの押し当て Duty」で、走行中の Duty は FF ＋ 積分（F）が決める |
| `kp_percent_per_100` | 2 | 6 | 比例ゲイン。誤差 100 count あたり 2 %。**0 は不可**（目標が止まると出力 0 で保持できない）。Issue #25 手順 9 の積分（F）は |I| ≤ 3 % を足すだけなので、P の役目（追従の補正、床 OFF 帯 11〜20 での引き戻し）は変わらない。設定の検査で `kp × following_error_limit_count / 100 ≥ duty_min_percent` を要求する: これを満たさないと、位置ループは FOLLOWING_ERROR で止まる前に Duty 下限より大きな値を出せない（2026-11-01: kp 2 × 400 / 100 = 8 % に対して自重のかかった軸の動き出しが 9 % → 静止なら動かず、目標が動けば FAULT。bringup は kp 6 / duty_min 9 で確認） |
| `ff_count_s_per_percent` | 100 | 100 | フィードフォワード。軌道速度 [count/s] ÷ これ = Duty [%]。実測の下側 |
| `deadband_count` / `deadband_out_count` | 10 / 20 | 10 / 20 | 目標が止まっているときの到達帯（入る側 / 出る側のヒステリシス）。`/v1/center` の到達判定に使う `[game].center_tolerance_count`（既定 50）はこの出る側以上でなければならない（ファームは出る側の内側では補正しないので、それより厳しい許容は永久に満たせない） |

ファーム側の固定値との関係（Issue #15 H4 / M1 / M2、2026-11-02。いずれも実機未検証）:

- **動き出し Duty と `duty_min`**: 11-02 の実機（kp 6 / duty_min 9）で、自重のかかった軸の動き出しは約 9 %、7〜8 % では動かなかった。位置ループに積分は無いので、`duty_min` が動き出し Duty より低いと、目標が止まっているときの静的誤差（不感帯の外）は詰められない。ファームはこれを **停止ブースト**で補う: 軌道が止まっていて下限 Duty でもカウントが 200 ms（`V1_STALL_BOOST_MS`）動かなければ下限を 1 % ずつ上げ、`duty_max` で頭打ち、不感帯に入る・誤差の符号が反転する・軌道が動き出すと 0 に戻す（protocol.md §5）。`duty_min` を下げる（§4 手順 3）と、静止時の押し当ては下限から動き出し Duty まで 200 ms × 差分だけ遅れる。上乗せには上限がある: 対象になってから `V1_STALL_BOOST_MAX_MS`（2 s）動かない、または `duty_max` で頭打ちのまま次の窓も動かなければ ENCODER_FAULT（詰まった機構を `duty_max` で押し続けない。基準は消える）。`duty_min` と `duty_max` の差が小さいほど早く頭打ちになる（差 0 なら最初の窓 200 ms で FAULT）。
- **床の適用状態**（E、Issue #25 手順 8 c。旧 M1 `V1_FLOOR_SIGN_GATE` に代わる。フラグは無い）: 床（`duty_min` ＋ 停止ブースト）は |誤差| > 20 かつ合成が誤差の向きのときに ON、|誤差| ≤ 15 か向きの不一致で OFF（15〜20 はヒステリシス）。OFF のときは生の FF + P。`duty_min`（9 % ≈ 900 count/s）が `max_vel_count_s`（bringup 600）より高くても、追従中は床が掛からないので「バースト → 停止」や反転は出ず、FF（`ff_count_s_per_percent`）が速度を決める。FF が負荷で足りなければ誤差が 20 を超えた所で床が入って追いつき、15 以下で抜ける。これが新しい限界サイクル（20 ⇄ 15 の往復）にならないかを実機で見る（protocol.md §5）。
- **有界の積分と保持バイアス（F、Issue #25 手順 9、2026-11-02。`V1_INTEGRAL` 既定 1）**: `I += Ki × 誤差 × dt`（Ki 0.01 %/(count·s)、|I| ≤ 3 %、|dI/dt| ≤ 1 %/s）。実出力でアンチワインドアップ（`duty_max` 飽和・床の持ち上げ・床の向き不一致では増やさない）。**不感帯の中でも出力 = I**（保持バイアス。従来の 0 ではない）なので、自重で下がる軸の保持は「静止 → 下がる → 床 9 % で蹴り戻す」の繰り返しではなく I で支える（I が 3 % で足りなければ従来どおり |誤差| > 20 で床 ＋ 停止ブースト）。FF が負荷で足りない追従（r4 で #2 の誤差が 111 まで開いた「のこぎり波」）では、床 OFF の帯で I が 1 %/s 以下で育って FF の不足（6 % に対して 8〜9 %）を埋める。上限 3 % なので FF 6 % ＋ I 3 % = 9 %。それ以上の不足は `ff_count_s_per_percent` を下げる（D。負荷測定の後）。DISARMED / FAULT / SET_REFERENCE / 系統切替で 0。STATUS には載らない（`pwm_permille` に含まれる）。ファーム画面の制御項の行 `I`
- **追従中の動き出し補助（T、Issue #25 手順 9、2026-11-02。`V1_TRACK_ASSIST` 既定 1）**: |軌道速度| > 100 count/s・|実測速度| < 20 count/s・|誤差| 増加が 150 ms 続けば床 `duty_min` を掛け、3 count 動けば外す。E の床（|誤差| > 20）より先に効くのは軌道 100〜130 count/s の低速だけ（600 count/s なら 33 ms で E の床が先）。保護は変わらない: 床は ENCODER_FAULT のしきい値（`max(10 %, duty_max × 75 %)`）に届かず、FOLLOWING_ERROR の判定は触らない。ファーム画面の制御項の行 `A`
- **整定判定（S、Issue #25 手順 10、2026-11-02。`V1_SETTLED_MEASURED` 既定 1）**: 不感帯に入るのは「軌道速度 0・|誤差| ≤ `deadband_count`・|実測速度| ≤ 10 count/s が 200 ms」（実測速度は 100 ms 窓なので最長 300 ms）。出るのは従来どおり軌道が動く・|誤差| > `deadband_out_count`。`/v1/center` の到達判定への影響は無い（到達判定は出る側 20 を見る）
- **`V1_STALL_DUTY`（10 %）と `duty_max`**: 無パルスの ENCODER_FAULT のしきい値は `max(10 %, duty_max × 75 %)`（bringup の duty_max 20 → 15 %、40 → 30 %）。動けない軸がどちらの異常になるかは `kp × following_error_limit_count / 100 + FF` とこのしきい値の大小で決まる: 未満なら FOLLOWING_ERROR（基準が残り、FAULT_RESET → ARM で続けられる）、以上なら ENCODER_FAULT（FAULT_RESET で基準が消え、SET_REFERENCE か校正が要る）。bringup（kp 6、FE 400、duty_max 20）は 24 % ≥ 15 % なので ENCODER_FAULT 側。**FOLLOWING_ERROR 側に置きたいなら `kp × FE / 100 < 0.75 × duty_max` にする**（PR #14 の `kp × FE / 100 ≥ duty_min` と合わせて、`duty_min ≤ kp × FE / 100 < 0.75 × duty_max`）。`duty_max_percent < 10` は受理されるが無パルス判定が決して成立しない（ファームの画面に「無パルス検出なし」）。**注意**: しきい値を上げたぶん、軌道が動いている間に `duty_min`（9 %）以上・しきい値（15 %）未満で止まっているモーターを直接守る検出は無い。そこで守るのは FOLLOWING_ERROR（誤差が `following_error_limit_count` を超える）だけで、誤差の増え方は軌道速度で決まる（max_vel 600 なら FE 400 まで約 0.7 s）。目標が止まっているときは停止ブーストの上限（2 s / `duty_max` 頭打ち → ENCODER_FAULT）が守る。

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
| 低速が段付き・かくつく | `pwm_permille` が生の FF（小さい値）と ±床の間を往復し、`following_error_count` が 15〜20 をまたぐ（ファーム画面の `FLR` が 0/1 で点滅） | E の床の往復。`ff_count_s_per_percent` を下げて FF を増やす（同じ速度に多めの Duty）か、`duty_min_percent` を下げる。残るなら `deadband_*`、Kp。r3 以前の「0 と ±下限の往復」はもう出ない |
| 止まったまま誤差が残る（目標静止） | `following_error_count` が不感帯の外で一定、`pwm_permille` が ±下限 → 200 ms ごとに 10 ずつ増える（停止ブースト） | 動き出し Duty が高い。`duty_min_percent` を動き出し Duty（実測 9 %）に近づける。ブーストが `duty_max` に張り付いても動かないなら機構・電源 |
| 誤差 11〜20 で止まったまま（目標静止。床 OFF・停止ブースト対象外） | `pwm_permille` が小さい P（kp 6 で 7〜12）＋ I（≤ 30）のまま、ファーム画面 `F0 B 0 I ±nn` | 床の手前の帯。I の上限 3 % では動き出し 9 % に届かない。`deadband_out_count` を下げる（床の ON を早く）か、`kp` を上げる（帯の中の P を増やす）。I が 30 に張りついていれば負荷が FF/床の想定より重い |
| 保持中に自重で下がって蹴り戻す（0 ↔ 床の繰り返し） | `pwm_permille` が 0 と ±90 を往復、ファーム画面 `I` が育たない | F（`V1_INTEGRAL`）が切れていないか（`I` が常に 0 なら切れている）。不感帯の中の出力は I なので、I が育つ前に 20 を超えて床に入るなら保持の時間が短い（I は 1 %/s 以下）。下がる速さが速ければ `duty_min` の床の帯を狭く（`deadband_out_count`） |
| 追従で FF が足りず誤差が 20 を越えて床で追いつく（のこぎり波） | `following_error_count` が 0 → 20 超 → 15 以下の繰り返し、ファーム画面 `F` が 0/1 で点滅、`I` が 30 に張りつく | I の上限（3 %）を超える負荷。`ff_count_s_per_percent` を下げる（負荷・方向別の測定の後。D）。I が 30 未満で落ち着いていれば待つ（I は 1 %/s 以下で育つ） |
| 追従の始動で 150 ms 止まってから床で動く | ファーム画面 `A1` が点く、`pwm_permille` が小さい FF → ±床 | T の動き出し補助。軌道が 100 count/s 超でカウントが動かない。正常（E の床より先に押し当てる）。頻発するなら `duty_min_percent` が動き出し Duty に足りているか、`ff` が低すぎないか |
| 動けない軸が ENCODER_FAULT になり基準が消える | FAULT_EVENT の直前の `pwm_permille` が `max(100, duty_max × 7.5)` 以上 | `kp × following_error_limit / 100 < 0.75 × duty_max` にする（§2.2）か、機構を直す |
| 高速側で頭打ち | `pwm_permille` が `duty_max` に張り付く | `duty_max_percent`（絶対上限 60 まで） |
| 移動中に FOLLOWING_ERROR | `following_error_count` の増え方 | `max_vel` を下げる → `ff` を小さく → それでも出るなら `following_error_limit_count` |
| OVERSPEED | `velocity_count_s` | `overspeed_count_s`（max_vel の 1.5 倍以上を保つ）。暴走なら配線と符号 |

## 6. 変更履歴

- 2026-11-02（Issue #25 手順 9・10、ファーム。実機未検証・未書き込み）: F 有界の積分と保持バイアス（`V1_INTEGRAL`。Ki 0.01 %/(count·s)、|I| ≤ 3 %、|dI/dt| ≤ 1 %/s、実出力でアンチワインドアップ、不感帯の中は出力 = I、DISARMED / FAULT / SET_REFERENCE / 系統切替で 0）、T 追従中の動き出し補助（`V1_TRACK_ASSIST`。軌道 > 100 count/s・実測 < 20 count/s が 150 ms で床 `duty_min`、3 count で解除）、S 整定判定（`V1_SETTLED_MEASURED`。軌道 0・|誤差| ≤ 10・実測 ≤ 10 count/s が 200 ms で不感帯に入る）。いずれも `-D<名前>=0` で切れる。`duty_min_percent` の意味は §2.2: 動き出しの押し当て（E の床と T の補助）、走行は FF ＋ I。

- 2026-11-02（Issue #25 手順 8 c、ファーム。実機未検証）: E。床（`duty_min` ＋ 停止ブースト）はヒステリシス付きの適用状態が ON のときだけ（ON: |誤差| > `deadband_out` かつ向き一致、OFF: |誤差| ≤ `V1_FLOOR_OFF_COUNT` 15 か向き不一致）。OFF では有界の生の FF + P を permille で出す（0 にしない）。Issue #15 M1 の 0 置換と `V1_FLOOR_SIGN_GATE` を廃止（向きの条件として残る）。`duty_min_percent` の意味は §2.2: 床は帯の外だけ。

- 2026-11-02（Issue #25 手順 8 b、ファーム。実機未検証）: 位置ループの分解能を permille（0.1 %）に。`v1_duty_command` → `motor_output_write_permille` → PWM まで permille で運び、STATUS の `pwm_permille` は実際に書いた値になる（旧: 整数 % × 10）。線上の `duty_min_percent` / `duty_max_percent` は整数 % のまま。しきい値は `config.h` の `V1_DUTY_MIN_PERMILLE` / `V1_DUTY_MAX_PERMILLE` / `V1_STALL_DUTY_PERMILLE` / `V1_STALL_BOOST_STEP_PERMILLE` / `V1_ABS_DUTY_MAX_PERMILLE` と `v1_permille()` で換算し、効き目は変えていない（ENCODER_FAULT のしきい値は整数 % で切り捨ててから × 10）。

- 2026-11-02（Issue #15 H4 / M1 / M2、ファーム。実機未検証）: 停止ブースト（`V1_STALL_BOOST_MS` 200 ms、`V1_STALL_BOOST_MIN_MOVE` 3 count。下限を 1 %/窓で `duty_max` まで）、Duty 下限を目標向きだけに（`V1_FLOOR_SIGN_GATE`、既定 1、`-D` で 0）、無パルスの ENCODER_FAULT のしきい値を `max(V1_STALL_DUTY 10, duty_max × V1_STALL_DUTY_RATIO_PERCENT 75 %)` に。`[limits]` との関係は §2.2: `duty_min ≤ kp × following_error_limit / 100 < 0.75 × duty_max` なら動けない軸は FOLLOWING_ERROR（基準が残る）。`duty_max_percent < 10` は受理されるが無パルス判定が無効（ファーム画面に警告）。

- 2026-11-01: `[game].max_gain` と `[[axis]]` の `max_vel_count_s` / `max_acc_count_s2` / `following_error_limit_count` / `overspeed_count_s` を `[limits]` へ（`schema_version` 1 → 2。旧ファイルは読み込みで失敗する）。SET_LIMITS（0x0D）/ LIMITS（0x88）、capabilities bit3、WS `limits`、`GET/POST /v1/limits`、status の `motion_limits` / `axes[].fw_limits`。`calibration_hash` は幾何だけに。
