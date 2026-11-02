# 動きの制限値（limits）

状態: **実装済み・模擬軸で試験済み。数値はどれも実機未検証の見積もり。** 設計は [docs/superpowers/specs/2026-11-01-motion-limits-design.md](../superpowers/specs/2026-11-01-motion-limits-design.md)。

座席の動きを制限する値は 2 段だけにしている。「いまどこで何に制限されているか」をこの 2 段で追えるようにするため、それ以外の段は作らない。

```text
ゲーム / 管理者 ─(要求)─▶ PC の現在値（3 軸共通。初期値は [limits]）─▶ clamp(上限) ─▶ 有効値
                                                                  │
   上限 = ファームの絶対上限（LIMITS で申告）と overspeed_count_s / 1.5 と gain 1.0
          ＋ ゲームが書いた項目には操作者の [limits] の値（ゲームは下げるだけ）
                                                                  │
   有効値 → 毎フレームの SET_POSITION（速度・加速度）、SET_LIMITS（Duty 上限など。変わったときだけ）
                                                                  │
   ファーム: 受けた値を絶対上限で黙ってクランプ（拒否しない）
```

## 1. 2 段の制限

| 段 | どこにあるか | 変え方 | 内容 |
| --- | --- | --- | --- |
| 1. 絶対上限 | ファーム `moter-controler/include/config.h` の定数 `V1_ABS_*`。PC へは HELLO の後の LIMITS で申告される（status の `axes[].fw_limits.abs_*`） | 焼き直し | Duty 60 %、速度 6000 count/s、加速度 40000 count/s² |
| 2. 現在値 | PC の設定ファイル `[limits]`（初期値）。起動後は 3 軸共通の 1 組（status の `motion_limits`） | 設定ファイル、`WS /v1/motion` の `limits`（操作者の初期値まで。下げるだけ）、`POST /v1/limits`（段 1 まで） | §2 の表 |

- 段 2 で段 1 を超える要求を受けても拒否はしない。段 1 でクランプして、有効値と理由（`ceiling_clamped`）を返す。gain の `gain_limited` と同じ流儀。
- **ゲーム経路は操作者の値を下げるだけ**（Issue #15）。WS `/v1/motion` の `limits`（トークン不要）で書いた項目は、`[limits]` の初期値（＝操作者が許した上限。status の `motion_limits.operator_max`）を超えない。超える要求は操作者の値でクランプして理由 `operator_limited` を返す（ファームの上限のほうが低い個体では `ceiling_clamped`）。操作者の `POST /v1/limits`（Bearer）だけがそれを超えて段 1 まで上げられる。項目ごとに「最後に書いたのがゲームか」を覚えていて（status の `motion_limits.game_fields`）、管理者が書いた項目はこの上限から外れ、ゲームが書き直せばまた入る。`POST /v1/session` で値とともに消える。
- クランプにはもう 1 つ、**停止時間の規則**（§1.1）がある。上限とは逆向き（下限）で、`max_acc_count_s2` を上げる（場合により `max_vel_count_s` を下げる）。理由は `stop_time_limited`。
- クランプした要求（target）がすぐ有効値になるとは限らない。Duty 上限が変わる要求は、全軸が新しい SET_LIMITS を ACK するまで planner に効かない（§1.2、status の `pending`）。
- CONFIG（ARM 時に MCU へ送る）の速度・加速度には段 1 の値を入れる。実際の上限は毎フレームの SET_POSITION が運ぶ有効値で決まる。CONFIG は「もう 1 つの上限」ではない。
- ソフトリミット（`soft_min/max_count`、`decel_zone_count`）と校正 3 点は幾何で、制限値ではない。`[[axis]]` に置いたままで、実行時には変えない。
- `POST /v1/session`（新しいゲームの接続）で現在値は `[limits]` の初期値に戻る。ゲームが上げた値は、別のゲームや再接続に引き継がれない。

### 1.1 停止時間の規則（Issue #15 H2）

STOP（管理の `/v1/stop`、鮮度切れ、`center_reached`）は planner が有効値の `max_acc_count_s2` で減速する。MCU は `mcu_stop_timeout_ms`（1500）、サーバーは `stop_settle_timeout_ms`（3000）で見切りをつけ、どちらも FAULT → **その時点の速度でブレーキ**になる。つまり `max_acc` を小さくし過ぎると「安全側に倒れる」はずの STOP が危険側に倒れる。2026-11-02 の静的レビューまで、ゲーム側の WS `limits`（トークン不要、ACTIVE 中も効く）は `max_acc_count_s2 = 1` を受け付けていた（600 count/s からの減速に 600 s）。

規則: **最悪の停止時間 `max_vel_count_s / max_acc_count_s2` ≤ 停止期限 = 0.8 × min(`mcu_stop_timeout_ms`, `stop_settle_timeout_ms`)**（既定の timing で 1.2 s。0.8 はリンク遅延・MCU 側のランプ・整定判定の分）。`control-server/src/limits.rs` の `stop_deadline` が唯一の定義で、設定の検査と実行時の clamp が同じ関数を使う。

- 設定の検査（起動時）: `[limits]` の `max_vel / max_acc` がこれを超えると起動しない（メッセージに停止時間・期限・必要な `max_acc` の下限を出す）。ファームの絶対上限（6000 / 40000 = 0.15 s）と `overspeed / 1.5` から作る上限の組も検査する。実行時の要求は上限まで上げられるので、上限の速度でも規則を満たす加速度が上限の内側に無いと、clamp が行き場を失うため。
- 実行時の clamp（`limits::clamp`。WS ack・`GET/POST /v1/limits`・制御ループが共有）: 上限でクランプしたあと、**有効値の `max_vel`** から `max_acc` の下限 `ceil(max_vel / 期限)` を作り、要求がそれより小さければ下限まで上げる。1 つの要求で速度と加速度を同時に変えても、新しい速度で判定する（要求は patch を全部当ててから clamp する）。下限が加速度の上限を超える個体（LIMITS で小さい `abs_max_acc` を申告した）では、加速度を上限にして `max_vel` を `floor(上限 × 期限)` まで下げる。動かした項目は `limited`、理由は `stop_time_limited`。
- `duty_max_percent` には下限を作っていない。Duty と減速度を結ぶ検証済みのモデルが無く、Duty 不足は FOLLOWING_ERROR（保護）で止まる。`duty_min ≤ duty_max` は `FwTuning` で保っている。
- 例（既定 timing、期限 1.2 s）: `max_vel 600` に `max_acc 1` → 500 に上がる。`max_vel 6000`（上限）に `max_acc 1000` → 5000。`{max_vel 300, max_acc 250}` の同時要求はそのまま通る。bringup の 600 / 2000（0.3 s）、example の 3000 / 15000（0.2 s）はどちらも通る。

### 1.2 要求と適用の分離（Issue #15 M3）

PC は制限値を 3 段で持つ: `requested`（API が書いた値）→ `target`（§1 の上限と §1.1 で clamp した値。SET_LIMITS はこれを運ぶ）→ `effective`（planner と毎フレームの SET_POSITION が使う値。status の `motion_limits.effective`）。

- target の SET_LIMITS 内容（Duty 上限）が effective と同じなら、target はその tick で effective になる。速度・加速度・gain だけの変更は上げても下げても待たない（SET_LIMITS を送らないので待つ相手がいない）。
- Duty 上限が変わる要求は、接続中で SET_LIMITS を知る**全軸**がその内容を ACK するまで effective にならない（`pending: true`）。待つ間、effective は確定済みの組を保ち、target のうち**下がった**成分（gain・速度・加速度）だけを先に取り込む。速度と Duty を同時に上げたとき、前の SET_LIMITS が飛行中の軸に旧 Duty 上限のまま速い目標が届いて FOLLOWING_ERROR になる窓（往復 1 回分、喪失なら 1 s）を閉じるため。成分ごとの最小は §1.1 を保つ（両組が `acc ≥ vel / 期限` なら最小同士も満たす）。
- 飛行中に要求がまた変わったら、前の ACK が届いた次の tick で新しい内容を送る（`LIMITS_RETRY` の 1 s は待たない）。SET_LIMITS が拒否された軸は `limits_rejected`（§3.3）で見え、`pending` は立ったまま。
- WS ack の `effective_limits` と `GET/POST /v1/limits` の `effective` は **target**（API スレッドは適用の完了を知らない）。適用済みかは status の `motion_limits.pending` で見る。

## 2. `[limits]` の項目と根拠

`control-server/config/*.toml` の `[limits]`（`schema_version = 2`）。3 軸共通。下の初期値は `example.toml` / `sim.toml` のもの（上げ切った先の目標）。`bringup.toml` は §4 の低い値で置いてある。

### 2.1 実行時に変えられる 4 つ

表の「上限」は管理者（`POST /v1/limits`）が上げられる限界。**ゲーム（WS `limits`）はどの項目も初期値まで**で、超える要求は初期値で切られて `operator_limited` になる（§1）。

| 項目 | 初期値 | 上限 | 根拠 / 注意 |
| --- | ---: | --- | --- |
| `max_gain` | 0.5 | 1.0 | 混合式 `m = 512 + gain·511·(roll ± pitch)` で、pitch と roll を同時に振り切っても 0.5 以下なら仮想 0〜1023 で飽和しない。旧 h3game の既定 0.35 は検証値ではなかった。`bringup.toml` は向きの確認用に 0.15 |
| `max_vel_count_s` | 3000 | min(絶対上限 6000, `overspeed_count_s` / 1.5) | 7.5 回転/s。ストローク 5500 count を加減速込みで約 2.3 s。実測の Duty 1 % ≈ 100〜130 count/s（2026-10-31）から、フィードフォワードだけで約 30 % |
| `max_acc_count_s2` | 15000 | 絶対上限 40000。**下限** `ceil(max_vel / 停止期限)`（§1.1。既定 timing で `max_vel / 1.2 s`） | 0 → 3000 count/s に 0.2 s。旧値 2000 では 1.5 s かかり、30 Hz の目標変化に追いつけなかった。下限を割る要求は下限まで上がる（`stop_time_limited`） |
| `duty_max_percent` | 40 | 絶対上限 60 | 外挿で約 4000〜5000 count/s。max_vel 3000 に対して FF + P の余裕を残す。旧ファームは 20 % 直書き |

### 2.2 初期値だけ（SET_LIMITS でファームへ配る。実行時には変えない）

| 項目 | 初期値 | 旧（ファーム直書き） | 役割 |
| --- | ---: | ---: | --- |
| `duty_min_percent` | 3 | 7 | **床**: 位置ループが「床の適用状態 ON」のときに出す最小の非 0 Duty（Issue #25 手順 8 c の E、2026-11-02）。ON になるのは |誤差| > `deadband_out_count`（20）で合成（FF + P）が誤差の向きのとき、OFF は |誤差| ≤ min(15（`V1_FLOOR_OFF_COUNT`）, `deadband_out_count` − 1) か向きの不一致（P = 0、つまり `kp` = 0 なら常に OFF で FF だけに床は掛からない）。**追従中の小さい誤差（|誤差| ≤ 15、または 15〜20 で OFF の履歴）では床は掛からず、生の FF + P（permille）がそのまま出る**ので、`duty_min` は「停止から動き出すときの押し当て」と「誤差 20 超の補正の最小値」だけを決める。旧（r3 以前）は常に掛けていたので 7 % ≈ 700 count/s より遅い目標速度が「7 % で動く / 0 で止まる」の往復（段付き）になったが、E ではその往復は原理的に出ない（FF が足りず誤差が 20 を超えた所で床が入り、15 以下に戻れば抜ける）。自動移動（端当て・中立）の 7 %（`NEUTRAL_DUTY_MIN`）は別の定数で、変えていない。**Issue #25 手順 9（2026-11-02）以降**は、追従中の動き出し補助（T、`V1_TRACK_ASSIST`）も同じ床を使う: 軌道が 100 count/s 超で動いているのにカウントが 150 ms 動かなければ |誤差| が 20 以下でも `duty_min` を掛け、3 count 動けば外す。つまり `duty_min` = 「動き出しの押し当て Duty」で、走行中の Duty は FF ＋ 積分（F）が決める |
| `kp_percent_per_100` | 2 | 6 | 比例ゲイン。誤差 100 count あたり 2 %。**0 は不可**（目標が止まると出力 0 で保持できない）。Issue #25 手順 9 の積分（F）は |I| ≤ 3 % を足すだけなので、P の役目（追従の補正、床 OFF 帯 11〜20 での引き戻し）は変わらない。設定の検査で `kp × following_error_limit_count / 100 ≥ duty_min_percent` を要求する: これを満たさないと、位置ループは FOLLOWING_ERROR で止まる前に Duty 下限より大きな値を出せない（2026-11-01: kp 2 × 400 / 100 = 8 % に対して自重のかかった軸の動き出しが 9 % → 静止なら動かず、目標が動けば FAULT。bringup は kp 6 / duty_min 9 で確認） |
| `ff_count_s_per_percent` | 100 | 100 | フィードフォワード。軌道速度 [count/s] ÷ これ = Duty [%]。実測の下側 |
| `deadband_count` / `deadband_out_count` | 10 / 20 | 10 / 20 | 目標が止まっているときの到達帯（入る側 / 出る側のヒステリシス）。`/v1/center` の到達判定に使う `[game].center_tolerance_count`（既定 50）はこの出る側以上でなければならない（ファームは出る側の内側では補正しないので、それより厳しい許容は永久に満たせない） |

**Issue #25 r6（2026-11-02、`config.h` `V1_FF_DIRECTIONAL` 既定 1。実機未確認）で効く項目**: 位置ループが方向別 FF ＋ PI・走行中の床なしになり（protocol.md §5 の先頭）、SET_LIMITS の 6 項目のうち位置ループに効くのは **`kp_percent_per_100` と `duty_max_percent` だけ**。`ff_count_s_per_percent` は受理して LIMITS で返すが使わない（FF は `config.h` の `V1_FF_A_UP_PERMILLE` 46 / `V1_FF_B_UP_PERMILLE_PER_COUNT_S` 0.10 / `V1_FF_A_DOWN_PERMILLE` −7 / `V1_FF_B_DOWN_PERMILLE_PER_COUNT_S` 0.107。変えるにはファームを焼き直す）。`duty_min_percent` は床が無いので使わない（動き出しだけ `V1_BREAKAWAY_UP_PERMILLE` 90 / `V1_BREAKAWAY_DOWN_PERMILLE` 60 のパルス）。`deadband_count` / `deadband_out_count` は保持の入口 `V1_HOLD_ENTER_COUNT` 2 / 出口 `V1_HOLD_EXIT_COUNT` 4 に代わる（`/v1/center` の到達判定 `center_tolerance_count` ≥ 20 はそのままで問題ない）。`duty_max_percent` は飽和と無パルスのしきい値 `max(10 %, duty_max × 75 %)` に効く。設定の検査 `kp × FE / 100 ≥ duty_min` は従来どおり通す（r6 では意味を持たないが害も無い）。**FAULT の境**: FF が 600 count/s で 10.6 %（旧 6 %）になったので、止まった軸は `FF + kp × FE / 100 ≥ 0.75 × duty_max` のとき ENCODER_FAULT 側（bringup kp 6 / FE 400 / duty_max 20: 10.6 + 24 ≥ 15）。FOLLOWING_ERROR 側に置くなら `duty_max` を上げる（40 → 30 %）か kp を下げる。以下の段落は `-DV1_FF_DIRECTIONAL=0` の経路（r5 まで）の説明。

ファーム側の固定値との関係（Issue #15 H4 / M1 / M2、2026-11-02。いずれも実機未検証）:

- **動き出し Duty と `duty_min`**: 11-02 の実機（kp 6 / duty_min 9）で、自重のかかった軸の動き出しは約 9 %、7〜8 % では動かなかった。位置ループに積分は無いので、`duty_min` が動き出し Duty より低いと、目標が止まっているときの静的誤差（不感帯の外）は詰められない。ファームはこれを **停止ブースト**で補う: 軌道が止まっていて下限 Duty でもカウントが 200 ms（`V1_STALL_BOOST_MS`）動かなければ下限を 1 % ずつ上げ、`duty_max` で頭打ち、不感帯に入る・誤差の符号が反転する・軌道が動き出すと 0 に戻す（protocol.md §5）。`duty_min` を下げる（§4 手順 3）と、静止時の押し当ては下限から動き出し Duty まで 200 ms × 差分だけ遅れる。上乗せには上限がある: 対象になってから `V1_STALL_BOOST_MAX_MS`（2 s）動かない、または `duty_max` で頭打ちのまま次の窓も動かなければ ENCODER_FAULT（詰まった機構を `duty_max` で押し続けない。基準は消える）。`duty_min` と `duty_max` の差が小さいほど早く頭打ちになる（差 0 なら最初の窓 200 ms で FAULT）。
- **床の適用状態**（E、Issue #25 手順 8 c。旧 M1 `V1_FLOOR_SIGN_GATE` に代わる。フラグは無い）: 床（`duty_min` ＋ 停止ブースト）は |誤差| > 20 かつ合成が誤差の向きのときに ON、|誤差| ≤ 15 か向きの不一致で OFF（15〜20 はヒステリシス）。OFF のときは生の FF + P。`duty_min`（9 % ≈ 900 count/s）が `max_vel_count_s`（bringup 600）より高くても、追従中は床が掛からないので「バースト → 停止」や反転は出ず、FF（`ff_count_s_per_percent`）が速度を決める。FF が負荷で足りなければ誤差が 20 を超えた所で床が入って追いつき、15 以下で抜ける。これが新しい限界サイクル（20 ⇄ 15 の往復）にならないかを実機で見る（protocol.md §5）。
- **有界の積分と保持バイアス（F、Issue #25 手順 9、2026-11-02。`V1_INTEGRAL` 既定 1）**: `I += Ki × 誤差 × dt`（Ki 0.01 %/(count·s)、|I| ≤ 3 %、|dI/dt| ≤ 1 %/s）。実出力でアンチワインドアップ（`duty_max` 飽和・床の持ち上げ・床の向き不一致では増やさない）。**不感帯の中でも出力 = I**（保持バイアス。従来の 0 ではない）なので、自重で下がる軸の保持は「静止 → 下がる → 床 9 % で蹴り戻す」の繰り返しではなく I で支える（I が 3 % で足りなければ従来どおり |誤差| > 20 で床 ＋ 停止ブースト）。FF が負荷で足りない追従（r4 で #2 の誤差が 111 まで開いた「のこぎり波」）では、床 OFF の帯で I が 1 %/s 以下で育って FF の不足（6 % に対して 8〜9 %）を埋める。上限 3 % なので FF 6 % ＋ I 3 % = 9 %。それ以上の不足は `ff_count_s_per_percent` を下げる（D。負荷測定の後）。DISARMED / FAULT / SET_REFERENCE / 系統切替で 0。STATUS には載らない（`pwm_permille` に含まれる）。ファーム画面の制御項の行 `I`
- **追従中の動き出し補助（T、Issue #25 手順 9、2026-11-02。`V1_TRACK_ASSIST` 既定 1）**: |軌道速度| > 100 count/s・|実測速度| < 20 count/s・|誤差| 増加が 150 ms 続けば床 `duty_min` を掛け、3 count 動けば外す。E の床（|誤差| > 20）より先に効くのは軌道 100〜130 count/s の低速だけ（600 count/s なら 33 ms で E の床が先）。保護は変わらない: 床は ENCODER_FAULT のしきい値（`max(10 %, duty_max × 75 %)`）に届かず、FOLLOWING_ERROR の判定は触らない。ファーム画面の制御項の行 `A`
- **整定判定（S、Issue #25 手順 10、2026-11-02。`V1_SETTLED_MEASURED` 既定 1）**: 不感帯に入るのは「軌道速度 0・|誤差| ≤ `deadband_count`・|実測速度| ≤ 10 count/s が 200 ms」（実測速度は 100 ms 窓なので最長 300 ms）。出るのは従来どおり軌道が動く・|誤差| > `deadband_out_count`。`/v1/center` の到達判定への影響は無い（到達判定は出る側 20 を見る）
- **r10（Issue #44・#46、ファーム `V1_SUPERVISOR` 既定 1。実機未検証）**: 方向別の経路では「動けない軸が FOLLOWING_ERROR と ENCODER_FAULT のどちらになるか」の境（下の段落）は無くなった。進まない軸は ENCODER_FAULT にならず、見えない走行の予算（中央付近で 1000 count = 400 ms、機械端から 400 以内は 0）で `FEEDBACK_UNCERTAIN`、少しずつ動いているなら飽和 1 s で `LOAD_LIMITED`。どちらも PWM 0 ＋ ブレーキ作動の持続する停止で、DISARM → 再 ARM で戻る（基準は残る）。誤差の上限は FOLLOWING_ERROR のまま（既定では猶予なし）。`duty_max_percent < 10` の「無パルス検出なし」の警告も出ない。詳細は protocol.md §5 の r10。以下の段落は `-DV1_SUPERVISOR=0`（または従来の経路）のときの説明。
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
- ack: `{"type":"ack","seq":1252,"accepted":true,"effective_gain":0.5,"effective_limits":{...4 項目...},"limits":["operator_limited"],"fresh":false}`。`effective_gain` は新しい gain の上限。`operator_limited` はゲームが書いた項目が操作者の `[limits]` の値で切られたとき（§1。ゲームは下げるだけ。4 項目とも）、`ceiling_clamped` はファームの上限で切られたとき、`stop_time_limited` は停止時間の規則（§1.1）で `max_acc` が上がった（または `max_vel` が下がった）とき。`limits` は要求全体のクランプ結果で、前に送って切られたままの項目も毎回入る。`effective_limits` は clamp した要求値（target）で、Duty 上限が変わる要求が planner に効くのは全軸の ACK 後（§1.2、status の `motion_limits.pending`）。
- 拒否: 非有限は `non_finite`、`max_gain < 0`・速度と加速度の 0 以下・`duty_max_percent` の 1〜100 の外は `out_of_range`。競合中（`axis_conflict`）は pose と同じく拒否。
- 新しい gain の上限は、いま追従中の姿勢にもその場で効く（`effective_gain` が下がり `gain_limited` が付く。鮮度は変えない）。Python クライアントは `submit_limits(...)` を pose より先に送る。
- bridge（`H3_BACKEND=server`）はゲームの POST に `"limits": {...}` があればそのまま渡す（`clients/h3game/README.md`）。

### 3.2 管理者から（REST）

```sh
curl -s $URL/v1/limits | python3 -m json.tool                          # 現在値（トークン不要・loopback だけ）
curl -sX POST -H "$AUTH" -d '{"duty_max_percent":30}' $URL/v1/limits   # 書いた項目だけ変える（Bearer、rate limit あり）
```

応答（GET / POST 共通）: `{"ok":true,"effective":{...},"requested":{...},"ceiling":{...},"defaults":{...},"operator_max":{...},"game_fields":[...],"limited":[...],"reasons":[...]}`。`requested` が要求値、`ceiling` がいまの上限、`effective` がその結果（clamp した要求値。適用済みかは status の `pending`）、`defaults` が `[limits]` の初期値、`operator_max` がゲーム経路の上限（＝ `defaults` の値。役割を名前にしたもの）、`game_fields` がいまゲームが書いた項目（この上限がかかる）、`limited` は clamp が動かした項目名、`reasons` はその理由（`ceiling_clamped` / `operator_limited` / `stop_time_limited`）。不正な値は 400（`non_finite` / `out_of_range`）。POST で書いた項目は `game_fields` から外れ、ファームの上限まで上がる。

### 3.3 status で見る

- `motion_limits`: 上の GET と同じ 8 つ（`effective` / `requested` / `ceiling` / `defaults` / `operator_max` / `game_fields` / `limited` / `reasons`）に `pending`（bool。Duty 上限が変わる要求の SET_LIMITS を全軸が ACK するまで true。その間 `effective` は確定済みの組＋下がった成分、§1.2）。
- `axes[].fw_limits`: その個体が LIMITS で返した有効値（`duty_max_percent` …）と絶対上限（`abs_duty_max_percent`、`abs_max_vel_count_s`、`abs_max_acc_count_s2`）。SET_LIMITS を受ける前は `null`。
- blockers: `firmware_limits_unsupported: axisN`（SET_LIMITS を知らない旧ファーム。焼き直すまで ARM しない）、`limits_rejected: axisN`（SET_LIMITS が拒否された。1 秒後に再送）、`limits_unconfirmed: axisN`（いまの調整値の SET_LIMITS が ACK され、かつ LIMITS が届くまで。ファーム既定の 7〜20 % や前の接続の値で ARM しないための阻害。ACK が来なければ 1 秒の期限切れの後に、ACK は来たが LIMITS が落ちたなら 1 秒後に、新しい request_id で送り直す）。
- events: `limits`（LIMITS の受信）、`ack_rejected`（SET_LIMITS の拒否）。

### 3.4 試験ページ（`GET /`）

「動きの制限」の 4 本のスライダー（WS で送るので、上限は status の `motion_limits.operator_max` と `ceiling` の小さいほうに追従）→「制限を送る」。WS の `limits` で送るので、姿勢送信（WS）を開始してから使う。「設定の初期値に戻す」は `defaults` を送る。limits の ack は姿勢の ack とは別に表示する。有効値の行には `limited` / `reasons` と、全軸の ACK 待ちなら `[pending]` を出す。

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

- 2026-11-02（Issue #15 追加分）: ゲーム経路の上限（§1）。WS `limits` で書いた項目は操作者の `[limits]` の値を超えない（`operator_limited`）。`POST /v1/limits` だけがファームの上限まで上げる。`GET/POST /v1/limits` と status の `motion_limits` に `operator_max` / `game_fields`。試験ページのスライダーの上限もそれに追従。
- 2026-11-02（Issue #15 H2 / M3）: 停止時間の規則（§1.1。設定の検査と実行時 clamp の下限、`stop_time_limited`、`GET/POST /v1/limits` と status に `reasons`）、要求と適用の分離（§1.2。Duty 上限が変わる要求は全軸の SET_LIMITS ACK まで planner に効かない、status `motion_limits.pending`）。同じ PR で START の姿勢関門 `[game].start_max_offset_count`（M4、DESIGN.md §5）。
- 2026-11-02（Issue #35 r9、ファーム。未焼き込み・実機未検証）: 方向別の経路に速度誤差の減衰項（`V1_VEL_DAMPING`、既定 1）。`u_d = clamp(0.05 × (軌道速度 − v_fast), ±10)` permille、v_fast は 40 ms 窓の速度推定。SET_LIMITS の項目と意味は変えない（Kd・上限・窓は `config.h` の定数で、変えるにはファームを焼き直す）。r8 と同じく kp 30 前提。protocol.md §5 の r9。
- 2026-11-02（Issue #25 r6〜r8、ファーム。PR #33。未焼き込み）: 方向別 FF ＋ PI・走行中の床なし（`V1_FF_DIRECTIONAL`、既定 1）。この経路では SET_LIMITS の `ff_count_s_per_percent` / `duty_min_percent` / deadband は出力に効かず、効くのは `kp_percent_per_100` と `duty_max_percent`。kp 6 では追従が柔らかすぎる（#1 で FE rms 15）ので、焼くなら kp 30（FE rms 3.6）。2026-11-02 時点で 3 台とも r3（PR #19 の 914cee8）・kp 6 のまま。経緯は Issue #25 と `runs/hw_20261102.md`。
- 2026-11-02（Issue #25 手順 9・10、ファーム。実機未検証・未書き込み）: F 有界の積分と保持バイアス（`V1_INTEGRAL`。Ki 0.01 %/(count·s)、|I| ≤ 3 %、|dI/dt| ≤ 1 %/s、実出力でアンチワインドアップ、不感帯の中は出力 = I、DISARMED / FAULT / SET_REFERENCE / 系統切替で 0）、T 追従中の動き出し補助（`V1_TRACK_ASSIST`。軌道 > 100 count/s・実測 < 20 count/s が 150 ms で床 `duty_min`、3 count で解除）、S 整定判定（`V1_SETTLED_MEASURED`。軌道 0・|誤差| ≤ 10・実測 ≤ 10 count/s が 200 ms で不感帯に入る）。いずれも `-D<名前>=0` で切れる。`duty_min_percent` の意味は §2.2: 動き出しの押し当て（E の床と T の補助）、走行は FF ＋ I。

- 2026-11-02（Issue #25 手順 8 c、ファーム。実機未検証）: E。床（`duty_min` ＋ 停止ブースト）はヒステリシス付きの適用状態が ON のときだけ（ON: |誤差| > `deadband_out` かつ向き一致、OFF: |誤差| ≤ `V1_FLOOR_OFF_COUNT` 15 か向き不一致）。OFF では有界の生の FF + P を permille で出す（0 にしない）。Issue #15 M1 の 0 置換と `V1_FLOOR_SIGN_GATE` を廃止（向きの条件として残る）。`duty_min_percent` の意味は §2.2: 床は帯の外だけ。

- 2026-11-02（Issue #44・#46、ファーム r10。実機未検証）: 保護の監視 `V1Supervisor`。方向別の経路で「進まない → ENCODER_FAULT」を出さず、見えない走行の予算（`FEEDBACK_UNCERTAIN`）・過負荷（`LOAD_LIMITED`）・端の手前の制動（`END_BRAKING`）・SOFT_LIMIT / FOLLOWING_ERROR の猶予（`LIMIT_RECOVERY`）に置き換え。STATUS flags の bit 7〜10（protocol.md §3.4）。`[limits]` の値の意味は変えていない。
- 2026-11-02（Issue #25 手順 8 b、ファーム。実機未検証）: 位置ループの分解能を permille（0.1 %）に。`v1_duty_command` → `motor_output_write_permille` → PWM まで permille で運び、STATUS の `pwm_permille` は実際に書いた値になる（旧: 整数 % × 10）。線上の `duty_min_percent` / `duty_max_percent` は整数 % のまま。しきい値は `config.h` の `V1_DUTY_MIN_PERMILLE` / `V1_DUTY_MAX_PERMILLE` / `V1_STALL_DUTY_PERMILLE` / `V1_STALL_BOOST_STEP_PERMILLE` / `V1_ABS_DUTY_MAX_PERMILLE` と `v1_permille()` で換算し、効き目は変えていない（ENCODER_FAULT のしきい値は整数 % で切り捨ててから × 10）。

- 2026-11-02（Issue #15 H4 / M1 / M2、ファーム。実機未検証）: 停止ブースト（`V1_STALL_BOOST_MS` 200 ms、`V1_STALL_BOOST_MIN_MOVE` 3 count。下限を 1 %/窓で `duty_max` まで）、Duty 下限を目標向きだけに（`V1_FLOOR_SIGN_GATE`、既定 1、`-D` で 0）、無パルスの ENCODER_FAULT のしきい値を `max(V1_STALL_DUTY 10, duty_max × V1_STALL_DUTY_RATIO_PERCENT 75 %)` に。`[limits]` との関係は §2.2: `duty_min ≤ kp × following_error_limit / 100 < 0.75 × duty_max` なら動けない軸は FOLLOWING_ERROR（基準が残る）。`duty_max_percent < 10` は受理されるが無パルス判定が無効（ファーム画面に警告）。

- 2026-11-01: `[game].max_gain` と `[[axis]]` の `max_vel_count_s` / `max_acc_count_s2` / `following_error_limit_count` / `overspeed_count_s` を `[limits]` へ（`schema_version` 1 → 2。旧ファイルは読み込みで失敗する）。SET_LIMITS（0x0D）/ LIMITS（0x88）、capabilities bit3、WS `limits`、`GET/POST /v1/limits`、status の `motion_limits` / `axes[].fw_limits`。`calibration_hash` は幾何だけに。
