# ゲーム開発者向け Tips（h3-control-server で座席を動かす）

対象: 座席（H3 改造機）に姿勢を送るゲーム・デモを書く人。[protocol.md](protocol.md)（USB の中身）は読まなくてよい。API の正確な形は [`control-server/DESIGN.md`](../../control-server/DESIGN.md) §7、最短の起動手順は [game-integration.md](game-integration.md)。この文書は「思いどおりに動かすにはどう送ればよいか」と「何をゲーム側に置かないか」だけを扱う。

数値は 2026-11-02 の bring-up リグの設定（`control-server/config/bringup.toml`）と実機記録（[runs/hw_20261102.md](runs/hw_20261102.md)）に基づく。操作者が設定を変えれば変わるので、動かす前に `GET /v1/status` の `motion_limits` で現在値を見る。

## 1. 契約の全体像

ゲームが送るのは **姿勢 3 つ（pitch / roll / yaw、各 −1..1）と gain（0..1）** だけ。ハッカソンの `h3_direct.py` と同じ値をそのまま送る。サーバーがそれを 3 本のモーターの位置に変える。

```text
ゲーム: pose(pitch, roll, yaw) + gain  ── WS /v1/motion ──▶  サーバー
   ① 向き補正      [game].swap_pitch_roll / invert_*（操作者の設定。ゲームは触らない）
   ② 混合          g = gain×511;  m1 = 512 + g(roll+pitch);  m2 = 512 + g(roll−pitch);  m3 = 512 + g·yaw
   ③ 飽和          仮想位置を 0..1023 に切る（virtual_saturated）
   ④ 校正          仮想 0 / 512 / 1023 → count（軸ごとの 3 点折れ線。中立 512 = 校正の中立 count）
   ⑤ ソフトリミット count を [soft_min, soft_max] に切る（soft_limited）
   ⑥ プランナー    10 ms ごとに max_vel / max_acc の範囲で目標へ近づく軌道を作る（velocity_limited / acceleration_limited）
   ⑦ ファーム      位置ループ（Duty 上限、不感帯）。追従誤差・過速度・ソフトリミット外で FAULT
```

①〜⑥はサーバー（`compat.rs` / `calibration.rs` / `planner.rs`）、⑦は CoreS3 ファーム[^src-path]。

| 決めるもの | 誰が | どこで |
| --- | --- | --- |
| 姿勢と gain（毎フレーム） | ゲーム | WS `pose` |
| 速度・加速度・Duty・gain の上限を **一時的に下げる** | ゲーム（任意） | WS `limits`（§3） |
| 上限の初期値、向き（swap / invert）、校正、ソフトリミット、START 関門の幅 | 操作者 | 設定ファイル `[limits]` `[game]` `[[axis]]`、`POST /v1/limits` |
| ARM / START / STOP / CENTER / FAULT の解除 | 操作者 | 管理 API（ゲームには権限がない） |
| 絶対上限（Duty 60 %、6000 count/s、40000 count/s²）、停止期限、保護 | ファーム＋サーバー | 変えられない |

目安の数（bring-up リグ、2026-11-02）:

- 機構の全ストローク ≈ 5500 count。ソフトリミット内で仮想 0→1023 に割り当てる範囲は約 5150 count（軸 1: 200〜5351）、つまり **仮想 1 ≈ 5 count**。
- 操作者の現在値: `max_gain` 1.0、`max_vel_count_s` 600、`max_acc_count_s2` 2000、`duty_max_percent` 20。実測 Duty 1 % ≈ 100〜130 count/s（無負荷）。
- `swap_pitch_roll = true`。この機体では混合式の「同方向」側が前後傾きなので、サーバーが入れ替える。**ゲームは pitch を前後、roll を左右の意味のまま送る**。

[^src-path]: 混合・飽和は `control-server/src/compat.rs`（`mix` / `saturate` / `orient`）、プランナーは `control-server/src/planner.rs`、制限値の clamp は `control-server/src/limits.rs`、START 関門と目標の経路（`pose_targets`）は `control-server/src/supervisor.rs`。

## 2. 全域を使いたい

- **gain 1.0 を送る。** 混合式は `gain × 511` が振れ幅なので、gain 1.0・pitch ±1 で仮想 1 / 1023 ＝ ソフトリミット内の端に届く。gain 0.5 では半分までしか動かない。「スティックを全開にしても端まで行かない」は、ほぼゲームが送っている gain が小さいせい（2026-11-02 の試験で、ジョイスティック画面の gain 既定が 0.1 だったのが原因だった）。
- **頭打ちは操作者の `max_gain`。** ゲームの gain が `max_gain` を超えると、その値に切られて pose の ack に `effective_gain` と `limits: ["gain_limited"]` が返る[^src-compat]。bring-up リグは 2026-11-02 に `max_gain = 1.0` になったので、いまは切られない。`POST /v1/session` の応答 `max_gain`、status の `motion_limits.effective.max_gain` で確認できる。
- **pitch と roll を同時に振ると飽和する。** m1 は `roll + pitch`、m2 は `roll − pitch` なので、gain 1.0 で |roll| + |pitch| > 1 になると仮想位置が 0..1023 を越えて切られ、status の `limits` に `virtual_saturated` が出る（pose の ack ではなく **status**。§3.3）。飽和しても止まらないし FAULT にもならないが、その軸は端で頭打ちになり、pitch と roll の比が崩れる。全域を使いつつ比を保ちたいなら、ゲーム側で `|roll| + |pitch| ≤ 1` に収める（例: 両方を `1 / max(1, |roll| + |pitch|)` 倍する）か、gain を 0.5 にする（それなら同時に振り切っても飽和しない）。yaw は独立で、gain 1.0・yaw ±1 で端。
- 姿勢そのものが −1..1 を越えた場合はクリップされ、ack に `pose_clamped` が付く。意図して越える必要はない。

[^src-compat]: `compat::check_pose`。範囲外の姿勢は `[game].clamp_out_of_range = true`（既定）でクリップ。gain が負なら 0 にされ `gain_negative`。

## 3. 速さと加速度を思いどおりにしたい

### 3.1 位置指令であることの帰結

ゲームが送るのは **位置**（姿勢）で、速度ではない。速さは「姿勢を時間でどう変えたか」で決まる。

- プランナーは 10 ms ごとに「いまの目標」へ、`max_vel_count_s`（600）と `max_acc_count_s2`（2000）の範囲で近づく軌道を作る。目標に着く前に減速し、目標を越えない[^src-planner]。
- **ステップ入力は常に最高速で動く。** 姿勢を 0 → 0.5 に一気に変えると、約 1290 count（gain 1.0）を 600 count/s で約 2.4 s かけて移動する。0.3 s（= 600 / 2000）で最高速に達し、終わりで 0.3 s かけて止まる。速度の大小を姿勢の大きさで表現することはできない。
- **ゆっくり変える姿勢は、少し遅れて追従する。** プランナーは毎 tick「目標で止まれる速度」`sqrt(2·max_acc·遅れ)` で走るので、目標が速度 v で動いているとき、定常では `v² / (2·max_acc)` だけ後ろを付いてくる。600 / 2000 なら最高速で約 90 count（時間にして約 0.15 s）、450 count/s で約 50 count、200 count/s で約 10 count。これはサーバー側の決まった遅れで、ファームの追従誤差（新ファームで平均 2〜4 count、最大 12〜57 count。旧ファーム平均 14〜16、`runs/hw_20261102.md`）とは別に乗る。音や映像と同期させるなら、この 0.1〜0.15 s を見込む。
- **3 軸は同じ時間軸で動く。** 速度制限は 3 軸に同じ倍率を掛ける（軸空間で方向を保つ）。1 軸が飽和すると他の 2 軸も同じ割合で遅くなる。大きな pitch 移動の最中に yaw を足しても、yaw はその分遅れる。

目安（gain 1.0、片軸だけ動かす）:

| 姿勢の変化 | count | 600 / 2000 での所要時間 |
| ---: | ---: | --- |
| 0.02 | ≈ 50 | 0.3 s（加速度が支配。最高速に届かない） |
| 0.1 | ≈ 260 | 0.7 s |
| 0.2 | ≈ 510 | 1.2 s |
| 0.5 | ≈ 1290 | 2.4 s |
| 1.0（中立 → 端） | ≈ 2570 | 4.6 s |
| 2.0（端 → 端） | ≈ 5150 | 8.9 s |

### 3.2 加速度を「リニアに」反映したいとき

やり方は 2 つ。向き不向きが違う。

**A. ゲーム側で姿勢を時間で滑らかにする（推奨）。** ゲームが毎フレーム補間した姿勢を送り、プランナーの制限に当てない。サーバーは §3.1 の小さな遅れで付いてくるだけになり、演出の速さ・緩急はゲームが決められる。

```python
# 一次遅れ（指数平滑）。tau が小さいほど速い。dt はフレーム時間
alpha = 1.0 - math.exp(-dt / tau)
cur = cur + (goal - cur) * alpha
client.submit_pose(cur.pitch, cur.roll, cur.yaw, gain=1.0, frame_seq=n)

# S 字（smoothstep）で t0 から T 秒かけて a → b へ
u = min(1.0, (now - t0) / T); s = u * u * (3 - 2 * u)
val = a + (b - a) * s
```

目安: 中立 → 端（≈ 2570 count）を S 字で動かすとき、ピーク速度 ≈ 1.5 × 2570 / T、ピーク加速度 ≈ 6 × 2570 / T²。600 / 2000 に収めるなら T ≥ 6.4 s（速度側が効く。加速度側は T ≥ 2.8 s）。**速度制限に当たれば演出の意図は失われる**ので、`limits` 配列（§3.3）を見ながら T を決める。

- 向く: 演出として緩急を付けたい、シーンごとに速度を変えたい、複数軸のタイミングを合わせたい。
- 向かない: 入力がもともと滑らか（物理シミュレーションの出力など）で、ただ安全な上限が欲しいだけ。

**B. `limits` でサーバーに丸めさせる。** 同じ WS に `{"type":"limits", ...}` を送って `max_vel_count_s` / `max_acc_count_s2` を下げると、以後のステップ入力もその速さ・加速度で動く[^src-limits]。

```json
{"type":"limits","protocol":1,"source_session":"...","seq":12,"max_vel_count_s":300,"max_acc_count_s2":800}
```

- 向く: 「このシーンは全体にゆっくり」「コックピット演出では全体の速度を半分に」のように、シーン単位で一律に抑えたい。姿勢の計算を変えずに済む。
- 向かない: 動きごとに違う速さを出したい（制限は 3 軸共通・全体に効く）。速度と加速度の上限は下げても、加速の立ち上がり方は台形のまま。
- 注意:
  - 値はサーバーが **操作者の `[limits]` の値**（status の `motion_limits.operator_max`。理由 `operator_limited`）と **ファームの絶対上限・`overspeed / 1.5`・gain 1.0**（`ceiling_clamped`）で切る。**ゲーム経路（WS `limits`）は操作者の値より上には上げられない**（2026-11-02 Issue #15 で強制に変更）。だからゲームは `limits` を **下げる用途にだけ使う**。上げたいときは操作者に `POST /v1/limits`（Bearer、ファームの上限まで）か設定変更を頼む（§6）。
  - **停止時間の規則**: 加速度を下げ過ぎると STOP で止まり切れなくなるので、サーバーが `max_acc ≥ max_vel / 1.2 s` になるまで `max_acc` を上げる（または `max_vel` を下げる）。ack の `limits` に `stop_time_limited`、`effective_limits` に実際の値が返る。例: `max_vel 600` に `max_acc 1` を要求 → 500 になる。
  - `duty_max_percent` は触らない（§6）。Duty を変える要求は全軸のファームが受理するまで `motion_limits.pending = true` のまま効かない。速度・加速度・gain は即時。
  - `limits` は鮮度を延ばさない（pose を送り続けること）。`POST /v1/session` のたびに初期値へ戻るので、再接続後は送り直す（Python クライアントは `submit_limits(...)` を pose より先に送る）。

### 3.3 どこで丸められたかを知る

「制限の理由」は 3 か所に分かれて出る。混同しやすい。

| 見る場所 | 入る理由 | 意味 |
| --- | --- | --- |
| pose の ack `limits` | `pose_clamped` / `gain_limited` / `gain_negative` | 受け付け時の検査。姿勢が −1..1 外、gain が `max_gain` 超 |
| **status の `limits`**（WS の `status`、`GET /v1/status`） | 上に加えて `virtual_saturated` / `soft_limited` / `velocity_limited` / `acceleration_limited` / `tracking_governed` / `governor_no_progress` | 直近 tick で混合・校正・プランナーが実際に丸めたもの。**速度・加速度の頭打ちはここにしか出ない**。`tracking_governed` は追従ガバナ（[limits.md](limits.md) §1.3）。`governor_no_progress` はその原因軸が止まって coordinated stop したとき |
| `limits` メッセージの ack `limits` | `operator_limited` / `ceiling_clamped` / `stop_time_limited` | 要求した制限値がサーバーで変えられた（操作者の値で / ファームの上限で / 停止時間の規則で）。`effective_limits` に結果 |

the server may slow all axes together when one can't keep up; status limits shows tracking_governed

status は `server.status_period_ms`（bring-up 100 ms）ごとに WS に届く。演出を作るときは `limits` に **`velocity_limited` が出ない**ことを確かめる（出ていれば、その瞬間は意図した速さではなくサーバーの上限 600 count/s で動いている。信頼できる印）。`acceleration_limited` は読み方に注意: 30〜50 Hz で送る姿勢は 100 Hz の tick から見ると階段で、段の縁ごとに目標速度が `max_acc × 送信間隔`（20〜60 count/s）跳ぶのに対し 1 tick の加速度の余地は `max_acc × 0.01` = 20 count/s なので、**制限の内側でもフレームごとに点滅する**。点滅は正常、**連続して立ち続ける**ならランプが急すぎる。`virtual_saturated` / `soft_limited` が出たら姿勢が端に当たっている。

[^src-planner]: `planner::step_track`。目標で止まれる速度 `sqrt(2·a·|err|)` 以下に切り、3 軸共通の倍率で `max_vel` に収め、軸ごとに `|Δv| ≤ a·dt` で加速度を切る。`dt` は 10 ms。
[^src-limits]: `limits::clamp`（WS の ack、`GET/POST /v1/limits`、制御ループが同じ関数を使う）。停止期限は `limits::stop_deadline` = 0.8 × min(`mcu_stop_timeout_ms` 1500, `stop_settle_timeout_ms` 3000) = 1.2 s。詳細は [limits.md](limits.md) §1.1。

## 4. 揺れ・振動・衝撃などの演出

正弦波（振幅 A count、周波数 f）のピーク速度は `2πfA`、ピーク加速度は `(2πf)²A`。600 count/s と 2000 count/s² から、振幅ごとに出せる最大周波数は:

| 振幅 A（count） | 姿勢の振幅（gain 1.0、片軸） | 速度側の上限 f | 加速度側の上限 f | **使える f** |
| ---: | ---: | ---: | ---: | ---: |
| 20 | 0.008 | 4.8 Hz | 1.6 Hz | （不感帯に飲まれる。下記） |
| 50 | 0.02 | 1.9 Hz | 1.0 Hz | ≈ 1.0 Hz |
| 100 | 0.04 | 0.95 Hz | 0.71 Hz | ≈ 0.7 Hz |
| 180 | 0.07 | 0.53 Hz | 0.53 Hz | ≈ 0.5 Hz（速度と加速度の境目） |
| 500 | 0.2 | 0.19 Hz | 0.32 Hz | ≈ 0.19 Hz |
| 1000 | 0.4 | 0.095 Hz | 0.22 Hz | ≈ 0.1 Hz |
| 2500 | 1.0 | 0.038 Hz | 0.14 Hz | ≈ 0.04 Hz（周期 26 s） |

- **「速く小さく」は加速度で、「大きく」は速度で頭打ちになる。** 振幅 180 count（姿勢 0.07）より小さい揺れは加速度 2000 が、大きい揺れは速度 600 が上限を決める。2026-11-02 の試験（pitch ±0.4 @0.07 Hz、roll ±0.4 @0.10 Hz、gain 0.3。1 軸あたり約 600 count の振幅、ピーク加速度は 200 count/s² 程度）で `acceleration_limited` が周期的に立っていたのは、この表の上限に当たったのではなく §3.3 の階段による点滅。
- **1 Hz を超える振動は出ない。** 600 / 2000 のリグでは、振幅を何 count にしても 1 Hz 前後が上限。「ガタガタ」「ブルブル」は周波数ではなく、0.5〜1 Hz の揺れに振幅や位相のばらつきを付けて表現する。
- **小振幅は不感帯に飲まれる。** ファームの不感帯は入る側 10 count、出る側 20 count（`[limits].deadband_count` / `deadband_out_count`）。振幅 20 count（姿勢 0.008）以下の揺れは、目標が不感帯の中を往復するだけでモーターが動かないことがある。最低でも振幅 50 count（姿勢 0.02）を目安にする。
- **低速は段付きになる（現状）。** 位置ループの最小 Duty は 9 %（`duty_min_percent`。自重のかかる軸が動き出す値）で、無負荷換算で約 900〜1200 count/s、自重のかかった軸の実測で 100〜240 count/s。これより遅い目標速度では「9 % で動く / 0 で止まる」の繰り返しになり、ゆっくりした揺れがなめらかに出ないことがある。新ファーム（PR #19）で往復は 1/8 に減ったが残っている。遅い揺れは、速度が遅くなり過ぎない振幅・周波数の組み合わせを選ぶ。
- **衝撃（ステップ）は「最高速で動いて止まる」にしかならない。** 0 → 姿勢 0.1 → 0 のパルスは 260 count を片道 0.7 s（往復 1.5 s）で動く。もっと短く感じさせたいなら振幅を小さくする（50 count なら片道 0.3 s）。加速度 2000 を超える「ドン」は出ない。
- 揺れの合成は、まずゲーム側で姿勢を合成してから送る（1 本の WS、1 つの姿勢）。揺れの大きさは姿勢の振幅で表し、gain を揺らして作らない（§6）。

## 5. 開始・停止・中立

### 5.1 つなぐ

1. `POST /v1/session` → `{"source_session":"...","protocol":1,"max_gain":1.0,"source_freshness_ms":250,"require_frame_seq":true}`。
2. `WS /v1/motion?session=<source_session>` を開き、毎フレーム `pose` を送る。
3. 座席は動かない。**ARM と START は操作者**（テストページか `curl`）。ゲームにできるのは「新鮮な中立姿勢を送って待つ」ことだけ。

```json
{"type":"pose","protocol":1,"source_session":"...","seq":1,"frame_seq":1,"source_age_ms":0,
 "pitch":0.0,"roll":0.0,"yaw":0.0,"gain":1.0}
```

Python なら `clients/python/h3_motion_client.py` の `MotionClient` が session 取得・WS・再接続をやる（`submit_pose(pitch, roll, yaw, gain, frame_seq=n)`、`submit_limits(...)`、`status()`）。

### 5.2 START は中立から

操作者の `POST /v1/start` は、ゲームの最新姿勢が作る目標が **保持位置から 300 count 以内**（`[game].start_max_offset_count`）でなければ 409 `pose_not_near_hold` で拒否される[^src-start]。gain 1.0 では姿勢 0.12 でもう 300 count なので、**START 待ちの間は pitch / roll / yaw = 0 を送る**。ARM 直後の保持位置は校正の中立とは限らない（ブレーキが掛かった位置のまま）ので、操作者が `POST /v1/center` で中立へ動かしてから START するのが普通の流れ。ゲームは「START されたら動かす」ではなく「中立を送り続けていたら、ある時点から座席が動き出す」と考える。新鮮な姿勢が無ければ 409 `no_fresh_pose`。

### 5.3 鮮度（止まらずに動かし続ける条件）

- `frame_seq` が**ゲームのフレームごとに進む**ことがゲームの生存の証明。同じ `frame_seq` の再送、`{"type":"gain"}`、`limits`、`GET /v1/status` の poll では鮮度は延びない[^src-session]。
- 鮮度は `source_age_ms` を差し引いた時刻から 250 ms（`source_freshness_ms`）。`source_age_ms` は「その姿勢を作ってからの経過 ms」を正直に（通常 0〜数 ms）。大きく盛ると止まりやすくなるだけ。
- 送信は **50 Hz を推奨**（最低でも 250 ms に数回。30 Hz でも動く）。描画と送信を同じループに置けば、描画が止まったとき自動的に座席も止まる。
- **タブが裏に回ると止まる。** ブラウザーの `requestAnimationFrame` は隠れたタブで止まる → `frame_seq` が進まない → 250 ms で STOPPING。これは仕様（`document.hidden` のあいだ、わざと中立を送って動かし続けない）。
- **再開は操作者。** 一度止まった後に `frame_seq` が進み出しても、座席は動かない。操作者が ARM → START をやり直す。

### 5.4 止まり方と、ゲームが見るもの

止まる引き金はすべて同じ経路に入る: **STOPPING**（有効な `max_acc` で減速。600 count/s からなら 0.3 s）→ 全軸 HOLDING かつ実測で静止（`rest_*`、200 ms）→ 1 s 保持 → ブレーキ → **BRAKED_DISARMED**。中立へは戻らない。その場でブレーキ。

| 引き金 | ゲームから見える印 |
| --- | --- |
| `frame_seq` が 250 ms 進まない / WS が切れた | status の `session.fresh: false`、`rig_state: STOPPING` → `BRAKED_DISARMED`、イベント `stopping {"reason": 2}` |
| 操作者の `POST /v1/stop`（トークン不要。誰でも呼べる） | 同上、`reason: 1` |
| 別のクライアントが `POST /v1/session` した（再読み込み、2 つ目の画面、bridge 再起動） | 自分の ack が `stale_session` になる。走行中なら停止 |
| 操作者の `/v1/center` が中立に着いた | `ACTIVE`（`active_source: "center"`）→ `HOLDING`。姿勢は使われない |
| FAULT（追従誤差、過速度、軸の切断、USB 無応答など） | `rig_state: FAULT_LATCHED`、`fault: {"kind": "...", ...}`。即ブレーキ（減速しない） |

ゲームの画面に出すとよいもの（`GET /v1/status` か WS の `status`）:

- `rig_state`（`ACTIVE` のときだけ座席が姿勢に追従している）と `active_source`（`"game"` / `"center"` / null）。
- `session.fresh`（自分の入力が生きているか）、`session.source_session_prefix`（自分の session の先頭 8 桁と比べて、取られていないか）。
- `effective_gain`、`limits`（§3.3）、`motion_limits.effective`。
- `fault` と `blockers`（`"<code>: <detail>"`。`:` の前の code で判定）。FAULT の解除と再 ARM は操作者なので、ゲームは「止まっています。操作者を呼んでください」が出せれば十分。

### 5.5 再接続とセッション

- セッションは **最後に発行した者が所有**。前の session は即無効（`stale_session`）。駆動は START 時の session に束縛されるので、走行中に自分が再接続（新 session）しても座席は止まる。再接続は「通信の回復」であって「運転の再開」ではない。
- 再接続後は `limits` を送り直す（新しい session は `[limits]` の初期値から始まる）。
- サーバーが再起動したら旧 session は全部無効。ゲームは `POST /v1/session` からやり直す。座席は再起動で動かない（自動 ARM / START / 中央復帰はしない）。
- 2 つのゲーム・画面を同時に動かさない。後から来た方が session を取り、先の方は `stale_session` になる（bridge は `/state` の `session_lost: true` で示す）。

[^src-start]: `supervisor.rs` の `Start` 処理: 最新の新鮮な姿勢を追従と同じ経路（orient → mix → saturate → 校正 → ソフトリミット）で count にし、保持位置（プランナーの位置。未初期化なら実位置）との差が全軸 `start_max_offset_count` 以内かを見る。blockers に `pose_not_near_hold: axis1 target 2000 is +2000 counts from hold 0 (max 300)` の形で軸ごとの距離が入る。
[^src-session]: `session::PoseIntake::accept_pose`。`fresh_until = (受信時刻 − source_age_ms) + source_freshness_ms`。`frame_seq` が前回と同じなら値は受理するが `fresh_until` は前のまま。小さければ `old_frame` で拒否。

## 6. やってはいけないこと／ゲーム側に置かないこと

| やらない | 理由・代わりに |
| --- | --- |
| **安全のゲーティングをゲームに置く**（ゲームが ARM / START / STOP を叩く、動いてよいかをゲームが判断する） | 駆動権限は管理 API（操作者）にある。ゲームにトークンを持たせない。止めたいときだけ `POST /v1/stop`（トークン不要）を呼んでよい |
| **`frame_seq` を偽装する**（タイマーで進める、隠れたタブで進め続ける、固定値を送る） | 鮮度は「ゲームが生きている証明」。描画ループが進んだときだけ +1。`allow_missing_frame_seq = true` に頼らない（有人では使わない） |
| **向きの補正をゲームに置く**（この機体は pitch と roll が逆だから入れ替えて送る、符号を反転して送る） | 向きは `[game].swap_pitch_roll` / `invert_*` で操作者が機体ごとに設定する。ゲームが補正すると、別の機体や設定変更で二重に反転する。ゲームは pitch = 前後、roll = 左右の意味のまま送る |
| **混合式を自分で計算して m1 / m2 / m3 を送る** | API は姿勢と gain だけ。混合はサーバー |
| **`limits` で操作者の値より上げようとする**（`max_vel` 600 → 1333、`duty_max` 20 → 60 など） | サーバーが操作者の値で切って `operator_limited` を返す（上がらない）。`[limits]` の値は操作者が機体の状態を見て決めた上限。上げるのは操作者が `POST /v1/limits` か設定で。ゲームは **下げる用途だけ** |
| **`duty_max_percent` を触る** | ファームの出力上限。モーターと機構の保護側の値で、操作者が実機を見ながら上げるもの |
| **ソフトリミット・校正値・中立位置を知っている前提で姿勢を計算する** | count はゲームに見えない単位。姿勢 −1..1 と gain だけで書き、端は `virtual_saturated` / `soft_limited` で知る |
| **gain で範囲を表現する**（「このシーンは小さく動かしたいから gain 0.3」） | 範囲は姿勢の振幅で表現する（gain 1.0 固定で pitch ±0.3）。gain は h3game 互換の「全体の効き」。pose の中で毎フレーム変えてよい（`{"type":"gain"}` 単独では鮮度が延びない）が、範囲表現に混ぜると飽和の条件（§2）が読みにくくなる |
| **止まった後に自動で再接続して動かそうとする** | 再接続しても ARM / START は操作者。ゲームは中立を送って待つ |
| **止める代わりに中立 (0,0,0) を送り続ける** | 止めたいなら `frame_seq` を止める（鮮度切れ → 減速 → ブレーキ）か `POST /v1/stop`。中立を送ると「中立へ最高速で動く」になる |

## 7. チェックリストと、よくある症状

### 7.1 リリース前に試すこと（模擬軸 `config/sim.toml` で全部できる）

1. `POST /v1/session` → WS → 中立を 50 Hz で送り、status の `session.fresh` が `true` になる。
2. 操作者が ARM → CENTER → START できる（409 が出ない）。START 待ちの姿勢が中立になっている。
3. gain 1.0・pitch ±1 で status の `virtual_targets` が 1 / 1023 付近に届く。pitch と roll 同時に ±1 で `virtual_saturated` が出ることを知っている（出てよいか判断済み）。
4. 一番速い演出で status の `limits` に `velocity_limited` が出ないこと、`acceleration_limited` が立ち続けないこと（出るなら意図した速さになっていない。フレームごとの点滅は正常、§3.3）。
5. タブを裏に回す・ゲームを一時停止する → 250 ms 以内に `STOPPING` → `BRAKED_DISARMED`。中立へ戻らない。
6. 画面を再読み込み → 旧 session が `stale_session`、座席は止まる、再開には操作者の ARM + START が要ることを画面に出せる。
7. `limits` を送っているなら、ack の `effective_limits` を表示し、`operator_limited` / `ceiling_clamped` / `stop_time_limited` が出たら要求値と違うことが分かる。再接続後に送り直している。
8. `rig_state` / `fault` / `blockers` を画面に出し、FAULT のとき「操作者を呼ぶ」導線がある。
9. 2 つ目の画面・bridge を同時に開いたとき、先の方が `stale_session` になることを知っている。
10. 実機では最初は操作者に `max_gain` を 0.3 程度に下げてもらい（`POST /v1/limits`）、向きと速さを目で確認してから 1.0 に戻す。

### 7.2 症状 → 原因

| 症状 | まず疑うもの | 見る場所 |
| --- | --- | --- |
| スティック全開でも端まで行かない | 送っている gain が小さい（例: 0.1）。次に操作者の `max_gain` | ack の `effective_gain` と `gain_limited`、status の `virtual_targets`（端なら 1 / 1023 付近） |
| 動きが鈍い・遅れる | `max_vel_count_s` / `max_acc_count_s2`（600 / 2000）に当たっている。遅れだけなら §3.1 の定常遅れ（最高速で約 90 count、0.15 s） | status の `limits` に `velocity_limited`（`acceleration_limited` は連続して立つか）、`motion_limits.effective` |
| 速くしたのに速度が変わらない | 位置指令なので姿勢の変化率が速度。ステップは常に最高速 | §3.1 |
| pitch と roll を同時に倒すと片方が効かない | 仮想位置の飽和（\|roll\| + \|pitch\| > 1 で gain 1.0） | status の `limits` に `virtual_saturated` |
| START が 409 `pose_not_near_hold` | START 時の姿勢が中立から 300 count 超（gain 1.0 なら姿勢 0.12 超） | 409 の `blockers` に軸ごとの距離 |
| START が 409 `no_fresh_pose` | 姿勢を送っていない、`frame_seq` が進んでいない、`missing_frame_seq` で拒否されている | ack の `reason`、status の `session.fresh` |
| すぐ止まる（数百 ms で STOPPING） | `frame_seq` が進んでいない（同じ値の再送、隠れたタブ、送信が 250 ms 空く）。`source_age_ms` を盛っている | status の `session.fresh`、`session.last_frame_seq` が増えるか、イベント `stopping reason 2` |
| ack が `stale_session` | 別のクライアントが session を取った、サーバーが再起動した、自分が再接続した | `session.source_session_prefix` と自分の id の先頭 8 桁 |
| ack が `old_seq` / `old_frame` | `seq` / `frame_seq` が巻き戻った（画面の再読み込みで 0 から数え直したが session を取り直していない） | 新しい session を取る（bridge は owner が変わると取り直す） |
| ack が `axis_conflict` | 2 台の CoreS3 が同じ系統を選んでいる（機材側の問題） | status の `axis_conflict`。操作者に伝える。送り続けてよい |
| 前後と左右が逆・符号が逆 | 操作者の `[game].swap_pitch_roll` / `invert_*`。ゲームで直さない | 操作者に伝える（hardware-bringup.md §7 の向きの確認） |
| 小さい揺れが出ない | 不感帯 10 / 20 count。振幅が 20 count（姿勢 0.008）以下 | 振幅を 50 count（0.02）以上に |
| ゆっくりの動きがカクつく | 最小 Duty 9 % の段付き（現状の機構・ファーム） | 操作者に伝える（`duty_min_percent`、limits.md §5） |
| `limits` を送ったのに `effective_limits` が違う | 絶対上限（`ceiling_clamped`）か停止時間の規則（`stop_time_limited`） | ack の `limits`、status の `motion_limits.ceiling` |
| `limits` を送ったのに効かない | Duty を変えた要求は全軸 ACK まで保留。または再接続で初期値に戻った | status の `motion_limits.pending`、`motion_limits.requested` |
| 座席が FAULT で止まった | 追従誤差・過速度・USB 無応答・軸の異常。ゲームの入力では直らない | status の `fault`、`fault_history`（最新の FAULT の最初の記録の `prehistory` に直前 2 s の全軸の実測・目標・追従誤差・Duty）。操作者が `fault/reset` → ARM → START |

質問・変更要望は Issue へ。WS の JSON の形・`protocol = 1`・拒否理由の文字列・`frame_seq` の意味は相談なく変えない（[game-integration.md](game-integration.md) §4）。
