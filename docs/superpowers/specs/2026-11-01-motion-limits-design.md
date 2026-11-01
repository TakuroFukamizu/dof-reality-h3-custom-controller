# 動きの制限値を 2 段にまとめ、実行時にゲームから変えられるようにする（設計）

日付: 2026-11-01。対象: `control-server`（PC）、`moter-controler` の v1 ファーム（CoreS3）、`clients/`、`docs/control-server/`。

## 0. 目的と背景

実機でゲームを動かしたとき、速度と可動範囲が元の DOF Reality の半分以下で、動きの解像度が低くかくついた。原因の候補は 3 つ。

1. PC 側の速度上限 600 count/s（1.5 回転/s）が低く、ゲームの動きに追いつけない。
2. ファームの Duty 下限 7 % が効いている。7 % は無負荷で約 700 count/s 出るので、それより遅い目標速度は「7 % で動く / 0 で止まる」の往復になり、低速が段付きになる（FF + P の位置ループで下限が限界サイクルを作る）。
3. Kp・フィードフォワード係数・不感帯・Duty 上下限がファームに直書きで、焼き直さないと調整できない。

加えて、動きを制限する値が `[game]`・`[[axis]]`・ファームの `config.h` に散らばっていて、「いまどこで何に制限されているか」をデバッグで追いにくい。

この設計で決めること:

- 制限値を **2 段** にする。ファームが持つ絶対上限（コンパイル時定数）と、PC 側で 3 軸に共有される 1 組の現在値。それ以外の段は作らない。
- PC 側の現在値は設定ファイルの `[limits]` を初期値にし、実行時にゲーム（WS）と管理者（REST）から変えられる。変更はファームの絶対上限で頭打ちにする。
- Duty の上下限・Kp・FF・不感帯もファームの直書きをやめ、PC の設定ファイルから配る。
- 初期値を見直し、安全余裕を残して緩める。**数値はどれも実機未検証の見積もり**で、実機で低い値から上げる手順を文書に残す。

## 1. 2 段の制限

```text
ゲーム / 管理者 ─(実行時の要求)─▶ PC の現在値（3 軸共有、初期値は [limits]）─▶ clamp(ファームの絶対上限)
                                                                              │
                                   SET_POSITION（毎フレーム max_vel / max_acc）◀┘
                                   SET_LIMITS（duty_max などが変わったとき）
                                                                              │
                                      ファーム: 受けた値を絶対上限で黙ってクランプ ◀┘
```

| 段 | 置き場所 | 変え方 | 内容 |
| --- | --- | --- | --- |
| 1. 絶対上限 | ファーム `config.h` の定数。HELLO の後に PC へ申告（LIMITS メッセージ） | 焼き直し | Duty 上限 60 %、速度 6000 count/s、加速度 40000 count/s² |
| 2. 現在値 | PC の `[limits]`（初期値）。起動後は共有メモリ 1 組 | 設定ファイル（初期値）、`WS /v1/motion` の `limits`、`POST /v1/limits` | §3 の表 |

守ること:

- 段 2 の現在値は、段 1 を超える要求を受けても段 1 でクランプして「有効値と理由」を返す。拒否しない（gain と同じ流儀）。
- CONFIG の `max_vel_count_s` / `max_acc_count_s2` には **段 1 の値**（ファームの申告値。未申告なら PC 内蔵の同じ定数）を入れ、毎フレームの SET_POSITION の値で実際の上限を決める。CONFIG の値を「もう 1 つの上限」にしない。
- ソフトリミット（`soft_min/max_count`、`decel_zone_count`）と校正 3 点は幾何で、制限値ではない。今までどおり `[[axis]]` に置き、実行時には変えない。
- 追従誤差・過速度の判定値（FOLLOWING_ERROR / OVERSPEED）は保護で、演出の調整対象ではない。`[limits]` に置くが実行時には変えない。

## 2. 設定ファイル（schema_version 2）

`[game]` から `max_gain` を外し、`[[axis]]` から `max_vel_count_s` / `max_acc_count_s2` / `following_error_limit_count` / `overspeed_count_s` を外し、新しい `[limits]` にまとめる。`schema_version = 2`。旧ファイルは `deny_unknown_fields` と版の検査で読み込み時にはっきり失敗する（黙って既定値にしない）。

```toml
schema_version = 2

[limits]
# 実行時に変えられる（WS limits / POST /v1/limits。POST /v1/session で初期値に戻る）
max_gain = 0.5                  # 上限 1.0
max_vel_count_s = 3000          # 上限 min(ファーム絶対上限, overspeed_count_s / 1.5)
max_acc_count_s2 = 15000        # 上限 ファーム絶対上限
duty_max_percent = 40           # 上限 ファーム絶対上限
# 初期値だけ（ファームへ SET_LIMITS で配る。実行時には変えない）
duty_min_percent = 3
kp_percent_per_100 = 2
ff_count_s_per_percent = 100
deadband_count = 10
deadband_out_count = 20
# 保護（CONFIG で配る。実行時には変えない）
following_error_limit_count = 800
overspeed_count_s = 9000
```

読み込み時の検査（`Config::validate`）:

- `0 < max_gain ≤ 1`、`max_vel_count_s > 0`、`max_acc_count_s2 > 0`、`1 ≤ duty_max_percent ≤ 100`、`duty_min_percent ≤ duty_max_percent`、`ff_count_s_per_percent > 0`、`deadband_out_count ≥ deadband_count`、`following_error_limit_count > 0`。
- `overspeed_count_s ≥ 1.5 × max_vel_count_s`（bringup.toml のコメントにある「max_vel の 2 倍では過渡で止まった」実測に基づく余裕。実行時に max_vel を上げるときも同じ比で頭打ちにする）。
- `kp_percent_per_100 ≤ 255`、`ff / deadband / deadband_out ≤ 65535`、`duty_* ≤ 255`（SET_LIMITS の幅）。

`calibration_hash` は幾何（3 点・機械端・ソフトリミット・減速帯）だけから計算する（外した 4 項目は含めない）。`calibration_export` が書き換える `[[axis]]` のキーは変わらない。

## 3. 初期値と根拠

| 項目 | 旧 | 新 | 根拠 / 注意 |
| --- | ---: | ---: | --- |
| `max_gain` | 0.15（bringup）/ 0.35 | 0.5 | 混合式 `m = 512 + gain·511·(roll ± pitch)` で、pitch と roll を同時に振り切っても 0.5 以下なら仮想 0〜1023 で飽和しない。旧 h3game の既定 0.35 は検証値ではなかった |
| `max_vel_count_s` | 600 | 3000 | 7.5 回転/s。ストローク 5500 count を加減速込みで約 2.3 s。Duty 1 % ≈ 100〜130 count/s（2026-10-31 実測）から FF だけで約 30 % |
| `max_acc_count_s2` | 2000 | 15000 | 0 → 3000 count/s に 0.2 s。旧値では 1.5 s かかり、30 Hz の目標変化に追いつかなかった |
| `duty_max_percent` | 20（直書き） | 40 | 外挿で約 4000〜5000 count/s。max_vel 3000 に対して FF + P の余裕を残す |
| `duty_min_percent` | 7（直書き） | 3 | 限界サイクルを小さくする仮説。7 % は自動移動（端当て・中立）の実測値で、そちら（`NEUTRAL_DUTY_MIN` 等）は変えない |
| `kp_percent_per_100` | 2 | 2 | 据え置き |
| `ff_count_s_per_percent` | 100 | 100 | 据え置き（実測の下側） |
| `deadband_count` / `_out` | 10 / 20 | 10 / 20 | 据え置き |
| `following_error_limit_count` | 400 | 800 | 速度を上げると追従遅れも増える。max_vel 3000 で 100 ms 遅れると 300 count |
| `overspeed_count_s` | 2000 | 9000 | 1.5 × 絶対上限 6000。暴走検出として残す |
| ファーム絶対上限 Duty | なし | 60 % | MD13S と 24 V モーターの余裕側。100 % は使わない |
| ファーム絶対上限 速度 | なし | 6000 count/s | 15 回転/s。Duty 60 % の外挿（6000〜7800）より下 |
| ファーム絶対上限 加速度 | なし | 40000 count/s² | 0 → 6000 に 0.15 s |

実機での上げ方は `docs/control-server/limits.md` に書く（低い値から、1 軸ずつ、物理非常停止に手が届く状態で）。どの値がどの症状向けかも書く: 速度上限と加速度上限は「追いつかない」、Duty 下限は「低速の段付き」、Duty 上限は「高速側の頭打ち」。

## 4. USB プロトコルの追加（v1 のまま、種別を足す）

protocol.md §2 の規則「既存種別の長さは変えない。新しい `msg_type` を足す」に従う。版は 1 のまま。

### 4.1 HELLO `capabilities` bit3 = `LIMITS`（0x08）

SET_LIMITS / LIMITS に対応するファーム。PC は立っていない軸を `firmware_limits_unsupported: axisN` で ARM 阻害にする（黙って旧動作にしない）。

### 4.2 SET_LIMITS（`0x0D`、PC → MCU、16 byte、ACK あり）

| オフセット | 型 | 名前 | 内容 |
| ---: | --- | --- | --- |
| 0 | u32 | request_id | |
| 4 | u8 | duty_max_percent | 0 は INVALID_PARAM。それ以外は絶対上限でクランプ（100 超も拒否せずクランプ。PC の API 側で 1〜100 に検査する） |
| 5 | u8 | duty_min_percent | 0〜duty_max |
| 6 | u8 | kp_percent_per_100 | |
| 7 | u8 | reserved | 0 |
| 8 | u16 | ff_count_s_per_percent | > 0 |
| 10 | u16 | deadband_count | |
| 12 | u16 | deadband_out_count | ≥ deadband_count |
| 14 | u16 | reserved | 0 |

- 管理命令の共通規則（request_id 0 / DUPLICATE / FAULT_ACTIVE）に従う。状態は問わない（ACTIVE 中も受理）。
- `duty_max_percent = 0`、`duty_min > duty_max`、`ff = 0`、`deadband_out < deadband` は `REJECTED/INVALID_PARAM`。
- 受理したら V1Tuning の該当項目を置き換え（絶対上限でクランプ）、ACK の後に LIMITS を 1 回送る。
- 受理前（起動直後）の値は従来どおり `config.h` の既定（Duty 7〜20 % など）。PC は識別直後の CONFIG（Sync）と一緒に SET_LIMITS を送る。

### 4.3 LIMITS（`0x88`、MCU → PC、24 byte）

| オフセット | 型 | 名前 | 内容 |
| ---: | --- | --- | --- |
| 0 | u32 | boot_id | |
| 4 | u8 | duty_max_percent | 有効値（クランプ後） |
| 5 | u8 | duty_min_percent | |
| 6 | u8 | kp_percent_per_100 | |
| 7 | u8 | abs_duty_max_percent | 絶対上限（60） |
| 8 | u16 | ff_count_s_per_percent | |
| 10 | u16 | deadband_count | |
| 12 | u16 | deadband_out_count | |
| 14 | u16 | reserved | 0 |
| 16 | u32 | abs_max_vel_count_s | 絶対上限（6000） |
| 20 | u32 | abs_max_acc_count_s2 | 絶対上限（40000） |

送るのは SET_LIMITS を受理した直後だけ。

### 4.4 既存メッセージの扱いの変更

- CONFIG の `max_vel_count_s` / `max_acc_count_s2` は絶対上限でクランプして保存する（拒否しない。§4 の「ハードウェア上限などで値を拒否しない」は「拒否はしないがクランプはする」に改める）。
- SET_POSITION の `max_vel` / `max_acc` は今までどおり CONFIG の値で頭打ち（CONFIG が絶対上限なので、結果として絶対上限）。
- STOPPING の減速は、CONFIG の `max_acc_count_s2` ではなく **最後に適用したフレームの `max_acc`**（`frame_max_acc_`）を使う。CONFIG に絶対上限が入るため、従来のままだと停止が 40000 count/s² の急減速になる。

### 4.5 テストベクタ

protocol.md §7 に SET_LIMITS / LIMITS を足す（Rust の `tests/protocol_vectors.rs` が生成し、`doc_matches_vectors` で一致を確認）。値:

- SET_LIMITS: `request_id 13, duty_max 40, duty_min 3, kp 2, ff 100, deadband 10, deadband_out 20`
- LIMITS: `boot_id 0x89ABCDEF, duty_max 40, duty_min 3, kp 2, abs_duty_max 60, ff 100, deadband 10, deadband_out 20, abs_max_vel 6000, abs_max_acc 40000`

ファームの `tests/test_v1_codec.h` は同じ hex を埋め込む。

## 5. PC 側（control-server）

### 5.1 新モジュール `limits.rs`

```rust
pub struct MotionLimits { max_gain: f64, max_vel_count_s: u32, max_acc_count_s2: u32, duty_max_percent: u8 }  // 実行時に変わる 4 つ
pub struct Ceilings    { max_gain: f64 /* 1.0 */, max_vel_count_s: u32, max_acc_count_s2: u32, duty_max_percent: u8 }
pub struct FwTuning    { duty_max, duty_min, kp, ff, deadband, deadband_out }   // SET_LIMITS の中身
pub struct FwAbsolute  { duty_max_percent: u8, max_vel_count_s: u32, max_acc_count_s2: u32 }  // LIMITS の申告値
pub const FW_ABSOLUTE_FALLBACK: FwAbsolute = { 60, 6000, 40000 };  // ファームの config.h と同じ値。模擬軸もこれを名乗る
pub fn clamp(requested: &MotionLimits, ceilings: &Ceilings) -> (MotionLimits, Vec<LimitedField>);
```

`clamp` は WS の ack と supervisor の両方が使う唯一の関数（2 か所で結果が食い違わないため）。

### 5.2 共有状態

`Arc<Mutex<MotionLimits>>`（要求値）を API と制御ループで共有する。

- `POST /v1/session` で `[limits]` の初期値に戻す。
- 制御ループは毎 tick、要求値を `Supervisor::set_requested_limits` に渡す。supervisor は接続中の軸の LIMITS 申告値の最小（未申告なら `FW_ABSOLUTE_FALLBACK`）と `overspeed / 1.5` から `Ceilings` を作り、`clamp` して有効値にする。
- planner の速度・加速度は有効値（center は `center_speed_fraction` 倍）。
- SET_LIMITS は、有効値から作った `FwTuning` が「その軸に最後に送って ACK された値」と違うときだけ送る。軸ごとに未応答は 1 つまで。識別直後は CONFIG（Sync）と一緒に送る。
- gain の上限は session の受理で使う（`accept_pose` / `accept_gain` に有効 `max_gain` を渡す）。

### 5.3 API

WS `/v1/motion`（ゲーム用。トークン不要）:

```json
{"type":"limits","protocol":1,"source_session":"...","seq":1252,
 "max_gain":0.5,"max_vel_count_s":3000,"max_acc_count_s2":15000,"duty_max_percent":40}
```

- 4 項目はどれも省略可（書いた項目だけ変える）。`seq` と `source_session` の規則は pose と同じ。鮮度は延長しない。
- ack: `{"type":"ack","seq":1252,"accepted":true,"effective_gain":<新しい gain の上限>,"effective_limits":{...4 項目...},"limits":["ceiling_clamped"],"fresh":...}`。非有限は `non_finite`、`max_gain < 0`・速度と加速度の 0 以下・`duty_max_percent` の 1〜100 の外は `out_of_range`（実装で決めた: 制限値の要求に「負の gain を 0 に丸める」互換は要らない）。`seq` は受理したときだけ消費する。
- 競合中（`axis_conflict`）は pose / gain と同じく拒否。

REST:

- `GET /v1/limits`（トークン不要、loopback のみ）: `{"effective":{...},"requested":{...},"ceiling":{...},"defaults":{...},"limited":[...]}`。
- `POST /v1/limits`（管理トークン、rate limit あり）: 本文は WS と同じ 4 項目（省略可）。応答は `GET` と同じ形。

`POST /v1/session` の応答の `max_gain` は現在の有効値（初期値に戻した直後なので `[limits].max_gain`）。

### 5.4 StatusSnapshot

- `motion_limits: {effective, requested, ceiling, defaults, limited}` を足す。
- `axes[].fw_limits: {duty_max_percent, duty_min_percent, kp_percent_per_100, ff_count_s_per_percent, deadband_count, deadband_out_count, abs_duty_max_percent, abs_max_vel_count_s, abs_max_acc_count_s2}`（LIMITS 受信後。null = 未受信）。
- `LimitReason` に `CeilingClamped` を足す。

### 5.5 模擬軸（`sim_axis.rs`）

ファームと同じ応答: capabilities に `LIMITS`、SET_LIMITS の検査・クランプ（`FW_ABSOLUTE_FALLBACK` と同じ値）・LIMITS 返信、CONFIG の速度・加速度のクランプ、STOPPING の減速に `frame_max_acc`。プラントの速度モデルは変えない。

## 6. ファーム側（moter-controler、`MC_PROTO_V1`）

- `config.h`: 絶対上限 `V1_ABS_DUTY_MAX = 60`、`V1_ABS_MAX_VEL_COUNT_S = 6000`、`V1_ABS_MAX_ACC_COUNT_S2 = 40000` を足す。`V1_DUTY_MIN` は `NEUTRAL_DUTY_MIN` の別名をやめて独立した定数 7 にする（自動移動側の値は触らない）。既定の `V1_DUTY_MAX = 20` は SET_LIMITS を受けるまでの初期値として残す。
- `v1_codec.h`: `msg::SET_LIMITS = 0x0D`（16 byte）、`msg::LIMITS = 0x88`（24 byte）、`capabilities::LIMITS = 1u << 3`、構造体 `SetLimits` / `Limits`、encode / decode、長さ表。
- `v1_axis.h`: `on_set_limits`（§4.2 の検査 → `tuning` を置き換え → LIMITS を push）。HELLO の capabilities に `LIMITS`。`on_config` で速度・加速度を絶対上限にクランプ。`update_setpoint` の STOPPING で `frame_max_acc_` を使う。`stall_duty`（10 %）は据え置き。
- `tests/`: codec のベクタ 2 つ、axis の試験（受理・INVALID_PARAM・クランプ・LIMITS の内容・ACTIVE 中の受理・FAULT 中の拒否・HELLO の capabilities・STOPPING の減速が frame の値）。
- **書き込みはしない。** ビルド（`~/.platformio/penv/bin/pio run`）とホストテストまで。

## 7. クライアント・試験ページ

- `clients/python/h3_motion_client.py`: `submit_limits(max_gain=None, max_vel_count_s=None, max_acc_count_s2=None, duty_max_percent=None)`（非ブロッキング、最新 1 件）、`last_limits_ack()`。
- `clients/h3game/h3_server_backend.py`: ゲームの POST に `limits` オブジェクトがあれば `submit_limits` に渡す。
- `control-server/static/test_page.html`: 速度・加速度・Duty 上限のスライダー（上限は status の `motion_limits.ceiling`）。WS の `limits` で送り、ack の `effective_limits` を表示。gain スライダーの上限も `motion_limits.ceiling.max_gain`（1.0）に。

## 8. 文書

- 新規 `docs/control-server/limits.md`: 2 段の説明、全項目の値と根拠と未検証の印、実行時 IF の使い方、実機で上げる手順、症状 → 調整項目、ファームの焼き直しが要ること。
- `protocol.md`: §3（capabilities）、§4（SET_LIMITS / LIMITS、CONFIG のクランプ、STOPPING の減速）、§7（ベクタ）。
- `README.md`（設定ファイル）、`DESIGN.md`（§7.1 / §7.2、モジュール表）、`hardware-bringup.md`（`max_gain 0.15` の記述を `[limits]` に）、`testing.md`（件数）、`clients/*/README.md`、`moter-controler/README.md`。

## 9. 実装の順序（それぞれ `cargo test && cargo clippy --all-targets -- -D warnings`、Python の unittest、`run_tests.sh`、`pio run` を通す）

1. 設定の統合（schema 2、`[limits]`、`calibration_hash` の変更、toml 3 本）。挙動は変えない。
2. PC 側だけの実行時制限（gain / vel / acc）。SET_POSITION が毎フレーム速度・加速度を運ぶので、旧ファームでも動く部分。
3. SET_LIMITS / LIMITS（ファーム・模擬軸・Rust protocol・ベクタ・supervisor の配布・阻害要因）。
4. Python クライアント、h3 backend、試験ページ。
5. 文書。

## 10. 決めなかったこと・残ること

- 数値はすべて実機未検証。PR では「どのつまみがどの症状向けか」「未検証」「どの個体に新ファームが要るか（68:EE、30:ED は v1 の焼き直し待ち）」を明記する。
- Duty 下限 3 % は仮説。限界サイクルが残るなら、停止中（軌道速度 0）だけ下限を外す、または P 項だけに下限を掛けるなどの改良が候補。
- LIMITS を HELLO 直後に自発送信することはしない（PC が SET_LIMITS を送るので十分）。必要になったら足す。
