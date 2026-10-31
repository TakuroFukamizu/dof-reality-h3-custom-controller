# h3-control-server 実装設計

元の計画: [docs/control-server/architecture.md](../docs/control-server/architecture.md)（以下「計画」）。
USB プロトコル: [docs/control-server/protocol.md](../docs/control-server/protocol.md)（規範。コードと常に一致させる）。

この文書は計画を実装に落とすときの決定事項。計画と矛盾したら計画を優先し、この文書を直す。

## 0. 守ること（全モジュール共通）

- 混合式は `h3_direct.py` のまま: `m1 = 512 + g(roll + pitch)`, `m2 = 512 + g(roll − pitch)`, `m3 = 512 + g·yaw`, `g = gain·511`。`docs/cores3-controller/software.md` の `T1 = C + aP + bR` 系の符号に「直さない」。
- 軸の向きの違いは校正（count_at_virtual_*）で吸収する。混合式では吸収しない。
- 型と名前で単位を分ける: `Pose`（無次元 −1..1）、`VirtualPos`（旧 0..1023 目標位置、PWM Duty ではない）、`Counts`（エンコーダ count）。
- 接続しただけで原点移動・中央復帰・ブレーキ解除・ARM・START をしない。再接続でも同じ。サーバー終了時に center しない。
- 安全停止（STOP / FAULT）を最新 Pose で上書きしない。STOP / FAULT 後に遅れて届いた Pose で駆動を再開しない。
- ゲーム入力の鮮度は `frame_seq` が進んだときだけ更新する。同じ frame_seq の再送、gain だけの更新、/state の poll、AI 応答、USB keepalive では延長しない。
- USB ポートは HELLO で軸が決まる（`selected_axis`、無ければ設定の `device_id`）まで HELLO_REQ 以外を送らない。未知の装置・系統の競合・設定の `device_id` との不一致があれば ARM 不可（§10）。
- 非有限値は拒否。範囲外は互換モードでクリップし、必ず `LimitReason` で報告する。
- 時間は単調時計（`tokio::time::Instant`）。純粋ロジック（compat / calibration / planner / session / supervisor / sim_axis の中核）は `now` を引数で受け取り、テストで時刻を注入できるようにする。非同期部分は `#[tokio::test(start_paused = true)]` で検証できるよう `tokio::time` だけを使う（`std::time::Instant::now()` / `SystemTime` を制御判断に使わない）。
- パニックしない: 外部入力（USB バイト列・HTTP/WS 入力・設定）で `unwrap` / 添字アクセスのパニックを起こさない。

## 1. モジュールと責任

| モジュール | 種別 | 責任 |
| --- | --- | --- |
| `types` | 共有 | 単位付きの型、状態 enum、異常コード（済） |
| `protocol` (`cobs`, `frame`, `mod`) | 純粋 | USB v1 のエンコード・デコード、ストリーム再同期 |
| `config` | 純粋 | TOML スキーマ（済）と `validate()` |
| `compat` | 純粋 | h3game-v1 の Pose 検査・gain 制限・混合式・飽和、旧 `H3` の送信値モデル（参照用） |
| `calibration` | 純粋 | 3 点区分線形の virtual→count、ソフトリミット、検証、ハッシュ |
| `sim_axis` | 純粋＋非同期 | **MCU 側の参照モデル**。protocol §5 の状態機械・期限・epoch/seq 検査・ACK・STATUS を実装し、簡単な位置追従プラントを持つ。異常注入可。CoreS3 ファームはこの挙動に合わせる |
| `planner` | 純粋 | 100Hz の 3 軸目標生成。速度制限は 3 軸一様スケール、加速度制限、停止（減速）モード |
| `session` | 純粋 | 駆動クライアントのセッション（単一所有）、Pose 受理判定と鮮度 |
| `supervisor` | 純粋 | リグ状態機械（計画 §10）。軸イベント・管理命令・tick を受けて軸ごとの送信メッセージを出す。Planner と校正を所有 |
| `axis_link` | 非同期 | 1 ポート 1 タスク。HELLO 照合、受信デコード、管理命令（優先・有界キュー）と SET_POSITION（最新 1 件）の送信、STATUS 途絶検出 |
| `telemetry` | 純粋＋非同期 | 遅延統計（p50/p95/p99/max）、カウンター、有界リングのイベントログ（落とした件数を記録）、非同期 JSONL 記録 |
| `api` | 非同期 | axum: Game API（WS /v1/motion、POST /v1/session）、GET /v1/status、管理 API。Origin 検査、トークン、本文長制限、rate limit |
| `server` | 非同期 | 配線。バックエンド（sim/serial）生成、リンク起動、制御ループ（`MissedTickBehavior::Skip` の interval）、API 起動。テスト用ハーネス |
| `main.rs` | | CLI: `--config <path>`、`check-config` |

## 2. スレッド・データの流れ

```text
 h3game bridge.py ──WS /v1/motion──▶ api ──(session 判定)──▶ watch<Option<AcceptedPose>>（最新 1 件）
                                      │                                 │
 管理 CLI / curl ──HTTP /v1/*──▶ api ──mpsc(有界)+oneshot──▶ 制御ループ（単一タスク, 100Hz, Skip）
                                                                 │  Supervisor（Planner・校正を所有）
                         axis_link ×3 ──mpsc(有界) LinkEvent──▶  │
                         ◀── 管理命令: mpsc(有界・優先) ─────────┤
                         ◀── SET_POSITION: watch（最新 1 件）────┘
                                                                 └──▶ watch<StatusSnapshot> ──▶ api（GET /v1/status, WS status）
```

- 制御ループの 1 tick: LinkEvent をすべて取り込み → 管理命令を取り込み → `supervisor.tick(now, latest_pose)` → 出力を各リンクへ。LinkEvent と管理命令はどちらも有界キュー。FAULT は Pose より優先（tick 内で先に処理される）。
- tick の `dt` は実測経過時間を `2 × planner_period` で頭打ちにする（遅延後に大きな一歩を出さない）。遅れた tick を連続実行しない（Skip）。
- SET_POSITION は `watch` で最新 1 件だけ。古い目標を FIFO に溜めない。管理命令は有界 mpsc（満杯なら送信失敗として FAULT 扱い）。

## 3. session（Game 入力の受理と鮮度）

- `POST /v1/session` で新しい `source_session`（ランダム 128bit の hex）を発行。発行すると前のセッションは無効（単一所有。旧 bridge の owner 番号と同じく「最後に開いた方」）。セッション発行は駆動権限ではない（ARM / START は管理 API）。
- **駆動は START 時のセッションに束縛する。** `PoseIntake` はセッション発行ごとに `session_gen` を +1 し、`AcceptedPose.session_gen` に載せる。supervisor は `Start` 時の Pose の `session_gen` を記録し、ACTIVE(game) 中は別の `session_gen` の Pose を「古い Pose」と同じに扱って `STOPPING(source_stale)` へ（タイミングによらず必ず止まる）。新しいセッションで駆動するには改めて ARM / START。Python クライアントの再接続（新しいセッション）も走行中なら座席を止める。HOLDING で START 確認待ちの間に入れ替わった場合も、ACTIVE に入った tick で止まる。
- Pose 受理規則（`session::PoseIntake::accept(msg, recv_at)`）:
  1. `protocol == 1`、`source_session` が現行と一致（不一致 → `stale_session`）。
  2. `seq` は前回より大きい（そうでない → `old_seq`）。
  3. pitch/roll/yaw/gain/source_age_ms は有限（そうでない → `non_finite`）。範囲は compat::check_pose。
  4. `source_age_ms` は 0 以上（負 → `invalid_age`）。`produced_at = recv_at − source_age`。
  5. `frame_seq` あり: 前回より大きければ鮮度更新 `fresh_until = produced_at + source_freshness_ms`。同じ → Pose 値は受理しても鮮度は延長しない。小さい → `old_frame`。
  6. `frame_seq` なし: `allow_missing_frame_seq = true` なら新しい seq ごとに鮮度更新し `freshness_mode = "timer_only"` と報告。false なら `missing_frame_seq` で拒否。
  7. `{"type":"gain"}` は gain だけ更新し鮮度を延長しない。
- 受理した Pose は `AcceptedPose { pose, requested_gain, checked: CheckedPose, fresh_until, frame_seq, seq, received_at, produced_at, freshness_mode, session_gen }` として watch に置く。

## 4. planner

- 状態: 軸ごとの `pos`（f64 count）、`vel`（f64 count/s）。ARM で HOLDING に達した時点の実測 `actual_count` で初期化、vel = 0。
- `step_track(targets: [f64; 3], limits: [AxisLimits; 3], dt) -> PlanOutput`:
  1. 各軸 `v_raw = (target − pos) / dt`。
  2. 減速可能速度で制限: `|v| ≤ sqrt(2·a·|target − pos|)`（目標で止まれる速度）。
  3. **一様スケール**: `s = min(1, min_i(vmax_i / |v_i|))` を 3 軸に同じだけ掛ける（軸空間で方向を保つ ＝ 共通の時間軸）。s < 1 なら `VelocityLimited`。
  4. 加速度制限: `|v_new − vel| ≤ a·dt` を軸ごとにクリップ。クリップしたら `AccelerationLimited`。
  5. `pos += v_new·dt`、`vel = v_new`。出力 `command_counts = round(pos)` を i32 に。
- `step_stop(limits, dt)`: 目標なしで各軸 `vel` を `a·dt` ずつ 0 へ。全軸 `vel == 0` で `settled = true`。
- `hold()`: vel = 0、pos 固定。
- 目標はソフトリミットで既にクリップ済み（supervisor が calibration で）。planner は受けた目標を越えない。

## 5. supervisor（リグ状態機械）

純粋な構造体。入力:

- `on_link_event(LinkEvent, now)`: Identified{axis, hello, port} / UnknownDevice{port, device_id} / Status{axis, status, received_at} / Ack / FaultEvent / Disconnected{axis} / DecodeErrors / PortClosed{port}。`Status.received_at` はリンクが復号した時刻で、STATUS の経過時間（`status_age_ms`・`status_stale`・STATUS 途絶 FAULT）はここから数える（`now` より後の値は `now` に丸める）
- `admin(AdminCmd, now) -> Result<AdminOk, AdminError>`: `Arm`, `Start`, `Stop`, `Disarm`, `FaultReset`, `Center`, `Reference{axis, count}`, `Fault{note}`
- `tick(now, dt, pose: Option<&AcceptedPose>) -> TickOutput { admin_msgs: [Vec<HostMsg>; 3], set_position: Option<[SetPosition; 3]> }`
- `snapshot(now) -> StatusSnapshot`

遷移（計画 §10 に対応）:

| 状態 | 条件・動作 |
| --- | --- |
| `OFFLINE` | 3 軸のどれかに primary が無い（未接続、または系統の競合 `axis_conflict`、§10） |
| `CALIBRATION_REQUIRED` | 3 軸接続済みだが、どれかが: 設定の `calibration_valid = false` / `POSITION_REFERENCED` なし / 設定の `device_id` との不一致（`axis_device_mismatch`）/ 系統を変えた個体の再基準化待ち（`reference_stale`）/ 未知の装置のポートあり（`unknown_device`）/ CONFIG 未受理 / MCU が DISARMED 以外（FAULT 等）/ 実位置（STATUS.actual_count）がソフトリミット外（MCU も ARM_PREPARE を BAD_STATE で拒否する）/ CONFIG 受理後の STATUS `POWER_CONTROLLER` が設定（`power_controller_axis` の軸だけ 1）と違う（`power_role_mismatch`。主電源リレーを 2 軸が操作しないように。計画 §10）。`blockers` に理由を列挙 |
| `READY_TO_ARM` | 上記がすべて解消。接続時は各軸へ CONFIG を送る（epoch は現行値）。CONFIG ACK を待って `configured` |
| `ARMING` | `Arm` で epoch += 1 → 全軸へ CONFIG（新 epoch）→ 全 ACK ACCEPTED → 全軸へ ARM_PREPARE → 全軸が STATUS で HOLDING → planner を実測位置で初期化して `HOLDING`。`arm_timeout_ms` 超過 → FAULT_LATCHED(ArmTimeout)。ARMING 中から SET_POSITION（現在位置＝保持目標）を毎 tick 送る（lease） |
| `HOLDING` | 毎 tick SET_POSITION（保持目標）。`Start`: 新鮮な Pose が無ければ拒否。あれば全軸へ START → 全軸 ACTIVE を確認して `ACTIVE(game)`。`Center`: 全軸へ START → `ACTIVE(center)`。MCU は期限内の SET_POSITION が無い START を BAD_STATE で拒否するので、START は SET_POSITION を送り続けている HOLDING からだけ出す |
| `ACTIVE(game)` | 新鮮な Pose → compat → calibration（ソフトリミットでクリップ）→ planner.step_track。Pose が古い/無い → `STOPPING(source_stale)`。center 目標へは動かない |
| `ACTIVE(center)` | 目標 = 各軸 neutral count、速度上限 × `center_speed_fraction`。到達（全軸 |pos − neutral| ≤ 1 かつ vel = 0）→ 全軸 CONTROLLED_STOP → `HOLDING`。Pose は使わない |
| `STOPPING` | `Stop` / 鮮度切れ: 全軸へ CONTROLLED_STOP、planner.step_stop。settled かつ全軸 HOLDING → `stop_hold_ms` 保持 → 全軸 DISARM → 全軸 DISARMED で `BRAKED_DISARMED`。`stop_settle_timeout_ms` 超過 → FAULT_LATCHED(StopTimeout) |
| `BRAKED_DISARMED` | 停止後。`Arm` は READY_TO_ARM と同じ前提で受ける（自動再 ARM しない）。前提が崩れたら CALIBRATION_REQUIRED / OFFLINE へ |
| `FAULT_LATCHED` | 原因: 軸の STATUS.fault_code ≠ NONE / FAULT_EVENT / 駆動中（ARMING〜STOPPING）の軸断・boot_id 変化・状態不一致（例: ACTIVE 中に軸が DISARMED）/ ARMING・STOPPING のタイムアウト / ACTIVE 中の frame skew（下記）> `max_frame_skew`（猶予: ACTIVE 到達から 200ms）/ 管理 `Fault`。ラッチ時に接続中の全軸へ HOST_FAULT（各軸は PWM 0・ブレーキ作動）。ラッチ中に再接続・再起動した軸にも HOST_FAULT を送る（CONFIG は送らない。CONFIG は解除後に `reconcile_config` が送る）。HOST_FAULT の ACK が `ACK_TIMEOUT` 内に来なければ再送（MCU 側で冪等）。`FaultReset`: 全軸接続、かつ各軸で **HOST_FAULT の ACK より後に受けた STATUS** がある（古い STATUS は「DISARMED・NONE」のままのことがあり、それで判断すると FAULT_RESET を送らずにラッチを外してしまう）、かつ管理以外の原因が解消していれば、FAULT の軸へ FAULT_RESET → 全 ACK かつ全軸がリセット受理後の STATUS で DISARMED・NONE を確認して再分類（自動再 ARM しない）。原因が残れば拒否 |

- `Disarm`: ARMING / HOLDING / ACTIVE / STOPPING から全軸 DISARM → 全軸 DISARMED で `BRAKED_DISARMED`。
- `Stop` は常に受理（どの状態でも冪等。駆動していなければ何もしない）。
- `Reference{axis, count}`: 全軸 DISARMED（駆動していない）ときだけ。その軸へ SET_REFERENCE。
- 駆動していない状態（OFFLINE / CALIBRATION_REQUIRED / READY_TO_ARM / BRAKED_DISARMED）での軸断は FAULT にしない（OFFLINE へ）。
- boot_id が前回と変わった軸は、その軸の `referenced` / `configured` を無効化（MCU 側でも消えている）。
- request_id は supervisor が全軸共通の単調カウンターで払い出す（0 は使わない、wrap したら 1 へ）。ACK は request_id と照合。DUPLICATE は「受理済み」と同じに扱う（MCU は受理した命令しか DUPLICATE にしない）。
- **request_id と session_epoch の初期値は起動ごとに乱数で決める**（固定値 1 から始めない）。MCU は PC の再起動を知らず、受理済み命令（16 件）と epoch ごとの `last_rx_seq` を保持している。request_id を再利用すると同じ内容の命令が DUPLICATE で捨てられ、epoch を再利用すると SET_POSITION がすべて stale になり ARM のたびに COMMAND_TIMEOUT になる（protocol.md §4）。epoch は `Arm` ごとに +1（wrapping）。SET_POSITION の seq は epoch ごとに 1 から始めてよい。
- 「後に受けた」は時刻ではなくイベント順（supervisor 内の受信通番）で判定する。同一ポートの受信は順序が保たれるので、ACK の後に来た STATUS は HOST_FAULT 処理後の状態を表す。
- frame skew: 各軸の STATUS を取り込んだ時点の `group_frame_id` と、その STATUS の `last_applied_frame` の差（遅れ量 lag）を軸ごとに記録し、3 軸の lag の最大−最小を skew とする。STATUS が遅れた・途絶えた軸は lag が古いままで skew にはならない（途絶は `axis_status_timeout_ms` で `AxisDisconnected`）。同時に適用されていないフレーム（SET_POSITION を取りこぼす軸）は lag が増えて検出される。
- 管理命令の拒否理由 `blockers` は `"<code>: <detail>"` 形式（例 `not_connected: axis2`、`rig_state: HOLDING`、`no_fresh_pose: …`）。状態名は JSON と同じ表記（`HOLDING` など）。クライアントは `:` より前の code だけで判定する。
- 状態のスナップショット（`StatusSnapshot`、§7）を毎 tick 作る。

## 6. sim_axis（MCU 参照モデル）

- `SimMcu`（純粋）: `handle(&HostMsg, now_us) -> Vec<McuMsg>`、`tick(now_us) -> Vec<McuMsg>`（STATUS 周期送信、期限監視、プラント更新）。protocol.md §3〜§5 の規則をすべて実装する: epoch 検査、seq 単調（epoch ごと）、`valid_for_ms` と `command_timeout_ms`、再送の検出（受理済みで msg_type・request_id・payload が同じ → `DUPLICATE` ACK・再実行しない。HOST_FAULT は常に実行）、CONFIG 未受理で ARM 拒否、REFERENCED でなければ ARM 拒否、実位置がソフトリミット外なら ARM 拒否、期限内の SET_POSITION が無ければ START 拒否、HOLDING は target 無視、ソフトリミット最終クリップ、追従誤差・過速度・ソフトリミットで FAULT、FAULT_RESET は原因が残っていれば拒否、DISARM は減速してから。
- プラント: 制御目標へ `plant_max_vel_count_s` と CONFIG/フレームの速度・加速度上限で追従する簡単なモデル。ARMING は電源→ブレーキ解除の待ちを短い固定時間（既定 50ms）で模擬。
- 異常注入（テスト用）: `freeze_status`（STATUS を止める＝USB 断模擬）、`drop_rx`（受信を捨てる）、`force_fault(code)`、`stall`（実位置が動かない → 追従誤差）、`reboot`（boot_id 変更・状態リセット）、`corrupt_next_tx`。
- `spawn_sim_axis(io, params) -> SimHandle`: `tokio::io::DuplexStream` の片側で COBS フレームを読み書きする非同期ランナー（`tokio::time` 使用）。`SimHandle` から異常注入できる。

## 7. API（JSON は Python クライアントと共有の契約）

既定 `127.0.0.1:8892`。Origin ヘッダーがあれば `allowed_origins` に含まれること（なければ 403）。本文は `max_body_bytes` まで。管理 API は `Authorization: Bearer <token>`（トークンは `admin_token_env` の環境変数、未設定なら起動時に生成して標準エラーへ 1 回出す）。

- loopback 以外への bind は既定で拒否。`server.allow_non_loopback = true` で許可できるが、そのときも **loopback 以外の相手には `GET /v1/status` と `POST /v1/stop` だけ**を開く（他は 403 `loopback_only`。相手アドレス不明も拒否）。Game 入力（session / motion）と管理トークンは LAN を通らない（計画 §11「LAN の観戦通信に駆動権限を付けない」）。
- 管理 API の rate limit は `admin_rate_limit_per_s`（トークン検査の後）。`/v1/stop`・`/v1/disarm`・`/v1/fault` は rate limit の対象外（安全側の操作を拒まない）。
- `/v1/fault` は本文に関係なくラッチする: `{"note": "..."}` として読めればその note、空なら空文字、それ以外は `"unparsed body: <先頭 200 文字>"` を note にする（`max_body_bytes` 超過だけは 413）。

| API | 認証 | 内容 |
| --- | --- | --- |
| `POST /v1/session` | なし | `{"source_session": "...", "protocol": 1, "max_gain": 0.35, "source_freshness_ms": 250, "require_frame_seq": true}` |
| `GET /v1/motion[?session=<id>]`（WS） | なし | §7.1。`session` は省略可（各メッセージの `source_session` で判定）。指定した場合は別セッション名のメッセージも `stale_session` |
| `GET /v1/status` | なし | StatusSnapshot（§7.2） |
| `GET /v1/events[?n=&after=]` | なし（loopback だけ） | 制御ループのイベントリング（障害前後の経過）。§7.3 |
| `POST /v1/stop` | **なし**（停止は誰でも要求できる。物理非常停止ではない） | `{"ok":true,"rig_state":...}` |
| `POST /v1/arm`, `/v1/start`, `/v1/disarm`, `/v1/fault/reset`, `/v1/center` | Bearer | 成功は 200 `{"ok":true,"rig_state":...,"note":null\|"..."}`（`note` は no-op などの補足）。拒否は 409 `{"ok":false,"error":"...","blockers":[...]}` |
| `POST /v1/reference` `{"axis":1,"count":0}` | Bearer | DISARMED 時のみ |
| `POST /v1/fault` `{"note":"..."}` | Bearer | 管理者による FAULT ラッチ |

エラー応答はすべて `{"ok":false,"error":"<code>"}`。`blockers` があるのは 409 だけ。

| HTTP | `error` | 意味 |
| --- | --- | --- |
| 400 | `bad_request` / `bad_axis` | `/v1/reference` の本文不正 / axis が 1..3 以外 |
| 401 | `unauthorized` | トークンなし・不一致（`WWW-Authenticate: Bearer` 付き） |
| 403 | `forbidden_origin` / `loopback_only` | Origin が許可外 / LAN の相手に閉じた API |
| 404 | `not_found` | 未知のパス |
| 405 | `method_not_allowed` | 既知のパスでメソッド違い |
| 409 | supervisor の code（下記） | 拒否。`blockers` 付き |
| 413 | `body_too_large` | 本文が `max_body_bytes` 超過 |
| 429 | `rate_limited` | 管理 API の rate limit |
| 503 | `control_loop_unavailable` | 制御ループが `ADMIN_TIMEOUT`（1 秒）内に答えない、または止まっている（`GET /v1/status` も 503） |

- 409 の code: `not_ready` / `already_armed` / `not_holding` / `busy` / `no_fresh_pose` / `fault_latched` / `not_faulted` / `reset_in_progress` / `axis_disconnected` / `axis_not_settled` / `not_disarmed` / `axis_conflict`（§10、arm / start / center）/ `shutting_down`。
- 503 の意味: 制御ループがまだ取り出していなかった命令は**実行されない**（遅れて実行しない。ループは実行前に要求を claim し、API の打ち切りとどちらか一方だけが成立する）。ただし `Stop` / `Disarm` / `Fault` は安全側なので、ループが追いつけば実行する（呼び出し側から結果は分からない）。ループが claim 済みなら API は応答を待って 200/409 を返す。

### 7.1 WS /v1/motion

クライアント → サーバー:

```json
{"type":"pose","protocol":1,"source_session":"...","seq":1250,"frame_seq":2408,"source_age_ms":3.0,
 "pitch":0.3,"roll":-0.2,"yaw":0.0,"gain":0.35}
{"type":"gain","protocol":1,"source_session":"...","seq":1251,"gain":0.3}
```

サーバー → クライアント:

```json
{"type":"ack","seq":1250,"accepted":true,"reason":null,"effective_gain":0.35,"limits":["pose_clamped"],"fresh":true}
{"type":"status", ...StatusSnapshot}
```

- ack は受理・拒否どちらも返す。拒否理由: `stale_session` / `old_seq` / `old_frame` / `non_finite` / `out_of_range` / `invalid_age` / `missing_frame_seq` / `bad_protocol` / `bad_message` / `axis_conflict`（§10。pose・gain とも拒否し、鮮度も更新しない）。
- status は `status_period_ms` ごと。WS 自体は駆動権限を与えない。
- 1 メッセージの最大長は `max_body_bytes`。不正 JSON は `bad_message` の ack（seq は null）で、接続は維持。

### 7.2 StatusSnapshot

```json
{
  "protocol": 1,
  "rig_state": "HOLDING",
  "armed": true,
  "active_source": null,
  "calibration_valid": true,
  "fault": null,
  "fault_history": [],
  "blockers": [],
  "session": {"source_session_prefix": "8b69173f", "fresh": false, "freshness_mode": "frame_seq", "source_age_ms": null, "last_seq": 0, "last_frame_seq": null},
  "requested_pose": null, "requested_gain": null, "effective_gain": null, "limits": [],
  "virtual_targets": [512.0, 512.0, 512.0],
  "target_counts": [0, 0, 0],
  "session_epoch": 3,
  "group_frame_id": 1234,
  "axes": [{"axis_id": 1, "name": "…", "connected": true, "port": "sim:2", "device_id": "5131000000000001",
            "expected_device_id": "5131000000000001", "assignment": "selected",
            "claimants": [{"port": "sim:2", "device_id": "5131000000000001"}], "reference_stale": false,
            "boot_id": 1, "mcu_state": "HOLDING", "fault_code": "NONE", "configured": true, "referenced": true,
            "calibration_valid": true, "actual_count": 0, "target_count": 0, "velocity_count_s": 0,
            "following_error_count": 0, "pwm_permille": 0, "power_on_cmd": true, "brake_release_cmd": true,
            "power_controller": true, "current_a": null, "last_applied_frame": 1234, "status_age_ms": 4, "rx_error_count": 0, "stale_drop_count": 0}],
  "axis_conflict": [],
  "measured_angles": null,
  "physical_estop": "not_monitored",
  "telemetry": {"tick_count": 0, "tick_overruns": 0,
                "tick_dt_ms": {"count": 100, "p50": 10.0, "p95": 10.1, "p99": 10.3, "max": 11.0, "max_all": 12.0},
                "pose_rx_to_tick_ms": {...}, "status_age_ms": {...}, "frame_skew": {...},
                "log_dropped": 0, "recorder_dropped": 0}
}
```

- `armed` = ARMING / HOLDING / ACTIVE / STOPPING。
- 電流は `CURRENT_VALID` のときだけ A で、そうでなければ `null`（0 と表示しない）。
- 実測角度は未校正なので `measured_angles: null`。`requested_pose` は指令値であって実測ではない。
- `session.source_session_prefix`: 現行セッション id の先頭 8 桁（なければ null）。完全な id は `POST /v1/session` の応答でだけ返す（id が Pose 入力の唯一の鍵なので、認証なしの status に載せない）。自分のセッションか確かめるときは先頭 8 桁を比べる。
- `active_source`: `"game"` / `"center"` / null（ACTIVE 以外）。
- `fault`: null または `kind` で分かれるオブジェクト: `{"kind":"axis_fault","axis":1,"code":"FOLLOWING_ERROR"}` / `{"kind":"axis_disconnected","axis":2}` / `{"kind":"axis_rebooted","axis":3}` / `{"kind":"axis_changed","axis":1}`（駆動中に担当個体の系統が変わった、§10）/ `{"kind":"axis_state_mismatch","axis":1,"expected":"ACTIVE","actual":"DISARMED"}` / `{"kind":"arm_timeout"}` / `{"kind":"stop_timeout"}` / `{"kind":"frame_skew","max_skew_frames":6}` / `{"kind":"operator","note":"..."}`。リンク送信失敗は `operator` で note が `link_send_failed: …`。
- `fault_history[]`: `{"at_ms": 14510.7, "fault": {…上と同じ形…}, "detail": null|"..."}`。`at_ms` は supervisor 起動からの ms（壁時計ではない）。最大 32 件、古いものから捨てる。ラッチ中に重なった原因も入る。
- `blockers`: ARM の前提条件だけ（`"<code>: <detail>"`）。armed 中は []。FAULT_LATCHED でも [] のことがある（ARM 拒否の理由は `fault`）ので、クライアントは `fault` も見る。
- `axes[].device_id` は接続中の個体（無ければ設定値）、`expected_device_id` は設定の照合値（0 = 照合なし）、`assignment` は `"selected"`（MCU の `selected_axis`）/ `"config"`（`device_id` の後方互換経路）/ null（未接続）、`claimants` はこの軸を名乗るポート（通常は primary 1 つ、競合中は 2 つ以上）、`reference_stale` は同じ boot のまま系統を変えた個体で SET_REFERENCE のやり直し待ち。最上位の `axis_conflict` は競合中の軸番号（空なら無し）。§10。
- `axes[].power_controller`: STATUS の `POWER_CONTROLLER` フラグ（MCU が自分を主電源担当と思っているか）。STATUS が無ければ null。
- telemetry の各統計は `{count, p50, p95, p99, max, max_all}`。`count` は起動以来の件数、p* と `max` は直近の窓、`max_all` は起動以来。サンプルがなければ p* / max / max_all は **null**（0 ではない）。`frame_skew` の単位はフレーム（上記の lag 差）。`recorder_dropped` は JSONL 記録で落とした件数（記録器は未配線なので常に 0）。`log_dropped` はイベントリング（§7.3）から失われた件数（上書き＋supervisor のキューで落ちた分）。
- telemetry に無いもの（計画 §8・§11 との差。未実装）: ゲーム側の source age の分布、SET_POSITION の送信→MCU 適用の遅延、管理・イベントのキュー深さ、STATUS の欠落数、USB 書き込み遅延、clock_sync の品質。サーバー受信→MCU 適用 p99 などの §8 の目標は、この telemetry では評価できない。

### 7.3 GET /v1/events

```json
{"ok": true, "now_ms": 15022.4, "next_id": 13, "capacity": 512, "dropped": 0,
 "events": [{"id": 11, "t_ms": 341.7, "kind": "fault",
             "detail": {"fault": {"kind": "axis_fault", "axis": 2, "code": "FOLLOWING_ERROR"}, "detail": "reported in FAULT_EVENT"}},
            {"id": 12, "t_ms": 341.7, "kind": "rig_state", "detail": {"from": "ACTIVE", "to": "FAULT_LATCHED"}}]}
```

- supervisor のイベント（`identified`・`disconnected`・`axis_rebooted`・`admin`・`ack_rejected`・`rig_state`・`stopping`・`center_reached`・`session_changed`・`ack_timeout`・`fault`・`fault_reset_timeout` など）を制御ループが有界リング（512 件、古いものから上書き）に積む。古い順。`id` は 1 からの通番で、抜けは上書きされた分。`t_ms` はサーバー起動からの ms（壁時計ではない）。
- `n`（既定 100、最大はリングの大きさ）: 新しい方から n 件。`after`: `id > after` のものだけ（ポーリングでは前回の最後の `id` を渡す）。`n` / `after` が数でなければ 400 `bad_request`。
- 制御ループが落ちた後（`GET /v1/status` が 503）も読める（落ちる前の経過を見るため）。
- 同じイベントは標準エラーにも出る（`fault` は error、`rig_state` は "rig state" の行、他は info。`RUST_LOG` 既定 `info`）。ファイルへの JSONL 記録（`telemetry.rs` の `Recorder`）は未配線。サーバーを再起動するとリングは空になる。

## 8. Python クライアントと h3game アダプター

- `clients/python/h3_motion_client.py`: `MotionClient(base_url)`。`start()`, `close()`, `submit_pose(pitch, roll, yaw, gain, frame_seq=None, source_age_ms=0.0)`（非ブロッキング、最新 1 件スロット）, `submit_gain(gain)`, `status() -> dict | None`, `connected`, `reconnect()`（通信の再確立だけ。ARM/START しない）, `stop()`（POST /v1/stop）。背景スレッドが session 取得 → WS 接続 → スロット送信・status 受信・再接続（指数バックオフ、新しい session）。`source_age_ms` は「submit 時の年齢 + submit から送信までの経過」で送り、キュー待ちで 0 に作り直さない。依存: `websockets>=13`（同期クライアント）。HTTP は標準ライブラリ。
- `clients/h3game/`: h3game リポジトリ（`TakuroFukamizu/20261004_shi3z_hackathon`）への小さいパッチと、bridge 用のバックエンド層 `h3_server_backend.py`。`H3_BACKEND=legacy|server|dry`（既定 legacy＝現状どおり）。server では旧 USB を開かず、60Hz 再送・1 秒で中央へ戻す処理・auto_sway を無効にし、POST の Pose をクライアントへ渡すだけ。`/state` の既存キーを残し、`backend`, `rig_state`, `armed`, `calibration_valid`, `fault` を追加。`index.html` は frame_seq と frame 経過時間を POST に追加する数行の変更。シーン（scene*.js）は変更しない。
- パッチは h3game リポジトリへは push しない。`clients/h3game/patches/` に置き、適用手順を README に書く。

## 9. 受け入れテスト（最低限）

- compat: 計画 §12 の表（gain 0.35: pitch=1 → 690/333/512 など、十分収束後）。`LegacyH3Model` が Python の float 累積・`int()` 切り捨て・送信時クリップ・cur は未クリップ保持を再現。`center()` 30 回の収束列。初回送信が max_step で異なるケース。飽和からの戻り。
- protocol: 全メッセージの往復（AXIS_SELECT 0x86・STATUS offset 13 の `selected_axis` を含む）、テストベクタ（protocol.md と一致）、CRC 既知値（"123456789" → 0x29B1）、COBS の 0x00 を含む/254・255 byte 境界、ゴミ・途中切断・過大フレーム後の再同期、長さ不一致・未知種別・版違いの破棄。
- calibration: 区分線形・逆写像・単調減少の軸・不正設定の拒否・ハッシュの安定性（設定順に依存しない）。
- planner: 1 軸だけ速度飽和したとき 3 軸の歩幅が同じ比で縮む、加速度制限、目標で止まる（オーバーシュートなし）、停止モードが減速で止まる、`MissedTickBehavior::Skip` を使っている（server 側テスト）、dt 頭打ち。
- session: 同じ frame_seq の再送で延長しない、gain だけで延長しない、古い session/seq を拒否、source_age を考慮、frame_seq なしの扱い。
- supervisor: 接続で center しない、再接続で ARM しない、calibration_valid=false / 系統の競合（`axis_conflict`）/ 設定 device_id との不一致（`axis_device_mismatch`）/ 再基準化待ち（`reference_stale`）/ 未知の装置で ARM 拒否、競合中は CONFIG を送らず Arm・Start・Center を `axis_conflict` で拒否、駆動中の 2 台目で減速停止してから競合、駆動中の担当個体の系統変更で FAULT（`axis_changed`。切り替え先を担当していた個体にも HOST_FAULT が届いてから降格、ACK が無ければ ACK タイムアウトで降格・再送なし）、競合解消で残った個体を primary に昇格、ゲーム途絶 → 協調減速 → 保持 → ブレーキ（center しない）、STOP 後の遅れた Pose で再開しない、1 軸の異常で全軸 HOST_FAULT、駆動中の軸断・再起動で FAULT、FaultReset は原因が残れば拒否・解除後も自動 ARM しない、frame skew で FAULT、gain 上限超過の要求は effective_gain と理由を返す。
- sim_axis: protocol §5 の全遷移、期限切れで COMMAND_TIMEOUT、epoch 不一致の拒否、request_id 重複で再実行しない。
- api: Origin 違反 403、管理トークン無し 401、本文過大 413、WS の ack と status、stop はトークン不要、競合中は pose / gain を `axis_conflict` で拒否し鮮度を更新しない、arm / start / center は 409 `axis_conflict`。
- 結合（sim）: サーバー起動 → session → WS pose → arm/start → 追従 → pose 停止 → HOLD → BRAKED_DISARMED。USB 断（freeze_status）で FAULT。系統選択（§10）: 割当は `selected_axis` で決まりポート順に依存しない、待機中の切り替えで再割当（新しい軸の CONFIG・`reference_stale` が SET_REFERENCE まで ARM を阻む）、同じ系統を 2 台が選ぶと `axis_conflict`（ARM 409・pose 拒否・stop は受理）で片方を戻すと解消、駆動中の切り替えで FAULT_LATCHED、`report_selected_axis = false` で旧 device_id 経路。

## 10. 系統選択と軸の割当（protocol.md §1.1）

軸の割当は **MCU が選択している系統** で決める。設定の `[[axis]].device_id` は照合用（0 = 照合しない）。

### 10.1 axis_link

- HELLO の検査は `protocol_version == 1` と POSITION_MODE だけ。軸は `selected_axis`（1〜3）、0 なら設定で `device_id` が一致する `[[axis]]`。どちらでも決まらなければ `UnknownDevice` でポートを閉じる（従来どおり）。
- リンクは「いま名乗っている軸」を 1 つ持つ。次のどれかで軸が変わったら、同じポートから `Disconnected{axis: 旧, reason: "axis_changed: <旧>-><新>"}` → `Identified{axis: 新, hello: 直近の HELLO に新しい selected_axis を入れたもの, port}` の順に送る: (a) AXIS_SELECT（`0x86`）、(b) `selected_axis` が違う HELLO、(c) `selected_axis` が 0 でなく違う STATUS（AXIS_SELECT の取りこぼし対策）。新しい値が 0 で設定からも決まらなければ `Disconnected` → `UnknownDevice` → ポートを閉じる。同じ値の AXIS_SELECT は無視する。
- `Hello.selected_axis`（offset 5）、`Status.selected_axis: u8`（offset 13）、`McuMsg::AxisSelect(AxisSelect{boot_id, selected_axis, prev_selected_axis, mcu_state})`（`0x86`）。protocol.md §7 のテストベクタ 7.18 が AXIS_SELECT。

### 10.2 supervisor

- `AxisTrack` は **claimants**（この軸を名乗るポートと device_id・HELLO の一覧）を持つ。primary（CONFIG・SET_POSITION を送り、STATUS を採用する相手）は次のとおり:
  - 駆動していない（Idle / Fault）: claimants がちょうど 1 つならそれが primary。2 つ以上なら primary なし = **競合**（`axis_conflict`）。競合中はどちらにも CONFIG を送らず、両方の STATUS / ACK を捨てる（server.rs の primary 判定で落ちる）。
  - 競合が解消して 1 つ残ったら、その claimant を `Identified` と同じ手順で primary にする（CONFIG 送信など）。
  - 駆動中（ARMING〜STOPPING）に 2 つ目の claimant が現れたら primary は変えず、`begin_stop`（協調減速 → 保持 → DISARM → BRAKED_DISARMED）を開始し、Idle に戻った時点で競合にする。駆動中に primary 自身の軸が変わった（`Disconnected` の reason が `axis_changed:`）ら `RigFault::AxisChanged{axis}` でラッチ（既存の `AxisDisconnected` と同じ経路、理由だけ区別）。
  - **FAULT ラッチ中の降格は HOST_FAULT の後**: ラッチで HOST_FAULT を送った primary が競合になっても（切り替えた個体が、駆動中の別の軸を選んだ場合）、その HOST_FAULT が ACK されるか ACK タイムアウト（`ACK_TIMEOUT`）になるまで primary のまま置く。降格（`drop_primary`）は outbox を消し、server.rs は競合軸に何も送らないので、先に降格すると HOST_FAULT が届かず、その個体は lease 切れでしか止まらない。競合軸への HOST_FAULT の再送はしない（1 回だけ。タイムアウトで降格）。切り替えた個体自身は競合軸の claimant（primary なし）なので HOST_FAULT を受け取らず、lease 切れで止まる（protocol.md §1.1）。
- `on_identified` の「device_id が設定と違えば unknown 扱い」の分岐は削除する。代わりに照合: 設定の `device_id` が 0 でなく、primary（または claimant）の device_id と違えば blocker `axis_device_mismatch: axisN device <id>, config expects <expected>`。逆に、その device_id が別の軸 M の設定に書かれていれば `axis_device_mismatch: device <id> is configured as axisM but selected axisN`。リンクは閉じない（操作者が選択し直せば解消する）。
- `device_id = 0` は「照合しない」の意味。未記入そのものを理由にする blocker は無い（未選択で設定にも無い個体は `unknown_device`）。
- blocker `axis_conflict: axisN claimed by <port>(<device_id>), <port>(<device_id>)`。
- `AdminCmd::Arm` / `Start` / `Center` は競合中 `AdminError{error: "axis_conflict", blockers}`。`Stop` / `Disarm` / `Fault` / `FaultReset` / `Reference` / セッション作成は従来どおり（Reference は primary が無ければ既存の `axis_disconnected`）。
- WS `/v1/motion` の pose / gain は競合中 `RejectReason::AxisConflict`（JSON `"axis_conflict"`）で拒否し、鮮度を更新しない。`Supervisor::axis_conflict() -> Vec<AxisId>` を snapshot 経由で api に渡す。
- 再基準化: supervisor は `device_id → (boot_id, axis)` の最終割当を覚える。同じ boot_id の個体が別の軸として `Identified` されたら、その軸の `reference_stale = true`。PC 自身の SET_REFERENCE が ACCEPTED されるか boot_id が変わるまで blocker `reference_stale: axisN re-reference required after axis change`。
  - 実装の補足: 記録は `Identified`（claim）時点で更新する（競合中で primary になれなかった claim も数える。1→2→1 と往復した個体は 1 に戻っても stale）。再起動の検出（`AxisRebooted`・stale の解除）は **同じ device_id** の boot_id 変化だけで、別の個体が同じ軸の primary になるのは再接続扱い（reboot ではない）。
- `StatusSnapshot`: 各軸に `device_id`（接続中の個体。無ければ設定値）、`expected_device_id`（設定値、0 = 照合なし）、`assignment`（`"selected"` / `"config"` / null）、`claimants: [{"port","device_id"}]`（競合中は 2 つ以上、通常は primary 1 つ）、`reference_stale: bool`。最上位に `axis_conflict: [axis_id, ...]`（空なら競合なし）。
- RigState は競合軸が primary を持たないので `OFFLINE`（`not_connected` と `axis_conflict` の両方を blockers に出す）。

### 10.3 sim_axis / server

- `SimParams.selected_axis`。`SimMcu::select_axis(new, now_us) -> Vec<McuMsg>`: DISARMED 以外では何もしない（`select_axis_unchecked` はテスト用に規則違反を起こす）。切り替えで CONFIGURED / POSITION_REFERENCED / calibration_hash を消し、AXIS_SELECT を返し、以後の STATUS / HELLO に新しい値を載せる。`SimHandle::select_axis` / `select_axis_unchecked`。規則違反（DISARMED 以外）の切り替えでは制限値と lease を保つ（POSITION_REFERENCED だけ消す）ので、PC が書かなくなった個体は COMMAND_TIMEOUT で FAULT になる（protocol.md §1.1 の要求。`tests/server.rs` で確認）。
- `[backend] kind = "sim"` に `report_selected_axis = true`（既定 true）。true なら各模擬軸の `selected_axis` = その軸番号（割当は選択で決まり、`device_id` は照合に使われる）。false なら 0（設定の `device_id` だけで決まる旧経路の試験用）。模擬軸の device_id は設定値、設定が 0（照合なし）なら `5131000000000000 + 軸番号`（個体は必ず device_id を持つ）。
- server.rs の「primary でないポートの Status / Ack / FaultEvent を捨てる」判定は `Supervisor::axis_port` のままでよい（競合中は `None` なので全部落ちる）。`Disconnected` が primary 以外から来たら `PortClosed` に変換する既存処理で claimants から外す。
