# h3-control-server の試験

状態: **architecture.md §12 の段階 1（実機なしの互換試験）と段階 2（サーバーとゲームの結合、模擬軸）を実施。段階 3〜5（実機）は未実施。**

ここに書く結果はすべて模擬軸（`control-server/src/sim_axis.rs`）と模擬ブラウザー（または ヘッドレス Chromium）でのもの。実機の挙動・停止距離・遅延の根拠にはならない。

## 実行方法

```sh
cd control-server && cargo test && cargo clippy --all-targets -- -D warnings && cd ..
clients/python/.venv/bin/python -m unittest discover -s clients/python/tests -v
clients/python/.venv/bin/python -m unittest discover -s clients/h3game/tests -v
control-server/scripts/e2e_sim.sh
```

- Python の結合試験は `control-server/target/debug/h3-control-server` を `config/sim.toml` で空きポートに起動する（`H3CS_BINARY` で変更可。なければ skip）。
- h3game の試験と e2e は h3game のクローンを一時ディレクトリに複製してパッチを `git am` する。元のクローンは変更しない。クローンの場所は `H3GAME_SRC`。未設定なら初回に `https://github.com/TakuroFukamizu/20261004_shi3z_hackathon.git` を `control-server/target/h3game-src` に clone し、パッチを作った元のコミット `649ab66` を checkout して使う（ネットワークが要る。以後はそのクローンを使う。`target/` の中なので git の管理外で、`cargo clean` で消える）。`H3GAME_SRC` もネットワークも無ければ、クローンを使う試験（`test_patch.py`・`test_integration.py`・`test_browser.py`）は理由を表示して skip し、`e2e_sim.sh` は理由を表示して終了コード 2 で終わる。
- `test_browser.py` は node・Playwright・Chromium が無ければ skip（`PLAYWRIGHT_MODULE`、`CHROMIUM_PATH`）。

## 一覧

### Rust（`control-server/tests/`、`cargo test`）

| スイート | 件数 | 内容 | 段階 |
| --- | ---: | --- | --- |
| `src/lib.rs`（単体） | 3 | モジュール内の小さい単体試験 | 1 |
| `compat.rs` | 16 | Pose 検査・gain 制限・混合式・飽和。旧 `h3_direct.H3` の送信値モデル（float 累積・`int()` 切り捨て・送信時クリップ）を Python の記録と照合、計画 §12 の表 | 1 |
| `calibration.rs` | 15 | 3 点区分線形・逆写像・単調減少の軸・不正設定の拒否・ハッシュの安定性 | 1 |
| `planner.rs` | 16 | 3 軸一様の速度制限、加速度制限、目標で止まる（行き過ぎなし）、停止モード、dt 頭打ち | 1 |
| `session.rs` | 25 | 単一所有の session、seq / frame_seq の規則、同じ frame_seq・gain だけでは鮮度を延長しない、source_age、frame_seq なしの扱い | 1 |
| `protocol.rs` | 32 | COBS・フレーム・CRC、全メッセージの往復、ゴミ・途中切断・過大フレーム後の再同期、長さ違い・未知種別・版違いの破棄、AXIS_SELECT（`0x86`）のレイアウト、STATUS offset 13 の `selected_axis`（旧フレームは 0 として読める） | 1 |
| `protocol_vectors.rs` | 4（＋1 ignored） | protocol.md §7 のテストベクタとバイト単位で一致（HELLO / STATUS のベクタは `selected_axis = 1` で、byte 5 / 13 を埋めないファームでは通らない。7.18 AXIS_SELECT）。ignored の 1 件は文書用ベクタの生成器 | 1 |
| `sim_axis.rs` | 67 | MCU 参照モデル: protocol.md §3〜§6 の全遷移、期限切れ、epoch・seq 検査、DUPLICATE、異常注入。ACK の後の STATUS は遷移後の状態、ACTIVE 以外での CONTROLLED_STOP・DISARMED/DISARMING での DISARM は ACCEPTED（no-op）、STATUS の `target_count` は軌道上の目標で `following_error_count = actual − target`、系統選択（`select_axis` は DISARMED だけ・CONFIGURED / POSITION_REFERENCED / calibration_hash を消して AXIS_SELECT を 1 回・`select_axis_unchecked` は規則違反・選択で count は動かず再起動後も残る） | 1 |
| `supervisor.rs` | 57 | リグ状態機械（3 台の SimMcu、10 ms の偽時計。管理メッセージの配送は server.rs と同じく primary のポートだけ）: 接続で動かない、ARM 阻害要因、ゲーム途絶で減速・保持・ブレーキ（中央へ戻さない）、STOP 後の Pose で再開しない、1 軸の異常で全軸 HOST_FAULT、FaultReset、frame skew、主電源担当（STATUS の `POWER_CONTROLLER`）が設定と違えば ARM しない、READY_TO_ARM で軸が再起動すると FAULT にせず `not_configured`・`not_referenced` で ARM を拒否し、SET_REFERENCE をやり直すまで READY_TO_ARM に戻らない。系統選択（DESIGN.md §10）: 割当は `selected_axis`、0 なら設定の device_id、どちらも無ければ `unknown_device`、設定 device_id との不一致は両方向とも `axis_device_mismatch`、待機中・ARMING 中・駆動中の 2 台目（競合 → 減速停止 / DISARM してから競合、競合解消で残った個体を昇格）、待機中の切り替えで再割当・CONFIG・`reference_stale`（再起動で解除）、駆動中の担当個体の切り替えで `axis_changed` ラッチ — 切り替え先を担当していた個体には HOST_FAULT が届いてから降格（`primary_changing_axis_while_driving_latches_axis_changed`）、ACK が無ければ ACK タイムアウトで降格し再送しない（`conflicted_primary_that_never_acks_host_fault_is_demoted_after_ack_timeout`）、未選択への切り替えは `unknown_device` | 1 |
| `axis_link.rs` | 30 | HELLO 照合までの送信制限、SET_POSITION 最新 1 件、管理命令の優先、STATUS 途絶、書き込みタイムアウトの隔離、系統選択: HELLO の `selected_axis` で割当（device_id ではない）、範囲外は `unknown_device`、AXIS_SELECT / 別の `selected_axis` の HELLO / STATUS で同じポートのまま `Disconnected{axis_changed}` → `Identified`、0 への切り替えは設定 device_id に戻すか `unknown_device` でポートを閉じる、STATUS の 0 では切り替えない | 1 |
| `config.rs` | 22 | 設定の読み込みと検証、`[backend] report_selected_axis` の既定 true、全軸 `device_id = 0`（照合なし）が有効 | 1 |
| `telemetry.rs` | 12 | 百分位、リング上書き、落とした件数、`after` 付きの読み出し、JSONL 記録器 | 1 |
| `types.rs` | 2 | AxisId の変換 | 1 |
| `api.rs` | 25 | Origin 403、トークン 401、本文過大 413、rate limit、503、WS の ack と status、LAN からの制限、`GET /v1/events`（`n`・`after`・不正な値は 400・制御ループが落ちた後も読める・LAN からは 403）、競合中は pose / gain を `axis_conflict` で拒否し intake に届けない、arm / start / center は 409 `axis_conflict` | 1・2 |
| `server.rs` | 15 | 実サーバー（sim）: 追従 → 停止 → ブレーキ（中央へ戻さない）、1 軸の STATUS 停止（`freeze_status`）で全体 FAULT、起動で動かない、再接続・サーバー再起動で ARM しない、走行中の終了は その場で DISARM、走行中の新 session で停止、走行中の軸異常（`force_fault`）の前後の経過（ARM・START・ACTIVE → fault → FAULT_LATCHED → 解除操作）を `GET /v1/events` で読み戻せる。系統選択: ポート順を入れ替えても `selected_axis` で割当、待機中の切り替えで再割当と `reference_stale`（SET_REFERENCE まで ARM 409）、同じ系統を 2 台が選ぶと `axis_conflict`（ARM 409・WS pose 拒否・stop は 200）で片方を戻すと解消、駆動中の切り替えで FAULT_LATCHED（切り替え先を担当していた個体も HOST_FAULT で止まる。lease 切れではない。切り替えた個体自身は PC から何も受けず COMMAND_TIMEOUT で止まる）、`report_selected_axis = false` で旧 device_id 経路 | 2 |

### Python

| スイート | 件数 | 内容 | 段階 |
| --- | ---: | --- | --- |
| `clients/python/tests/test_motion_client.py` | 27 | 偽サーバー相手: 送信規則（新しい submit だけ送る、seq 単調、source_age をキュー待ちで作り直さない）、再接続とバックオフ、stale_session 後に取り返さない、送信がブロックしない、float にできない巨大な整数・2⁶⁴−1 を超える frame_seq は例外ではなく `False`、`axis_conflict` の拒否では再接続せず送り続けてログは 1 回（他の拒否理由では `axis_conflict` を立てない）、`_summary` に各軸の割当（port / device_id / claimants / `reference_stale`）と競合を出し古い形の status でも落ちない | 1 |
| `clients/python/tests/test_against_server.py` | 7 | 実サーバー（sim）: ARM しない、gain 上限、frame_seq なしの拒否、同じ frame_seq で鮮度が切れる、トークンなしの stop、走行中の再接続で停止、クライアントが許す最大の frame_seq（2⁶⁴−1）をサーバーが受理 | 2 |
| `clients/h3game/tests/test_server_backend.py` | 31 | bridge 用バックエンド層（偽クライアント）: frame_seq の扱い、owner 変更、無効値（float にできない巨大な整数、2⁶⁴−1 を超える frame_seq は `invalid` で、以後のフレームを塞がない）、`/state` のキー（session を取られたら `connected` は false、status の `axis_conflict` と blockers・ack の拒否理由から `hud_note`「系統の競合(軸1、入力拒否)」「個体の不一致」「再基準化が必要」を作る、他の blocker は載せず届いていなければ `null`、壊れた入力でも落ちない）、並行 POST | 2 |
| `clients/h3game/tests/test_patch.py` | 11 | パッチの適用（blob SHA・変更ファイル・正本との一致）、index.html の隠れたときの規則（0 ＋表示中に最後に送った frame_seq）と server 用の HUD 表示（`hud_note` を `srvNote()` に足す）、legacy は元の bridge と同じ応答、server・dry は serial / h3_direct を import しない、サーバー不在でも POST が待たされない、壊れた数値の POST にも応答して `invalid` に数える、ポート使用中の 2 つ目の bridge はサーバーへ 1 回も要求せずに終わる、LAN のアドレスからの `GET /reconnect`・`/claim` は無視（LAN の IPv4 アドレスが無ければこの 1 件は skip） | 2 |
| `clients/h3game/tests/test_integration.py` | 7 | 実サーバー（sim）＋パッチ後の bridge: 追従、ゲーム停止で STOPPING → BRAKED_DISARMED（中央へ戻さない）、POST が続いても frame_seq が止まれば停止、ACTIVE 中に画面を隠しても目標が動かずその位置で停止（ページの規則を模擬ブラウザーで再現したもの。ページ自体の確認は test_browser.py）、再読み込みで新 session・自動再開なし、走行中に bridge を二重起動しても session は変わらず走行が続く・別のクライアントに session を取られると bridge の `/state` が `connected: false`・`session_lost: true`、frame_seq なしは START 不可、dry は何も送らない | 2 |
| `clients/h3game/tests/test_browser.py` | 1 | ヘッドレス Chromium でパッチ後の index.html: frame_seq が増える。rAF 直後（未送信のフレームがある瞬間）に `document.hidden` を偽装し、隠した後の最初の POST から全て「0 ＋表示中に最後に送った frame_seq」。サーバー接続時は live 操作で pitch=+1 を保ったまま ARM・START し、隠しても ACTIVE の間の目標カウントが動かず、BRAKED_DISARMED でその位置に止まる。サーバーが frame_seq モードで受理 | 2 |

### e2e（`control-server/scripts/e2e_sim.sh`）

サーバー（sim）＋パッチ後の bridge（`H3_BACKEND=server`）＋模擬ブラウザー（30 Hz POST）＋オペレーター（管理 API）で、次を 1 本の流れで確認する（段階 2）。記録は [runs/](runs/)。

接続しても動かない（1 秒、Pose が届いている状態で）→ ARM（その場で保持）→ START → pitch / roll / yaw 単独と複合の姿勢（`virtual_targets` が混合式と一致、`target_counts` と実位置が校正後の値に収束）→ gain 0.6（`effective_gain` 0.35、`gain_limited`）→ ゲーム停止（同じ frame_seq を送り続ける）で STOPPING → 全軸 HOLDING → BRAKED_DISARMED（その場でブレーキ）→ ゲーム再開・bridge 再接続でも止まったまま → 明示の ARM ＋ START で再開 → 再読み込み（新 session）で停止 → トークンなしの stop → `/v1/center` → 管理者 FAULT とリセット → SIGTERM で正常終了。

模擬軸の異常注入（`stall`・`freeze_status`・`force_fault`・`reboot`）と系統の切り替え（`select_axis`）は HTTP から呼べないので e2e では行わない（Rust の `server.rs`・`supervisor.rs`・`sim_axis.rs` で確認）。e2e は 3 台がそれぞれ別の系統を選んだ通常の割当（`report_selected_axis = true`）で動き、競合・切り替え・`hud_note` の表示は試験しない。e2e の FAULT は管理 API の `/v1/fault`。

## 試験していないこと

- **実機（段階 3〜5）**: CoreS3 ファームウェアが未実装。単軸の位置追従・保持・ブレーキ引継ぎ、軸 2・3 の校正、3 軸同期差、USB 抜去、パケット破損、センサー GND 断、停止距離はどれも未確認。
- **serial backend の実ポート**: USB CDC を開いての運転は未実施（`axis_link` は tokio の duplex で試験）。ESP32-S3 の USB CDC で、ポートを開いたときの DTR / RTS による MCU リセットの有無と、それが boot_id 変化 → FAULT 扱いになるかは未確認。
- **時刻同期**: CLOCK_SYNC と `apply_at_us` は v1 で未実装。3 軸の「同時適用」は `last_applied_frame` の差（frame skew）でしか見ていない。片道遅延は測っていない。
- **負荷と OS**: 遅延の値は macOS（Apple Silicon）・無負荷・debug ビルドのもの。Windows、CPU / GPU 高負荷、ゲームと同じ PC での長時間運転は未計測。
- **ゲーム全体**: h3game の各シーン・AI 応答待ち・音声を通した試験はしていない（e2e と結合試験は POST を模擬）。実際の（ヘッドありの）Chrome でタブを裏に回したときの requestAnimationFrame 停止も未確認（[clients/h3game/README.md](../../clients/h3game/README.md)「確認できていないこと」）。
- **サーバー再起動を bridge 側から通した試験**: bridge を動かしたままサーバーを再起動する試験はない（クライアントの再接続は偽サーバー、サーバー再起動時に ARM しないことは Rust の `server.rs` で別々に確認）。
- **系統選択の端から端まで**: 競合（`axis_conflict`）・待機中 / 駆動中の切り替え・`hud_note` の画面表示は、Rust の結合試験（`server.rs`）と Python の偽サーバー / 偽クライアント試験で別々に確認しただけ。実サーバー ＋ パッチ後の bridge ＋ ページで競合を起こす試験（e2e）は無い（模擬軸の切り替えを HTTP から起こせないため）。
- **物理非常停止**: サーバーは監視していない（`physical_estop: "not_monitored"`）。
- **telemetry の不足**（計画 §8・§11 との差。未実装）: SET_POSITION の送信→MCU 適用の遅延、ゲーム側の source age の分布、管理・イベントのキュー深さ、STATUS の欠落数、USB 書き込み遅延、clock_sync の品質。§8 のベンチ目標（サーバー受信→MCU 適用 p99 20 ms）は今の telemetry では評価できない。ファイルへの JSONL 記録も未配線（イベントはメモリ上のリング `GET /v1/events` と標準エラーだけ）。
- **CoreS3 側の規則への適合**: ACK と STATUS の順序（protocol.md §1）、ACTIVE 以外での停止命令の扱い、CONFIG の検査範囲などは、PC が依存していることを模擬軸の試験で確かめただけ。実ファームウェアがそうなっているかは CoreS3 側の実装・試験で確かめる。

## 最新の結果（2026-10-31、系統選択 DESIGN.md §10 の実装後）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、Python 3.14.4 ＋ websockets 17（`clients/python/.venv`）。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 344 passed、0 failed、1 ignored（上の表の合計。doc-tests 0 件。2 回連続） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| `clients/python/tests` | Ran 34、OK |
| `clients/h3game/tests` | Ran 50、OK（skip なし。`test_browser.py` もヘッドレス Chromium で実行。`H3GAME_SRC` 未設定で、`control-server/target/h3game-src` のクローンを使用。パッチは `hud_note` 対応で作り直したもの） |
| `e2e_sim.sh` | PASS、チェック 120 / 120（[runs/e2e_sim_20261031-161917/summary.md](runs/e2e_sim_20261031-161917/summary.md)。`H3GAME_SRC` 未設定。主な値: Pose の WS 受信 → tick 取り込み p50 5.3 ms / p99 10.6 ms、tick 周期 p99 11.1 ms（overrun 0 / 2240 tick）、frame skew max 1、ゲーム停止 → STOPPING 266 ms、STOPPING → BRAKED_DISARMED 1056 ms、管理者 FAULT → 全軸 FAULT 23 ms） |

この回で足した回帰試験（レビュー指摘の修正）: `supervisor.rs` の `primary_changing_axis_while_driving_latches_axis_changed`（切り替え先を担当していた個体に HOST_FAULT が届き、ACK の後に降格する）と `conflicted_primary_that_never_acks_host_fault_is_demoted_after_ack_timeout`（ACK が無ければ ACK タイムアウトで降格・再送なし）、`server.rs` の `unit_switching_axis_while_active_latches_axis_changed` は軸 2 の個体の停止理由を HOST_FAULT に限定、`test_patch.py` の `test_index_html_shows_server_rig_state_and_lost_session` は `hud_note` の表示を確認。`protocol_vectors.rs` の HELLO / STATUS ベクタは `selected_axis = 1` に変えた（protocol.md §7 も再生成）。

## 2026-10-30 の結果（系統選択の実装前）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、Python 3.14.4 ＋ websockets 17（`clients/python/.venv`）。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 314 passed、0 failed、1 ignored（2 回連続） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| `clients/python/tests` | Ran 30、OK（3 回連続） |
| `clients/h3game/tests` | Ran 46、OK（3 回連続。skip なし。`test_browser.py` もヘッドレス Chromium で実行。`H3GAME_SRC` 未設定で、`control-server/target/h3game-src` のクローンを使用） |
| `e2e_sim.sh` | PASS、チェック 120 / 120（[runs/e2e_sim_20261030-120325/summary.md](runs/e2e_sim_20261030-120325/summary.md)。`H3GAME_SRC` 未設定） |

Python 3.9.6 ＋ websockets 13.1 でも、`clients/python/tests`（Ran 30、OK・2 回）、`clients/h3game/tests`（Ran 46、OK・2 回）と `e2e_sim.sh`（120 / 120、[runs/e2e_sim_20261030-120408/summary.md](runs/e2e_sim_20261030-120408/summary.md)。このときはキャッシュのクローンを消してから実行し、スクリプトが GitHub から clone して `649ab66` を使うことも確認）が通った。3.9 の `clients/h3game/tests` は、これとは別の 1 回で `test_frozen_game_loop_stops_rig_although_posts_continue` が ERROR になった（実行中に Mac がスリープし（pmset のログで 11:46〜12:01）、模擬軸の STATUS が途絶えて軸の `actual_count` が null になった。環境要因と判断した。その後に再実行した 2 回は OK）。

画面を隠した瞬間の POST の修正（段階 C のレビュー指摘）:

- 修正前（確定、sim ＋ヘッドレス Chromium、live 操作で pitch=+1、ACTIVE で [700, -140, 0] を保持）: rAF は POST より先に進むので、隠した直後の POST が「まだ送っていない frame_seq ＋ 0」になり、サーバーは新しいフレームとして受理した。ACTIVE のまま目標が 700 → 605 と中央へ動き、約 250 ms 後に STOPPING、BRAKED_DISARMED で [477, -51, 0]。
- 修正後（確定、同じ手順）: 隠している間の POST は 0 ＋表示中に最後に送った frame_seq。bridge で repeat になりサーバーへ渡らない。目標は動かず、STOPPING → BRAKED_DISARMED で [700, -140, 0]。
- 回帰試験は `test_browser.py`（修正前のパッチに対して実行すると失敗することを確認）。`test_integration.py` の同名の場面は模擬ブラウザーがページの規則をまねたもので、ページ自体の確認ではない。
- legacy の bridge には従来どおり 0 が届く（index.html の 0 の送り方は変えていない）。

e2e の主な値（模擬軸、1 回分。推定値の目安で、保証値ではない）:

| 項目 | 値 |
| --- | --- |
| Pose の WS 受信 → 制御 tick が取り込むまで | p50 5.3 ms、p95 10.2 ms、p99 10.7 ms、max 11.1 ms |
| 制御 tick の周期（目標 10 ms） | p50 10.0 ms、p99 11.1 ms、max 16.4 ms（overrun 1 回 / 2240 tick） |
| 3 軸の frame skew | p99 1 フレーム、max 1 フレーム |
| ゲーム停止（最後に frame_seq が進んだ POST）→ STOPPING | 264 ms（`source_freshness_ms` 250。status の poll 10 ms 分の誤差を含む） |
| STOPPING → BRAKED_DISARMED | 1057 ms（`stop_hold_ms` 1000 を含む） |
| 管理者 FAULT → 全軸 FAULT | 21 ms |

値は [runs/e2e_sim_20261030-120325/summary.md](runs/e2e_sim_20261030-120325/summary.md)（Python 3.14）のもの。
