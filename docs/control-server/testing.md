# h3-control-server の試験

状態: **architecture.md §12 の段階 1（実機なしの互換試験）と段階 2（サーバーとゲームの結合、模擬軸）を実施。段階 3〜5（実機）は未実施。** CoreS3 の v1 ファーム（`moter-controler`、`env:m5stack-cores3-v1*`）はビルドとホストテストまで（実機未確認）。

ここに書く結果はすべて模擬軸（`control-server/src/sim_axis.rs`）と模擬ブラウザー（または ヘッドレス Chromium）でのもの。実機の挙動・停止距離・遅延の根拠にはならない。

## 実行方法

```sh
cd control-server && cargo test && cargo clippy --all-targets -- -D warnings && cd ..
clients/python/.venv/bin/python -m unittest discover -s clients/python/tests -v
clients/python/.venv/bin/python -m unittest discover -s clients/h3game/tests -v
control-server/scripts/e2e_sim.sh
sh moter-controler/tests/run_tests.sh                                    # CoreS3 ファームのホストテスト（c++ だけ。実機不要）
cd moter-controler && /Users/takuro/.platformio/penv/bin/pio run && cd ..  # 6 env のビルド（書き込みはしない）
```

- Python の結合試験は `control-server/target/debug/h3-control-server` を `config/sim.toml` で空きポートに起動する（`H3CS_BINARY` で変更可。なければ skip）。
- h3game の試験と e2e は h3game のクローンを一時ディレクトリに複製してパッチを `git am` する。元のクローンは変更しない。クローンの場所は `H3GAME_SRC`。未設定なら初回に `https://github.com/TakuroFukamizu/20261004_shi3z_hackathon.git` を `control-server/target/h3game-src` に clone し、パッチを作った元のコミット `649ab66` を checkout して使う（ネットワークが要る。以後はそのクローンを使う。`target/` の中なので git の管理外で、`cargo clean` で消える）。`H3GAME_SRC` もネットワークも無ければ、クローンを使う試験（`test_patch.py`・`test_integration.py`・`test_browser.py`）は理由を表示して skip し、`e2e_sim.sh` は理由を表示して終了コード 2 で終わる。
- `test_browser.py` は node・Playwright・Chromium が無ければ skip（`PLAYWRIGHT_MODULE`、`CHROMIUM_PATH`）。

## 一覧

### Rust（`control-server/tests/`、`cargo test`）

| スイート | 件数 | 内容 | 段階 |
| --- | ---: | --- | --- |
| `src/lib.rs`（単体） | 11 | モジュール内の小さい単体試験（`calibration_export` の UTC 時刻・ファイル名・`key = value # comment` の分解、`limits` の clamp・上限・patch の規則・SET_LIMITS の内容を含む） | 1 |
| `compat.rs` | 16 | Pose 検査・gain 制限・混合式・飽和。旧 `h3_direct.H3` の送信値モデル（float 累積・`int()` 切り捨て・送信時クリップ）を Python の記録と照合、計画 §12 の表 | 1 |
| `calibration.rs` | 15 | 3 点区分線形・逆写像・単調減少の軸・不正設定の拒否・ハッシュの安定性。ハッシュは幾何だけ（速度・加速度・保護の判定値は `[limits]` へ移ったので含まない。golden 値を更新） | 1 |
| `planner.rs` | 16 | 3 軸一様の速度制限、加速度制限、目標で止まる（行き過ぎなし）、停止モード、dt 頭打ち | 1 |
| `session.rs` | 29 | 単一所有の session、seq / frame_seq の規則、同じ frame_seq・gain だけでは鮮度を延長しない、source_age、frame_seq なしの扱い、`limits` メッセージ（header の規則、`seq` は成功時だけ消費、鮮度と最新 Pose に触れない、省略項目の解釈、ack の `effective_limits`、`reapply_gain_ceiling` が最新 Pose を鮮度を変えずに再クランプ） | 1 |
| `protocol.rs` | 34 | COBS・フレーム・CRC、全メッセージの往復、ゴミ・途中切断・過大フレーム後の再同期、長さ違い・未知種別・版違いの破棄、AXIS_SELECT（`0x86`）のレイアウト、STATUS offset 13 の `selected_axis`（旧フレームは 0 として読める）、CALIBRATE（`0x0C`、8 byte）と CAL_RESULT（`0x87`、28 byte: `reason` の NUL 埋め・未知の `result` は破棄・予約バイトは無視）、SET_LIMITS（`0x0D`、16 byte）と LIMITS（`0x88`、24 byte）の往復と長さ表 | 1 |
| `protocol_vectors.rs` | 4（＋1 ignored） | protocol.md §7 のテストベクタとバイト単位で一致（HELLO / STATUS のベクタは `selected_axis = 1` で、byte 5 / 13 を埋めないファームでは通らない。7.18 AXIS_SELECT、7.19 CALIBRATE、7.20 CAL_RESULT）。ignored の 1 件は文書用ベクタの生成器。7.21 SET_LIMITS、7.22 LIMITS（ファームの `test_v1_codec.h` と同じ hex） | 1 |
| `sim_axis.rs` | 82 | MCU 参照モデル: protocol.md §3〜§6 の全遷移、期限切れ、epoch・seq 検査、DUPLICATE、異常注入。ACK の後の STATUS は遷移後の状態、ACTIVE 以外での CONTROLLED_STOP・DISARMED/DISARMING での DISARM は ACCEPTED（no-op）、STATUS の `target_count` は軌道上の目標で `following_error_count = actual − target`、系統選択（`select_axis` は DISARMED だけ・CONFIGURED / POSITION_REFERENCED / calibration_hash を消して AXIS_SELECT を 1 回・`select_axis_unchecked` は規則違反・選択で count は動かず再起動後も残る）。校正（protocol.md §1.2）: 端当て → CAL_RESULT(OK, step 3, n_end 6000, neutral 3000, sign 1, `ok`) が DISARMED の STATUS より前・P 端 = 0 で REFERENCED・CONFIG は残る、DISARMED 以外は BAD_STATE・mode 2 は INVALID_PARAM・同じ CALIBRATE の再送は DUPLICATE、CALIBRATE(0) / DISARM で CANCELLED（`stop`）と REFERENCED 消去、HOST_FAULT は CAL_RESULT(FAILED, `fault`) → FAULT_EVENT(`mcu_state_before` 8)、校正中の CONFIG / ARM_PREPARE / START / SET_REFERENCE / FAULT_RESET は BAD_STATE・SET_POSITION は数えずに捨てる・lease なし、注入した失敗（`slow`）と `stall`、機構の端を越えない・端は SET_REFERENCE / 再起動と一緒に動く。SET_LIMITS / LIMITS（design 2026-11-01）: HELLO の capabilities bit3、どの状態でも受理・ACK の後に LIMITS 1 回・絶対上限 60 % でクランプ・INVALID_PARAM の 4 条件・DUPLICATE と FAULT_ACTIVE では LIMITS なし、CONFIG の速度・加速度を 6000 / 40000 でクランプ、STOPPING の減速が最後のフレームの `max_acc`（2000 で約 1 s、40000 で 50 ms） | 1 |
| `supervisor.rs` | 75 | リグ状態機械（3 台の SimMcu、10 ms の偽時計。管理メッセージの配送は server.rs と同じく primary のポートだけ）: 接続で動かない、ARM 阻害要因、ゲーム途絶で減速・保持・ブレーキ（中央へ戻さない）、STOP 後の Pose で再開しない、1 軸の異常で全軸 HOST_FAULT、FaultReset、frame skew、主電源担当（STATUS の `POWER_CONTROLLER`）が設定と違えば ARM しない、READY_TO_ARM で軸が再起動すると FAULT にせず `not_configured`・`not_referenced` で ARM を拒否し、SET_REFERENCE をやり直すまで READY_TO_ARM に戻らない。系統選択（DESIGN.md §10）: 割当は `selected_axis`、0 なら設定の device_id、どちらも無ければ `unknown_device`、設定 device_id との不一致は両方向とも `axis_device_mismatch`、待機中・ARMING 中・駆動中の 2 台目（競合 → 減速停止 / DISARM してから競合、競合解消で残った個体を昇格）、待機中の切り替えで再割当・CONFIG・`reference_stale`（再起動で解除）、駆動中の担当個体の切り替えで `axis_changed` ラッチ — 切り替え先を担当していた個体には HOST_FAULT が届いてから降格（`primary_changing_axis_while_driving_latches_axis_changed`）、ACK が無ければ ACK タイムアウトで降格し再送しない（`conflicted_primary_that_never_acks_host_fault_is_demoted_after_ack_timeout`）、未選択への切り替えは `unknown_device`。校正（DESIGN.md §11）: 全軸 CALIBRATE(1) → `calibrating: axisN` で ARM / START / CENTER / REFERENCE / Calibrate が `calibrating`・SET_POSITION も CONFIG も出ない → CAL_RESULT が snapshot に入り `referenced`・SET_REFERENCE なしで READY_TO_ARM、`power_axis_required` / `axis_disconnected` / `mcu_not_disarmed` / `not_disarmed`（ARM 中）/ `fault_latched`、cancel と Stop で CALIBRATE(0) → CANCELLED・`not_referenced`、電源担当軸の REJECTED で他を中止・`calibrate_rejected`・FAULT にしない、ACK が来ない CALIBRATE は ACK タイムアウトで忘れ（電源担当軸なら他を中止）次の校正を受ける、FAILED は記録して `not_referenced`、結果は同じ boot の再接続で残り再起動で消える、校正中の STATUS 途絶は FAULT にしない、OK の結果で `reference_stale` が消える。動きの制限値: 識別直後に CONFIG（絶対上限入り・`[limits]` の保護値）と SET_LIMITS（`[limits]` の値）を 1 回ずつ、LIMITS が `axes[].fw_limits` に入り `motion_limits` が初期値、上限超えの要求は 4 項目ともクランプされ SET_LIMITS は変わった分だけ 1 回・同じ要求では再送なし・速度だけの変更では MCU へ送らない、ACTIVE 中の速度・加速度の変更が次の SET_POSITION から効く、再接続した軸にだけ再送、capabilities bit3 の無い軸は `firmware_limits_unsupported` で SET_LIMITS を送らない、リンクを保ったままの再起動で SET_LIMITS を再送し古い LIMITS を消す、SET_LIMITS が ACK され LIMITS が届くまで `limits_unconfirmed` で ARM 阻害（ACK タイムアウト後と、ACK は来たが LIMITS が落ちたときは 1 秒後に新しい request_id で再送） | 1 |
| `axis_link.rs` | 32 | HELLO 照合までの送信制限、SET_POSITION 最新 1 件、管理命令の優先、STATUS 途絶、書き込みタイムアウトの隔離、系統選択: HELLO の `selected_axis` で割当（device_id ではない）、範囲外は `unknown_device`、AXIS_SELECT / 別の `selected_axis` の HELLO / STATUS で同じポートのまま `Disconnected{axis_changed}` → `Identified`、0 への切り替えは設定 device_id に戻すか `unknown_device` でポートを閉じる、STATUS の 0 では切り替えない。シリアルを開いたあとの DTR: `assert_cdc_dtr` が DTR を 1 回だけ立て RTS に触らない・ioctl 失敗は panic せず Err（偽の SerialPort で。実ポートは開かない） | 1 |
| `config.rs` | 26 | 設定の読み込みと検証、`[backend] report_selected_axis` の既定 true、`end_min_count / end_max_count` の既定 ±3000 と順序の検査、全軸 `device_id = 0`（照合なし）が有効、`[limits]` の規則（gain は (0, 1]、Duty 1〜100、duty_min ≤ duty_max、ff > 0、deadband_out ≥ deadband、overspeed ≥ 1.5 × max_vel、既定値）、schema 1 のファイル・`[game].max_gain`・`[[axis]]` の速度は読み込みで失敗 | 1 |
| `calibration_export.rs` | 4 | CAL_RESULT → 設定値（sign ±1、i32 からの逸脱はエラー）、`config/bringup.toml` と `config/sim.toml` のテキストの `[[axis]]` ブロック書き換え（対象の行だけ変わりコメントは残る・結果が `Config` として読める・コメント中の `[[axis]]` は見出しにしない・キー欠落と重複はエラー）、ファイル名 | 1 |
| `telemetry.rs` | 12 | 百分位、リング上書き、落とした件数、`after` 付きの読み出し、JSONL 記録器 | 1 |
| `types.rs` | 5 | AxisId の変換、FaultCode の wire 値（0〜12 が定義、`LOCAL_ESTOP` = 12、それ以外は UNKNOWN）と JSON 名（SCREAMING_SNAKE_CASE）、`McuState::Calibrating` = 8（JSON `CALIBRATING`） | 1 |
| `api.rs` | 36 | Origin 403、同一 origin（Host が loopback で `Origin` = `http://<Host>`）は通す・Host と一致しても loopback でない名前（DNS リバインディング）/ 別ポート / https / 末尾スラッシュ / LAN アドレス / Host 無しは 403・Origin 無しは従来どおり・同一 origin でも本文過大 413 と rate limit はそのまま、試験ページ `GET /`・`/index.html`（200 text/html・`no-store`・埋め込んだ HTML と同一・外部資源なし・POST は 405・LAN からは 403）、ジョイスティック試験ページ `GET /joystick`・`/joystick.html`（同じ検査。`window.H3Joy`・`navigator.getGamepads`・`requestAnimationFrame`・`frame_seq` を含む、他の Origin からは 403、LAN からは 403）、トークン 401、本文過大 413、rate limit、503、WS の ack と status、LAN からの制限、`GET /v1/events`（`n`・`after`・不正な値は 400・制御ループが落ちた後も読める・LAN からは 403）、競合中は pose / gain を `axis_conflict` で拒否し intake に届けない、arm / start / center は 409 `axis_conflict`。校正: `/v1/calibrate`（本文省略 / `axes` / `bad_axis` / `bad_request`）・`/v1/calibrate/cancel`・`/v1/calibration/export` はトークン必須・Origin 検査・LAN から 403、supervisor の拒否は 409 の code と blockers、export は設定ファイルの隣に書き（sign +1 と −1 の値、FAILED の軸はそのまま、コメントと `# 模擬軸のみ` が残る、`Config::load` で読める）・同名を上書きしない（409 `exists`）・OK の結果なしは 409・不正な結果は 500 で何も書かない・config path なしは 500。動きの制限値: `GET /v1/limits`（初期値・模擬軸の上限）→ WS `limits`（gain 0.8 と速度 7000 → 6000 で `ceiling_clamped`・`fresh` false → 次の pose は 0.7 で通る → 不正値は `out_of_range` で seq を消費しない → status と `axes[].fw_limits` に反映）→ `POST /v1/limits`（401 / 400 / 200）→ 新 session で初期値に戻る、競合中の `limits` は `axis_conflict`、status の `motion_limits` と `axes[].fw_limits` の形 | 1・2 |
| `server.rs` | 20 | 実サーバー（sim）: 追従 → 停止 → ブレーキ（中央へ戻さない）、1 軸の STATUS 停止（`freeze_status`）で全体 FAULT、起動で動かない、再接続・サーバー再起動で ARM しない、走行中の終了は その場で DISARM、走行中の新 session で停止、走行中の軸異常（`force_fault`）の前後の経過（ARM・START・ACTIVE → fault → FAULT_LATCHED → 解除操作）を `GET /v1/events` で読み戻せる。系統選択: ポート順を入れ替えても `selected_axis` で割当、待機中の切り替えで再割当と `reference_stale`（SET_REFERENCE まで ARM 409）、同じ系統を 2 台が選ぶと `axis_conflict`（ARM 409・WS pose 拒否・stop は 200）で片方を戻すと解消、駆動中の切り替えで FAULT_LATCHED（切り替え先を担当していた個体も HOST_FAULT で止まる。lease 切れではない。切り替えた個体自身は PC から何も受けず COMMAND_TIMEOUT で止まる）、`report_selected_axis = false` で旧 device_id 経路。`LOCAL_ESTOP`（12）を模擬軸が報告すると FAULT_LATCHED になり、`/v1/status` の `fault.code`・`axes[].fault_code` が `LOCAL_ESTOP`、blockers に `axis_fault: axis2 fault_code LocalEstop`、FAULT RESET で READY_TO_ARM に戻る。校正: 未基準の sim 3 台に `POST /v1/calibrate` → 全軸 CALIBRATING（ARM は 409 `calibrating`）→ CAL_RESULT OK・`referenced`・P 端 = 0 → blockers が `calibration_invalid` だけ → export したファイルが読めて値（mechanical [0, 1000]、soft [300, 700]、count_at_virtual [300, 500, 700]、`calibration_valid = false`）が合う → そのファイルで起動したサーバーでも校正後は `calibration_invalid` だけ、`{"axes":[2,3]}` は `power_axis_required`、校正中の `/v1/stop` で CANCELLED・未基準に戻り export は 409 | 2 |

### Python

| スイート | 件数 | 内容 | 段階 |
| --- | ---: | --- | --- |
| `clients/python/tests/test_motion_client.py` | 32 | 偽サーバー相手: 送信規則（新しい submit だけ送る、seq 単調、source_age をキュー待ちで作り直さない）、再接続とバックオフ、**WS の再接続は同じ session を使い回す**（切断 3 回で `POST /v1/session` は 1 回、受信停止の再接続でも発行しない、`stale_session` の後の再接続では新 session、`reconnect()` の POST が 503 で失敗したら次も POST し直して古い id に戻らない、POST は成功したが WS の接続に失敗したら次は**新しい** id で繋ぐ（古い id に戻らない）。Issue #15 M5）、stale_session 後にその場では取り返さない、送信がブロックしない、float にできない巨大な整数・2⁶⁴−1 を超える frame_seq は例外ではなく `False`、`axis_conflict` の拒否では再接続せず送り続けてログは 1 回（他の拒否理由では `axis_conflict` を立てない）、`_summary` に各軸の割当（port / device_id / claimants / `reference_stale`）と競合を出し古い形の status でも落ちない、`submit_limits`（合成・Pose より先に送る・省略項目は送らない・ack が `last_limits_ack`・拒否も入る） | 1 |
| `clients/python/tests/test_against_server.py` | 8 | 実サーバー（sim）: ARM しない、gain 上限（0.5）、`submit_limits` で gain の上限が変わり速度は 6000 で切られ status / `GET /v1/limits` に出て新 session で戻る、frame_seq なしの拒否、同じ frame_seq で鮮度が切れる、トークンなしの stop、走行中の再接続で停止、クライアントが許す最大の frame_seq（2⁶⁴−1）をサーバーが受理 | 2 |
| `clients/python/tests/test_motion_client.py` | 28 | 偽サーバー相手: 送信規則（新しい submit だけ送る、seq 単調、source_age をキュー待ちで作り直さない）、再接続とバックオフ、stale_session 後に取り返さない、送信がブロックしない、float にできない巨大な整数・2⁶⁴−1 を超える frame_seq は例外ではなく `False`、`axis_conflict` の拒否では再接続せず送り続けてログは 1 回（他の拒否理由では `axis_conflict` を立てない）、`_summary` に各軸の割当（port / device_id / claimants / `reference_stale`）と競合を出し古い形の status でも落ちない、`submit_limits`（合成・Pose より先に送る・省略項目は送らない・ack が `last_limits_ack`・拒否も入る） | 1 |
| `clients/python/tests/test_against_server.py` | 8 | 実サーバー（sim）: ARM しない、gain 上限（0.5）、`submit_limits` でゲームが上げた gain 0.8 / 速度 7000 は操作者の 0.5 / 3000 で切られ（`operator_limited`）、下げた gain 0.3 は効き、status（`operator_max` / `game_fields`）/ `GET /v1/limits` に出て新 session で戻る、frame_seq なしの拒否、同じ frame_seq で鮮度が切れる、トークンなしの stop、走行中の再接続で停止、クライアントが許す最大の frame_seq（2⁶⁴−1）をサーバーが受理 | 2 |
| `clients/h3game/tests/test_server_backend.py` | 32 | bridge 用バックエンド層（偽クライアント）: frame_seq の扱い、owner 変更、無効値（float にできない巨大な整数、2⁶⁴−1 を超える frame_seq は `invalid` で、以後のフレームを塞がない）、`/state` のキー（session を取られたら `connected` は false、status の `axis_conflict` と blockers・ack の拒否理由から `hud_note`「系統の競合(軸1、入力拒否)」「個体の不一致」「再基準化が必要」を作る、他の blocker は載せず届いていなければ `null`、壊れた入力でも落ちない）、並行 POST、POST の `limits` オブジェクトの転送（姿勢と独立・未知キーは無視・数値でなければ `invalid`） | 2 |
| `clients/h3game/tests/test_patch.py` | 11 | パッチの適用（blob SHA・変更ファイル・正本との一致）、index.html の隠れたときの規則（0 ＋表示中に最後に送った frame_seq）と server 用の HUD 表示（`hud_note` を `srvNote()` に足す）、legacy は元の bridge と同じ応答、server・dry は serial / h3_direct を import しない、サーバー不在でも POST が待たされない、壊れた数値の POST にも応答して `invalid` に数える、ポート使用中の 2 つ目の bridge はサーバーへ 1 回も要求せずに終わる、LAN のアドレスからの `GET /reconnect`・`/claim` は無視（LAN の IPv4 アドレスが無ければこの 1 件は skip） | 2 |
| `clients/h3game/tests/test_integration.py` | 7 | 実サーバー（sim）＋パッチ後の bridge: 追従、ゲーム停止で STOPPING → BRAKED_DISARMED（中央へ戻さない）、POST が続いても frame_seq が止まれば停止、ACTIVE 中に画面を隠しても目標が動かずその位置で停止（ページの規則を模擬ブラウザーで再現したもの。ページ自体の確認は test_browser.py）、再読み込みで新 session・自動再開なし、走行中に bridge を二重起動しても session は変わらず走行が続く・別のクライアントに session を取られると bridge の `/state` が `connected: false`・`session_lost: true`、frame_seq なしは START 不可、dry は何も送らない | 2 |
| `clients/h3game/tests/test_browser.py` | 1 | ヘッドレス Chromium でパッチ後の index.html: frame_seq が増える。rAF 直後（未送信のフレームがある瞬間）に `document.hidden` を偽装し、隠した後の最初の POST から全て「0 ＋表示中に最後に送った frame_seq」。サーバー接続時は live 操作で pitch=+1 を保ったまま ARM・START し、隠しても ACTIVE の間の目標カウントが動かず、BRAKED_DISARMED でその位置に止まる。サーバーが frame_seq モードで受理 | 2 |

### CoreS3 ファーム（`moter-controler/tests/run_tests.sh`、ホストの c++ だけ）

| スイート | 件数（CHECK） | 内容 | 段階 |
| --- | ---: | --- | --- |
| `tests/test_host.cpp`（従来 env の純粋部分） | 236 | プロファイル・端ラベル・自動移動（`auto_move.h`）・手動速度の守り（`manual_guard.h`）など | 1 |
| `tests/test_v1_codec.h` | — | COBS・CRC・フレーム、protocol.md §7 の全ベクタの encode / decode（7.21 SET_LIMITS、7.22 LIMITS を含む）、CRC 不一致・長さ違い・版違い・未知種別・256 byte 超の破棄 | 1 |
| `tests/test_v1_axis.h` | — | `V1Axis` の規則（設計 §6 の各項目）。レビュー修正の回帰試験: DISARM 後の残り誤差で `ENCODER_FAULT` にならず DISARMED に着く（`test_disarm_with_residual_error_reaches_disarmed_not_encoder_fault`）、HOLDING で目標が止まっているときの最小 Duty の静止は `ENCODER_FAULT` にせず、軌道が動けば従来どおり出る（`test_static_hold_error_at_min_duty_is_not_encoder_fault`）、ACTIVE で到達後の残り誤差も出ない（`test_active_settled_with_residual_error_is_not_encoder_fault`）、`device_id` は MAC を表記順に詰める（`test_device_id_packs_mac_in_printed_order`）。SET_LIMITS / LIMITS（2026-11-01）: どの状態でも受理、INVALID_PARAM の各条件で `tuning` が変わらず LIMITS も出ない、90 → 60 のクランプ、LIMITS の中身と絶対上限、FAULT 中は FAULT_ACTIVE、DUPLICATE は LIMITS なし、HELLO の capabilities、CONFIG 99999 → 6000、STOPPING の減速がフレームの `max_acc`（2000 で約 1 s、40000 で ≤ 100 ms） | 1 |
| 合計 | 3758 | 件数は CHECK の数（スイートごとの内訳は出さない）。v1 追加前は 236。2026-11-01 に速度フィードフォワードと不感帯のヒステリシス（`v1_duty_command`）の試験を追加（1676）、同日に校正（CALIBRATE / CALIBRATING / CAL_RESULT。模擬機構での成功・失敗・中止・FAULT とベクタ）の試験を追加（2720）、同日に KEEP_POWER（3542）と SET_LIMITS / LIMITS（3758）の試験を追加 | 1 |

### e2e（`control-server/scripts/e2e_sim.sh`）

サーバー（sim）＋パッチ後の bridge（`H3_BACKEND=server`）＋模擬ブラウザー（30 Hz POST）＋オペレーター（管理 API）で、次を 1 本の流れで確認する（段階 2）。記録は [runs/](runs/)。

接続しても動かない（1 秒、Pose が届いている状態で）→ ARM（その場で保持）→ START → pitch / roll / yaw 単独と複合の姿勢（`virtual_targets` が混合式と一致、`target_counts` と実位置が校正後の値に収束）→ gain 0.6（`effective_gain` 0.5 = `[limits].max_gain`、`gain_limited`）→ ゲーム停止（同じ frame_seq を送り続ける）で STOPPING → 全軸 HOLDING → BRAKED_DISARMED（その場でブレーキ）→ ゲーム再開・bridge 再接続でも止まったまま → 明示の ARM ＋ START で再開 → 再読み込み（新 session）で停止 → トークンなしの stop → `/v1/center` → 管理者 FAULT とリセット → SIGTERM で正常終了。

模擬軸の異常注入（`stall`・`freeze_status`・`force_fault`・`reboot`）と系統の切り替え（`select_axis`）は HTTP から呼べないので e2e では行わない（Rust の `server.rs`・`supervisor.rs`・`sim_axis.rs` で確認）。e2e は 3 台がそれぞれ別の系統を選んだ通常の割当（`report_selected_axis = true`）で動き、競合・切り替え・`hud_note` の表示は試験しない。e2e の FAULT は管理 API の `/v1/fault`。

## 試験していないこと

- **実機（段階 3〜5）**: CoreS3 の v1 ファームは実機未確認（ビルドとホストテストだけ。手順は [hardware-bringup.md](hardware-bringup.md)）。単軸の位置追従・保持・ブレーキ引継ぎ、軸 2・3 の校正、3 軸同期差、USB 抜去、パケット破損、センサー GND 断、停止距離はどれも未確認。位置制御の仮値（`[limits]`: KP 2 %/100 count、Duty 3〜40 %、速度 3000 count/s、加速度 15000 count/s²。[limits.md](limits.md)）、無パルス判定（軌道が動いている間だけ）、ブレーキ・電源の待ち時間（各 300 ms）が実機で成り立つかも未確認。
- **serial backend の実ポート**: USB CDC を開いての運転は未実施（`axis_link` は tokio の duplex で試験）。`open_serial` はポートを開いたあと DTR を明示的に立てる（ESP32-S3 の HWCDC は DTR が無いと送信しない。RTS には触らない）が、これは偽の SerialPort で呼び出しを確かめただけで、実機で HELLO が届くこと、ポートを開いたときの DTR / RTS による MCU リセットの有無と、それが boot_id 変化 → FAULT 扱いになるかは未確認。
- **試験ページ（`GET /`）の実機操作**: 模擬軸とヘッドレス Chromium で一通り操作しただけ（下の 2026-10-31 の結果）。実機 3 台をつないだときの表示・操作は未確認。
- **時刻同期**: CLOCK_SYNC と `apply_at_us` は v1 で未実装。3 軸の「同時適用」は `last_applied_frame` の差（frame skew）でしか見ていない。片道遅延は測っていない。
- **負荷と OS**: 遅延の値は macOS（Apple Silicon）・無負荷・debug ビルドのもの。Windows、CPU / GPU 高負荷、ゲームと同じ PC での長時間運転は未計測。
- **ゲーム全体**: h3game の各シーン・AI 応答待ち・音声を通した試験はしていない（e2e と結合試験は POST を模擬）。実際の（ヘッドありの）Chrome でタブを裏に回したときの requestAnimationFrame 停止も未確認（[clients/h3game/README.md](../../clients/h3game/README.md)「確認できていないこと」）。
- **サーバー再起動を bridge 側から通した試験**: bridge を動かしたままサーバーを再起動する試験はない（クライアントの再接続は偽サーバー、サーバー再起動時に ARM しないことは Rust の `server.rs` で別々に確認）。
- **系統選択の端から端まで**: 競合（`axis_conflict`）・待機中 / 駆動中の切り替え・`hud_note` の画面表示は、Rust の結合試験（`server.rs`）と Python の偽サーバー / 偽クライアント試験で別々に確認しただけ。実サーバー ＋ パッチ後の bridge ＋ ページで競合を起こす試験（e2e）は無い（模擬軸の切り替えを HTTP から起こせないため）。
- **物理非常停止**: サーバーは監視していない（`physical_estop: "not_monitored"`）。
- **telemetry の不足**（計画 §8・§11 との差。未実装）: SET_POSITION の送信→MCU 適用の遅延、ゲーム側の source age の分布、管理・イベントのキュー深さ、STATUS の欠落数、USB 書き込み遅延、clock_sync の品質。§8 のベンチ目標（サーバー受信→MCU 適用 p99 20 ms）は今の telemetry では評価できない。ファイルへの JSONL 記録も未配線（イベントはメモリ上のリング `GET /v1/events` と標準エラーだけ）。
- **CoreS3 側の規則への適合**: ACK と STATUS の順序（protocol.md §1）、ACTIVE 以外での停止命令の扱い、CONFIG の検査範囲などは、PC が依存していることを模擬軸の試験で確かめただけ。実ファームウェアがそうなっているかは CoreS3 側の実装・試験で確かめる。

## 2026-11-02 の結果（Issue #36: ジョイスティックページで START などの拒否理由・「中央へ」の案内・session 切れのバナーを出す）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、node 26.3、Playwright 1.65 alpha（npx のキャッシュ）＋ chromium_headless_shell 1243 を `executablePath` で指定。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 439 passed、0 failed、1 ignored（`api.rs` 37: `joystick_page_is_served_and_self_contained` に `#opResult` / `#startHint` / `#sessionBanner`・`adminReject` / `startGateHint`・`j.blockers`・`pose_not_near_hold`・`stale_session`・案内文の検査と「`innerHTML =` を使わない」を追加） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| ページの JS | 2 つの `<script>` を `vm.Script` で構文検査、id の重複なし、`$()` の id は全部存在。`H3Joy` を node で評価（24 項目）: `adminReject` は成功で null、409 `pose_not_near_hold` で 1 行目「START 拒否: 409 pose_not_near_hold — 説明」・blockers をそのまま 1 行ずつ・速度モードは「速度モードは「中央へ」で姿勢を 0 に戻してから START」、位置モードは「スティックを中央」、指令姿勢がもう 0（または 0 固定）なら「CENTER で中立へ」の案内、401 `unauthorized`（blockers なし）はトークンの説明、503 `control_loop_unavailable` は「実行されていない」、JSON でない本文（502 Bad Gateway）は本文をそのまま、届かない（`netError`）は「サーバーに届かない」、未知の符号と配列でない blockers でも落ちない。`startGateHint` は速度モード・START の前の状態（HOLDING / READY_TO_ARM など）・姿勢あり・0 固定でない・gain > 0・\|pitch\| か \|roll\| が 0.02 超のときだけ文を返す（位置モード・ACTIVE・FAULT_LATCHED・姿勢なし・0 固定・gain 0・0.02 ちょうどは null） |
| ジョイスティック試験ページ（ヘッドレス Chromium、sim、空きポート） | 手で書いた Playwright スクリプト（リポジトリには入れていない。`navigator.getGamepads` を差し替え。`config/sim.toml` を作業用ディレクトリに複製して `start_max_offset_count = 50` にしたもの（`kind = "sim"`、シリアルポートは開かない）。サーバーはスクリプトが起動・強制終了・再起動する）。52 項目すべて通過、ページエラー・console エラー 0（サーバー停止中の接続失敗は除く）: 読み込みだけでは何も出ない → トークン無しの ARM → ボタンの下に赤で「ARM 拒否: 401 unauthorized — 管理トークンが無いか違う」→ トークンを入れて ARM 200 で消える → HOLDING → 速度モードで roll を 0.7 s 倒して離す（指令 roll 0.40）→ START の下に黄で「指令姿勢が中央から離れている（roll +0.40 / pitch +0.00）。速度モードは「中央へ」で姿勢を 0 に戻してから START」、START の title も同じ文、START は押せる → **START → 「START 拒否: 409 pose_not_near_hold」とサーバーの blockers（`axis1 target 403 is +403 counts from hold 0 (max 50)`、axis2 も）が 1 行ずつ、「→ 速度モードは「中央へ」で…」**、HOLDING のまま → 「中央へ」で黄の「中央へ: 指令姿勢を 0 にした → START を押し直す」に置き換わり、案内が消え、サーバーの `requested_pose` が 0 → START 200 で消え ACTIVE（案内は出ない）→ 別のクライアントの `POST /v1/session` → `stale_session` で送信停止と赤いバナー「session が無効になった（stale_session: …）。もう一度「送信開始」を押してください」→ 送信開始でバナーが消える → **送信中にサーバーを強制終了** → バナー「WS が切れた（close 1006。サーバーの再起動か接続断）…「送信開始」」、WS は赤の `closed (close 1006)` → 停止中の STOP は「STOP: サーバーに届かない（TypeError: Failed to fetch）」→ サーバー再起動で status のバナーは消えるが session のバナーは残る → 送信開始で ack accepted、バナーが消える → STOP 200 で拒否表示が消える → 操作者の「送信停止」ではバナーを出さない → **スティックの無いページ**で送信開始（姿勢が無いので何も送らず ack も来ない）→ 別のクライアントが session を発行 → status の確認（2 回続けて）で送信が止まり（`closed (stale_session (status))`、ボタン「送信開始」）バナー「サーバーの session がこのページのものでなくなった…送信は止めた」→ 「送信開始」1 回で開きバナーが消える |

変えたこと: `static/joystick_page.html`（`#opResult`・`#startHint`・`#sessionBanner`、`H3Joy.adminReject` / `H3Joy.startGateHint`、STOP の結果も表示、送信中の意図しない WS close を赤に）、`tests/api.rs`（ページの文字列検査）、`DESIGN.md` §7.5。サーバーも WS の約束も変えていない。実機ではまだ使っていない。

## 2026-11-02 の結果（Issue #15: ゲーム経路の制限値の上限・模擬軸の開け直し・StartTimeout）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、Python 3.14.4 ＋ websockets 17（`clients/python/.venv`）。サーバーだけの変更（ファームは触っていない）。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 438 passed、0 failed、1 ignored（この PR で 5 本追加: `limits.rs` 2・`supervisor.rs` 2・`server.rs` 1。WS の limits 試験は操作者の上限で書き直し） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| `clients/python/tests` | Ran 36、OK（`test_against_server.py` の limits 試験を `operator_limited` に書き直し） |
| `clients/h3game/tests` | Ran 51、OK（`H3GAME_SRC` に既存のクローン。パッチの `h3_motion_client.py` の複製は docstring だけ更新） |
| `e2e_sim.sh` | PASS、チェック 120 / 120（`--out` を作業用ディレクトリにしたので `runs/` には置いていない。e2e は limits を送らないので操作者の上限は通らない） |

変えたこと: WS `limits` で書いた項目は `[limits]` の初期値を超えない（`operator_limited`、status / `GET/POST /v1/limits` に `operator_max` / `game_fields`、limits.md §1）、模擬軸のポートもリンクが閉じたら `SERIAL_RETRY` 後に同じ `SimMcu` へ開け直す（`sim_port_task`、DESIGN.md §10.3）、START が確認できないときの FAULT を `StartTimeout { axis }` にして detail に `ack_timeout: …` か未到達の状態を書く（DESIGN.md §5 / §7.2）。

## 2026-11-01 夕方の結果（トークン既定・`sim_axes`・USB 方式の戻し・ファームの KEEP_POWER の後）

実機の記録は [runs/hw_20261101.md](runs/hw_20261101.md)。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 390 passed、0 failed、1 ignored（校正 386 ＋ `admin_routes_need_no_token_by_default`・`serial_backend_sim_axes_stand_in_for_missing_units`・`non_loopback_bind_requires_admin_token`・`serial_sim_axes_validation`） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| `clients/python/tests` / `clients/h3game/tests` | 34 OK / 50 OK（校正の取り込み時に実行） |
| `e2e_sim.sh` | PASS、120 / 120（校正のサーバー側を取り込んだ直後に scratch へ出力。`runs/` には残していない） |
| `moter-controler/tests/run_tests.sh` | 5364 passed、0 failed（CHECK の数。KEEP_POWER の 9 本を追加。前回 2720） |
| `pio run` | `m5stack-cores3-v1` と `m5stack-cores3` を SUCCESS で確認（他の 4 env は前回のまま） |

この回の変更: `server.require_admin_token`（既定 false）、`[backend] sim_axes`、status の `decode_errors`、テストページのタブ非表示停止の撤去、v1 ファームの USB を loop() 方式に戻し（専用タスクは CRC 不一致・USB 無応答を起こした）、校正の `KEEP_POWER`（ファーム側。サーバー側は別途）。

## 2026-11-01 の結果（2026-11-01、CoreS3 v1 ファームのレビュー修正後）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、Python 3.14.4 ＋ websockets 17（`clients/python/.venv`）、PlatformIO（`~/.platformio/penv/bin/pio`、espressif32@6.12.0、Arduino core 2.0.17）、Apple clang（`c++ -std=c++17`）。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 354 passed、0 failed、1 ignored（前回と同じ。サーバー側の変更なし） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| `clients/python/tests` | Ran 34、OK |
| `clients/h3game/tests` | Ran 50、OK（skip なし。`test_browser.py` もヘッドレス Chromium で実行） |
| `e2e_sim.sh` | PASS、チェック 120 / 120（[runs/e2e_sim_20261101-112206/summary.md](runs/e2e_sim_20261101-112206/summary.md)。主な値: ARM → HOLDING 90 ms、ゲーム停止 → STOPPING 255 ms、STOPPING → BRAKED_DISARMED 1064 ms、管理者 FAULT → 全軸 FAULT 20 ms） |
| `h3-control-server check-config config/bringup.toml` | OK。静的な阻害要因は `calibration_invalid` ×3 だけ（意図どおり。`device_id` は 0 = 照合なし） |
| `moter-controler/tests/run_tests.sh` | 2720 passed、0 failed（CHECK の数。従来 266（main 95e0069 の中立・手動速度の修正を含む）＋ v1。前回 1676。校正の追加分 1044） |
| `pio run`（6 env、書き込みなし） | 全 SUCCESS、警告 0。`m5stack-cores3` RAM 24080 / Flash 786833 byte、`-usbpower` 24080 / 786777、`-pmiclog` 24128 / 788021、`-pmiclog-nobatt` 24128 / 788441、`-v1` 27800 / 1059725、`-v1-usbpower` 27800 / 1059649（RAM 327680、Flash 6553600 byte 中） |

この回の修正（レビュー指摘。ファーム設計 §3 を先に直した）:

- `v1_axis.h`: `ENCODER_FAULT` の無パルス判定を「ブレーキ解除指令中に軌道が動いている間」だけに限定。従来は DISARMING（ブレーキ作動指令中）や、目標が止まっているときに最小 Duty で動かない静止でも 200 ms で FAULT になり、解除のたびに `POSITION_REFERENCED` が消えて再基準化が要った。回帰試験は上の表（修正前のコードで 11 件 FAIL することを確認）。
- `v1_axis.h` / `v1_link.cpp`: `device_id` を `0x5131` + MAC の表記順（先頭オクテットが上位）に変更（`ESP.getEfuseMac()` は逆順）。画面の `dev XXXX` = MAC の末尾 2 オクテット、status の `device_id` = `5131` + MAC の 12 桁。
- `platformio.ini`: v1 の 2 env を `CORE_DEBUG_LEVEL=0`（Arduino core の `log_e()` が USB CDC に文字を出さないように）。`main.cpp` の `esp_log_level_set` を `M5.begin()` より前へ。
- `fault_cause_present()` は画面の非常停止のラッチだけ（設計 §3 の「エンコーダが無パルスのまま」を削除。FAULT 中は観測できない）。
- `hardware-bringup.md`: `fault/reset` は 200 を返し拒否は `blockers` の `fault_reset_rejected` に出る（§6.3 / §10）、`device_id` のバイト順（§1.1 / §4.1 / §4.2）、未設定の個体は `#1` を名乗る（§2 / §10）、HELLO 途絶は 3 s ごとに開き直す（§10）、ソフトリミットの検査式（§5.3）、409 の code 名（§10）、§6.3 の期待（ずれが小さければ `ENCODER_FAULT` にならず ACTIVE のまま）。

## 2026-11-01 の結果（校正 CALIBRATE / CAL_RESULT・`/v1/calibrate`・`/v1/calibration/export` の追加後）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、Python 3.14.4 ＋ websockets 17（`clients/python/.venv`）。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 386 passed、0 failed、1 ignored（上の表の合計。前回 354 ＋ `lib.rs` 3・`protocol.rs` 2・`sim_axis.rs` 7・`supervisor.rs` 9・`config.rs` 1・`calibration_export.rs` 4・`types.rs` 1・`api.rs` 3・`server.rs` 2） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| `clients/python/tests` | Ran 34、OK |
| `clients/h3game/tests` | Ran 50、OK（skip なし。`H3GAME_SRC` に既存のクローンを指定） |
| `e2e_sim.sh` | PASS、チェック 120 / 120（`--out` を作業用ディレクトリにしたので runs/ には置いていない。e2e は校正を行わない） |
| `check-config` | `config/bringup.toml`（静的な阻害要因は `calibration_invalid` × 3）、`config/sim.toml`（なし）とも OK |

校正の試験は模擬軸（`end_min_count / end_max_count`、P 端 = count 最小の端、sign +1）に対するもので、実機の端当て（`auto_move.h` の無パルス・遅すぎ・移動量・時間の守り）と CAL_RESULT の `reason` の実際の値は CoreS3 側のホストテストと実機で確認する。

## 2026-10-31 の結果（`LOCAL_ESTOP`・試験ページ・DTR の追加後）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、Python 3.14.4 ＋ websockets 17（`clients/python/.venv`）。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 354 passed、0 failed、1 ignored（上の表の合計。前回 344 ＋ `types.rs` 2・`api.rs` 5・`axis_link.rs` 2・`server.rs` 1） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| `clients/python/tests` | Ran 34、OK |
| `clients/h3game/tests` | Ran 50、OK（skip なし。`test_browser.py` もヘッドレス Chromium で実行） |
| `e2e_sim.sh` | PASS、チェック 120 / 120（`--out` を作業用ディレクトリにしたので runs/ には置いていない） |
| 試験ページ（ヘッドレス Chromium、sim） | 手で書いた Playwright スクリプトで確認（リポジトリには入れていない）: 読み込みだけでは POST も session 発行も無し → ARM が 401 の本文ごとログに出る → トークン（sessionStorage）→ session（`max_gain` 0.35 がスライダー上限）→ pitch=+1 で送信開始（50 Hz、ack accepted、`effective_gain=0.35`）→ ARM → HOLDING → START → ACTIVE で目標 [700, −140, 0] → `document.hidden` を偽装すると送信が止まり WS が閉じ（seq 138 のまま）、サーバーは鮮度切れで STOPPING → BRAKED_DISARMED、その間も目標は [700, −140, 0] のまま（中央へ動かない）→ 表示に戻しても自動再開しない → トークン無しの STOP 200 → FAULT → FAULT_LATCHED 表示 → FAULT RESET → READY_TO_ARM → REFERENCE の count 欄に 250 ms 間隔で `2731` を打っても status の再描画（200 ms）で消えない → REFERENCE（確認ダイアログ）200 → DISARM 200。ページエラー・console エラー 0。ブラウザーが同一 origin の POST に付けた `Origin: http://127.0.0.1:<port>` をサーバーが通した |

## 2026-11-02 の結果（ジョイスティックページの入力モード: 速度モードを追加、既定に）

環境: 前節と同じ（rustc / cargo 1.96.0、node 26.3、Playwright 1.65 alpha（npx のキャッシュ）＋ chromium_headless_shell 1243 を `executablePath` で指定）。

| 試験 | 結果 |
| --- | --- |
| `cargo test --test api` | 37 passed（ページの配信・埋め込み HTML・外部資源なしの検査は変更なし） |
| ページの JS | 2 つの `<script>` を `vm.Script` で構文検査、`$()` の id は全部存在、重複なし。`H3Joy.rateStep` を node で評価: 全開 0.02 s × rate 1.0 で +0.02、100 刻みで 1.0 に張りつく（±1 で切る）、dt 5 s でも 0.1 s 分しか進まない、倒し量 0 なら保持、`rtc` は rtcSpeed で 0 へ寄せ行き過ぎず **ちょうど 0** で止まる、`mult` は 1 未満を 1 に、null / NaN の入力で {0, 0}。`H3Joy.momentum`: 5 刻みで 0 → 1 の弾き → 倍率 1.93、保持で約 1 秒後に 1.0x 近くへ減衰、1 秒かけてゆっくり押す → 1.3 未満、離す動き（|stick| が減る）は数えない |
| ジョイスティック試験ページ（ヘッドレス Chromium、sim、空きポート） | 手で書いた Playwright スクリプト（リポジトリには入れていない。`navigator.getGamepads` を差し替え。**ページは `include_str!` でバイナリに入るので `cargo build` してから**）。41 項目すべて通過、ページエラー・console エラー 0: 読み込みだけでは session なし、既定は速度モード・rate 1.0・戻る切・勢い切、指令姿勢は「—」→ スティック中央で指令 0.00 → **roll 全開 0.5 s で指令 0.50**、1.3 s で 1.00 に張りつく → 離すと 1.00 のまま保持、frame_seq は進む → 倒し量 0.2 で 0.5 s → 0.10（ゆっくり）→ 「中央へ」で 0.00 → 戻る入（2.0 /s）: −1 で −0.40 → 離して 0.6 s で 0.00 → 送信開始で ack accepted、サーバーの `requested_pose.roll` が指令姿勢（0.300）と一致 → **機器を消す**と frame_seq 停止・seq 継続・指令保持・バナー → 戻すと frame_seq 再開、指令は跳ばない（dt を 0 から取り直す）→ 「姿勢を 0 に固定」で 0 を送り指令も 0 → 勢い入: 弾きで ×1.80、保持で ×1.02 へ → **位置モード**に切り替えると rate 系の操作が無効になり、`axes [0.5, −0.3]` が指令 0.47 / −0.26 でサーバーの `requested_pose` も 0.4737 → 別クライアントの `POST /v1/session` で `stale_session` 停止 → 再読み込みでモード（位置）と rate 2.5 が残る → 壊れた保存値（mode "x"、rate 99、rtcSpeed −1）は 速度 / 3.0 / 0.1 に丸める → token → 送信開始 → ARM → HOLDING → START → ACTIVE、`POST /v1/limits` で max_vel 60 / max_acc 200 に下げて全開 → status の `limits` に `velocity_limited, acceleration_limited` が出て札が点く（WS の ack の `limits` は「なし」のまま: planner の札は ack に乗らない） |

変えたこと: `static/joystick_page.html` だけ（入力モード「位置」/「速度」、rate / 中央へ / 離すと中央へ戻る / 勢い、倒し量と指令姿勢の併記、status の `limits` の札とヒント）。サーバーも WS の約束（pose + gain）も変えていない。実機ではまだ使っていない。

## 2026-11-02 の結果（Issue #15 M5 / M4（ページ）/ L2: Python の session 使い回し、ジョイスティックページの入力衛生、stale_session で送信停止）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、node 26.3、Python 3.14 ＋ websockets 17（`clients/python/.venv`）、Playwright（npx のキャッシュ）＋ chromium_headless_shell 1243。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 全スイート通過（`api.rs` 36 を含む。ページの配信・埋め込み HTML の検査は変更なし） |
| `test_motion_client.py` | 32 passed（28 ＋ 4。既存の「切断で新 session」「受信停止で新 session」「session 拒否で止める」は使い回しの規則に合わせて書き換え） |
| `test_against_server.py` | 8 passed（実サーバー sim。`reconnect()` で走行中に止まる挙動は変わらず） |
| ページの JS | 2 つの `<script>` を `vm.Script` で構文検査、`$()` の id は全部存在、重複なし。`H3Joy` を node で評価: `pickDefault` はペダルだけ／ハンドルだけ（Logitech G29、Thrustmaster T300RS）／id 不明で軸 4＋ボタン 12（旧ヒューリスティックなら stick）／軸 1 本の joystick → **null**、ペダル＋T.16000M → T.16000M、T.Flight Hotas → 選ぶ、`mapping = standard` のパッド → 選ぶ、`poseFrom` は割り当てた軸が機器に無ければ null、`readPads` が `mapping` を写す（無ければ `""`） |
| ジョイスティック試験ページ（ヘッドレス Chromium、sim、空きポート） | 手で書いた Playwright スクリプト（リポジトリには入れていない。`navigator.getGamepads` を差し替え）。44 項目すべて通過、ページエラー・console エラー 0: 読み込みだけでは session なし → **ペダルだけ**では自動で選ばず（padState「スティックと判定できる機器が無い」、バナー「一覧から選んでください」、localStorage に保存しない、frame_seq 0 のまま）→ T.16000M が出ると自動で選びバナーが消え frame_seq が進む → 送信開始で ack accepted、`requested_pose` roll 0.4737 → **pitch を axis 7（機器に無い）に割り当てる**と frame_seq が止まり（22 → 22）seq は進み（最後の姿勢を送り続ける）、バナー「割り当てた軸が機器に無い（roll axis 0 / pitch axis 7、機器の軸は 4 本）」、サーバーの `requested_pose` は 0.4737 のまま（0 にならない）で `fresh = false` → axis 1 に戻すと再開しバナーが消える → **別のクライアントが `POST /v1/session`** → 次の ack が `stale_session` → WS が閉じ（`closed (stale_session)` 赤）、ボタンが「送信開始」に戻り、seq が止まり、サーバーの `last_seq` も増えない → 再読み込み（選択は残る）で機器なしのまま送信開始 → 「姿勢がまだ無い: 送らない」で seq 0、サーバーの `requested_pose` null（**0 をでっち上げて送らない**）→ 機器が戻ると実値で流れ始める |
| `clients/h3game/tests` | Ran 51、OK（skip なし。`H3GAME_SRC` に既存のクローン、`H3CS_BINARY` にこの worktree のビルド。パッチは M5 のクライアント本体と `h3_server_backend.py` の docstring（WS 切断は同じ session、新 session は owner 変更の `reconnect()` と `/reconnect` だけ）で作り直したもの） |
| 試験ページ（同上） | token → session 発行 → pitch 0.5 で送信開始、accepted → **送信中に「session 発行」**: 送信が先に止まり（`closed (session 発行)`）、stale_session は 1 件も出ない、seq 停止 → 送信開始で新 session の WS が開き accepted → 別のクライアントが session を発行 → ack `stale_session` で送信停止・ボタン「送信開始」・seq 停止 → session 発行 → 送信開始 → 「制限を送る」の ack が従来どおり limAck に出る（stale_session の判定を limits の ack より前に置いた影響なし）。`motion_limits.pending` が無いサーバーでは「(pending」を出さない |

変えたこと: `clients/python/h3_motion_client.py`（WS 再接続で session を使い回す。新 session は「無い／stale_session／reconnect()」のときだけ）、`static/joystick_page.html`（自動選択はスティックと判定できる機器だけ・null 姿勢 = 喪失・0 を作らない・`motion_limits.pending` の表示・stale_session で停止）、`static/test_page.html`（stale_session で停止・送信中の「session 発行」は先に送信を止める・`pending` の表示）。サーバーは変えていない。実機ではまだ使っていない。

既知の副作用（設計判断として残す）: Python クライアントは**サーバー再起動**のあと、使い回した id が新プロセスに知られていないので最初の Pose が `stale_session` になり `session_lost` で止まる（以前は自動で新 session を取って復帰した）。復帰は `reconnect()`（bridge は `/reconnect`）か次の WS 切断。status の `session.source_session_prefix` が null（誰も持っていない）なら誰も追い出さずに発行できるので、それを見て自動復帰する案はあるが未実装。

## 2026-11-01 の結果（ジョイスティック試験ページ `GET /joystick` の追加後）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、node 26.3、Playwright（npx のキャッシュ）＋ chromium_headless_shell 1243。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 417 passed、0 failed、1 ignored（前回 416 ＋ `api.rs` 1。`api.rs` は 36） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| ページの JS | 2 つの `<script>` を抜き出して `node --check` が通る。`id` の重複なし、`$()` で参照する id はすべて存在 |
| ジョイスティック試験ページ（ヘッドレス Chromium、sim、`--bind` で空きポート） | 手で書いた Playwright スクリプトで確認（リポジトリには入れていない。`navigator.getGamepads` を `addInitScript` で差し替え、模擬の T.16000M と T-Rudder を出す）。53 項目すべて通過、ページエラー・console エラー 0: 読み込みだけでは session も WS も無し → `window.H3Joy` の純粋な写像（`isStick` / `isPedal` / `pickDefault` はペダルより T.16000M、軸 4＋ボタン 4 もスティック、`norm` の center / min / max / 反転 / ±1 クランプ、不感帯 0.05 と h3game の 0.12 の再スケール、`poseFrom` は roll ← `axes[0]`・pitch ← `axes[1]` で符号を変えない、`mostMoved`、`findPad` の同名フォールバック）→ 機器が出ると自動で T.16000M を選び（一覧 2 台、「使用中」、localStorage に保存）、`axes [0.5, −0.3]` が roll 0.47 / pitch −0.26 と表示、frame_seq が rAF で進む → 「送信開始」で session → WS open → 50 Hz の pose（roll 0.4737、pitch −0.2632、yaw 0、gain 0.10、`source_age_ms` 0、seq は +1、frame_seq は単調に進む）、ack accepted `effective_gain=0.1`、サーバーの `requested_pose` が一致 → ARM → HOLDING → START → ACTIVE で `virtual_targets` [522.8, 549.7, 512]（m1 = 512 + g(roll + pitch)、m2 = 512 + g(roll − pitch)、m3 = 512）、`target_counts` [42, 29, 0]、表の actual 列が追従 → 「姿勢を 0 に固定」で `requested_pose` 0・fresh のまま → 切ると元に戻る → **機器を消すと**バナーが出て frame_seq が止まり、最後の姿勢（roll 0.47）を送り続けて 0 は送らず、サーバーは鮮度切れで STOPPING → BRAKED_DISARMED（`requested_pose` は最後の値のまま）→ 機器が戻ると frame_seq が再開しバナーが消える → 「動かして割り当て」で axis 2 を大きく往復させると 1.5 s 後に pitch ← axis 2（center 0、min −0.9、max 0.9）、反転で符号が変わる、「中央を取り直す」で raw 0.2 が 0 → 送信停止で WS closed → トークン無しの STOP 200 → 再読み込みで選択と割当が残る → 「割当を初期化」で axis 1 |

変えたこと: `GET /joystick`・`/joystick.html`（`static/joystick_page.html`、DESIGN.md §7.5）。サーバーの他の経路は変えていない。実機ではまだ使っていない。

## 2026-11-01 の結果（動きの制限値の 2 段化、[limits.md](limits.md)）

環境: macOS 26.5（arm64）、rustc / cargo 1.96.0、clippy 0.1.96、Python 3.14.4 ＋ websockets 17（`clients/python/.venv`）。

| 試験 | 結果 |
| --- | --- |
| `cargo test` | 416 passed、0 failed、1 ignored（上の表の合計。doc-tests 0 件） |
| `cargo clippy --all-targets -- -D warnings` | 警告 0 |
| `clients/python/tests` | Ran 36、OK |
| `clients/h3game/tests` | Ran 51、OK（skip なし。`test_browser.py` もヘッドレス Chromium で実行。パッチは `submit_limits` と `limits` の転送で作り直したもの） |
| `e2e_sim.sh` | PASS、チェック 120 / 120（[runs/e2e_sim_20261101-183215/summary.md](runs/e2e_sim_20261101-183215/summary.md)。gain 0.6 は `effective_gain` 0.5 に制限） |
| `moter-controler/tests/run_tests.sh` | 3758 passed、0 failed |
| `pio run` | 6 env すべて SUCCESS（書き込みはしていない。実機 3 台はこのファームを焼き直すまで `firmware_limits_unsupported` で ARM できない） |
| Codex の静的レビュー（`codex review` と深掘り 1 回、実機試験の前） | 指摘 2 件を修正: (1) SET_LIMITS が未 ACK / LIMITS 未受信のまま ARM できた → `limits_unconfirmed` の阻害要因と、LIMITS が落ちたときの再送（新しい request_id）。(2) 実行時に gain の上限を下げても追従中の Pose に効かなかった → `reapply_gain_ceiling` で受理済みの最新 Pose をその場で再クランプ（WS・REST とも）。正しいと確認された点: ファームの `on_set_limits` の検査とクランプ、STOPPING の減速のフォールバック、SET_LIMITS の ACK が ARM グループの ACK と混ざらない、再送は 1 秒周期、`send_config` は軸ごとの申告値を使う、API の seq 消費と session 再発行の排他、Python クライアント・試験ページの例外経路。残る注意: 模擬軸のプラントは Duty / Kp / FF を見ないので、低 Duty での保持や段付きは模擬では確かめられない |
| MCU 再起動後の再配布 | `supervisor.rs` の `limits_are_resent_after_an_mcu_reboot`: リンクを保ったまま個体が再起動すると SET_LIMITS を 1 回だけ再送し、古い LIMITS は status から消える |

変えたこと: `[limits]`（schema 2）、SET_LIMITS / LIMITS、WS `limits`、`GET/POST /v1/limits`、status の `motion_limits` / `axes[].fw_limits`、試験ページの「動きの制限」、Python クライアントの `submit_limits`、bridge の `limits` 転送。**数値は実機未検証**（limits.md §4 の手順で上げる）。

## 2026-10-31 の結果（系統選択 DESIGN.md §10 の実装後）

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
| STOPPING → BRAKED_DISARMED | 1057 ms（`stop_hold_ms` 1000 を含む）。実測の静止判定（`rest_dwell_ms` 200、Issue #15 H1）を入れた後は、鮮度切れからの停止で 1051 ms（止まっていた軸は dwell が済んでいる）、走行中の `POST /v1/stop` で 1252 ms（減速の末尾に dwell が足される）。[runs/e2e_sim_20261102-013725](runs/e2e_sim_20261102-013725/summary.md) |
| 管理者 FAULT → 全軸 FAULT | 21 ms |

値は [runs/e2e_sim_20261030-120325/summary.md](runs/e2e_sim_20261030-120325/summary.md)（Python 3.14）のもの。
