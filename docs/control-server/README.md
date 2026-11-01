# h3-control-server（PC 側コントロールサーバー）

状態: **PC 側（Rust サーバー・Python クライアント・h3game アダプター）は実装済みで、模擬軸（sim）でだけ試験済み。CoreS3 側ファームウェアは未実装。実機での運転は未実施。**

既存ゲーム h3game（`TakuroFukamizu/20261004_shi3z_hackathon` の `h3game/`）の姿勢指令を受け、3 台の CoreS3 軸コントローラーへ位置目標（count）を送るサーバー。

| 文書 | 内容 |
| --- | --- |
| [architecture.md](architecture.md) | 元の計画（互換契約・責任分界・状態と安全契約・導入順序） |
| [protocol.md](protocol.md) | PC ⇄ CoreS3 の USB プロトコル v1（規範。CoreS3 ファームはこれに合わせる） |
| [testing.md](testing.md) | 試験の一覧・未試験の範囲・最新の結果 |
| [hardware-bringup.md](hardware-bringup.md) | 実機 bring-up 手順（CoreS3 ×3 を初めてつないで 3 軸を動かすまで。設定は `control-server/config/bringup.toml`） |
| [limits.md](limits.md) | 動きの制限値。ファームの絶対上限と PC の共有現在値の 2 段、`[limits]` の各値と根拠（実機未検証）、実行時に変える IF（WS `limits`、`/v1/limits`）、実機で上げる手順 |
| [game-integration.md](game-integration.md) | ゲーム（h3game）実装者向けの入口。実機なしで動かす手順、ゲームがやること・やらないこと、画面に出す状態、変えてはいけないこと |
| [`control-server/DESIGN.md`](../../control-server/DESIGN.md) | 実装設計（API・状態機械・JSON の形） |
| [`clients/python/README.md`](../../clients/python/README.md) | Game API の Python クライアント |
| [`clients/h3game/README.md`](../../clients/h3game/README.md) | h3game へのパッチと bridge 用バックエンド層 |
| [runs/](runs/) | e2e 試験の記録 |

## 構成

```text
 Chrome: h3game web/index.html
   │  POST /  30Hz  {owner, pitch, roll, yaw, gain, frame_seq, frame_age_ms}
   ▼
 bridge.py :8891  (H3_BACKEND=server、パッチ適用後)
   │  h3_server_backend.py → h3_motion_client.py
   │  POST /v1/session、WS /v1/motion（Pose、新しい frame_seq のときだけ送る）
   ▼
 h3-control-server 127.0.0.1:8892 ◀── 管理 API（curl / オペレーター、Bearer トークン）
   │  混合式（h3_direct.py と同じ）→ 校正（3 点区分線形・ソフトリミット）
   │  → 100Hz プランナー（3 軸一様の速度制限・加速度制限）→ 状態機械
   │  USB CDC ×3、COBS + CRC16 フレーム（protocol.md）
   ▼              ▼              ▼
 CoreS3 軸1      CoreS3 軸2      CoreS3 軸3     位置制御・PWM・ブレーキ・期限切れ停止は MCU 側
 (motor #1)      (motor #2)      (motor #3)
```

sim backend では 3 台の CoreS3 の代わりに `control-server/src/sim_axis.rs`（MCU 側の参照モデル）がプロセス内で動く。

## 確定していること・未確定のこと

| 区分 | 項目 |
| --- | --- |
| 確定（実装・試験済み） | Game API と管理 API（DESIGN.md §7）。混合式は h3_direct.py と同じ（段階 1 の表と一致）。接続・再接続・サーバー終了で中央復帰・ARM・START をしない。ゲーム入力の鮮度は `frame_seq` が進んだときだけ延長。走行中に session が変わると必ず停止。停止・FAULT を後から来た Pose で上書きしない。いずれも **模擬軸での確認** |
| 確定（実機で確認、2026-11-01。[runs/hw_20261101.md](runs/hw_20261101.md)） | USB プロトコル v1 の PC 側と CoreS3 ファーム（`moter-controler` の env `m5stack-cores3-v1`）: HELLO による `selected_axis` の割当（3 台）、CONFIG / SET_REFERENCE、ARM_PREPARE → HOLDING（ブレーキ解除後の保持）、DISARM、校正 `POST /v1/calibrate`（#1 は完走。#2 / #3 は電源担当軸が先に主電源を切って中立で失敗 → `KEEP_POWER` を追加、実機は未試験）。serial backend の DTR、`sim_axes` |
| 推定 | 模擬軸での遅延（[runs/](runs/) の値: Pose 受信→tick 取り込み p99 約 10 ms、tick 周期 p99 約 11 ms）は macOS・無負荷での値。実機・高負荷・Windows で同じとは限らない |
| 未確定 | START → ACTIVE の追従（START の `BAD_STATE` は直したが、再試験前に USB 無応答が起きた）。位置制御の定数（フィードフォワード・KP・Duty 上限、仮値）。仮想 0→1023 の向き。`max_gain` 0.35 / bring-up 0.15、`source_freshness_ms` 250、bring-up の `set_position_valid_for_ms` 100・`mcu_command_timeout_ms` 200（USB の揺らぎから置いた値）。校正値は当日実測（#1 5551 / 2775、#2 5463 / 2731、#3 6138 / 3069。CAL ごとに ±20）。USB 無応答（10/31 に 3 回、11/1 に 2 台同時）の原因 |
| 未実装 | CLOCK_SYNC と `apply_at_us`（v1 は 0 固定）。実測角度（`measured_angles` は常に null）。物理非常停止の監視（`physical_estop: "not_monitored"`）。サーバー側の JSONL 記録（`telemetry.rs` に記録器はあるが設定・配線していない。`recorder_dropped` は常に 0。障害前後の経過はメモリ上のリング `GET /v1/events` と標準エラーのログで見る。サーバーを再起動すると消える）。telemetry の一部（計画 §8・§11）: SET_POSITION 送信→MCU 適用の遅延、source age の分布、キュー深さ、STATUS の欠落数、USB 書き込み遅延、clock_sync の品質は測っていない（サーバー受信→MCU 適用 p99 20 ms の目標は評価できない） |

## ビルド・起動・試験

Rust（確認した版: rustc / cargo 1.96.0）。Python は 3.9 以上、クライアントの依存は `websockets>=13` だけ。

```sh
cd control-server
cargo build                                    # target/debug/h3-control-server
cargo test                                     # Rust の全試験
cargo clippy --all-targets

./target/debug/h3-control-server check-config config/example.toml   # 設定検査と静的な ARM 阻害要因
./target/debug/h3-control-server list-ports                         # シリアルポート一覧（開かない）

# 模擬軸で起動（管理トークンは環境変数。未設定なら起動時に生成して標準エラーへ 1 回出す）
H3CS_ADMIN_TOKEN=secret ./target/debug/h3-control-server --config config/sim.toml
H3CS_ADMIN_TOKEN=secret ./target/debug/h3-control-server --config config/sim.toml --bind 127.0.0.1:18892
```

試験ページ（オペレーター用、DESIGN.md §7.4）: サーバーを起動して、同じ PC のブラウザーで **<http://127.0.0.1:8892/>**（`--bind` を変えたらそのアドレス。`localhost` でもよい）を開く。管理トークン欄に `H3CS_ADMIN_TOKEN` の値を入れると、状態（`rig_state`・blockers・fault・3 系統の表）、イベントの末尾、ARM / START / STOP / DISARM / CENTER / FAULT / FAULT RESET / 軸ごとの REFERENCE、スライダーによる姿勢送信（50 Hz、タブが隠れると止まる）が使える。読み込んだだけでは何もしない。ページは LAN からは開けない（loopback だけ）。ブラウザーが同一 origin の POST / WS に付ける `Origin`（`http://127.0.0.1:8892` など、Host が loopback で一致するもの）は `allowed_origins` に無くても通る。他の origin の扱い（403）と `Origin` 無しのクライアントの扱いは変わらない（DESIGN.md §7.4）。

Python の試験（リポジトリ直下で。結合試験は `cargo build` 済みのバイナリを使い、なければ skip）:

```sh
clients/python/.venv/bin/python -m unittest discover -s clients/python/tests -v
clients/python/.venv/bin/python -m unittest discover -s clients/h3game/tests -v   # h3game: H3GAME_SRC、未設定なら初回に GitHub から control-server/target/h3game-src へ clone（649ab66）
```

e2e（サーバー＋パッチ後の bridge＋模擬ブラウザー。結果を `docs/control-server/runs/e2e_sim_<日時>/` に書く）:

```sh
control-server/scripts/e2e_sim.sh                 # H3GAME_SRC・PYTHON で上書き可（h3game の既定は上と同じクローン）
control-server/scripts/e2e_sim.sh --out /tmp/e2e  # 記録先を変える
```

ゲームをつないで動かす手順は [`clients/h3game/README.md`](../../clients/h3game/README.md)「server モードでの運転の流れ」。

## 設定ファイル

| ファイル | 用途 |
| --- | --- |
| `control-server/config/example.toml` | 実機用の雛形。`backend.kind = "serial"`、`device_id` は 0（照合しない。軸は各 CoreS3 で選択した系統で決まる）、全軸 `calibration_valid = false`。**このままでは ARM できない**（`check-config` が阻害要因を列挙する） |
| `control-server/config/sim.toml` | 模擬軸用。`calibration_valid = true` は模擬軸だから許される。実機の設定にコピーしない |

- 校正・符号・ソフトリミットの変更は DISARMED のときだけ（サーバー再起動で反映）。軸 2・3 の値は未測定の仮値。軸 1 の値をコピーしない（architecture.md §6）。
- 動きの制限値（gain・速度・加速度・Duty・位置ループの調整値・保護の判定値）は `[limits]` に 3 軸共通で 1 組（`schema_version = 2`。旧ファイルの `[game].max_gain` と `[[axis]]` の `max_vel_count_s` などは読み込みで失敗する）。最初の 4 つは初期値で、実行時にゲーム（WS `limits`）と管理者（`POST /v1/limits`）が変えられ、ファームの絶対上限で頭打ちになる。`POST /v1/session` で初期値に戻る。[limits.md](limits.md)。
- 実機で使う前に: 各 CoreS3 で担当する系統（軸 1〜3）を選択する（電源ボタン操作、NVS に保存。protocol.md §1.1）。未選択のまま PC に挿した個体（設定の `device_id` にも無いもの）は `unknown_device` でポートを閉じられるので、選択してから **USB を抜き差しする**（PC は装置が消えるまでそのポートを開き直さない）→ 必要なら status の `axes[].device_id` を見て `[[axis]].device_id` に記入する（「この軸はこの個体のはず」という照合。0 なら照合しない。違う個体が選ばれていれば `axis_device_mismatch` で ARM しない）→ 校正値を実測 → `calibration_valid = true` は人が確認してから。
- 軸の割当は MCU 側の選択で決まる。同じ系統を 2 台が選んでいると `axis_conflict`（その軸は OFFLINE、ARM・START・ゲーム入力を拒否）。片方を別の系統にするか抜けば解消する。同じ boot のまま系統を変えた個体は `reference_stale`（`/v1/reference` をやり直すまで ARM しない）。
- bind は既定で loopback だけ。`server.allow_non_loopback = true` でも LAN の相手に開くのは `GET /v1/status` と `POST /v1/stop` だけ。
- `power_controller_axis` の軸だけが主電源（G2）を操作する。各軸の STATUS の `POWER_CONTROLLER` がこの設定と違えば ARM しない（`power_role_mismatch`）。

- `[backend] sim_axes = [3]`（serial backend）: 実機が足りないとき、その軸を模擬軸で埋める（`sim:3` として接続・基準済みで現れ、目標に完全に追従する）。その軸の `ports` は書かない。実機を戻したら消す。
- `[server] require_admin_token`: 既定 `false`（下の「管理操作」）。

## 管理操作（curl）

管理トークンは **既定で不要**（`server.require_admin_token = false`。bind が loopback のときだけ許される。LAN に開く `allow_non_loopback = true` では `true` が必須）。必要にするときは `require_admin_token = true` にして `export H3CS_ADMIN_TOKEN=...`、`AUTH="Authorization: Bearer $H3CS_ADMIN_TOKEN"` とし、下の `-H "$AUTH"` を付ける。`GET /v1/status` の `admin_token_required` で分かる（テストページはこれを見てトークン欄を無効にする）。

```sh
URL=http://127.0.0.1:8892
AUTH="Content-Type: application/json"   # トークン不要のときはこのままでよい（require_admin_token = true なら Bearer に置き換える）

curl -s $URL/v1/status | python3 -m json.tool      # 状態（認証なし）
curl -s "$URL/v1/events?n=50" | python3 -m json.tool   # 直近のイベント（障害前後の経過。認証なし・loopback だけ。?after=<id> で続きだけ）
curl -sX POST $URL/v1/session                       # Game の session 発行。通常は bridge が行う。前の session は無効になり、走行中なら停止する
curl -sX POST -H "$AUTH" $URL/v1/arm                # ARM: 全軸 HOLDING（その場で保持）まで
curl -sX POST -H "$AUTH" $URL/v1/start              # START: 新鮮な Pose が無ければ 409 no_fresh_pose、Pose が保持位置から離れ過ぎなら 409 pose_not_near_hold
curl -sX POST $URL/v1/stop                          # 停止要求（トークン不要）→ 減速 → 保持 → ブレーキ
curl -sX POST -H "$AUTH" $URL/v1/disarm             # DISARM: 減速停止してブレーキ
curl -sX POST -H "$AUTH" -d '{"note":"手で止めた"}' $URL/v1/fault   # 管理者による FAULT ラッチ
curl -sX POST -H "$AUTH" $URL/v1/fault/reset        # 原因が残っていれば 409。解除後も自動 ARM しない
curl -sX POST -H "$AUTH" $URL/v1/center             # HOLDING から中立へ低速移動（速度・加速度 × center_speed_fraction）→ HOLDING
curl -sX POST -H "$AUTH" -d '{"axis":1,"count":0}' $URL/v1/reference   # 全軸 DISARMED のときだけ。現在位置を count 0 と定義
curl -sX POST -H "$AUTH" $URL/v1/calibrate          # 端当て校正（機構が両端まで動く！）。全軸 DISARMED・電源担当軸を含む 3 軸。{"axes":[1,2]} で一部（1 を含むこと）
curl -sX POST -H "$AUTH" $URL/v1/calibrate/cancel   # 校正の中止（CALIBRATE(0)）。/v1/stop でも止まる
curl -sX POST -H "$AUTH" $URL/v1/calibration/export # OK の結果を設定ファイルの複製に書き出す（下記）
curl -s $URL/v1/limits | python3 -m json.tool       # 動きの制限値（effective / requested / ceiling / defaults / limited。認証なし・loopback だけ）
curl -sX POST -H "$AUTH" -d '{"max_vel_count_s":1500,"duty_max_percent":30}' $URL/v1/limits   # 書いた項目だけ変える。上限で切られた項目は limited に出る（limits.md）
```

- 校正（DESIGN.md §11、protocol.md §1.2）: MCU が自分で P 端 → N 端 → 中立と端当てし、CAL_RESULT を返す。status の `axes[].mcu_state` が `CALIBRATING`、終わると `axes[].calibration`（`result` / `reason` / `n_end_count` / `neutral_count` / `sign`）と `referenced = true`（P 端 = 0 の基準を MCU が作る。/v1/reference は不要）。校正中は ARM / START / CENTER / REFERENCE が 409 `calibrating`。`export` は、OK の結果がある軸の `mechanical_* / soft_* / count_at_virtual_* / device_id` を入れ替えた **設定ファイルの複製** を `--config` と同じディレクトリに `<rig_id>.calibration-<YYYYmmdd-HHMMSS>.toml`（時刻は UTC）として書く。`calibration_valid` は false のままなので、値を確認して true にし、そのファイルで起動し直す（hardware-bringup.md §5）。同名があれば 409 `exists`、OK の結果が無ければ 409 `no_calibration_result`。

- 成功は 200 `{"ok":true,"rig_state":...}`、拒否は 409 `{"ok":false,"error":"<code>","blockers":[...]}`。`blockers` は `"<code>: <detail>"`。
- 系統の割当（DESIGN.md §10）は status で見る: 最上位 `axis_conflict`（2 台以上が選んでいる軸の一覧。空なら競合なし）、各軸の `device_id`（接続中の個体）・`expected_device_id`（設定値、0 = 照合なし）・`assignment`（`"selected"`＝MCU の選択 / `"config"`＝`device_id` だけで決まった / null）・`claimants`（この軸を名乗るポートと個体。競合中は 2 つ以上）・`reference_stale`。blockers の code は `axis_conflict` / `axis_device_mismatch` / `reference_stale`。各軸の `decode_errors` は PC 側で復号に失敗したフレーム数（USB の品質。0 が正常）。競合中は arm / start / center が 409 `axis_conflict`、WS の pose / gain も ack `reason: "axis_conflict"` で拒否（鮮度は延びない）。
- 503 `control_loop_unavailable`: 制御ループが 1 秒以内に答えなかった。arm / start / center / reset / reference は **実行されない**。stop / disarm / fault は**実行されたかもしれない**（ループが追いつけば実行する）。いずれも status で確かめる。`GET /v1/status` は制御ループが終わっている（落ちた）とき 503。終了処理（SIGTERM など）が始まると新しい接続は受け付けず、503 ではなく接続拒否になる（確定: sim で ARM 中に SIGTERM、5 ms ごとの status は全て接続拒否）。終了処理の直前に受け付けた要求に 503 が返ることはありうる（推定、未観測）。
- 管理 API は rate limit（既定 20 回/秒）。stop / disarm / fault は対象外。
- FAULT になったら `fault`・`fault_history`（status）に加えて `GET /v1/events` で直前の経過（ARM・START・軸の接続・ACK の拒否や期限切れ・状態遷移）と、その後の状態遷移を見る。リングは 512 件（古いものから上書き、`dropped` に数える）。制御ループが落ちた後も読める。同じイベントは標準エラーにも info で出る。
- Python からは `python3 clients/python/h3_motion_client.py status|stop`。

運転の順序: 起動（READY_TO_ARM）→ ゲームが Pose を送る → **人が** ARM → HOLDING → **人が** START → ACTIVE。ゲームが止まる（`frame_seq` が進まない）・画面の再読み込み・bridge の再接続（新しい session）・stop のどれでも STOPPING → 保持 → BRAKED_DISARMED になり、**その場でブレーキ**（中央へ戻さない）。再開は人が ARM ＋ START をやり直す。

## 安全上の注意

- **API の stop は非常停止ではない。** ソフトウェアの減速停止要求で、サーバー・USB・MCU が正常に動いていることが前提。人が乗る前に、3 軸の動力を切る物理的な非常停止経路（ソフトを通らない）を用意する。サーバーは非常停止の状態を監視していない（`physical_estop: "not_monitored"`）。
- サーバーは自動で中央へ戻さない。接続・再接続・ゲーム停止・サーバー終了のどれでも、その位置で保持してブレーキ。中立へ動かすのは `/v1/center` を人が呼んだときだけ。
- 状態の表示（`power_on_cmd`、`brake_release_cmd`）は**指令**であって実状態の確認ではない。`requested_pose` も指令値で、実測角度ではない。
- 模擬軸での試験結果は、実機の停止距離・荷重保持・3 軸同期の根拠にならない。有人運転の可否は architecture.md §12 段階 5 の後に別に判断する。

## CoreS3 側でやること（protocol.md から）

参照する挙動は `control-server/src/sim_axis.rs`（`SimMcu`。ENCODER_FAULT・CONTROL_OVERRUN の検出は模擬しておらず、異常注入 `force_fault` で与えるだけ）。同じ入力に同じ応答を返すことを目標にする。フレームのバイト列は protocol.md §7 のテストベクタと一致させる。

1. **フレーミング**（§2）: COBS（区切り 0x00）、header 4 byte＋payload＋CRC-16/CCITT-FALSE（LE）。復号後 256 byte 超・CRC 不一致・版違い・未知種別・長さ違いは破棄して `rx_error_count` を数え、応答しない。
2. **HELLO**（§4）: HELLO_REQ に `selected_axis`（選択中の系統 1〜3、0 = 未選択）・`device_id`（eFuse MAC 由来）・起動ごとの乱数 `boot_id`・FW 版・capabilities・受理済み `calibration_hash` を返す。
3. **STATUS を 50 Hz で自発送信**（§4）: 状態・`selected_axis`・異常コード・flags・PWM・実位置・制御目標・速度・追従誤差・`last_applied_frame/seq`・`last_rx_seq`・電流（未校正なら CURRENT_VALID を立てない）・エラー数。
4. **状態機械**（§3.1、§5）: BOOT → DISARMED → ARMING → HOLDING → ACTIVE → STOPPING → HOLDING、DISARM で（動いていれば減速して）DISARMING → DISARMED、異常で FAULT。図にない組み合わせ（HOLDING での CONTROLLED_STOP、DISARMED・ARMING での DISARM など）は拒否せず ACCEPTED（§4 の表。PC はこれらを送り、拒否されると停止失敗として FAULT にする）。USB 再接続・CDC 再オープンで自動 ARM / START しない。NVS に ARM 状態を保存しない。
   - **ACK と STATUS の順序**（§1）: 命令による状態遷移は ACK を送る前に確定させ、ACK の後の STATUS はすべて遷移後の状態にする。USB を Core 0、制御を Core 1 に分けたときに、命令前に取った STATUS を ACK の後に送らない（PC は ARM を状態不一致で中断し、FAULT_RESET の判断を誤る）。
5. **ARM の前提**（§4 ARM_PREPARE）: DISARMED・CONFIG 受理済み・SET_REFERENCE 済み・epoch 一致・実位置がソフトリミット内。拒否理由の順序も表のとおり。ARMING の順序は §5（主電源は電源担当軸だけ → 範囲外ならブレーキを解除せず FAULT → 現在位置で位置制御開始 → ブレーキ解除）。
6. **SET_POSITION**（§4）: ACK なし。epoch ごとに seq 単調（古いものは `stale_drop_count`）、`valid_for_ms` と `command_timeout_ms` の期限監視、HOLDING / STOPPING では目標を使わず期限監視だけ、ソフトリミットで最終クリップ、`apply_at_us ≠ 0` は破棄。START は期限内の SET_POSITION がなければ拒否。
7. **管理命令の共通規則**（§4）: `request_id` = 0 拒否、直近 16 件の受理済み命令と同一なら DUPLICATE（再実行しない）、`fault_code ≠ NONE` なら FAULT_RESET 以外を FAULT_ACTIVE で拒否。HOST_FAULT は常に即実行（冪等）。
8. **異常検出**（§3.2）: COMMAND_TIMEOUT（動いていれば減速してから FAULT）、FOLLOWING_ERROR、OVERSPEED、SOFT_LIMIT、ENCODER_FAULT、CONTROL_OVERRUN、STOP_TIMEOUT。FAULT は PWM 0・ブレーキ作動・主電源 OFF 指令。FAULT_RESET は原因が残っていれば FAULT_CAUSE_PRESENT、ソフトリミット外でも解除（再 ARM は ARM_PREPARE が止める）。
9. **SET_REFERENCE / CONFIG** は DISARMED のときだけ。CONFIG は §4 の表にある検査（DISARMED、`soft_min < soft_max`、6 項目 > 0）以外で拒否しない。`role_flags` bit0 を STATUS の `POWER_CONTROLLER` に出す。CONFIG の epoch が変わったら `last_rx_seq` を 0 に戻す（同じ epoch なら保持）。boot ごとに REFERENCED / CONFIGURED は消える。
   - STATUS の `target_count` は軌道上の目標（受けた SET_POSITION の目標ではない）、`following_error_count` は `actual_count − target_count`（§4）。FAULT_EVENT は FAULT へ遷移するたびに 1 回（§4）。
   - PC 側の期限（ACK 1 s、HELLO_REQ 200 ms / 2 s、ARM 5 s、停止 3 s）に収まるようにする（§6）。
10. **系統選択**（§1.1）: 担当する系統（軸 1〜3）は MCU 側で選ぶ（電源ボタン操作、NVS に保存）。選択中の値を HELLO と毎回の STATUS の `selected_axis` に出す（0 = 未選択。PC はこれで軸を割り当て、設定の `device_id` は照合にだけ使う）。切り替えは **DISARMED のときだけ** 受け付け、それ以外（FAULT 中も）は操作を無視する。切り替えたら ACK と同じ順序規則で状態を確定してから、`CONFIGURED`・`POSITION_REFERENCED` を消し `calibration_hash` を 0 にし、AXIS_SELECT（`0x86`。`boot_id`、新旧の `selected_axis`、`mcu_state`。ACK なし）を 1 回送り、以後の STATUS に新しい値を載せる。選択で `actual_count` の向き・原点・スケールは変えない（符号・ストローク・中立は PC の校正で扱う）。受理済み命令の記憶と `last_rx_seq` は消さなくてよい。
11. PC 側にない責務: 位置制御周期（案 200 Hz 以上）を USB・画面描画に依存させない。PC が止まっても自律的に停止へ進む（期限切れ → 減速 → FAULT・ブレーキ）。

v1 で扱わないもの: CLOCK_SYNC による時刻同期と `apply_at_us`（CLOCK_SYNC_REPLY は PC 側で捨てる）。
