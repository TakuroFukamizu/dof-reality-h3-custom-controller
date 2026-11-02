# h3game 接続アダプター（bridge.py → h3-control-server）

状態: **パッチ作成済み・模擬軸（sim）サーバーと結合試験済み。実機、実際の Chrome（ヘッドあり）での長時間運転は未実施。**

既存ゲーム（`TakuroFukamizu/20261004_shi3z_hackathon` の `h3game/`）を、USB 直結の旧 H3 の代わりに `h3-control-server` へつなぐための小さいパッチと、bridge 用のバックエンド層。計画は [architecture.md](../../docs/control-server/architecture.md) §4、実装設計は [DESIGN.md](../../control-server/DESIGN.md) §8。ゲーム側で知っておくべき制御の前提（gain と全域、速度・加速度の丸め、開始・停止・鮮度）は [game-tips.md](../../docs/control-server/game-tips.md)。

h3game リポジトリへは push していない。パッチはこのディレクトリに置くだけ。

## ファイル

| ファイル | 内容 |
| --- | --- |
| `patches/0001-*.patch` | h3game への 1 コミット分のパッチ（`git am` で適用） |
| `h3_server_backend.py` | bridge 用バックエンド層の正本。パッチで `h3game/h3_server_backend.py` に同じものが入る |
| `../python/h3_motion_client.py` | Game API クライアントの正本。パッチで `h3game/h3_motion_client.py` に先頭 2 行のコメント付きで複製される |
| `tests/` | 単体・パッチ・結合・ブラウザー試験 |

正本を直したらパッチを作り直す（`tests/test_patch.py` の `test_copies_match_canonical_sources` が食い違いを検出する）。

パッチを作ったときの元ファイル（blob SHA、適用前に一致を確認済み）:

| ファイル | blob SHA |
| --- | --- |
| h3game/bridge.py | 807b4d9c45e6ef04550a64e34ceaf89a01599c18 |
| h3game/web/index.html | 55827600269b6b840b43f864718d19d7ac4b68ca |
| h3game/h3_direct.py | 06bc7f0bdda2c5ebff5f9d323074b582629582a0（変更しない） |

## パッチが変えること

### bridge.py（+37 / −6 行）

- 環境変数 `H3_BACKEND=legacy|server|dry`。
  - 未設定（または空）＝ `legacy`。**座席への経路は旧動作のまま**（USB 直結、`/state` のキーも同じ。試験で元の bridge と応答を比較済み）。下の「全モード共通」の 2 点だけ変わる。
  - `dry`＝ `H3_DRY=1` と同じ（どこにも送らない）。`H3_DRY=1` は `server` より優先する。
  - それ以外の値は起動時にエラー終了する（打ち間違いで legacy＝USB 駆動にならないように）。
  - `H3_SERVER_URL`（既定 `http://127.0.0.1:8892`）。
- `server` のとき:
  - `h3_direct`・`serial` を import しない（試験で確認）。
  - `connect()` は制御サーバーへの通信（session 取得・WS 接続）を始めるだけ。ARM・START・中央へ動かすことはしない。`watcher()` は何もしない（再接続は `h3_motion_client` が行う）。
  - 60Hz の再送、指令が 1 秒途切れたら中央へ戻す処理、`auto_sway` を止める。`{"auto": true}` は受け付けず、`/state` の `auto_ignored` に数だけ残す。
  - `do_POST`: 既存の owner 検査を通った本文（pitch, roll, yaw, gain, frame_seq, frame_age_ms）をバックエンドへ渡す。渡すのは最新 1 件の枠に置くだけで、HTTP ハンドラーはサーバーの応答を待たない（サーバー停止中でも POST は待たされないことを試験で確認）。バックエンドで例外が起きても POST には応答し、`/state` の `backend_errors` に数えてログに出す（席へは何も送らない）。
  - `/state`: 既存のキー（pitch, roll, yaw, connected, port, gain, auto, last_cmd_age …）を残し、下表のキーを足す。
  - `/reconnect`: 通信の張り直し（新しい session）だけ。運転は再開しない。
  - 終了時に接続を閉じる。座席は止めない（指令が途切れればサーバーが止める）。
- `/state` の 1 秒 poll はゲームの生存扱いにしない（読むだけ）。
- 全モード共通:
  - **HTTP のポート（8891）を先に取ってから** `connect()` する。ポートが使用中（bridge の二重起動など）なら `Address already in use` で終わり、座席にも制御サーバーにも触らない。以前は server モードで先にサーバーの session を取ってから終わっていたため、走行中のゲームが止まり、動いている方の bridge は session を取られたまま（`session_lost`）になった。
  - LAN の他の機器からの `GET /reconnect`・`GET /claim` は `{"ignored": true}` を返して何もしない（POST は元から同じ扱い）。どちらも走行中の座席を止められるため（`/reconnect` は新しい session、`/claim` は遊んでいる画面の操作権を奪う）。他の PC から開いた画面は元から「見るだけ」。

`/state` で server のときに意味が変わる・増えるキー:

| キー | 内容 |
| --- | --- |
| `pitch` / `roll` / `yaw` | 画面から受けた指令値（−1..1 にクリップ）。実測角度ではない |
| `connected` | サーバーに届いている かつ `rig_state` が `OFFLINE` 以外 かつ session を取られていない（`session_lost` でない）。**座席が動ける状態という意味ではない**（ARM・START は人が行う。FAULT_LATCHED・BRAKED_DISARMED でも true） |
| `port` | `"<server_url> <rig_state>"`、session を取られていれば末尾に ` SESSION_LOST`（画面の接続表示の横に出る） |
| `backend` | `"server"`（dry では `"dry"`。legacy では付かない） |
| `server_url` | 接続先 |
| `server_reachable` | session 取得済み・WS 接続中で、2 秒以内に status を受けている |
| `rig_state`, `armed`, `calibration_valid`, `fault`, `blockers`, `effective_gain` | サーバーの status の値。届いていなければ `null`（0 や false で埋めない） |
| `session_fresh` | 自分の session の Pose が新鮮か（自分の session でなければ `null`） |
| `session_lost` | 別のクライアントに session を取られた。この bridge の Pose はサーバーへ送られない。`/reconnect`（「H3 をつなぎ直す」）か画面の再読み込みで取り返す |
| `reject_reason` | 最新の ack の拒否理由（例 `missing_frame_seq`、`axis_conflict`） |
| `axis_conflict` | 2 台以上の CoreS3 が同じ系統を選んでいる軸の一覧（status の最上位 `axis_conflict`。空なら競合なし、届いていなければ `null`）。競合中はサーバーが ARM・START・Pose を拒否する（DESIGN.md §10） |
| `hud_note` | 系統の割当に関する画面向けの日本語（status の `blockers` の code と ack の拒否理由から作る）: `axis_conflict` →「系統の競合(軸1)」、ack が `axis_conflict` なら「(軸1、入力拒否)」、`axis_device_mismatch` →「個体の不一致」、`reference_stale` →「再基準化が必要」。複数は ` / ` でつなぐ。何もなければ `null`。パッチ後の index.html は `srvNote()` でこれを接続表示に足す |
| `frame_seq` | 最後にサーバーへ渡した frame_seq |
| `pose_stats` | `sent` / `gain_only` / `repeat` / `old_frame` / `invalid` / `new_page` の件数 |
| `auto_ignored` | 受け付けなかった auto 要求の数 |
| `backend_errors` | バックエンドへの受け渡しで起きた例外の数（起きたときだけ現れる） |

### web/index.html（+10 / −4 行）

- `frame()`（requestAnimationFrame）の 1 回分の処理（`Game.update` / `renderSetup`、平滑化）が最後まで済むたびに `frameSeq` を +1 し、その時刻（`performance.now()`）を記録する。
- `sendSeat()` の JSON に `frame_seq` と `frame_age_ms`（POST 時点の now − 最後のフレーム時刻）を足す。
- 画面が隠れている（`document.hidden`）あいだは、従来どおり 0 を送る（legacy の動作は変わらない）。ただし `frame_seq` は**表示中に最後に送った値**を繰り返し、`frame_age_ms` もそのフレームからの経過時間にする。rAF（約 60 Hz）は POST（約 30 Hz）より先に進むので、隠れる直前の「描いたがまだ送っていないフレーム」の番号を 0 と組み合わせて送ると、サーバーには新しいフレームの 0（中央へ）に見える。そうならないよう、隠れている間の POST が新しい frame_seq を持つことはない。server では repeat（gain が変わったときだけ gain_only）になり、座席は動かない。
- server モードのときだけ（`/state` の `backend` が `"server"`）、プレイ中の HUD と確認画面の接続表示に `rig_state`、session を取られたこと、FAULT、gain がサーバーで制限されていること（`effective_gain` が画面の値と違うとき「揺れは 0.35 に制限」）を足す。`connected` が false のときは「● 制御サーバーで運転できない」と `port` を出す（USB の案内は出さない）。legacy の画面の表示は変わらない。
- 同じく server モードのときだけ、`/state` の `hud_note`（系統の競合 / 個体の不一致 / 再基準化が必要）を `srvNote()` で接続表示に足す（`+(st.hud_note?' '+st.hud_note:'')`）。競合中はサーバーが入力を拒否するので、プレイヤーには「系統の競合(軸1、入力拒否)」と見える。詳細は `curl -s http://localhost:8891/state` か制御サーバーの status。
- `scene*.js`・`game.js` は変更しない。

旧 bridge は未知のキーを無視するので、パッチ後の画面は旧 bridge（legacy）でもそのまま動く。

### バックエンド層 `h3_server_backend.py` の規則

- 同じ `frame_seq` の POST（ゲームループが止まって setInterval だけ動いている、画面が隠れて 0 を送っている）は Pose としてサーバーへ渡さない。gain が変わったときだけ gain のみのメッセージ（鮮度を延長しない）を送る。値を変えずに生存を延長するものは何も送らない。
- 同じ owner で `frame_seq` が戻ったら捨てる。
- **owner が変わったら**（画面の再読み込み・別タブ。`frame_seq` が 0 からやり直しになる）新しい session で張り直す。
- 系統の割当の問題（DESIGN.md §10）は bridge では何もしない: `axis_conflict` の ack が来ても再接続せず、Pose は送り続ける（オペレーターが CoreS3 の選択を直せばサーバーがまた受理する）。画面向けには `/state` の `hud_note` と `axis_conflict` に出すだけ。
- 範囲外・gain 上限の判定はサーバーに任せる（bridge 側でクリップした値ではなく、受けた値を渡す）。非有限値（float にできない巨大な整数を含む）・数値でない値・不正な frame_seq（整数でない、負、2⁶⁴−1 を超える。サーバーは u64 として読むので、超えた値を覚えると以後のフレームがすべて old_frame になる）・不正な frame_age_ms はサーバーへ渡さず `invalid` に数える。

## 適用方法

```sh
cd 20261004_shi3z_hackathon            # h3game リポジトリの作業コピー
git switch -c control-server-backend   # 任意
git am /path/to/dof-reality-h3-custom-controller/clients/h3game/patches/0001-*.patch
```

`git am` が失敗したら `git am --abort` で戻す。元ファイルが上の blob SHA と違う場合は手で当てる必要がある。

## 実行

server モードだけ追加の依存がある:

```sh
python3 -m pip install 'websockets>=13'
```

`h3_motion_client` が WebSocket（`/v1/motion`）の持続接続に `websockets` の同期クライアントを使うため。標準ライブラリには WebSocket クライアントがない。legacy / dry では不要（import もしない。試験で確認）。Python は 3.9 以上（3.9.6 ＋ websockets 13.1 と 3.14 ＋ websockets 17 で全試験通過。結果は [testing.md](../../docs/control-server/testing.md)）。

```sh
# legacy（旧 USB 直結。これまでどおり）
./start.command                       # または python3 bridge.py

# server（h3-control-server 経由）
H3_BACKEND=server python3 bridge.py
H3_BACKEND=server H3_SERVER_URL=http://127.0.0.1:8892 python3 bridge.py

# dry（どこにも送らない。画面の確認用）
H3_BACKEND=dry python3 bridge.py      # H3_DRY=1 python3 bridge.py と同じ
```

注意: `H3_DRY` は「値があれば dry」（`H3_DRY=0` でも dry）。既存の挙動のまま。

### server モードでの運転の流れ

1. `h3-control-server` を起動する（`--config`、管理トークンは `H3CS_ADMIN_TOKEN`）。
2. `H3_BACKEND=server python3 bridge.py`、Chrome で `http://localhost:8891` を開く。この時点では座席は動かない。
3. オペレーターが管理 API で ARM → START する（新鮮な Pose が届いていないと START は `409 no_fresh_pose`）:

   ```sh
   curl -sX POST -H "Authorization: Bearer $H3CS_ADMIN_TOKEN" http://127.0.0.1:8892/v1/arm
   curl -sX POST -H "Authorization: Bearer $H3CS_ADMIN_TOKEN" http://127.0.0.1:8892/v1/start
   ```

4. 止めるとき: `curl -sX POST http://127.0.0.1:8892/v1/stop`（トークン不要）、または `python3 h3_motion_client.py stop`。物理非常停止の代わりではない。
5. 画面を閉じる・隠す・ゲームが固まる → `frame_seq` が進まなくなり（隠したときの 0 もサーバーへは渡らない）、`source_freshness_ms`（既定 250 ms）後にサーバーが STOPPING → BRAKED_DISARMED にする。**中央へは戻さない**（その位置で保持してブレーキ）。再開は ARM ＋ START。

**走行中に session が変わると座席は止まる（意図した動作）。** 駆動権限は START 時の session に束縛される。次のどれでも新しい session になり、STOPPING → BRAKED_DISARMED へ進む。再開はオペレーターが ARM ＋ START をやり直す。

- 画面の再読み込み・別タブで開く（owner が変わる）
- 「H3 をつなぎ直す」ボタン（`/reconnect`。bridge を動かしている PC の画面からだけ効く）
- 通信断からの自動再接続、bridge の再起動
- 別のクライアントが `POST /v1/session` する（このとき bridge の `/state` は `session_lost: true`・`connected: false` になり、画面に「別の画面か bridge に取られた」と出る）

bridge を二重に起動しても、2 つ目はポートが取れずに終わるだけで、走行中の座席は止まらない（試験で確認）。

その他:

- `GET /v1/status` は制御ループが終わっている（落ちた）とき 503 を返す。サーバーの終了処理中は新しい接続を受け付けない（接続拒否。確定: sim で確認）。直前に受け付けた要求への 503 はありうる（推定）。どの場合も bridge の `/state` では `server_reachable=false`・`rig_state=null` になる。
- 管理 API の 503 で「実行されたかもしれない」のは stop / disarm / fault だけ。arm / start などは 503 なら実行されない。状態は status で確かめる。
- gain の上限はサーバー設定 `[limits].max_gain`（既定 0.5）。画面のスライダーは 0.6 まで動くが、超えた分は `effective_gain` が上限になり、ack の `limits` に `gain_limited` が入る。画面の接続表示にも「揺れは 0.5 に制限」と出る（スライダーの数字は画面の設定値のまま）。
- 動きの制限（docs/control-server/limits.md §3.1）: ゲームの POST に `"limits": {"max_gain": 0.8, "max_vel_count_s": 3000, "max_acc_count_s2": 15000, "duty_max_percent": 40}`（どの項目も省略可）を付けると、backend がそのまま `limits` メッセージとして送る（姿勢と独立。`/state` の `stats.limits` に数える。数値でない値は `invalid`）。サーバーは上限で切って ack に `effective_limits` を返す。新しい session（再接続・再読み込み）で初期値に戻る。いまの index.html は送らない（必要なら画面側で付ける）。
- 「● H3 接続中」は「サーバーに届いていて session を持っている」という意味で、座席が動いているとは限らない。横に出る `rig_state`（`ACTIVE` で走行中）を見る。

## frame_seq のない画面

既定設定（`allow_missing_frame_seq = false`）では、`frame_seq` のない Pose は `missing_frame_seq` で拒否される。パッチを当てていない index.html を server モードの bridge で開いても座席は動かず、START は `no_fresh_pose` で拒否される（試験で確認）。`/state` の `reject_reason` に `missing_frame_seq` が出る。

`allow_missing_frame_seq = true` にすると受理されるが、鮮度は送信タイマーだけで決まり（`freshness_mode = "timer_only"`）、ゲームの描画が止まっていても検出できない。有人運転では使わない。

## 意図的な非互換（architecture.md §2.3）

server モードでは旧 H3 と次の点が違う。

- 接続・再接続しただけでは中央へ動かさない。ARM・START しない。
- bridge 終了時・接続を閉じたときに中央へ動かさない。
- owner 番号は駆動権限ではない。owner が変わると session が変わり、走行中なら座席は止まる。
- 非有限値は拒否。範囲外はサーバーでクリップし、制限理由を報告する。
- 60Hz 再送でゲーム入力の鮮度を延長しない。画面からの指令が止まったら、中央へゆっくり戻すのではなく、その場で減速・保持・ブレーキ。
- `auto_sway`（自動でゆっくり揺れる）は使えない。

## 試験

```sh
(cd control-server && cargo build)     # 結合試験に必要（なければ skip）
clients/python/.venv/bin/python -m unittest discover -s clients/h3game/tests -v
```

| ファイル | 内容 | 必要なもの |
| --- | --- | --- |
| `test_server_backend.py` | バックエンド層の単体試験（偽クライアント）。frame_seq の扱い、owner 変更、無効値、/state のキー（session を取られたら `connected` false、`axis_conflict`・`hud_note` の生成と壊れた入力の耐性）、並行 POST | なし |
| `test_patch.py` | 元リポジトリの一時クローンに `git am`。blob SHA、変更ファイル、正本との一致。legacy は h3_direct を遅延 import し元の bridge と同じ応答・`websockets` 不要。server・dry は `serial`・`h3_direct` を import しない（import すると印を書いて失敗する偽 `serial` で確認）。サーバー不在でも POST が待たされない、壊れた数値（巨大な整数、2⁶⁴ 以上の frame_seq）の POST にも応答して `invalid` に数える。ポート使用中の 2 つ目の bridge はサーバーへ 1 回も要求を出さずに終わる（要求を数える偽サーバーで確認）。LAN のアドレスからの `/reconnect`・`/claim` は無視される（LAN の IPv4 アドレスが無ければその 1 件は skip）。HUD の server 表示 | h3game のクローン（下記） |
| `test_integration.py` | 実サーバー（sim）＋パッチ後の bridge。ブラウザーの代わりに 30Hz で POST。ARM/START 後に実カウントが期待値（混合式＋校正）に追従、`/state` の新旧キー、POST 停止で STOPPING → BRAKED_DISARMED かつ中立へ戻らない、frame_seq が止まると POST が続いても停止、ACTIVE 中に画面を隠しても中央へ動かない（ページの規則を模擬したもの）、再読み込みで新 session・自動再開なし、bridge の二重起動で走行が止まらない・session を取られたら `/state` が `connected: false`・`session_lost: true`、frame_seq なしは START 不可、dry はサーバーに何も届かない | 上記＋サーバーのバイナリ、websockets |
| `test_browser.py` ＋ `browser_check.mjs` | ヘッドレス Chromium でパッチ後の画面を開き、POST の `frame_seq` が増え続け `frame_age_ms` が小さいこと、`document.hidden` を偽装すると、隠した直後の最初の POST から `frame_seq` が表示中に最後に送った値のまま 0 が送られること（隠す瞬間に未送信のフレームがある状況を作って確認）、サーバー接続時は ACTIVE 中に隠しても目標・実カウントが中央へ動かず STOPPING → BRAKED_DISARMED でその位置に止まること、サーバーが `freshness_mode=frame_seq` で受理したことを確認 | node、Playwright（`PLAYWRIGHT_MODULE`）、Chromium（`CHROMIUM_PATH`）。なければ skip |
| `e2e_driver.py` | e2e の本体（unittest では拾わない）。`control-server/scripts/e2e_sim.sh` から起動し、サーバー・bridge を起動して接続 → ARM/START → 姿勢 → gain 上限 → ゲーム停止 → 再 ARM → 再読み込み → stop → center → 管理者 FAULT を通し、`docs/control-server/runs/e2e_sim_<日時>/` に summary.md と timeline.jsonl を書く | h3game のクローン（下記）、websockets の入った Python（`PYTHON`）。バイナリはスクリプトがビルドする |

h3game のクローン: 環境変数 `H3GAME_SRC` にクローンの場所を指定する。未設定なら、初回に `https://github.com/TakuroFukamizu/20261004_shi3z_hackathon.git` を `control-server/target/h3game-src` へ clone し、パッチを作った元のコミット `649ab66` を checkout して使う（ネットワークが要る。2 回目からはそのクローンを使う。`target/` の中なので git の管理外で、`cargo clean` で消える）。`H3GAME_SRC` もネットワークも無いときは、クローンを使う試験（`test_patch.py`・`test_integration.py`・`test_browser.py`）が理由を表示して skip し、`e2e_sim.sh` は理由を表示して終了コード 2 で終わる。試験は元のクローンを変更しない（一時ディレクトリに複製してから `git am` する）。

### 確認できていないこと

- 実際の（ヘッドありの）Chrome で、タブを裏に回したとき requestAnimationFrame が止まること。ヘッドレス Chromium では裏のタブでも `visibilityState` は `visible` のままで rAF も進んだ（確認時の値: 0.8 秒で 51 フレーム）。そのため `document.hidden` を偽装して index.html 側の歯止めだけを確認した。実ブラウザーで隠れたときは rAF が止まり、歯止めがなくても `frame_seq` は進まない想定（推定）。
- `do_POST` の例外の捕捉（`backend_errors`）。今のバックエンドで例外になる入力は見つかっていない（巨大な整数などは `invalid` になる）ので、この経路は試験で通していない。
- 実機（CoreS3 軸）での運転。結合試験は模擬軸だけ。
- ブラウザー → bridge の HTTP 転送時間は `frame_age_ms` に含まれない（同一 PC のループバックなので小さいと推定。未計測）。
