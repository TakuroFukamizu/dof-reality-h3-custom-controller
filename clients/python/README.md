# h3_motion_client（Game 側 Python クライアント）

状態: **実装済み・模擬軸（sim）サーバー、パッチ後の h3game bridge と結合試験済み。実機は未実施。**

`h3-control-server` の Game API（[DESIGN.md](../../control-server/DESIGN.md) §7、§8）に Pose を送るクライアント。h3game の `bridge.py` から使う想定。管理 API（ARM / START / DISARM、管理トークン）は含めない。

- 依存: Python 3.9 以上、標準ライブラリ＋ `websockets>=13`（同期クライアント）。
- 確認した組み合わせ: Python 3.14 ＋ websockets 17.3、Python 3.9.6 ＋ websockets 13.1（どちらも全テスト通過）。

## 使い方

```python
from h3_motion_client import MotionClient

client = MotionClient("http://127.0.0.1:8892")
client.start()                      # 背景スレッド開始。すぐ戻る
client.submit_pose(pitch, roll, yaw, gain, frame_seq=n, source_age_ms=age)  # すぐ戻る
client.status()                     # 最新の StatusSnapshot（2 秒より古い・未受信なら None）
client.stop()                       # POST /v1/stop。確認できたら True。例外を出さない
client.close()                      # 座席は止めない（Pose が途絶えればサーバーが止める）
```

| API | 内容 |
| --- | --- |
| `submit_pose(pitch, roll, yaw, gain, frame_seq=None, source_age_ms=0.0)` | 最新 1 件スロットに置くだけ。通信を待たない。非有限値（float にできない巨大な整数を含む）・負の age・整数でない／0〜2⁶⁴−1 の外の frame_seq は保存せず `False`（例外は出さない） |
| `submit_gain(gain)` | gain だけの更新（`{"type":"gain"}`）。未送信の Pose があればその gain を書き換える。鮮度は延長しない |
| `status()` | WS で受けた最新 status（`GET /v1/status` と同じ形、`type` キーは除く） |
| `last_ack` | 最新の ack（dict） |
| `last_reject` | 最後に拒否された ack（`reason` が拒否理由。拒否が無ければ `None`。受理されても消えない） |
| `axis_conflict` | 直近の ack が `axis_conflict`（2 台の CoreS3 が同じ系統を選んでいる。DESIGN.md §10）で拒否された。受理された・別の理由の ack で戻る。**送信は止めず、再接続もしない**（オペレーターが CoreS3 側の選択を直せばそのまま受理される）。警告ログは競合 1 回につき 1 行 |
| `connected` | session 取得済みかつ WS 接続中（通信だけ。ARM 状態ではない） |
| `session_id` / `session_info` | 現行 session の完全な id / `POST /v1/session` の応答（`max_gain`, `source_freshness_ms`, `require_frame_seq`） |
| `session_lost` | `stale_session` の ack を受けた（別クライアントが session を取った）。以後 Pose を送らない |
| `reconnect()` | 通信を切って新しい session で張り直す。ARM / START はしない |
| `stop()` | `POST /v1/stop`（トークン不要） |
| `on_status=` | status 受信ごとに受信スレッドから呼ぶ。重い処理を入れない |

## 送信の規則（確定。テストで確認済み）

- 新しい submit があったときだけ送る。何も submit されなければ何も送らない（生存維持の再送をしない）。
- 送信のたびに `seq` を +1（クライアントの寿命を通して単調増加。session をまたいでも戻さない）。
- `source_age_ms` = submit 時の値 ＋（送信時刻 − submit 時刻）。キュー待ちで 0 に作り直さない。
- 切断中に submit され、再接続時点で `source_freshness_ms` より古くなった Pose は送らずに捨てる。
- `frame_seq` は渡された値をそのまま送る。クライアントは作らない。`None` なら `frame_seq` キーを付けない。
- 通信エラー時は接続を閉じ、0.2 秒から倍々で最大 2 秒のバックオフで、**新しい session** で再接続する。1 秒以上続いた接続のあとだけバックオフを 0.2 秒に戻す。
- 10 秒間何も受信しなければ（status は既定 100 ms ごと）切断して再接続する。
- `stale_session` の ack を受けたら自動で session を取り返さない（2 つのクライアントが奪い合わないため）。`reconnect()` で取り返す。
- `axis_conflict` の ack は通信の問題ではないので、session も接続も変えず送り続ける（`axis_conflict` プロパティで分かる。警告は競合 1 回につき 1 行）。

## bridge 側が知っておくこと

- **既定設定では `frame_seq` 必須**（`allow_missing_frame_seq = false`）。`frame_seq` なしの Pose は `missing_frame_seq` で拒否される。ゲーム更新が正常に終わるたびに増やす。同じ `frame_seq` の再送は受理されるが鮮度は延長されず、`source_freshness_ms`（既定 250 ms）後に `fresh=false` になる。
- `max_gain` は 0.35。超えた gain は `effective_gain` が 0.35 に制限され、ack の `limits` に `"gain_limited"` が入る。
- **走行中（ACTIVE）に再接続すると座席は止まる。** 駆動権限は START 時の session に束縛されるので、`reconnect()` や通信断からの自動再接続（新しい session）で必ず STOPPING → BRAKED_DISARMED になる。再開はオペレーターが ARM ＋ START をやり直す。意図した動作（模擬軸で確認済み）。
- 自分の session か確かめるには `status()["session"]["source_session_prefix"] == client.session_id[:8]`。status には完全な id は載らない。
- `GET /v1/status` の 503 `control_loop_unavailable` は制御ループが終わっている（落ちた）とき。終了処理（SIGTERM など）が始まるとサーバーは新しい接続を受け付けないので、503 ではなく接続拒否になる（確定: sim で確認）。終了処理の直前に受け付けた要求に 503 が返ることはありうる（推定、未観測）。`stop()` の `False` は「停止を確認できなかった」であって「停止していない」ではない（503 の stop はループが追いつけば実行される）。
- 接続しただけでは ARM・START・中央復帰をしない。`close()` も座席を止めない（Pose 途絶でサーバーが止める）。すぐ止めたいときは `stop()` を先に呼ぶ。

## 運用 CLI

```sh
python3 h3_motion_client.py status [--url http://127.0.0.1:8892] [--json]
python3 h3_motion_client.py stop   [--url http://127.0.0.1:8892]   # 確認できなければ終了コード 1
```

`status` は rig_state・blockers・`axis_conflict`（競合している軸）に加え、軸ごとに `device_id`（接続中の個体）・`expected`（設定の `device_id`、0 = 照合なし）・`assignment`（`selected`＝MCU の系統選択 / `config` / None）・`claimants`（`port(device_id)` の一覧。競合中は 2 つ以上、`CONFLICT` 印）・`reference_stale` を出す（DESIGN.md §10）。

## テスト

```sh
(cd control-server && cargo build)   # 実サーバー試験用（なければその試験は skip）
python3 -m unittest discover -s clients/python/tests -v   # websockets が入った python で
```

- `tests/test_motion_client.py`: プロセス内の偽サーバー（HTTP は標準ライブラリ、WS は `websockets.sync.server`）。
- `tests/test_against_server.py`: `control-server/target/debug/h3-control-server` を `config/sim.toml` で空きポートに起動して試験する。`H3CS_BINARY` で別のバイナリを指定できる。
