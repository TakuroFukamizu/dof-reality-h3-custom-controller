# AI エージェント向け: サーバーを見張りながら実機をデバッグする

h3-control-server を動かしたまま、実機（CoreS3 ×3 ＋ MD13S）で起きる停止や挙動の異常を調べるための手順。2026-11-02 に実機で使って役に立ったやり方をまとめた。人が操作している最中に AI が並行して見張る場面を想定している。

ここに書いた手順は、サーバーの HTTP API とログを読むだけのものと、リグを動かすもの（校正・ARM・試験走行）に分かれる。**リグを動かす操作は、必ず操作者に確認してから行う**（下の「やってはいけないこと」）。

## 1. 前提

| 項目 | 内容 |
| --- | --- |
| サーバー | `http://127.0.0.1:8892`（`server.bind`）。管理 API はループバックならトークン不要（`server.require_admin_token = false` のとき） |
| 設定 | 2026-11-02 午後の実機用: `control-server/config/h3-bringup-01-live-20261102.toml`（kp 30、端の余裕 350、#3 は模擬軸、`swap_pitch_roll = true`）。先頭のコメントに焼いてあるファームと前提がある |
| 起動 | `cd control-server && RUST_LOG=h3cs=debug nohup ./target/debug/h3-control-server --config <設定> >> <ログ> 2>&1 &`。古いサーバーが :8892 を持ったままだと、新しい方は何も言わずに終わる。起動前に `pkill -f 'h3-control-server --config'` |
| 模擬軸 | 起動のたびに `POST /v1/reference {"axis":3,"count":3069}` で基準を入れる（実機の軸は MCU が基準を覚えている） |
| Mac のスリープ | スリープで USB が止まる（Issue #11）。`caffeinate -i -s -w <サーバーの PID> &` |
| シリアルポート | サーバーが 3 台分のポートを開いている。AI が別にポートを開くと取り合いになる。ファームを焼くときはサーバーを止めてから |

## 2. 見るもの

### 2.1 `GET /v1/status`（主な情報源）

| キー | 使い方 |
| --- | --- |
| `rig_state` | `READY_TO_ARM` / `ARMING` / `HOLDING` / `ACTIVE` / `STOPPING` / `BRAKED_DISARMED` / `FAULT_LATCHED` / `CALIBRATION_REQUIRED` など。見張りはこの変化を拾う |
| `fault` | 直近の FAULT（`{"axis":2,"code":"ENCODER_FAULT","kind":"axis_fault"}` など） |
| `fault_history[]` | FAULT の履歴。**最新の FAULT の最初の記録にだけ `prehistory`（直前 2 秒の全軸の経過）が付く**（PR #43）。次の FAULT が起きると古い記録からは消えるので、**FAULT を見つけたらすぐ status 全体をファイルに保存する**。`fault_event` は MCU の FAULT_EVENT（`actual_count`、`mcu_state_before`、`mcu_time_us`） |
| `blockers` | ARM・START できない理由（`pose_not_near_hold`、`calibration_stale`、`position_outside_soft_limits` など） |
| `session` | `fresh`・`last_frame_seq`・`source_session_prefix`。**誰かが姿勢を送っているか**はここで分かる（`fresh: true` なら送信中） |
| `armed`、`requested_pose`、`effective_gain`、`target_counts` | いまの指令 |
| `axes[]` | 軸ごと: `mcu_state`、`fault_code`、`actual_count`、`target_count`、`following_error_count`（= 実測 − 目標）、`pwm_permille`（符号はカウント増が正）、`velocity_count_s`（ファームの 100 ms 窓）、`power_on_cmd`、`brake_release_cmd`、`configured`、`referenced`、`calibration_stale`、`fw_limits`（kp・duty_max・boot_id）、`status_age_ms`、`port` |

### 2.2 `prehistory` の読み方

列ごとの配列で、同じ添字が同じ STATUS（約 20 Hz に間引き、FAULT の時刻を 0 として `t_ms` が負）。軸ごとに `actual_count`、`target_count`（ファームの参照）、`server_target_count`（サーバーが送った目標）、`requested_count`（姿勢から計算した目標、プランナーの前）、`following_error_count`、`pwm_permille`、`velocity_count_s`、`mcu_state`、`fault_code`、`flags`。全体に `rig_state_before`、`requested_pose`、`effective_gain`。

```sh
python3 - <<'EOF'
import json
s = json.load(open("fault_XXXXXX.json"))
for f in s["fault_history"]:
    ph = f.get("prehistory")
    if not ph: continue
    print(f["fault"], f.get("fault_event"), ph["rig_state_before"], ph["requested_pose"], ph["effective_gain"])
    for ax in ph["axes"]:
        print("== axis", ax["axis"])
        for i in range(len(ax["t_ms"])):
            print(f'{ax["t_ms"][i]:6d} srv={ax["server_target_count"][i]} tgt={ax["target_count"][i]} '
                  f'act={ax["actual_count"][i]} fe={ax["following_error_count"][i]} '
                  f'pwm={ax["pwm_permille"][i]} v={ax["velocity_count_s"][i]} {ax["mcu_state"][i]}')
EOF
```

読むときの観点: (1) 実測が動いていたか（同じ値が続く = 動いていない）、(2) Duty はどこまで出ていたか（上限 = `duty_max` × 10 permille まで出ているか、小さいまま頭打ちか）、(3) 目標は動いていたか（サーバーの目標とファームの参照の差）、(4) 複数の軸が同時か（同時なら主電源・ブレーキ・指令など共通の原因）。

### 2.3 `GET /v1/events?n=<件数>&after=<id>`

サーバーのイベント（`identified`、`limits`、`cal_result`、`fault`、`fault_event`、`fault_prehistory`、`admin` など）。`after` に前回の最大 `id` を渡すと差分だけ取れる。

### 2.4 サーバーのログ

`RUST_LOG=h3cs=debug` で、`rig state from=… to=…`、`supervisor event kind="admin" detail={"cmd":"Start","error":{"blockers":[…]}}`（拒否の理由）、`rig fault`、`fault pre-history`（WARN、JSON）、`cal_result`（校正の結果。`n_end_count`・`neutral_count`・`reason`）が出る。時刻は UTC（日本時間 − 9 時間）。ANSI の色を落として読む: `sed 's/\x1b\[[0-9;]*m//g'`。

## 3. 見張りの仕掛け方

状態が変わったときだけ 1 行出し、FAULT ならその場で status 全体を保存する。Claude Code なら Monitor ツールで回す（30 分で切れるので張り直す）。

```sh
python3 -u - <<'EOF'
import json, time, urllib.request
OUT = "/path/to/scratch"   # 保存先
prev = None
while True:
    try: s = json.load(urllib.request.urlopen("http://127.0.0.1:8892/v1/status", timeout=2))
    except Exception: s = None
    cur = "SERVER_DOWN" if s is None else s["rig_state"]
    if cur != prev:
        if cur == "FAULT_LATCHED":
            fn = f"{OUT}/fault_{time.strftime('%H%M%S')}.json"
            json.dump(s, open(fn, "w"), ensure_ascii=False)
            print(time.strftime("%H:%M:%S"), "FAULT", json.dumps(s["fault"]), "saved", fn, flush=True)
        elif cur == "SERVER_DOWN" or prev in ("FAULT_LATCHED", "SERVER_DOWN"):
            print(time.strftime("%H:%M:%S"), cur, flush=True)
        prev = cur
    time.sleep(0.3)
EOF
```

**展示モードなど、FAULT を自動で解除するクライアントがいるとき**は、上の「状態の変化」を見る方式では取りこぼす（2026-11-02、ゲームの展示モードが FAULT を約 90 ms で解除し、0.3 s ごとの status では一度も `FAULT_LATCHED` が見えなかった）。その場合はイベントの `fault_prehistory`（FAULT のたびに 1 回出る。中身は prehistory 付き）を差分で拾う:

```sh
python3 -u - <<'EOF2'
import json, time, urllib.request
OUT = "/path/to/scratch"
def get(u):
    try: return json.load(urllib.request.urlopen(u, timeout=2))
    except Exception: return None
after = None
while True:
    e = get("http://127.0.0.1:8892/v1/events?n=200" + (f"&after={after}" if after is not None else ""))
    if e is None: after = None; time.sleep(1); continue          # サーバー再起動で id は振り直し
    evs = e.get("events", [])
    if after is None: after = max([x["id"] for x in evs], default=0); time.sleep(0.3); continue
    for x in evs:
        after = max(after, x["id"])
        if x["kind"] == "fault_prehistory":
            fn = f"{OUT}/fault_ev_{time.strftime('%H%M%S')}.json"; json.dump(x, open(fn, "w"), ensure_ascii=False)
            print(time.strftime("%H:%M:%S"), "FAULT", json.dumps(x["detail"].get("fault")), "saved", fn, flush=True)
        elif x["kind"] in ("stopping", "axis_inhibited", "disconnected"):
            print(time.strftime("%H:%M:%S"), x["kind"], json.dumps(x.get("detail"))[:160], flush=True)
    time.sleep(0.3)
EOF2
```

`stopping` の `reason`: 1 = 管理操作（STOP）、2 = ゲーム入力の途絶（`source_freshness_ms`）、3 = 中立到達。ログからも取れる: `fault pre-history`（WARN）の行の `detail=` 以降が同じ JSON。

- 出力は状態の変化だけに絞る（毎回の status を出すと通知が溢れる）
- 失敗の状態（`FAULT_LATCHED`、`SERVER_DOWN`、軸の切断）を必ず拾う。成功だけを拾う見張りは、止まっていても黙ったままになる

## 4. やってはいけないこと

- **操作者が動かしている最中に、AI がセッションを取らない。** `POST /v1/session`（`MotionClient` の起動を含む）は、いま送っているクライアント（ジョイスティックのページ、ゲーム）のセッションを無効にし、走行中なら止める。試験の前に `session.fresh` と `armed` を見て、送信中なら操作者に声をかける（2026-11-02 に AI の確認スクリプトが操作中のページを止めた）
- **校正（`POST /v1/calibrate`）は軸を機械端まで動かす。** 人が乗っていない・触れていないことを操作者に確認してから。ARM・START・試験走行も同じ
- 保護の上限（追従誤差・無パルス判定など）を、試験を通すために緩めない
- サーバーが動いている間にシリアルポートを開かない。`pio device monitor` も開かない
- ポート名で個体を決めない（ハブの差し替えで変わる）。`ioreg -r -c IOUSBHostDevice -l -w0` の `USB Serial Number`（1C:DB = #1・主電源担当、68:EE = #2、30:ED = #3）とポートを対応させてから

## 5. 復旧の手順

| 状態 | 手順 |
| --- | --- |
| `FAULT_LATCHED`（ENCODER_FAULT など、基準を失うもの） | 操作者に「シートが空いているか」を確認 → `POST /v1/fault/reset` → `POST /v1/calibrate {"axes":[1,2]}`（#3 単独は不可。主電源担当の #1 が要る）→ 両軸が `CALIBRATING` を抜けるまで待つ（約 20 s）→ ログの `cal_result` が `"reason":"ok"`（#1 5555〜5556 / 2777〜2778、#2 5508〜5509 / 2754 がいつもの値）→ **サーバーを再起動**（校正後は `calibration_stale` で ARM できない）→ `POST /v1/reference {"axis":3,"count":3069}` → `READY_TO_ARM` |
| `FAULT_LATCHED`（FOLLOWING_ERROR など、軸がソフトリミットの内側） | `POST /v1/fault/reset` だけで `READY_TO_ARM` に戻ることがある（status の `blockers` を見る） |
| `FAULT_LATCHED`（画面の非常停止 `LOCAL_ESTOP` だけ） | ボード画面でラッチを解除する。報告した軸が FAULT を出なくなり、サーバーが協調で止めた他軸の `HOST_FAULT` へ FAULT_RESET を送ると、`POST /v1/fault/reset` なしで `READY_TO_ARM` に戻る（自動では ARM しない。ログ `fault_auto_cleared`、`cause` は `local_estop_released`）。別の軸の `LOCAL_ESTOP` が残っている間、または `ENCODER_FAULT` などが混ざっているときは、従来どおり `POST /v1/fault/reset` |
| SOFT_LIMIT で軸がソフトリミットの外 | ARM できない（`position_outside_soft_limits`）。校正で戻す。機械端のすぐそばから校正すると失敗することがある（Issue #34）。失敗したら繰り返さず、操作者に実機を見てもらう |
| 校正が `stall` / `slow` で失敗 | 2 回失敗したら止める（モーターを端に押し付け続けない）。端に当たっていないか・主電源・MD13S の LED を操作者に確認してもらう |
| USB の無応答（status_age が伸びる、全軸 `sof0`） | Issue #11。ハブの上流を差し直すと戻る（MCU は再起動せず基準も残る） |

ファームを焼くとき: サーバーを止める → 個体とポートを確認 → `~/.platformio/penv/bin/pio run -e m5stack-cores3-v1 -t upload --upload-port /dev/cu.usbmodemXXXX`（1 台ずつ。PATH の `pio` は壊れている）→ 焼くと基準が消えるので校正する。

## 6. 2026-11-02 に見つかった「症状 → 原因」

| 症状（status / prehistory で見えるもの） | 原因 | 記録 |
| --- | --- | --- |
| ARM 直後（HOLDING の直後）に START すると、**2 軸同時に ENCODER_FAULT**。prehistory で Duty は上限まで出ているのに実測が 1 count も動かない | ARMING の電源待ちが 300 ms で、24 V が立ち上がる前に HOLDING になっていた（校正は 5000 ms 待つ）。2500 ms にした | `hw-r10a-test-20261102` の最後のコミット |
| 端の角（roll / pitch = ±1）のあと ENCODER_FAULT。目標の少し外で止まり、Duty は 10 % 前後で頭打ち、2 s 動かない | 小さな誤差では制御が上限まで出さない（積分の上限 3 %）。荷重の大きい姿勢で戻れない状態を、静止の監視が「エンコーダ断」と誤判定 | Issue #44 |
| 保持中（HOLDING）に、目標の数 count 下で 9 % のパルスと 1〜4 % を 0.2〜0.3 s ごとに往復 → ENCODER_FAULT。人が乗っていた | 保持中は出力 0・積分 3 % 頭打ちで、乗員の重さで沈む軸を支えられない。持ち上げては沈む往復が体感のガクガク | Issue #44（実測 2） |
| 端へ速く動かすと SOFT_LIMIT。サーバーの最下端の目標より 150 count 以上行き過ぎ | 端に向かう速度・制動の不足。端の余裕を 350 に広げた（暫定） | Issue #34、#46 |
| ゆっくり動かすとき（特に反転付近）にときどきガクッとする | ファームが目標を毎回「行って止まる」移動先として扱い、追いつくと速度を 0 にして FF が抜ける | Issue #45、PR #49 |
| START を押しても何も起きない | `pose_not_near_hold`（指令姿勢が中立から 300 count 以上）。ログの admin の `blockers` に出る | PR #41 で画面にも出る |
| #2 だけが下向きで止まりかける、遅れる | #2 の機構・エンコーダ（Issue #23）。制御の問題と混ぜない | Issue #23 |
| HOLDING（出力 0）で \|誤差\| 3〜4 のまま 2 s 静止して ENCODER_FAULT（2026-11-02 16:10 の #3） | r7 で静止の監視の対象を \|誤差\| > 2 に広げたが、保持（出るのは > 4）の中でも監視が回っていた。何も押していないのに「押しても動かない」と判定 | `hw-r10a-test-20261102` acc3a6d で修正 |
| 校正の直後に CONTROL_OVERRUN（出力 0、CALIBRATING のまま） | 新しいストロークを NVS に保存する間、フラッシュの書き込みで両コアが止まり制御の周期が 15 ms を超えた（見立て。保存済みの値で再校正すると出ない） | 未修正 |
| FAULT が見張りに映らないのに、気づくと CALIBRATION_REQUIRED | クライアント（ゲームの展示モード）が FAULT を自動で解除した。イベントの `fault_prehistory` で拾う（§3） | — |

## 7. 計測スクリプト

`clients/python/tools/`（README あり）。保持 ＋ 正弦波の A/B（`hw_ab3.py`）、CSV の解析、CENTER の再現など。**どれもリグを動かすので、操作者に確認してから**。状態の取得は 10 Hz なので、クライアント側の時刻で割った速度は使えない（ファームの `velocity_count_s` を使う）。数 Hz 以上の振動は取りこぼす。Python は `-P` を付けて実行する（作業ディレクトリのファイルが標準モジュールを隠すことがある）。
