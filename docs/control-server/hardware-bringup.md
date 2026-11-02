# 実機 bring-up 手順（CoreS3 ×3 を h3-control-server につないで 3 軸を初めて動かす）

状態: **未実施の手順書**。PC 側（サーバー・クライアント）は模擬軸でだけ試験済み、CoreS3 の v1 ファーム（`env:m5stack-cores3-v1*`）は実機未確認。ここに書いた期待値のうち「過去の記録」と付けたものは従来 env（`moter-controler`）や encoder-tester での測定値で、今回の構成で再現するとは限らない。期待と違うことが起きたら先へ進まず §10 を見る。

対になる設定: [`control-server/config/bringup.toml`](../../control-server/config/bringup.toml)。参照: [protocol.md](protocol.md) §1.1 / §5 / §6、[`control-server/DESIGN.md`](../../control-server/DESIGN.md) §5 / §10、[README.md](README.md)（管理操作・安全上の注意）、[`moter-controler/README.md`](../../moter-controler/README.md)（配線・HOME/UPPER/CAL・NVS）、[cores3-controller/safety.md](../cores3-controller/safety.md)、ファーム設計 [2026-10-31-moter-controler-v1-protocol-design.md](../superpowers/specs/2026-10-31-moter-controler-v1-protocol-design.md)。

以下、`REPO` はリポジトリ直下、`PIO` は `/Users/takuro/.platformio/penv/bin/pio`（PATH の `pio` は使わない）、`URL=http://127.0.0.1:8892`、`AUTH="Authorization: Bearer $H3CS_ADMIN_TOKEN"`。

## 0. 前提と禁止事項

- **座席に人を乗せない。** この手順の全工程が無人。有人運転の可否はこの手順の結果で決めない（architecture.md §12 段階 5 の後に別に判断する）。
- **物理非常停止を用意する。** モーター用 24V を遮断する手段（AC プラグ・ブレーカー・非常停止ボタン）を、動かす前に手の届く場所に置く。サーバーの `POST /v1/stop` と CoreS3 画面の非常停止はソフトの停止で、サーバー・USB・MCU が正常に動いていることが前提（README「安全上の注意」）。サーバーは物理非常停止の状態を監視していない（`physical_estop: "not_monitored"`）。
- **誰が止めるかを決めてから始める。** 2 人いるなら、PC を操作する人と、機構を見て物理非常停止を押す人を分ける。1 人なら、動かす操作（ARM / START / Pose 送信）は物理非常停止に手が届く位置から行い、「異常と思ったらまず 24V を切り、理由は後で見る」と決めておく。
- **ノート PC のスリープを切る。** サーバーが止まると SET_POSITION が途絶え、MCU 側は `COMMAND_TIMEOUT`（100 ms）で減速 → FAULT・ブレーキになる（安全側だが、試験としては失敗になる）。システム設定でスリープを切るか、サーバーを `caffeinate -dims` 越しに起動する（§3）。蓋を閉じない。USB ハブは電源付きのものを使い、途中で挿し替えない。
- **ブレーキは無通電で作動、24V 通電で解除。** 解除しただけで荷重により機構が動くことがある（#1 は 3 % の Duty でも重力で沈むと記録されている。過去の記録）。ブレーキ解除の指令が出る操作（ARM）は、機構が自由に動いてよい状態で行う。
- **GND を共通にする。** エンコーダとリレーは H3 側と CoreS3 の GND が共通でないと動かない（カウント 0、A/B が High のまま。過去の記録）。
- **給電。** Base15 の DC ジャックと USB の同時接続は未検証（DIN Base では USB + 12V で約 85 秒後に電源断。Issue #1）。この手順では **USB のみで給電する env**（`-usbpower`）を使い、Base15 の DC は接続しない。DC 給電の env を使うなら、その組み合わせを先に別途確認する。
- **ポートを開く操作に注意。** macOS のポート名（`/dev/cu.usbmodemXXXX`）は物理ポートで決まり、個体の識別には使えない。同じ名前に別の ESP32-S3（別プロジェクトの StampS3A など）が現れたことがある（過去の記録）。ポートを開くと、そこにつながっている装置がリセットされることがある。サーバー・書き込み・モニターのどれも、対象のポートに何がつながっているかを `ioreg` の USB Serial Number（= eFuse MAC）で確かめてから使う（§1.1）。
- **従来 env と v1 env を同じ USB ポートで混ぜない。** v1 env は USB CDC をバイナリ専用にする（テキストの `?` や `HOME` は効かない）。従来 env の画面・シリアルで動かすのは §1 と、§5 でやり直すときだけ。
- 期待と違う動き（向きが逆・止まらない・音が変）が出たら、原因を考える前に 24V を切る。

## 1. ファームの書き込み

ここでは **操作者が** 書き込む。手順書を書いている側（AI）は書き込みもポートのオープンもしない。

### 1.1 ポートと個体の対応を取る

```sh
ioreg -p IOUSB -l -w0 | grep -E "USB Serial Number|USB Product Name"   # 装置をリセットせずに MAC を読む
$PIO device list                                                        # ポート名と SER=（MAC）の対応
```

CoreS3 は「USB JTAG/serial debug unit」（VID:PID 303a:1001）で、SER= が eFuse MAC。過去に使った個体（過去の記録。今回どれを使うかは操作者が決める）:

| 個体（eFuse MAC。呼び名は先頭 2 オクテット） | 画面 `dev` / status `device_id` | 過去の記録 |
| --- | --- | --- |
| `1C:DB` = `1C:DB:D4:BA:8D:58` | `dev 8D58` / `51311cdbd4ba8d58` | 2026-11-01 に v1 の `/v1/calibrate` で #1 を校正（5551 / 2775、OK）。2026-10-31 に #1 を校正（読み値 stroke 5551 / neutral 2775 / sign +1。記録には 5553 / 2776 もあり、CAL ごとに ±20 程度ぶれる）。17:19 のホットプラグ直後、USB には見えるがシリアルに応答しない状態になり、リセットで復帰。Base15 構成での最新の動作確認はこの個体 |
| `68:EE` = `68:EE:8F:D8:42:F0` | `dev 42F0` / `513168ee8fd842f0` | 2026-11-01 に従来 env の CAL で 5463 / 2731、v1 の `/v1/calibrate` で行程 5463（中立で失敗、電源）。2026-10-31 に #2 を校正（stroke 5462 / neutral 2731 / sign +1） |
| `30:ED` = `30:ED:A0:0B:20:40` | `dev 2040` / `513130eda00b2040` | 2026-11-01 に v1 の `/v1/calibrate` で行程 6138（中立で失敗、電源）。2026-10-25 に #3 を校正（stroke 6160 / neutral 3080 / sign +1、スロット #3）。10-31 の再校正 6138 / 3069 は同じ個体の **スロット #2** に入っている（§2 で `UNCAL` してから系統を選ぶ）。10-31 午前に 2 回、USB には見えるがシリアルに応答しない状態になりリセットで復帰（同日の 3 系統運転では、ポートを閉じて開き直しても 6 回とも固まらなかった。「ホストがポートを閉じると固まる」説の証拠は無い） |
| `48:27` = `48:27:E2:66:B0:68` | `dev B068` / `51314827e266b068` | 記録なし（未校正） |

この表の呼び名（`1C:DB` など）は MAC の **先頭** 2 オクテット。v1 ファームの画面の `dev XXXX` は MAC の **末尾** 2 オクテット、status の `device_id` は `5131` + MAC を表記順に並べた 12 桁（小文字 hex）。`ioreg` / `pio device list` の SER= と全桁で突き合わせる。

書き込みは 1 台ずつ、`--upload-port` を必ず付ける。出力を `head` などにパイプしない（途中で殺されて書き込みが完了しなかったことがある）。

### 1.2 従来 env で校正と中立出し（各台）

v1 env には手動移動・端当て・校正の操作が無い（画面は系統選択と非常停止だけ）。校正と「中立で止めてブレーキを掛ける」は従来 env で行う。**v1 は基準位置（/v1/reference）をソフトリミットの内側で取る必要があるので、P 端（HOME の位置 = count 0）で止めたまま v1 に移らない**（§5 に理由）。

```sh
cd REPO/moter-controler
pgrep -fl "pio run"                                   # 他のビルドが走っていれば待つ（.pio を共有）
$PIO run -e m5stack-cores3-usbpower
$PIO run -e m5stack-cores3-usbpower -t upload --upload-port /dev/cu.usbmodemXXXX
$PIO device monitor -p /dev/cu.usbmodemXXXX -b 115200   # 従来 env のときだけ
```

各台で（配線先のモーターに合わせて）:

1. `?` → `ENC motor=<n> cal=<0|1> stroke=... neutral=... sign=...` を読む。`motor=` が配線先の系統と違えば、電源 OFF の状態で `M <n>`（または画面左上の `#n` タップ）。**`OK motor=<n>` が返ることを確認する**（返らないまま校正すると別のスロットに入る。過去の記録）。
2. 機構を支持し、可動端から余裕があることを目視。`R 1`（主電源 ON）→ `B 1`（ブレーキ解除）。
3. `CAL`（HOME → UPPER → 保存 → NEUTRAL。10 % で端に当てる）。`AUTO saved motor=<n> stroke=<S> neutral=<N> sign=<±1>` を控える。`sign` が −1 の系統が出たら §5.2 を読む。既に校正済みで値を信じるなら `HOME` → `NEUTRAL` でもよい。
4. `NEUTRAL` が `done pos=<P>` で止まったら、`B 0`（ブレーキ作動）→ `R 0`（主電源 OFF）。**最後の `pos=<P>` を控える**（画面のセンサー行でも読める）。
5. 機構の位置に印を付ける（リフトの柱にテープ、ヨーは床に目印）。§5 で「動いていない」ことを目で確かめるため。
6. bringup.toml の値と比べる: `<S>` が `mechanical_max_count` と ±30 以内か。違えば §5.3 のとおり書き直す。

### 1.3 v1 env を書き込む（各台）

> **どのファームを焼くか（2026-11-02 時点）**: 3 台で検証済みの既知の良品はタグ **`fw-v1-r3`**（914cee8、PR #19 の先頭）で、運用設定 `[limits]` の kp 6 / duty_min 9 / ff 100 と組。**main の既定は r8**（`V1_FF_DIRECTIONAL=1`、方向別 FF ＋ PI、床なし）で **kp 30 が前提**（kp 6 のままだと追従が柔らかすぎる。#1 で FE rms 15）。r8 は #1 でしか試しておらず、#2（Issue #23）では未検証。迷ったら `fw-v1-r3` を焼く（Issue #35）。
>
> ```sh
> git worktree add ../fw-r3 fw-v1-r3      # main の作業ツリーを汚さない
> cd ../fw-r3/moter-controler && $PIO run -e m5stack-cores3-v1 -t upload --upload-port /dev/cu.usbmodemXXXX
> ```

```sh
cd REPO/moter-controler
$PIO run -e m5stack-cores3-v1-usbpower                                  # ビルドだけ
$PIO run -e m5stack-cores3-v1-usbpower -t upload --upload-port /dev/cu.usbmodemXXXX
```

書き込み中・リセット中はブレーキ作動（無通電）・主電源 OFF のまま。GPIO の浮きでリレーが入らないことは Base15 のプルダウンに依存する（safety.md「ハードウェア側の安全条件」）。起動後、画面に系統 `#n`、`DISARMED`、`device_id` の下 4 桁、`CONFIGURED` / `REFERENCED` の無いことが出る。**v1 env の後は `pio device monitor` を開かない**（バイナリが流れる。開くとポートを取り合ってサーバーがつなげない）。

## 2. 各台で系統を選ぶ

- **先にスロットの中身を揃える。** v1 ファームは選んだ系統のスロットの `sign` で駆動の向きを決める（`stroke` / `neutral` は使わない）。別の系統の校正値が入ったスロット（例: 30:ED の `#2` に #3 の 6138 / 3069）は、従来 env のうちに電源 OFF で `M 2` → `UNCAL` で消しておく。`sign` は 3 系統とも +1 の記録だが、スロットを間違えたまま CAL すると上書きされる。
- 系統は MCU 側で選ぶ（protocol.md §1.1）。NVS のキー `motor` は従来 env と同じなので、§1.2 で `M <n>` した個体は v1 でも同じ `#n` を表示するはず（推定。画面で確認する）。
- 違っていれば、**DISARMED のときだけ** 画面の系統表示をタップして 1 → 2 → 3 → 1 と送る。選ぶと `CONFIGURED` / `POSITION_REFERENCED` が消え、AXIS_SELECT が 1 回出る。
- v1 ファームは未選択（`selected_axis = 0`）を名乗らない（NVS に `motor` が無ければ `#1`。`control_v1_begin` が 1〜3 以外を 1 に丸める）。新品や初期化した個体を混ぜて起動すると `#1` が 2 台になり、`axis_conflict: axis1 ...`・`not_connected: axis1`・`rig_state = OFFLINE` になる。これは正常で、片方の画面で系統をタップして直す。1 → 2 → 3 の切り替えに USB の挿し直しは要らない（PC は AXIS_SELECT と STATUS の `selected_axis` で追い、同じポートのまま割当を変える。切り替えた個体には `reference_stale` が付くので §5.1 の基準を取り直す）。
- `unknown_device` になるのは、`[backend] ports` に CoreS3 以外の装置（別の ESP32-S3 など）のポートを書いたときだけ。PC はそのポートを閉じ、装置が消えるまで開き直さない。その場合は bringup.toml の `ports` を直してサーバーを起動し直す（挿し直しでは直らない）。
- 3 台が 1・2・3 を 1 つずつ。同じ系統を 2 台が選ぶと `axis_conflict` で ARM できない（§4.3 でわざと起こして見る）。

## 3. サーバー起動

```sh
cd REPO/control-server
cargo build                                               # target/debug/h3-control-server
./target/debug/h3-control-server list-ports               # ポート一覧。開かない
```

`list-ports` は Bluetooth や他の ESP32-S3 のポートも出す。**bringup.toml の `backend.ports` には CoreS3 3 台のポートだけ書く。** サーバーは書かれたポートを全部開いて HELLO_REQ を送るので（HELLO で軸が決まるまでそれ以外は送らない）、他人の装置のポートを書くと、その装置を開いて（場合によってはリセットして）しまう。

```sh
$EDITOR config/bringup.toml                               # backend.ports を §1.1 の対応に合わせる
./target/debug/h3-control-server check-config config/bringup.toml
```

`check-config` の期待出力: `OK (rig_id h3-bringup-01, ...)`、3 軸の `calibration_hash`、`static ARM blockers:` に `calibration_invalid: axisN calibration_valid = false` が 3 行。これ以外の阻害要因や `invalid config:` が出たら直す。

```sh
# 管理トークンは既定で不要（server.require_admin_token = false、loopback 専用）。要るなら require_admin_token = true にして export H3CS_ADMIN_TOKEN=... とする
mkdir -p ../docs/control-server/runs/bringup_$(date +%Y%m%d)
RUST_LOG=info caffeinate -dims ./target/debug/h3-control-server --config config/bringup.toml \
  2> >(tee -a ../docs/control-server/runs/bringup_$(date +%Y%m%d)/server.log >&2)
```

- 起動時に 3 ポートを開き、HELLO_REQ を 200 ms ごと・最長 2 s 送る。**ポートを開いたときに CoreS3 がリセットされることがある**（従来 env で「開くとリセットされることも、されないこともある」と記録。v1 でどうなるかは未確認）。リセットされると `boot_id` が変わり、基準位置は消える。だから基準位置（§5）は必ずサーバー起動の **後** に取る。サーバーを再起動したら §5 をやり直す。
- 別のシェルで:

```sh
URL=http://127.0.0.1:8892; AUTH="Content-Type: application/json"   # トークンを要るようにしたときは "Authorization: Bearer $H3CS_ADMIN_TOKEN"
curl -s $URL/v1/status | python3 -m json.tool
REPO/clients/python/.venv/bin/python REPO/clients/python/h3_motion_client.py status     # 要約表示（--json で全部）
```

- ブラウザーで <http://127.0.0.1:8892/> を開く（試験ページ、§4）。以下の管理操作は curl で書くが、ページの同名のボタン（ARM / START / STOP / DISARM / CENTER / FAULT / FAULT RESET / 軸ごとの REFERENCE）でも同じ。どちらで操作しても、確認は status の値で行う。

## 4. テストページで接続確認

サーバーは `GET /` で試験ページを返す（DESIGN.md §7.4。`control-server/static/test_page.html`、外部資源なし）。同じ PC のブラウザーで **<http://127.0.0.1:8892/>** を開き、管理トークン欄に `H3CS_ADMIN_TOKEN` の値を入れる。読み込んだだけでは何もしない（session 発行・ARM・姿勢送信はすべてボタン）。ページは `GET /v1/status` を 200 ms ごとに読んで `rig_state`・`blockers`・`fault`・`axis_conflict`・3 系統の表（接続 / port / device_id / expected / assignment / claimants / mcu_state / fault_code / flags（P=power_on_cmd、B=brake_release_cmd、C=configured、R=referenced、PC=power_controller）/ actual / target / velocity / f.err / pwm‰ / status_age / ref_stale）を出し、`GET /v1/events` の末尾を 1 秒ごとに出す。以下の表の項目名は status の JSON のもの。同じ値を curl（`$URL/v1/status`）や `h3_motion_client.py status` でも読める。**このタブは隠さない**（姿勢送信はタブが隠れると止まり、サーバーが減速停止する。安全側だが試験は中断する）。

### 4.1 3 軸がつながっている

status の期待値:

| 項目 | 期待 |
| --- | --- |
| `rig_state` | `CALIBRATION_REQUIRED`（3 軸接続済み、`calibration_valid = false` と基準未確立で止まっている） |
| `axes[].connected` | 3 軸とも `true` |
| `axes[].assignment` | 3 軸とも `"selected"`（MCU の選択で決まった。`"config"` は `device_id` だけで決まった旧経路、`null` は未接続） |
| `axes[].device_id` | `5131` で始まる 16 桁 hex（小文字）。下 12 桁は eFuse MAC を表記順に並べたもの（例 `1C:DB:D4:BA:8D:58` → `51311cdbd4ba8d58`。設計 §3: ファームは `esp_efuse_mac_get_default()` の 6 byte を先頭から上位に詰める）。画面の `dev XXXX` は MAC の末尾 2 オクテット（`dev 8D58`） |
| `axes[].mcu_state` / `fault_code` | `DISARMED` / `NONE` |
| `axes[].configured` | `true`（`CALIBRATION_REQUIRED` でもサーバーは接続時に CONFIG を送る）。`false` のままなら §10 `not_configured` |
| `axes[].referenced` | `false`（まだ） |
| `axes[].power_controller` | 軸 1 だけ `true`、軸 2・3 は `false`（CONFIG の `role_flags` を MCU が STATUS に返した値） |
| `axes[].power_on_cmd` / `brake_release_cmd` | 3 軸とも `false` |
| `axes[].actual_count` | 起動時 0 からの相対値。触っていなければ 0 付近。触って増減すること（機構を動かせる軸だけ。リフトは動かせないので飛ばす） |
| `axis_conflict` | `[]` |
| `blockers` | `calibration_invalid: axisN ...` ×3、`not_referenced: axisN ...` ×3、`position_outside_soft_limits: axisN actual 0 not in [300, ...]` ×3 **だけ**。3 つ目は基準前の `actual_count ≈ 0` がソフトリミット `[300, stroke−300]` の外にあるためで、この段階では正常。§5.1 の基準で消える |

`blockers` に `not_connected` / `unknown_device` / `status_stale` / `power_role_mismatch` / `protocol_mismatch` があれば §10。

### 4.2 device_id を転記 → 再起動で `axis_device_mismatch` が出ないこと

1. 3 軸の `device_id` を bringup.toml の `[[axis]].device_id` に写す（16 桁 hex そのまま）。画面の `dev XXXX`（下 4 桁）が `ioreg` / `pio device list` の MAC の **末尾 2 オクテット** と、status の `device_id` の下 12 桁が MAC 全体と一致していることを見て、個体と系統の対応を記録する（§9。§1.1 の表に各個体の期待値）。
2. サーバーを Ctrl-C で止めて起動し直す。status の `blockers` に `axis_device_mismatch` が **無い** こと、`expected_device_id` が転記した値になっていることを確認。
3. 以後、違う個体がその系統を選んだときに `axis_device_mismatch: axisN device <id>, config expects <expected>` で ARM が止まる。これが「個体と軸の照合」。

### 4.3 わざと競合を作って `axis_conflict` を見る

1. 系統 3 の個体（DISARMED）の画面で系統をタップして `#1` にする（3 → 1）。
2. status: `axis_conflict: [1]`、軸 1 の `claimants` が 2 つ、`assignment` が `null`、`blockers` に `axis_conflict: axis1 claimed by <port>(<id>), <port>(<id>)` と `not_connected: axis1`、軸 3 は `not_connected`。`rig_state` は `OFFLINE`。`curl -sX POST -H "$AUTH" $URL/v1/arm` は 409 `axis_conflict`。
3. タップで 1 → 2 → 3 と戻す。`axis_conflict` が `[]` に戻り、軸 1・3 が `connected`。
4. 戻した個体には **`reference_stale: axis3 re-reference required after axis change`** が付く（同じ boot のまま系統を変えた個体は、PC が自分で SET_REFERENCE を送り直すまで ARM しない。protocol.md §1.1）。§5 の /v1/reference で消える。USB を挿し直して boot を変えても消える。
5. 系統 1 の個体にも同じ競合中、`axis_device_mismatch`（§4.2 を済ませていれば）が一時的に出る。戻せば消える。

## 5. 基準位置

### 5.0 サーバーから校正する（v1 ファームが CALIBRATE に対応していれば、こちらが先）

v1 ファームの端当て校正（protocol.md §1.2、DESIGN.md §11）を使うと、§5.1〜§5.3 の「従来 env で校正 → 値を転記 → 中立で止めて /v1/reference」が 1 回の操作になる。**機構が P 端・N 端の両方まで動く**ので、座席に人がいない・可動範囲に物がない・物理非常停止を押せる人がいる、を確認してから。

1. 3 軸が接続され、全軸 `DISARMED`（status の `axes[].mcu_state`）であること。`calibration_valid = false` のままでよい（校正に CONFIG も基準も要らない）。
2. 試験ページの「校正（3 軸）」（確認ダイアログあり）、または `curl -sX POST -H "$AUTH" $URL/v1/calibrate`。電源担当軸（系統 1）を含めないと 409 `power_axis_required`（他の軸は 24V が無く端当てに失敗する）。
3. status: 各軸が `CALIBRATING`（blockers に `calibrating: axisN`。この間 ARM / START / CENTER / REFERENCE は 409 `calibrating`）。止めるなら「校正を中止」/ `POST /v1/calibrate/cancel` / STOP。
4. 終わると各軸の `calibration` に `result`（`OK` / `FAILED` / `CANCELLED`）・`reason`・`n_end_count`（N 端の count。P 端 = 0）・`neutral_count`・`sign`。`OK` の軸は `referenced = true`（MCU が P 端 = 0 の基準を自分で作る。/v1/reference は不要）で中立に止まっている。同時にその軸は `calibration_stale = true` になり、blockers に `calibration_stale: axisN recalibrated (stroke S, neutral N); export and reload config before ARM` が出る（MCU の原点・行程・向きが測り直されたので、いま動いているサーバーの設定の幾何は承認済みとみなさない。Issue #15 H3）。**この印は 5.→6. で export したファイルでサーバーを起動し直すまで消えない**（CoreS3 の再接続・再起動でも消えない。旧設定のまま再起動しても消えるので、起動する `--config` が export したファイルであることは操作者が確かめる）。`FAILED`（`stall` / `slow` / `travel` / `timeout` …）なら従来 env の §1.2 と同じ切り分け。
5. 「校正結果を設定に書き出す」/ `POST /v1/calibration/export`。`bringup.toml` と同じディレクトリに `h3-bringup-01.calibration-<YYYYmmdd-HHMMSS>.toml`（UTC）が書かれる: OK の軸の `mechanical_* / soft_* / count_at_virtual_* / device_id` が結果から置き換わり、コメントと他の軸はそのまま、`calibration_valid = false` のまま、先頭に生の結果と置き換え前の値。
6. そのファイルを見て、`n_end_count` が過去の記録（§5.3、±30）と合うこと・`sign` が §5.2 の前提と合うことを確かめ、§5.5 の手順で `calibration_valid = true` にして `--config` にそのファイルを指定して起動し直す（この再起動で 4. の `calibration_stale` が消える。再起動せずに ARM はできない）。基準は boot ごとに消える（§5.1 の末尾）ので、再起動後は校正をやり直すか、中立で止まっていれば §5.1 の /v1/reference で `count_at_virtual_512` を入れる。

以下 §5.1〜§5.4 は、校正に対応していないファーム・校正が失敗するときの手動の手順（そのまま使える）。

### 5.1 手順

前提: §1.2 で各軸を従来 env の `NEUTRAL` で止め、`B 0` `R 0` でブレーキ作動・電源 OFF にし、`pos=<P>` を控え、印を付けてある。それ以降、機構が動いていないことを印で確かめる（動いた疑いがあれば §5.4）。

```sh
curl -sX POST -H "$AUTH" -d '{"axis":1,"count":2776}' $URL/v1/reference    # <P> を軸ごとに入れる
curl -sX POST -H "$AUTH" -d '{"axis":2,"count":2731}' $URL/v1/reference
curl -sX POST -H "$AUTH" -d '{"axis":3,"count":3080}' $URL/v1/reference
```

- 試験ページでは 3 系統の表の REFERENCE 列に `count` を入れてボタン（確認ダイアログあり）。既定の 0 のまま押さない。
- `/v1/reference` は全軸が駆動していない（DISARMED）ときだけ。その軸へ SET_REFERENCE を送り、**現在の計数値を `count` と定義する**（MCU 側: `reference_offset = raw − count`、以後 `actual_count = raw − reference_offset`）。
- 期待: 200 `{"ok":true,...}`、status で `axes[].referenced = true`、`actual_count` が入れた値（触っていなければ ±数カウント）、`blockers` から `not_referenced` と `position_outside_soft_limits` が消える。
- `<P>` は `count_at_virtual_512`（stroke/2）と ±30 以内のはず（NEUTRAL の到達帯は ±10、過去の記録でブレーキ解除・PWM 0 のまま 65 秒で +9 のドリフト）。外れていれば、従来 env の校正値と bringup.toml の stroke / neutral が一致しているか見直す。
- なぜ P 端（HOME の `pos=0`）で基準を取らないか: bringup.toml は `mechanical_min_count = 0`（P 端）、`decel_zone_count = 300`、`soft_min_count = 300`。実位置がソフトリミット `[300, stroke−300]` の外だと、サーバーは `position_outside_soft_limits` で ARM せず、MCU も ARM_PREPARE を `BAD_STATE` で拒否する（protocol.md §4）。v1 env には手動移動が無いので、P 端から範囲内へ戻す手段が無い。だから **範囲内（中立）で止めてから基準を取る**。P 端で基準を取るなら、`count` に 0 を入れた後に従来 env で中立へ動かし直す必要があり、二度手間になる。
- 基準は MCU の boot ごとに消える（`boot_id` が変わる → `referenced = false`）。USB の挿し直し・サーバー起動時のリセット・ENCODER_FAULT の解除（`POSITION_REFERENCED` を消す）のたびにやり直す。やり直すときの `count` は「機構が動いていなければ前回と同じ値」。動いた後なら、動く前に status で読んだ最後の `actual_count`（ブレーキが保持していた前提）。確信が無ければ §5.4。

### 5.2 符号を count に掛けない理由と、座標の約束

- 従来 env は `position = sign × (raw − origin)` で表示する（`sign = +1` なら速度 − でカウント増、P 端 = 0、N 端 = +stroke）。v1 は **`actual_count = raw − reference_offset` で、系統の符号を掛けない**（protocol.md §1.1「系統の選択で `actual_count` の向き・原点・スケールを変えない」）。count は常に生のエンコーダ計数の平行移動で、向き・ストローク・中立は PC 側の `[[axis]]` で扱う。MCU が系統ごとに向きを変えると、PC の校正と MCU の座標が二重になり、どちらが反転したか分からなくなるため。
- 3 系統とも過去の校正で `sign = +1` なので、v1 の count は従来 env の表示と同じ向き（P 端 0、N 端 +stroke）になる。bringup.toml はその前提で `mechanical = [0, stroke]`。
- **`sign = −1` の系統が出たら**（§1.2 の `AUTO saved ... sign=-1`）: その軸の生カウントは N 端へ向かって **減る**。P 端を 0 とすると N 端は −stroke、中立は −stroke/2。その軸だけ `mechanical_min_count = −stroke`、`mechanical_max_count = 0`、`soft = [−stroke+300, −300]`、`count_at_virtual_512 = −stroke/2`、`count_at_virtual_0 / 1023` もその範囲内に置く。/v1/reference の `count` は −<P>（従来 env の表示 `<P>` は符号を掛けた後の値なので、生に戻す）。v1 のモーター出力は NVS の `sign` から向きを決める（`duty_signed = −sign × duty`）ので、ファーム側は何も変えない。

### 5.3 `count_at_virtual_*` がストロークから決まる仕組み

bringup.toml（軸 1、stroke 5553 の例）:

```text
P 端                                                          N 端
0 ──300──┤ soft_min 300 ……… 512 → 2776 ……… soft_max 5253 ├──300── 5553
   decel                                                  decel
   count_at_virtual_0 = 300        count_at_virtual_1023 = 5253
```

- `mechanical_min/max` = 従来 env の HOME / UPPER で当てた端（10 % Duty で止まる位置。到達位置は 0〜16 / 6155〜6171 の幅でばらつく。過去の記録）。目標には使わない。
- `soft_min/max` = 機械端から `decel_zone_count`（300）だけ内側。サーバーは `soft_min − mechanical_min ≥ decel_zone_count`（かつ `soft_min > mechanical_min`）、`mechanical_max − soft_max ≥ decel_zone_count`（かつ `soft_max < mechanical_max`）を `check-config` で検査し（bringup.toml はちょうど等号で通る）、PC も MCU も目標をここでクリップし、実位置がここを外れると `SOFT_LIMIT`。
- `count_at_virtual_512` = stroke / 2（従来 env の `neutral` と同じ値）。仮想位置 512（Pose 0）がここ。
- `count_at_virtual_0` / `count_at_virtual_1023` = `soft_min` / `soft_max`。仮想 0〜1023 を運用範囲いっぱいに割り当てる。実際に動く量はゲームの gain で決まる: `g = gain × 511`、`m1 = 512 + g (roll + pitch)`、`m2 = 512 + g (roll − pitch)`、`m3 = 512 + g × yaw`（architecture.md §2.2）。gain 0.15 では仮想 512 ± 77（入力 1 つのとき）→ 軸 1 で約 ±370 count（モーター約 0.9 回転、座席の角で 10 mm 前後。過去の記録 0.03 mm/count から推定）。gain の上限は `[limits].max_gain`（bringup.toml は 0.15、example / sim は 0.5。[limits.md](limits.md) §4）。
- **向き**: 仮想 0 → 1023 で座席が上がるか下がるか（ヨーは右か左か）は新しい機構で **未確認**（architecture.md §6）。bringup.toml は全軸「count 増 = 仮想位置 増」。過去の記録では #1 / #2 の N 端 = 上（P 端 = 沈み切り）、#3 の P 端 = 右端なので、この仮定のもとでは `pitch +` で #1 が上がり #2 が下がる、`roll +` で両方上がる、`yaw +` で #3 が左へ動く。ゲームの意図と逆な軸は、**その軸の `count_at_virtual_0` と `count_at_virtual_1023` の値を入れ替える**（`count_at_virtual_512` はそのまま。単調減少は許される。3 点がソフトリミット内で単調なら `check-config` が通る）。入れ替えたらサーバー再起動 → §5.1 からやり直し。
- ストロークが記録と違った（§1.2 の `<S>` が ±30 を超える）: `mechanical_max_count = <S>`、`soft_max_count = <S> − 300`、`count_at_virtual_1023 = <S> − 300`、`count_at_virtual_512 = <S>/2` に書き直し、`check-config`。

### 5.4 位置が信用できないとき

機構が動いた疑い（印がずれた、リフトが沈んだ、ENCODER_FAULT の後など）があれば、推測で `count` を入れない。該当する個体に従来 env を書き戻し（§1.2）、`HOME` → `NEUTRAL` → `B 0` `R 0` → `pos=` を控え直し、v1 env を書き戻して §2 以降をやり直す。時間はかかるが、基準を誤ると `SOFT_LIMIT` の保護が機能しない。

### 5.5 `calibration_valid = true` にする

§5.1 が 3 軸とも通り、各軸の `actual_count` が `count_at_virtual_512` ± 30、§1.2 の stroke が bringup.toml と一致、座席に人がおらず、物理非常停止を押せる人がいる。この 4 点を確認したら bringup.toml の 3 軸を `calibration_valid = true` にしてサーバーを再起動し、§5.1 をやり直す（再起動で CoreS3 がリセットされた場合に備える。`referenced` が `true` のままなら不要）。status: `rig_state = READY_TO_ARM`、`blockers = []`。

`true` は「この軸でこの値を測った」という操作者の宣言で、有人運転の可否ではない。向きの確認（§7）が終わるまで `[limits].max_gain` は bringup.toml の 0.15 のまま（limits.md §4 の手順 2）。

## 6. 無負荷・モーター電源 OFF で ARM / START の流れ

**モーター用 24V を物理的に切り離した状態**（AC プラグを抜く・24V 出力を外す。配線に合わせて、主電源リレーを閉じても 24V が出ない点で切る）で、指令の流れとフラグだけを確認する。24V が無いのでモーターもブレーキ（24V で解除）も動かない。

### 6.1 Pose の送り方

ゲームは使わない。2 通りある。どちらも ARM / START はしない（駆動は人の管理 API だけで始まる）。

- **試験ページ**（§4）: 「session 発行」→ pitch / roll / yaw（−1..1）と gain（0..`max_gain`）のスライダー → 「送信開始」。50 Hz で送り、最後の ack（accepted / reason / effective_gain / limits / fresh）を表示する。「送信停止」またはタブが隠れると止まり、サーバーが 250 ms 後に減速 → 保持 → ブレーキ。
- **スクリプト**: 値を正確に（`roll = pitch` のように）与えたいときは、`clients/python/h3_motion_client.py` の `MotionClient` で一定周期に送る最小のスクリプトを使う。スクラッチ領域に保存して実行する:

```python
# bringup_pose.py: 指定の Pose を 30 Hz で送り続ける。Ctrl-C で送信をやめる → サーバーが 250 ms 後に減速 → 保持 → ブレーキ。
import sys, time
sys.path.insert(0, "REPO/clients/python")
from h3_motion_client import MotionClient
pitch, roll, yaw, gain = (float(x) for x in sys.argv[1:5])
c = MotionClient("http://127.0.0.1:8892"); c.start()
seq = 0
try:
    while True:
        seq += 1
        c.submit_pose(pitch, roll, yaw, gain, frame_seq=seq)
        time.sleep(1 / 30)
except KeyboardInterrupt:
    pass
finally:
    c.close()
```

```sh
REPO/clients/python/.venv/bin/python bringup_pose.py 0 0 0 0.05     # pitch roll yaw gain。管理 curl とは別のターミナルで前面実行
```

- スクリプトは **管理操作とは別のターミナルで前面で** 動かす（バックグラウンド `&` にすると Ctrl-C が届かない）。止めるのはそのターミナルの Ctrl-C。
- `frame_seq` を進め続ける限り鮮度が保たれる。送信を止める（Ctrl-C、試験ページなら「送信停止」）と `source_freshness_ms`（250 ms）で `STOPPING → HOLDING → 1 s → DISARM → BRAKED_DISARMED`。**その場で止まり、中央へは戻らない。**
- 以下「送信開始」「送信停止」は、試験ページのボタンかスクリプトの起動 / Ctrl-C のどちらか。
- このスクリプトは ARM / START しない。駆動は人の管理 API だけで始まる。

### 6.2 ARM

```sh
REPO/clients/python/.venv/bin/python bringup_pose.py 0 0 0 0.05     # 別ターミナルで前面実行（試験ページなら pitch=roll=yaw=0、gain 0.05 で送信開始）。Pose 0 = 仮想 512 = count_at_virtual_512
curl -sX POST -H "$AUTH" $URL/v1/arm
curl -s $URL/v1/status | python3 -m json.tool
```

期待（protocol.md §5 の ARMING の順序）:

| 時点 | 期待 |
| --- | --- |
| ACK 直後 | `rig_state = ARMING`。3 軸とも `mcu_state = ARMING` |
| 主電源 ON 指令 | 軸 1 だけ `power_on_cmd = true`（`POWER_CONTROLLER` の軸だけが G2 を操作。軸 2・3 は `false` のまま）。主電源リレーの動作音が聞こえることがある（Q1 → HRS4H の経路に給電があれば。未確認） |
| `POWER_SETTLE_MS`（300 ms）後 | 実位置がソフトリミット内 → 現在位置を保持目標にして位置制御開始 → 3 軸とも `brake_release_cmd = true` |
| さらに `BRAKE_RELEASE_WAIT_MS`（300 ms）後 | 3 軸とも `mcu_state = HOLDING`、`rig_state = HOLDING`、`armed = true`、`target_count ≈ actual_count`、`pwm_permille = 0` 付近（誤差 5 count 以内は Duty 0） |

24V が無いので機構は動かない。`following_error_count` は 0 付近のまま。5 秒以内に HOLDING にならなければ `ArmTimeout`（§10）。

### 6.3 START（期待どおりに FAULT する場合がある）

```sh
curl -sX POST -H "$AUTH" $URL/v1/start
```

- START の受理には、期限内の SET_POSITION（ARM 中からサーバーが送り続けている保持目標）と、新鮮な Pose（スクリプトが送っている）が要る。`409 no_fresh_pose` ならスクリプトが止まっている。その Pose が作る目標は保持位置から `[game].start_max_offset_count`（300 count）以内でなければならない（`409 pose_not_near_hold`。上の `0 0 0 0.05` なら中立で問題ない。§10 の表）。
- 受理後 `rig_state = ACTIVE`、`active_source = "game"`、3 軸 `mcu_state = ACTIVE`。目標は `count_at_virtual_512` へ `[limits].max_vel_count_s` 以内で動く（limits.md §4 の低い値 600 count/s で始める）。
- **ここで 3 通りに分かれる**（ファームの無パルス判定は「ブレーキ解除指令中に目標（軌道）が動いている間」だけ働く。設計 §3）:
  - `|actual_count − count_at_virtual_512| ≤ 5`（基準がぴったり）: 目標 = 実位置で Duty 0 のまま。ACTIVE が続く。送信停止 → `BRAKED_DISARMED` まで進むことを見る。
  - ずれが小さい（目安: 数十 count 未満。`max_acc` 2000 count/s² では目標が 200 ms で動く距離が約 40 count）: 目標は 200 ms 以内に `count_at_virtual_512` に着いて止まり、無パルスの窓が閉じる。位置ループは Duty 下限（`[limits].duty_min_percent`。SET_LIMITS を受ける前のファーム既定は 7 %）を出し続けるが 24V が無いので動かず、**ACTIVE のまま `following_error_count` がそのずれで一定**（`pwm_permille` ≈ ±70）。FAULT にはならない。送信停止 → STOPPING → HOLDING → `BRAKED_DISARMED` まで進むことを見る（DISARMING でもこの残り誤差で FAULT にならない）。
  - ずれが大きい（数十 count 以上）: 目標が動いている間に Duty ≥ 7 % でカウントが変わらないので、`STALL_MS`（200 ms）で **`ENCODER_FAULT`**。サーバーは FAULT_EVENT / STATUS でラッチし、他の軸へ HOST_FAULT。status: `rig_state = FAULT_LATCHED`、`fault = {"kind":"axis_fault","axis":N,"code":"ENCODER_FAULT"}`、3 軸とも `power_on_cmd = false`・`brake_release_cmd = false`、FAULT の軸 `fault_code = ENCODER_FAULT`、他は `HOST_FAULT`。**これは異常検出が働いた正常な結果**。`GET /v1/events` に `fault` と `rig_state` の遷移が並ぶ（§9）。
- FAULT からの戻し: 送信を止める → `curl -sX POST -H "$AUTH" $URL/v1/fault/reset`。`fault/reset` は 200（`{"ok":true,"rig_state":"FAULT_LATCHED"}`）を返すが、解除の成否はこの応答では分からない（MCU の ACK は後から届く）。数百 ms 後に status を読み、`rig_state` が `READY_TO_ARM`（または `CALIBRATION_REQUIRED`）になっていれば解除済み。`FAULT_LATCHED` のままで `blockers` に `fault_reset_rejected: axisN FaultCausePresent` が出ていれば MCU が「原因が残っている」と判断している（v1 ファームでこれになるのは画面の非常停止がラッチ中のときだけ。ENCODER_FAULT は常に解除できる。`GET /v1/events` に `ack_rejected` が並ぶ）。原因を除いてからもう一度 `fault/reset` を送る。ENCODER_FAULT を解除した軸は `POSITION_REFERENCED` が消える（protocol.md §4）ので `not_referenced` に戻る。機構は動いていないので §5.1 と同じ `count` で `/v1/reference` をやり直す。
- ここまでで確認できたこと: ACK → STATUS の順序、ARMING の電源 → ブレーキの順、`POWER_CONTROLLER` の担当分け、FAULT 時の全軸停止指令、FAULT_RESET と再基準化。24V を入れる前にこの流れを一度通す。

### 6.4 DISARM と stop

HOLDING のまま `curl -sX POST -H "$AUTH" $URL/v1/disarm` → 3 軸 `DISARMING`（ブレーキ給電 OFF → 300 ms → PWM 0 → 主電源 OFF 指令）→ `DISARMED`、`rig_state = BRAKED_DISARMED`。ブレーキが掛かってから `actual_count` が 8〜38 count 動くのは正常（2026-10-31 の実測: #2 +8、#3 +38。ブレーキの食い込み）。基準位置はブレーキ後の値で考える（§5.1 の「動いていなければ前回と同じ値」は ±40 の幅がある）。`curl -sX POST $URL/v1/stop`（トークン不要）はどの状態でも 200。

### 6.5 実機が 3 台そろわないとき（模擬軸で代替）

サーバーは 3 軸そろわないと ARM しない。1 台を別件で外しているときは `[backend]` に `sim_axes = [<軸>]` を足し、その軸のポートを `ports` から外す（`control-server/config/bringup.toml` のコメント）。その軸は模擬軸（`sim:<軸>`）として接続・基準済みで現れ、目標に完全に追従する。その軸の `calibration_valid = true` は模擬軸用（実機の宣言ではない）。残りの実機だけを Pose の選び方（§7）で動かして確認する。実機を戻したら `sim_axes` を消す。

## 7. 1 軸ずつ電源 ON で低ゲイン

**24V を入れる前に**: 座席に人がいない、機構の可動範囲に物も人もない、物理非常停止に手が届く、§6 が通っている、`bringup.toml` の `[limits]` が低い値のまま（`max_gain 0.15`、`max_vel 600`、`max_acc 2000`、`duty_max 20`、`duty_min 7`。出荷時のまま。limits.md §4）、印の位置が §5.1 の基準と合っている。

制約: 主電源は軸 1 の G2 が共通に入れるので、**24V を入れると 3 軸とも通電する**（safety.md「主電源リレーのインターロック」）。また、サーバーは 3 軸そろわないと ARM せず、24V の無い軸は目標が動く（中立まで数十 count 以上離れている）と `ENCODER_FAULT` になる（§6.3）ので、「2 軸を電気的に切り離して 1 軸だけ動かす」は ARM / HOLDING の確認にしか使えない。動かす確認は **3 軸とも通電したまま、Pose の選び方で 1 軸ずつ動かす**:

| 動かす軸 | Pose（pitch roll yaw） | 他軸 |
| --- | --- | --- |
| #3 後部ヨー | `0 0 +1` / `0 0 −1` | m1 = m2 = 512 のまま |
| #1 右手前リフト | `+1 +1 0` / `−1 −1 0`（roll = pitch） | m2 = 512 のまま、m3 = 512 |
| #2 左リフト | `−1 +1 0` / `+1 −1 0`（roll = −pitch） | m1 = 512 のまま、m3 = 512 |

### 7.1 最初の通電と ARM

1. 24V を入れる（主電源リレーはまだ開いているので、モーター・ブレーキには来ない。DC_STOP1 経由の配線のとおりかは機体で確認）。
2. 送信開始（`0 0 0 0.05`） → `/v1/arm`。§6.2 と同じ遷移。**HOLDING になった瞬間に機構が動かないこと**（ブレーキ解除 → 位置制御の引き継ぎ）。リフトが沈む・ヨーが流れるなら即 24V を切る（位置制御の向きが逆、または KP 不足。§10 `FOLLOWING_ERROR`）。
3. 10 秒 HOLDING のまま status を 2〜3 回見る: `following_error_count` が ±20 以内、`pwm_permille` が小さい、`fault_code = NONE`。**START はさらに 10 秒待ってから**（2026-10-31 の記録: 24V 投入から 4 秒では両方向とも無パルスで動かず、12 秒後は動いた。動かなければ `ENCODER_FAULT` になる）。
4. `/v1/disarm` → `BRAKED_DISARMED`。ブレーキ作動後に機構が動かないこと。

参考（2026-10-31 の実測、従来 env の手動速度。無人・USB 給電・24V あり）: Duty 8 % ≈ 540〜550 count/s（#3）、10 % ≈ 750〜810、13 % ≈ 920〜985、15 % ≈ 1150〜1360、自動移動 15 % の最大 #1 1460 / #3 1400 / #2 1300。#2 の持ち上げ側は 8 % 台で 200 count/s 未満。v1 の `max_vel 600` は 7〜8 % 相当。

### 7.2 1 軸ずつ動かす（gain 0.05 から）

1. 送信開始（`0 0 0 0.05`） → `/v1/arm` → `/v1/start`。ACTIVE で目標が `count_at_virtual_512` へ寄る（基準の誤差ぶん、数十 count 以内の小さな動き）。
2. 送信停止 → `BRAKED_DISARMED` になるのを待つ（その場で止まる）。
3. 送信開始（`0 0 1 0.05`） → `/v1/arm` → `/v1/start`。#3 の目標が `count_at_virtual_512 + 0.05 × 511 × (5560/1023) ≈ +139 count` へ動く。**向きを記録する**（右か左か）。#1 / #2 は動かない。
4. 送信停止。`0 0 -1 0.05` で反対へ。
5. 同様に `1 1 0 0.05`（#1 だけ、roll + pitch = 2 なので約 +250 count）、`-1 1 0 0.05`（#2 だけ）。それぞれ **上がったか下がったか** を記録。
6. ゲームの意図（`pitch +` で前が上がるのか、`roll +` で右が上がるのか、`yaw +` で右へ向くのか）と比べる。混合式は h3game のまま（`roll` = #1・#2 同方向、`pitch` = #1・#2 逆方向）なので、**前後と左右が入れ替わっていれば `[game].swap_pitch_roll = true`**、向きだけ逆なら `invert_pitch` / `invert_roll` / `invert_yaw`（いずれもゲーム側の軸の意味で指定。ゲームは変えない）。1 軸だけ逆（そのモーターの配線・符号）なら §5.3 の方法で `count_at_virtual_0 / 1023` を入れ替える。どれもサーバー再起動 → §5.1 → この節をやり直す。組み直した H3 は 2026-11-02 に `swap_pitch_roll = true` と判明（ジョイスティックの前後で左右に傾いた）。
7. 1 軸ずつ通ったら gain 0.10、0.15 で同じ Pose。`max_gain`（0.15 にしてある）を超える値は `effective_gain` が 0.15 に制限される（`limits` に `gain_limited`）。
8. 各回、status の `following_error_count`（`following_error_limit_count` に近づくと FAULT）、`velocity_count_s`（`max_vel_count_s` 以下）、`telemetry.tick_dt_ms.p99`（10 ms 前後）を控える。
9. 向きが 3 軸とも合ったら、limits.md §4 の手順で `duty_min` → 速度・加速度 → Duty 上限の順に上げる（試験ページの「動きの制限」か `POST /v1/limits`。サーバーを再起動せずに変えられる）。
10. 固定の Pose の代わりに手元のスティックで roll / pitch を動かして確かめるときは **<http://127.0.0.1:8892/joystick>**（DESIGN.md §7.5。h3game と同じ符号: 左右 = roll、前後 = pitch、奥へ倒す = マイナス。yaw は 0。gain はスライダー、`max_gain` で頭打ち）。スティックを離せば 0、抜けば鮮度切れで減速停止する（中央へは戻らない）。

### 7.3 `/v1/center`

HOLDING（ARM 直後、START 前）で `curl -sX POST -H "$AUTH" $URL/v1/center` → `active_source = "center"`、3 軸が `count_at_virtual_512` へ `max_vel × center_speed_fraction`（0.25。`max_vel` 600 なら 150 count/s）で動き、到達（目標と**実位置**の両方が中立 ± `center_tolerance_count`）で CONTROLLED_STOP → HOLDING。実位置が `center_reach_timeout_ms`（5 s）以内に入らなければ FAULT になるので、その軸は自重に対して Duty が足りない（`duty_min` / `kp` を見直す）。1 軸ずつ動かした後に中立へ戻す手段として使う。

## 8. 3 軸

1. §7 の向きが 3 軸とも確定していること。`bringup_pose.py` で複合の Pose（`1 0 0`、`0 1 0`、`1 1 1`、gain 0.10）を 1 つずつ、ARM → START → 送信停止。3 軸が同時に動き始め・同時に止まること（`telemetry.frame_skew` が 5 未満。超えると `frame_skew` で FAULT）。
2. ゲームをつなぐ（`clients/h3game/README.md`「server モードでの運転の流れ」）: サーバーはそのまま、`H3_BACKEND=server python3 bridge.py`、Chrome で `http://localhost:8891`。接続しただけでは動かない。ゲームの Pose が届いていることを status の `session.fresh = true` で見てから `/v1/arm` → `/v1/start`。画面のスライダーの gain は `max_gain` で頭打ち。
3. 止める: `curl -sX POST $URL/v1/stop`（トークン不要）か画面を閉じる・隠す（`frame_seq` が止まり 250 ms で STOPPING）。どちらもその場で保持 → ブレーキ。
4. 停止距離・揺れ・保持の様子を見て、`timing` と `[limits]`（limits.md §4）を決める材料にする。ここで決めた値は模擬軸の結果に基づかない実測として §9 に残す。
5. 有人運転の可否はここでは判断しない（README「安全上の注意」）。

## 9. 記録

`docs/control-server/runs/bringup_<日付>/` に残す（§3 の `server.log` と同じ場所）:

| 何を | どう取る |
| --- | --- |
| 個体と系統の対応 | `ioreg` の MAC、画面の下 4 桁、status の `device_id`、bringup.toml に転記した値（§4.2） |
| 従来 env の校正値 | §1.2 の `AUTO saved motor= stroke= neutral= sign=` と最後の `pos=`（画面の写真でもよい） |
| 設定 | 使った `bringup.toml` のコピー（向きを入れ替えた後の最終形と、変更の理由） |
| サーバーのログ | `server.log`（`RUST_LOG=info`。`rig state` 行・`fault` 行・ACK の拒否） |
| イベント | 各段階の終わり、特に FAULT の直後に `curl -s "$URL/v1/events?n=512" > events_<段階>.json`。リングは 512 件で上書きされ、サーバー再起動で消えるので **再起動の前に取る** |
| 状態 | 節目（接続直後・基準後・HOLDING・ACTIVE・停止後・FAULT）で `curl -s $URL/v1/status > status_<節目>.json` |
| 向きの結論 | 各軸: 仮想 0→1023 で上/下・右/左、`count_at_virtual_0 < 1023` か `>` か、確認に使った Pose と gain |
| 観察 | 停止距離（送信停止から静止までの count と秒）、HOLDING の `following_error_count` の幅、ブレーキ作動後の沈み、リレーの音、USB が応答しなくなった個体（§10） |
| 未解決 | 再現しなかった事象、推定のまま残した値 |

FAULT のたびに `fault_history`（status。最大 32 件）と `GET /v1/events` を取ってから `fault/reset` する。

## 10. トラブル表

status の `blockers` の code（`:` より前）、`fault.kind` / `code`、HTTP の応答ごと。

| 症状 | 意味 | 見るところ・対処 |
| --- | --- | --- |
| `not_connected: axisN` | その系統を名乗る個体が無い（未接続、HELLO 失敗、競合中）。**USB には見えるがシリアルに応答しない個体**が 2026-10-31 に 3 回（1C:DB 1 回、30:ED 2 回）あり、どれもボードのリセットで復帰（ポートを閉じたことが原因ではない） | `backend.ports` にその個体のポートがあるか（§3）。画面の `#n` がその系統か。競合なら `axis_conflict` も出る。ポートを開いて 2 s（`timing.hello_timeout_ms`）以内に HELLO が無ければいったん閉じ、3 s 後に開き直して HELLO_REQ をやり直す（`unknown_device` のときだけ開き直さない）。開くたびにリセットされて 2 s 以内に起動しない個体なら、この繰り返しから抜けないので `timing.hello_timeout_ms` を延ばす（`server.log` の `port opened` が 3 s ごとに続くのが目印。USB の挿し直しでは直らない）。従来 env のままの個体はテキストを返すので HELLO にならない（`rx_error` 扱い）→ §1.3 |
| `unknown_device: device <id> on port <port>` | `selected_axis = 0`（未選択）で、設定の `device_id` にも無い個体。サーバーはそのポートを閉じ、装置が消えるまで開き直さない | v1 ファームは 0 を名乗らない（未設定なら `#1`。§2）ので、CoreS3 以外の装置（別の ESP32-S3）のポートを `ports` に書いていないかを疑う。`ports` を直してサーバー再起動 |
| `axis_conflict: axisN claimed by ...`、`axis_conflict: [N]`、arm が 409 `axis_conflict` | 同じ系統を 2 台が選んでいる。その軸は primary 無し（OFFLINE）、CONFIG も送らない | どちらかの画面で別の系統をタップ（DISARMED のときだけ）。解消後、系統を変えた個体に `reference_stale` |
| `axis_device_mismatch: axisN device <id>, config expects <expected>` | 設定の `device_id` と違う個体がその系統を選んだ（または、その個体は別の軸として設定されている） | 個体を入れ替えたのか、系統の選択を間違えたのかを画面と `ioreg` で確認。意図した入れ替えなら bringup.toml の `device_id` を直してサーバー再起動 |
| `reference_stale: axisN re-reference required after axis change` | 同じ boot のまま系統を変えた個体。古い基準で動かさないための保留 | `/v1/reference`（§5.1）。USB の挿し直し（boot 変更）でも消える |
| `not_referenced: axisN POSITION_REFERENCED not set` | この boot で SET_REFERENCE 未了（起動直後、リセット後、ENCODER_FAULT の解除後） | §5.1。機構が動いた疑いがあれば §5.4 |
| `not_configured: axisN CONFIG not accepted yet` / `CONFIG rejected (...)` | CONFIG の ACK が無い、または拒否 | `INVALID_PARAM` なら bringup.toml の値（6 項目 > 0、soft_min < soft_max）。`BAD_STATE` なら MCU が DISARMED でない（下の `mcu_not_disarmed`）。ACK が来ないなら USB の往復に問題（`rx_error_count`、`status_stale`） |
| `power_role_mismatch: axisN reports POWER_CONTROLLER=..., config expects ...` | CONFIG の `role_flags` bit0 と STATUS の `POWER_CONTROLLER` が食い違う | `power_controller_axis = 1` で、軸 1 だけ `true` になるはず。違えばファームの `role_flags` → STATUS フラグの経路の不具合（PC 側では直せない）。主電源を 2 軸が操作しないための検査なので、無視して進まない |
| `mcu_not_disarmed: axisN state <STATE>` | MCU が DISARMED 以外（FAULT・HOLDING など）で止まっている | `FAULT` なら `fault_code` を見て `/v1/fault/reset`。画面の非常停止がラッチ中なら先に画面で解除。HOLDING 等で取り残されていれば `/v1/disarm` |
| `fault_reset_rejected: axisN <reason>` | 直前の `fault/reset`（HTTP は 200）を MCU が ACK で拒否した。`FaultCausePresent` = 原因が残っている（v1 ファームでは画面の非常停止のラッチだけ）、`BadState` = その軸は FAULT でない。`rig_state` は `FAULT_LATCHED` のまま、`GET /v1/events` に `ack_rejected` | 原因（画面の非常停止ラッチ）を除いて `fault/reset` をもう一度 |
| `position_outside_soft_limits: axisN actual X not in [..]` | 実位置がソフトリミット外。MCU も ARM_PREPARE を `BAD_STATE` で拒否 | **`referenced = false` のとき**（起動直後・リセット後）: `actual ≈ 0` は起動時からの相対値で、正常。§5.1 の基準で消える。**`referenced = true` のとき**: 基準の取り違え（P 端で 0 を入れた等。§5.1）か、機構が端まで動いた。v1 では動かせないので §5.4 |
| `status_stale: axisN` / `no_status: axisN` | STATUS が 200 ms 以上来ない / 1 回も来ない | USB の応答停止（個体 30:ED で記録あり。画面は生きている）→ その個体をリセット（USB 挿し直し）→ 基準からやり直し。ハブの電源。他のプログラムがポートを掴んでいないか（`pio device monitor` を閉じる） |
| `calibration_invalid: axisN calibration_valid = false` | 意図した阻害（§5.5 まで） | §5.5 の 4 点を確認してから `true` |
| `fault.kind = "arm_timeout"`（ArmTimeout） | ARM 開始から 5 s 以内に全軸 HOLDING にならなかった | `GET /v1/events` で、どの軸がどこで止まったか（`ack_rejected`、`ack_timeout`、`ARMING` のまま）。ARM_PREPARE の `REJECTED` 理由: `NOT_CONFIGURED` / `NOT_REFERENCED` / `EPOCH_MISMATCH` / `BAD_STATE`（範囲外）。ACK はあるが HOLDING にならない → ファームの ARMING シーケンス（電源 300 ms + ブレーキ 300 ms で 1 s 以内のはず） |
| `fault_code = COMMAND_TIMEOUT` | 100 ms 以上 SET_POSITION が届かなかった。動いていれば減速してから FAULT | PC 側の停止（スリープ、サーバー停止、USB 抜け）。`telemetry.tick_dt_ms.max` が 100 を超えていないか。ノート PC のスリープ設定（§0）。系統を駆動中に切り替えた個体もこれで止まる（protocol.md §1.1） |
| `fault_code = FOLLOWING_ERROR` が **動き始めた直後・向きが逆** | 位置ループの向きが逆で、目標から離れる方向に駆動している（誤差が急増して 300 を超える） | **24V を切る。** NVS の `sign` と実際の向きが食い違っている（`sign` は従来 env の CAL が判定して保存。校正したスロットと `motor=` が違うと起きうる、§1.2 の `OK motor=`）。従来 env で `?` の `sign=` を確かめ、必要なら `UNCAL` → `CAL`。bringup.toml の `count_at_virtual_*` の入れ替えでは直らない（それは PC 側の仮想→count の向きで、MCU の駆動の向きではない） |
| `fault_code = FOLLOWING_ERROR` が **移動中・徐々に** | 位置ループが目標に追いつけない（速度フィードフォワード 100 count/s あたり 1 %、KP 2 %/100 count、Duty 上限はすべて仮値。`[limits]`） | `following_error_limit_count` を上げる前に `max_vel_count_s` を下げる（`POST /v1/limits` で再起動なしに変えられる）。重力軸（#1 / #2）の上昇で出やすい（2026-10-31 の実測: #2 の持ち上げは 8 % 台で 200 count/s 未満に落ちる。9 % 以上なら沈む側と同程度）。恒常的に出るなら `[limits].ff_count_s_per_percent` を 100 → 80 に（1 % あたりの速度を小さく見積もる＝同じ速度に多めの Duty）。limits.md §5 |
| `fault_code = OVERSPEED` | 100 ms 窓の速度が 2000 count/s 超（Duty 上限 20 % ≈ 1900 count/s、15 % で 1300〜1460 の実測） | 重力で落ちた（ブレーキ解除で保持できていない）か、カウントのノイズ。24V を切って機構を支持、§5.4 |
| `fault_code = SOFT_LIMIT` | 位置制御中に実位置がソフトリミットを外れた（ARMING 中はブレーキを解除せず FAULT） | 基準の誤り（§5.1）か、追従できずに流れた。FAULT_RESET は範囲外でも通るが、ARM は `position_outside_soft_limits` で止まる → §5.4 |
| `fault_code = ENCODER_FAULT` | ブレーキ解除指令中に目標（軌道）が動いている間、Duty ≥ 10 % が 200 ms 続いてカウントが変わらない（目標が止まっているときの静止・DISARMING 中・下限 7 % の押し当てでは判定しない） | 24V が来ていない（§6.3 では期待どおり）、エンコーダのケーブル・GND 共通、レベル変換。`fault/reset` は通る（v1 ファームはこれを「残る原因」にしない）が、解除で `POSITION_REFERENCED` が消えるので §5.1 をやり直す |
| `fault_code = LOCAL_ESTOP`（12） | CoreS3 画面の非常停止ボタン（protocol.md §3.2）。ラッチ中に `fault/reset` を送ると HTTP は 200 だが MCU が `FAULT_CAUSE_PRESENT` で拒否し、`rig_state` は `FAULT_LATCHED` のまま、`blockers` に `fault_reset_rejected: axisN FaultCausePresent` | 画面のボタンで「解除」→ `/v1/fault/reset`。`UNKNOWN` と表示されるなら PC 側が 12 を知らない古いバイナリ（`cargo build` し直す） |
| `fault_code = HOST_FAULT` | 他の軸の FAULT を受けて PC が止めた | `fault` の `axis` / `code` が原因の軸。`events` で最初の `fault` を見る |
| `fault.kind = "axis_disconnected"` / `"axis_rebooted"` / `"axis_changed"` | 駆動中に軸が切れた / `boot_id` が変わった / 担当個体の系統が変わった | USB・給電。`axis_changed` は駆動中に画面をタップした（DISARMED 以外では MCU が無視するはずなので、起きたらファームの不具合）。再基準化から |
| `fault.kind = "start_timeout"`（`axis` 付き） | START が確認できなかった。detail が `ack_timeout: START on axisN not ACKed … (MCU reports ACTIVE)` なら、MCU は START を適用したのに ACK が PC に届かなかった（USB の取りこぼし。2026-11-02 03:48 の形）。detail がその軸の状態（`not ACTIVE within 5000 ms (reports HOLDING)`）なら ACK は来たが ACTIVE にならなかった | `GET /v1/events` の `ack_timeout` と、その軸の `rx_error_count` / `decode_errors`。再現するなら USB ハブ・ケーブル（Issue #11 と同じ経路）。FAULT_RESET → ARM → START でやり直す |
| `fault.kind = "axis_state_mismatch"` | ACK の後の STATUS が遷移前の状態だった（例: ARM_PREPARE の ACK の後に DISARMED） | ファームの ACK → STATUS の順序の不具合（protocol.md §1）。PC 側では直せない。`events` を保存して報告 |
| `fault.kind = "frame_skew"` | 3 軸の `last_applied_frame` の差が 5 を超えた | 1 軸だけ SET_POSITION を取りこぼしている。その軸の `stale_drop_count` / `rx_error_count`、USB ハブ |
| HTTP 401 / 403 | トークン無し・不一致（`require_admin_token = true` のときだけ） / Origin 違反 | `H3CS_ADMIN_TOKEN` が起動時と同じか。curl は Origin を付けないので 403 はブラウザー側 |
| 3 台とも USB に見えるのに HELLO にも書き込み（esptool）にも答えない | USB-JTAG が固まっている。Base15 の DC を切っても USB ハブの VBUS で CoreS3 は動き続け、再起動しない（`ioreg -p IOUSB -l -w0 \| grep sessionID` が変わらない） | **ハブの上流 USB ケーブルを抜き差し**（または各 USB を抜き差し）して再列挙させる。2026-11-01 に 2 台同時に起きた（[runs/hw_20261101.md](runs/hw_20261101.md)） |
| HTTP 409 `no_fresh_pose` | START に新鮮な Pose が無い | `bringup_pose.py` が動いているか、`session.fresh`。`frame_seq` の無い Pose は拒否（`allow_missing_frame_seq = false`） |
| HTTP 409 `pose_not_near_hold` | START 時の Pose が作る目標が、保持位置から `[game].start_max_offset_count`（既定 300 count）を超えて離れた軸がある | `blockers` に軸ごとの距離。送っている Pose を中立（0, 0, 0）か gain 0 に戻してから START（忘れたタブが極端な姿勢を流していないか） |
| HTTP 409 `not_holding` / `not_disarmed`（`blockers` に `rig_state: <状態>`） | その状態では受けない命令（例: READY_TO_ARM で start、ACTIVE で center、HOLDING 中に reference） | 順序: arm → start。center は HOLDING で、reference は全軸 DISARMED で |
| HTTP 429 | 管理 API の rate limit（20 回/秒） | status のポーリングは `GET /v1/status`（認証なし・対象外）で |
| HTTP 503 `control_loop_unavailable` | 制御ループが 1 s 以内に答えない | arm / start / center / reset / reference は **実行されない**。stop / disarm / fault は **実行されたかもしれない**。status で確かめる。`GET /v1/status` 自体が 503 なら制御ループが落ちている → `GET /v1/events` は読めるので保存し、**24V を切ってから** サーバーを再起動（MCU は lease 切れで既に止まっているはず） |
| 画面は動くが USB が応答しない | 個体 30:ED で 2 回記録（ホストがポートを閉じた直後）。原因不明 | その個体をリセット。再発するなら別の個体に替え、§9 に記録 |
| ARM で HOLDING になった瞬間に沈む / 流れる | ブレーキ解除から位置制御への引き継ぎが効いていない | **24V を切る。** 位置ループの向き（上の FOLLOWING_ERROR 逆向き）、`V1_DUTY_MIN` 7 % で保持できるか、ブレーキの `BRAKE_RELEASE_WAIT_MS` |
