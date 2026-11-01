# moter-controler に USB プロトコル v1 を載せる（設計）

目的: 3 台の CoreS3 を `h3-control-server` に USB でつなぎ、3 系統を実機で動かせるようにする。仕様は [docs/control-server/protocol.md](../../control-server/protocol.md)（正）と、PC 側が依存する挙動の参照モデル `control-server/src/sim_axis.rs`。この文書は「どう載せるか」だけを決める。

## 0. 守ること

- **書き込み（`pio run -t upload`）と USB ポートのオープン（`pio device monitor`、pyserial）はしない。** 実機は別で使用中。ビルドとホストテストまで。
- 既存の env（`m5stack-cores3`、`-usbpower`、`-pmiclog*`）の挙動を変えない。v1 は新しい env でだけ有効。
- protocol.md と `sim_axis.rs` に無い振る舞いを勝手に足さない。足す必要があれば protocol.md を先に直す（この設計では `LOCAL_ESTOP` を 1 つ足す）。
- ピン・リレー論理・PWM・PCNT・NVS の既存モジュールはそのまま使う（`config.h` の定数を変えない）。

## 1. ビルド構成

`platformio.ini` に追加:

```ini
[env:m5stack-cores3-v1]           ; Base15 の DC 給電 + USB プロトコル v1
extends = env:m5stack-cores3
build_flags = -DCORE_DEBUG_LEVEL=0 -DMC_USB_POWER=0 -DMC_PROTO_V1=1

[env:m5stack-cores3-v1-usbpower]  ; USB のみ給電 + v1（机上確認用）
extends = env:m5stack-cores3
build_flags = -DCORE_DEBUG_LEVEL=0 -DMC_USB_POWER=1 -DMC_PROTO_V1=1
```

`MC_PROTO_V1=1` のとき USB CDC（`Serial`）は **v1 のバイナリ専用**になる。テキストコマンド（`R` `S` `HOME` …）とテキストの `STAT` / `ENC` / `AUTO` 行は出さない（COBS の区切り 0x00 と混ざるため）。デバッグ出力が要るときは画面に出す。v1 の env は `CORE_DEBUG_LEVEL=0`（既存 env の 1 ではない）: Arduino core の `log_e()` はこの値でコンパイル時に選ばれ、`esp_log_level_set` では止まらず `ets_printf` 経由で USB CDC（IDF の第 2 コンソール）に文字が出るため。`esp_log_level_set("*", ESP_LOG_NONE)` は `M5.begin()` より前に呼ぶ。

## 2. ファイル構成

| ファイル | 内容 | ホストテスト |
| --- | --- | --- |
| `include/v1_codec.h` | 純粋（Arduino 非依存、ヘッダのみ）。COBS、CRC-16/CCITT-FALSE、フレーム組み立て・分解、protocol.md §4 の全メッセージの構造体と LE エンコード・デコード。復号は長さ・版・種別・CRC を検査し、不正は `rx_error` として数えるだけ | protocol.md §7 の全テストベクタ（frame / wire の hex をそのまま埋め込む）で encode / decode の両方向 |
| `include/v1_axis.h` | 純粋（ヘッダのみ）。1 軸の状態機械 `V1Axis`。入力は時刻（us）と生の PCNT カウント、出力は電源 ON 指令・ブレーキ解除指令・符号付き Duty（%）と送信メッセージ列。`sim_axis.rs` の `SimMcu` と同じ入出力で同じ応答を返すことを目標にする | §3 の各規則。`sim_axis.rs` のテスト（`control-server/tests/sim_axis.rs`）の主要ケースを C++ に写す |
| `src/v1_link.cpp` / `include/v1_link.h` | Arduino 側。`Serial` のバイト受信 → COBS 区切りで 1 フレームずつ FreeRTOS キュー（受信）へ。送信キューからフレームを取り出して `Serial.write`。`device_id`（eFuse MAC）と `boot_id`（`esp_random()`）の生成 | なし |
| `src/control_task.cpp` | `MC_PROTO_V1` のとき `V1Axis` を 5 ms タスク内で回す（§4） | なし |
| `src/main.cpp` | `MC_PROTO_V1` のとき: テキストコマンド無効、画面は v1 表示（§5）、系統選択と非常停止だけ操作できる | なし |
| `tests/test_host.cpp` | 既存に `v1_codec` / `v1_axis` のテストを追加（`tests/run_tests.sh` は変えない） | |

`v1_codec.h` と `v1_axis.h` は `#include <stdint.h>` / `<string.h>` だけに依存し、`millis()` や `Serial` を使わない（既存の `auto_move.h` と同じ流儀）。

## 3. V1Axis の規則（protocol.md の実装箇所の対応）

- 状態: BOOT → DISARMED → ARMING → HOLDING ⇄ ACTIVE、STOPPING、DISARMING、FAULT（§3.1、§5）。起動直後の BOOT は `begin()` で即 DISARMED にしてよい（HELLO はどの状態でも返す）。
- 管理命令の共通規則（§4）: `request_id = 0` は拒否、直近 16 件の受理済み命令（種別・request_id・payload が同一）は DUPLICATE で再実行しない、`fault_code ≠ NONE` なら FAULT_RESET と HOST_FAULT 以外は FAULT_ACTIVE。HOST_FAULT は毎回実行。
- **ACK は状態遷移を確定させてから送る。ACK の後に出す STATUS は遷移後の状態**（§1）。`V1Axis` は 1 つの出力列に ACK → STATUS の順で積むので、`v1_link` が順序を保てば満たされる。
- CONFIG: DISARMED のときだけ。§4 の表の検査だけ（6 項目 > 0、`soft_min < soft_max`）。epoch が変われば `last_rx_seq = 0`。`role_flags` bit0 → STATUS `POWER_CONTROLLER`。
- SET_REFERENCE: DISARMED のときだけ。`reference_offset = raw − reference_count`。以後 `actual_count = raw − reference_offset`。**生のカウントに系統の符号を掛けない**（§1.1。符号は PC 側の校正）。
- ARM_PREPARE: DISARMED・CONFIGURED・REFERENCED・epoch 一致・実位置がソフトリミット内（拒否理由の順序は §4 の表）。加えて **ローカル操作中（§5: 系統選択画面の非常停止ラッチ中）は BAD_STATE**。ARMING の順序は §5: 主電源 ON 指令（電源担当軸のみ）→ `POWER_SETTLE_MS` → 実位置がソフトリミット外なら FAULT(SOFT_LIMIT)（ブレーキは解除しない）→ 現在位置を保持目標にして位置制御開始 → ブレーキ解除指令 → `BRAKE_RELEASE_WAIT_MS` → HOLDING。
- SET_POSITION: ACK なし。`seq <= last_rx_seq`（同じ epoch）は `stale_drop_count`、epoch 違い・`apply_at_us ≠ 0` は `rx_error_count`。有効なら lease と `valid_for_ms` を更新。HOLDING / STOPPING では目標を使わず期限だけ。ACTIVE では目標をソフトリミットでクリップして軌道の目標にする。
- START: HOLDING かつ期限内の SET_POSITION があるときだけ。CONTROLLED_STOP: ACTIVE なら STOPPING、HOLDING などでは何もせず ACCEPTED。DISARM: 動いていれば減速 → DISARMING → DISARMED、DISARMED / DISARMING では ACCEPTED（no-op）。
- 軌道: 速度・加速度制限つきの setpoint（`sim_axis.rs` の `update_setpoint` と同じ考え方。目標へ向かって `max_acc` で加減速、`max_vel` を超えない、到達で速度 0）。STATUS の `target_count` はこの setpoint、`following_error_count = actual − target`。
- 位置制御（Duty）: `err = target_setpoint − actual`、`vel` = 軌道の目標速度。指令（カウント増が正の %）= `vel / V1_FF_COUNT_S_PER_PERCENT`（フィードフォワード）+ `err × V1_KP_PERCENT_PER_100 / 100`（比例）。`vel = 0` のときだけ不感帯を使い、`|err| ≤ V1_DEADBAND` で 0 に入り、`|err| > V1_DEADBAND_OUT` まで再駆動しない（ヒステリシス。2026-10-31 の検証で #3 が到達帯の縁で 36 秒往復したため）。0 でなければ `V1_DUTY_MIN`〜`V1_DUTY_MAX` に収める。向きは NVS プロファイルの `sign`（`+1` なら速度 − でカウント増）から決める: `duty_signed = −sign × 指令`。定数は `config.h`（初期値: KP 2 %/100 count、FF 100 count/s あたり 1 %、DEADBAND 10 / OUT 20、MIN 7、MAX 20。`moter-controler/docs/multi_axis_20261031.md` の実測「1 % ≈ 100〜130 count/s、始動 7〜9 %、15 % ≈ 1300〜1460 count/s」から）。実機で詰めるまでの仮値と明記。純粋関数 `v1_duty_command` に切り出してホストテストする。
- 異常検出（§3.2）: COMMAND_TIMEOUT（動いていれば減速してから FAULT、止まっていれば即）、FOLLOWING_ERROR、OVERSPEED（100 ms 窓の速度）、SOFT_LIMIT、STOP_TIMEOUT、ENCODER_FAULT（**ブレーキ解除指令中に軌道が動いている（setpoint の速度 ≠ 0）間だけ**、`|duty| ≥ V1_DUTY_MIN` が `STALL_MS` 続いてカウントが変わらない。`manual_guard.h` の無パルス判定と同じ窓。ブレーキ作動指令中（DISARMING）はカウントが変わらないのが正常で、目標が止まっているときの静止（荷重とのつり合いで最小 Duty では動かない）はエンコーダの死と区別できないので、どちらも見ない。目標が止まっているときの誤差の上限は FOLLOWING_ERROR が守る。**代償**: エンコーダが死んだまま目標が止まっている間（HOLDING、到達後の ACTIVE）は検出できず、目標が動き出してから STALL_MS で出る。旧規則もこの状況は |誤差| > V1_DEADBAND のときしか捉えられなかった）、CONTROL_OVERRUN（tick 間隔が `V1_OVERRUN_MS = 15` を超えた）、HOST_FAULT、**LOCAL_ESTOP = 12（新設。画面の非常停止ボタン）**。FAULT は PWM 0・ブレーキ作動・主電源 OFF 指令。FAULT_EVENT は FAULT へ入るたびに 1 回。
- FAULT_RESET: FAULT 以外は BAD_STATE。原因が残っていれば FAULT_CAUSE_PRESENT（v1 で「残る原因」= **画面の非常停止ボタンのラッチが解除されていない、だけ**。ENCODER_FAULT の無パルスは FAULT 中（ブレーキ作動・PWM 0）には観測できないので原因として残さず、解除は通す）。解除後 DISARMED。CONFIG と位置基準は残す（ENCODER_FAULT の解除では `POSITION_REFERENCED` を消す）。
- 系統選択（§1.1）: `select_axis(n)` は DISARMED のときだけ。`CONFIGURED` / `POSITION_REFERENCED` / `calibration_hash` を消し、AXIS_SELECT を 1 回、以後の HELLO / STATUS に新しい値。選択は NVS の `motor`（`profile_store_select_motor`）に保存。
- STATUS: 20 ms ごと、および状態・fault_code・flags が変わったとき即時。`boot_id`・`mcu_time_us`（`esp_timer_get_time()`）・`selected_axis`・`last_applied_frame/seq`・`last_rx_seq`・`current_ma = 0`（`CURRENT_VALID` は立てない）・`rx_error_count`・`stale_drop_count`。
- HELLO: `protocol_version = 1`、`selected_axis`、`device_id`（eFuse MAC の 48 bit を **表記順**（先頭オクテットが上位）で u64 に。上位 16 bit は 0x5131。例 `30:ED:A0:0B:20:40` → `0x513130EDA00B2040`。`ESP.getEfuseMac()` は先頭オクテットを最下位に置くので使わず、`esp_efuse_mac_get_default()` の 6 byte を詰める。画面の `dev XXXX`（下 4 桁）= MAC の末尾 2 オクテット）、`boot_id`、`fw_version = {0, 1, 0}`、`capabilities = POSITION_MODE(bit0)` のみ、`calibration_hash` = 受理済み CONFIG の値。
- 時間定数（`config.h` に追加、protocol.md §5・§6）: `POWER_SETTLE_MS = 300`、`BRAKE_RELEASE_WAIT_MS = 300`、`BRAKE_APPLY_WAIT_MS = 300`（合計が ARM の 5000 ms・停止の 3000 ms より十分短い）。`RELAY_SETTLE_MS`（100）はそのまま。
- 校正（protocol.md §1.2、§4 の CALIBRATE `0x0C` / CAL_RESULT `0x87`、§3.1 の `CALIBRATING = 8`）: 従来 env で実機確認済みの端当て状態機械 `AutoMachine`（`auto_move.h`、`AutoKind::Cal`: P 端当て → 退避 → N 端当て → 保存 → 中立。無パルス・遅すぎ・移動量・時間の守り付き）を `V1Axis` に内蔵のまま回す。`AutoInputs` は `V1Axis` の時刻（ms）・生カウント・`relay_live = true`（settle 済み。電源担当でない軸も同じ時間を待ったものとみなす）・`brake_released = brake_release_cmd` から作り、`duty_signed`（MD13S の符号付き %）を**そのまま** `duty_percent` に出す（従来の制御タスクが `motor_output_write(machine.duty_signed)` と書くのと同じ。`sign_` は掛けない）。
  - 受理: CALIBRATE `mode = 1` は DISARMED のときだけ（CONFIG・基準は不要。それ以外 `BAD_STATE`。`mode` が 0・1 以外は `INVALID_PARAM`。`request_id = 0` / DUPLICATE / `FAULT_ACTIVE` の共通規則はそのまま）。受理で `CALIBRATING`、`POSITION_REFERENCED` を**開始時に消す**（端当てで原点を取り直すため。成功したときだけ立て直す）。`mode = 0` は CALIBRATING（POWER_HOLD を含む）なら中止、それ以外は何もせず ACCEPTED。payload byte 5 の `flags`（`v1::cal_flag`）: bit0 = `KEEP_POWER` は **電源担当軸（受理時点の CONFIG の `role_flags` bit0）だけが解釈**し（`cal_keep_power_`）、他の軸と bit1〜7（予約）は無視する（拒まない）。
  - 手順（`CalPhase`）: 主電源 ON 指令（`powered_state` に CALIBRATING を含める。電源担当軸だけ G2 が ON）→ `CAL_POWER_SETTLE_MS`（5000。2026-10-31 に電源投入 4 s 後は動かず 12 s 後に動いた記録があるため ARMING の 300 より長い。仮値）→ ブレーキ解除指令 → `BRAKE_RELEASE_WAIT_MS` → `AutoMachine::request(Cal)` → tick ごとに `AutoMachine::tick` → Done / Failed / Cancelled で終了手順（ブレーキ給電 OFF → `BRAKE_APPLY_WAIT_MS` → PWM 0 のまま → CAL_RESULT を 1 回 → `DISARMED`（ここで主電源 OFF 指令）→ 即時 STATUS）。終了手順の間も `mcu_state = CALIBRATING`。AutoMachine は finish で `duty_signed = 0` にするので、ブレーキ作動待ちの間も PWM 0（校正には位置ループが無いので「保持」はしない）。
  - 成功: `reference_offset = AutoMachine.origin_raw`（P 端 = 0）、`POSITION_REFERENCED` を立て、`sign_` を校正で決まった符号に更新、`MotorProfile{motor = selected_axis, stroke, neutral, sign, calibrated}` を `profile_dirty` に出す。NVS への保存は**制御タスクではなく loop()** が行う（`V1Snapshot.profile_dirty` → `profile_store_save` → `control_v1_clear_profile_dirty()`。Preferences の書き込みは数十 ms 止まることがあり、5 ms の制御タスクでは `CONTROL_OVERRUN` になる。従来 env も loop() 側で保存している）。制御タスクは `g_signs[motor]` だけ更新する（次の系統切り替えの `set_sign` 用）。失敗・中止: `POSITION_REFERENCED` は消えたまま、保存しない、FAULT にしない。中立への移動で失敗（step 3）したときは N 端まで測れた値を CAL_RESULT に載せるが、保存も基準の確立もしない（§1.2 の「成功したら」に従う。再校正が要る）。
  - CAL_RESULT: `n_end_count = stroke × sign`（= raw_N − raw_P）、`neutral_count = neutral × sign`（符号は掛けない座標。`actual_count` と同じ）、`sign` は系統プロファイルの符号（成功なら新しい値、それ以外は従来の値）、`step` は終了時の `CalStep`、`reason` は AutoMachine の理由（`ok` / `stall` / `slow` / `travel` / `timeout` / `stop`）を 8 byte に NUL 埋め。
  - 中止: CALIBRATE(0) と DISARM は同じ（ACCEPTED）。AutoMachine が動いていれば `stop` を渡して "stop" で止め（reason `stop`）、settle / ブレーキ解除待ちの間なら step 0 で直接終了手順へ。ACK の時点でブレーキ給電 OFF・PWM 0（ACK の後の STATUS がそれを示す）。終了手順の間の 2 回目は何もせず ACCEPTED。
  - FAULT（HOST_FAULT・画面の非常停止 `LOCAL_ESTOP`・`CONTROL_OVERRUN`）: `latch_fault` が出力を即落とす（終了手順は踏まない）。校正の終わりなので CAL_RESULT(`CANCELLED`、reason `estop` / `fault`）を **FAULT_EVENT より前に 1 回**送る（protocol.md の「校正が終わるたびに 1 回」の解釈。PC 側はどの状態で受けても最新の結果として扱う）。FAULT_RESET 後は DISARMED で基準なし。終了手順（ブレーキ作動待ち）の間に FAULT になったときは結果が確定済みなので、その結果（成功なら `OK`、基準・プロファイルも保つ）をそのまま送る。
  - 校正中の他の命令: ARM_PREPARE / START / CONFIG / SET_REFERENCE / FAULT_RESET → `BAD_STATE`、CONTROLLED_STOP → 何もせず ACCEPTED、SET_POSITION は捨てる（`stale_drop_count` / `rx_error_count` / `last_rx_seq` / lease を触らない）、`select_axis` は無視、HELLO は `mcu_state = 8`。lease・FOLLOWING_ERROR・OVERSPEED・SOFT_LIMIT・ENCODER_FAULT は見ない（AutoMachine の守りが代わりに失敗で止める）。`CONTROL_OVERRUN` は見る（CALIBRATING は parked ではない）。
  - STATUS: `mcu_state = 8`、`POWER_ON_CMD` / `BRAKE_RELEASE_CMD` は実際の指令、`pwm_permille` は `−sign × duty × 10`（P 端に当たって向きが分かった後は AutoMachine の `working_sign`、それまでは NVS の `sign`）、`fault_code = NONE`。
  - 画面: 状態 `CALIBRATING` と段階（P 端へ / N 端へ / 中立へ / 電源保持）、AutoMachine の理由（`retreat` など）、終わった後は最後の CAL_RESULT（結果 / 理由 / N 端 / 中立 / sign）と「保存待ち」。系統のタップは DISARMED のときだけ（校正中は拒む）。
  - 電源の保持（`KEEP_POWER`、protocol.md §1.2 最終項・§4 CALIBRATE。2026-11-01 の実機で、電源担当軸 #1 が先に終わって主電源を切り #2 / #3 が中立への移動中に無パルスで失敗したため）: 電源担当軸として `KEEP_POWER` を受理した校正は、**成功・失敗・中止を問わず**、終了手順（ブレーキ給電 OFF → `BRAKE_APPLY_WAIT_MS` → PWM 0）の後に CAL_RESULT を 1 回送った **あと主電源 OFF 指令を出さず DISARMED にも落ちず**、`CalPhase::PowerHold`（`mcu_state = CALIBRATING`、`POWER_ON_CMD` = 1、`BRAKE_RELEASE_CMD` = 0、PWM 0）に留まる。CAL_RESULT の `step` は校正が終わった段階（通常 3）のまま（4 は入れない）。成功時の基準・符号・`profile_dirty` は終了手順の前（`cal_finish`）に確定するので PowerHold に依らない。PowerHold を出るのは CALIBRATE(0) / DISARM（ACCEPTED → 主電源 OFF → DISARMED。**CAL_RESULT は送らない**）、または PowerHold に入ってから `CAL_POWER_HOLD_TIMEOUT_MS`（120000、仮値。`V1Tuning::cal_power_hold_timeout_ms`）経過（同じく主電源 OFF → DISARMED）。HOST_FAULT / `LOCAL_ESTOP` / `CONTROL_OVERRUN` はいつも通り即 FAULT（主電源 OFF。結果は送信済みなので CAL_RESULT は増えず、FAULT_EVENT の `mcu_state_before` は CALIBRATING）。PowerHold 中の他の命令は校正中と同じ（ARM_PREPARE / START / CONFIG / SET_REFERENCE / FAULT_RESET / CALIBRATE(1) → `BAD_STATE`、SET_POSITION は捨てる、`select_axis` は無視、lease は見ない）。`cal_live_step()` は PowerHold で `cal_step::POWER_HOLD = 4` を返し、画面は「電源保持」。電源担当でない軸（CONFIG なし、または `role_flags` bit0 なし）は `KEEP_POWER` を無視して従来どおり DISARMED で終わる。PC 側（control-server/DESIGN.md §11）は対象に電源担当軸以外が含まれるときだけ `KEEP_POWER` を付け、他の軸の CAL_RESULT が揃ってから電源担当軸へ CALIBRATE(0) を送る。

protocol.md への追記（PC 側も同時に対応する）: §3.2 に `12 | LOCAL_ESTOP | MCU の画面の非常停止ボタン。解除はボタンのラッチ解除 → FAULT_RESET`。`control-server/src/types.rs` の `FaultCode` に `LocalEstop = 12` を足し、名前は `LOCAL_ESTOP`。

## 4. タスクとデータの流れ（Arduino 側）

```text
Core 0 loop():  Serial.read → COBS 区切り → rx キュー（フレーム）     tx キュー → Serial.write
                画面描画・タッチ（系統選択・非常停止）
Core 1 control task（5 ms、既存）:
                rx キューを全部取り出して V1Axis::handle_frame
                V1Axis::tick(now_us, raw_count) → power_on_cmd / brake_release_cmd / duty
                GPIO（主電源 G2・ブレーキ G13）と PWM を **制御タスクから直接**書く
                出力メッセージ → tx キュー
```

- キューは FreeRTOS の `xQueueCreate`（受信 16 フレーム・送信 32 フレーム、1 フレーム最大 260 byte）。送信キューが満杯なら STATUS は捨ててよい（次の周期で送る）が、ACK・FAULT_EVENT・AXIS_SELECT・HELLO は捨てない（満杯なら STATUS を 1 つ捨てて入れる、または `V1Axis` 側で保留）。
- `Serial.write` は `setTxTimeoutMs(20)` のまま。PC が読んでいないときに制御タスクが止まらないよう、書き込みは loop() 側だけで行う。
- 既存の `Inputs.fed_ms` の鮮度監視は残す。v1 では loop() が止まっても制御タスクは止まらないが、画面が止まっていることを操作者に知らせる手段として、`fed_ms` が古ければ画面側の問題として STATUS に出す必要はない（PC は STATUS が続く限り正常とみなす）。
- 主電源 G2 は CONFIG の `role_flags` bit0 が立っている軸だけ操作する。それ以外の軸は G2 に触らない（OFF のまま）。
- `MC_PROTO_V1` のとき、`control_task.cpp` の手動速度・自動移動（`AutoMachine`・`ManualGuard`）は使わない（コンパイルは残してよい）。モーター出力は `V1Axis` だけが書く。

## 5. 画面と操作（v1）

- 表示: 系統 `#n`、状態名（DISARMED / ARMING / …、FAULT は赤で異常コード名）、電源・ブレーキの指令、`actual_count` / `target_count` / 速度 / 追従誤差、リンク（最後に PC から何か受けてからの経過 ms、`rx_error_count`）、`device_id` の下 4 桁、`CONFIGURED` / `REFERENCED` の有無。
- 操作: **系統選択**（DISARMED のときだけ。タップで 1 → 2 → 3 → 1。選ぶと `V1Axis::select_axis`）、**非常停止**（いつでも。`V1Axis` に LOCAL_ESTOP をラッチさせる。ラッチ中はボタンが「解除」になり、解除すると FAULT_CAUSE が消える。その後 PC が FAULT_RESET を送る）。
- 手動の電源・ブレーキ・速度・自動移動のボタンは v1 では出さない。校正（端当て）は PC からの CALIBRATE で MCU が行う（§3 の校正の項。従来 env の `CAL` と同じ状態機械）。手順は `docs/control-server/hardware-bringup.md`。

## 6. 受け入れ（ビルドとホストテスト）

- `pio run -e m5stack-cores3-v1` と `-v1-usbpower` が通る。既存 4 env も通る（`pio run` 全 env）。
- `tests/run_tests.sh` が通る（既存 236 件 + v1 のテスト）。
- v1_codec: protocol.md §7 の全ベクタの encode / decode 一致、CRC 不一致・長さ違い・版違い・未知種別・256 byte 超の破棄。CALIBRATE / CAL_RESULT のベクタ（CRC は独立実装でも確認）。
- v1_axis（校正）: CALIBRATE の拒否（mode・request_id 0・DUPLICATE・BAD_STATE・FAULT_ACTIVE）/ 模擬機構（Duty に比例して動き、端で止まる）での成功（電源担当軸・非担当軸・符号が逆の機構）: 手順の時刻、CAL_RESULT の値と順序（DISARMED の STATUS より前に 1 回）、P 端 = 0 の基準、`profile_dirty` / 無パルスで退避後も止まる失敗（FAILED、基準なし、FAULT なし）/ 中立で失敗（値は載るが保存しない）/ CALIBRATE(0)・DISARM の中止（settle 中・動作中）/ 校正中の他の命令の拒否・SET_POSITION の破棄・系統切り替えの無視 / HOST_FAULT・LOCAL_ESTOP・CONTROL_OVERRUN で即 FAULT と CAL_RESULT(CANCELLED) / `KEEP_POWER`（電源担当軸: 成功・失敗・中止の後も CAL_RESULT 1 回 → CALIBRATING(POWER_HOLD、主電源 ON 指令・ブレーキ作動・PWM 0・段階 4）、CALIBRATE(0) / DISARM / `CAL_POWER_HOLD_TIMEOUT_MS` で主電源 OFF → DISARMED（2 回目の CAL_RESULT 無し）、HOST_FAULT / LOCAL_ESTOP で FAULT、PowerHold 中の他の命令の拒否・SET_POSITION の破棄・系統切り替えの無視、電源担当でない軸と予約ビットの無視）。
- v1_axis: HELLO はどの状態でも返す / CONFIG の拒否順 / ARM_PREPARE の拒否順と ARMING の順序（電源 → settle → ソフトリミット確認 → 位置制御 → ブレーキ → wait → HOLDING）/ SET_POSITION の stale・epoch・apply_at / START の条件 / lease 切れで減速 → FAULT / valid_for 切れで STOPPING → HOLDING / CONTROLLED_STOP・DISARM の no-op ACCEPTED / DUPLICATE / FAULT_ACTIVE / HOST_FAULT 常時実行 / FAULT_RESET と FAULT_CAUSE_PRESENT / ENCODER_FAULT は軌道が動いている間だけ（DISARM 後の残り誤差・目標停止中の静止・到達後の残り誤差では出ない）/ `device_id` の詰め順 / SET_REFERENCE と actual_count に符号を掛けない / select_axis は DISARMED だけ・CONFIGURED/REFERENCED/hash を消す・AXIS_SELECT 1 回・STATUS に載る / ACK の後の STATUS が遷移後 / FAULT_EVENT 1 回 / LOCAL_ESTOP。
- Duty の符号: `sign = +1` の系統で目標がカウント増の向きなら Duty は負。
