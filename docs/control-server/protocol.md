# PC ⇄ CoreS3 軸コントローラー USB プロトコル v1

状態: **v1 案（PC 側実装済み・CoreS3 側未実装）**。PC 側の参照実装は `control-server/src/protocol/`、MCU 側の参照モデルは `control-server/src/sim_axis.rs`（模擬軸）。CoreS3 ファームウェアはこの文書と末尾のテストベクタに合わせて実装する。

元になった設計: [H3 Control Server 初期アーキテクチャ・互換仕様案 v0.1](architecture.md) §9。

## 1. 基本方針

- 1 本の USB CDC に 1 軸。PC は軸ごとに独立したタスクで読み書きし、1 軸の書き込み待ちで他軸を待たせない。
- PC は **HELLO で device_id を確認するまで、その USB ポートに HELLO_REQ 以外を送らない**（同じ PC に別の ESP32-S3 が刺さっていることがある）。
- 位置制御・PWM・ブレーキシーケンス・期限切れ停止は MCU 側で完結する。PC は位置目標（count）だけを送る。
- 古い目標の再送はしない。SET_POSITION は ACK を返さず、定期 STATUS の `last_applied_*` で適用を確認する。管理命令は `request_id` 付きで ACK を返し、受理済み命令の再送（`msg_type`・`request_id`・payload がすべて同じフレーム）は再実行しない。HOST_FAULT だけは例外で、毎回実行する（§4「管理命令の共通規則」）。
- ACK の `ACCEPTED` は「受理」であって、動作の完了ではない（減速・ARMING の待ち時間などは後で進む。到達は STATUS の `mcu_state` で確認する）。
- **ただし、命令による状態遷移そのものは ACK を送る前に確定させる**（DISARMED→ARMING、HOLDING→ACTIVE、ACTIVE→STOPPING、→DISARMING、任意→FAULT、FAULT→DISARMED、`CONFIGURED`・`POSITION_REFERENCED` の変化）。ACK より後に送る STATUS はすべて遷移後の状態を示す。ACK より前に取ったスナップショットを ACK の後に送らない（USB 送信と制御が別コアでも、STATUS は送信時点の状態から作る）。PC はこれに依存している: ARM_PREPARE の ACK の後に DISARMED の STATUS が来ると軸の状態不一致として FAULT にし、HOST_FAULT の ACK の後の最初の STATUS を見て FAULT_RESET の送り先を決める。

## 2. フレーミング

```text
USB バイト列:  COBS(frame) 0x00  COBS(frame) 0x00 ...
frame:         header(4) payload(N) crc(2)
```

| オフセット | サイズ | 名前 | 内容 |
| ---: | ---: | --- | --- |
| 0 | 1 | `version` | プロトコル版。v1 は `0x01` |
| 1 | 1 | `msg_type` | §4 の種別 |
| 2 | 2 | `payload_len` | N（u16 LE） |
| 4 | N | `payload` | 種別ごとの固定長レイアウト（§4） |
| 4+N | 2 | `crc` | CRC-16/CCITT-FALSE（poly 0x1021、init 0xFFFF、反転なし、xorout 0x0000）を offset 0〜4+N−1 に対して計算。u16 LE |

- すべての整数は **リトルエンディアン**。符号付きは 2 の補数。浮動小数点は使わない。
- COBS は標準 COBS（Cheshire & Baker）。区切りは `0x00`。復号前のフレーム最大長 300 byte、復号後のフレーム最大長 **256 byte**。超えたら破棄して次の `0x00` まで読み飛ばす（再同期）。
- COBS の細目: 非ゼロ 254 byte の連続でデータが終わるときは末尾に空ブロック `0x01` を付けない（Wikipedia の例と同じ）。受信側は付いていても付いていなくても受け付ける。`0x00` が連続した空フレームは数えずに無視する。
- 受信側は、CRC 不一致・`version` 不一致・未知の `msg_type`・`payload_len` が種別の定義長と異なるフレームを **破棄** し、破棄数を数える（STATUS の `rx_error_count`）。破棄したフレームに応答しない。`rx_error_count` には、v1 で扱えない `apply_at_us ≠ 0` の SET_POSITION（§4）も数える。
- 将来の拡張でも既存種別の長さは変えない。必要なら新しい `msg_type` を足す。

## 3. 共通の値

### 3.1 MCU 状態 `mcu_state`（u8）

| 値 | 名前 | 意味 |
| ---: | --- | --- |
| 0 | `BOOT` | 起動直後。出力は安全側 |
| 1 | `DISARMED` | PWM 0、ブレーキ作動（解除給電なし）、主電源 OFF 指令 |
| 2 | `ARMING` | ARM_PREPARE 受理後。主電源投入待ち → 現在位置で位置制御確立 → ブレーキ解除待ち |
| 3 | `HOLDING` | ブレーキ解除・位置制御で現在目標を保持。SET_POSITION の `target_count` は無視し保持目標のみ（ただし期限監視のため SET_POSITION を受け続ける） |
| 4 | `ACTIVE` | SET_POSITION の `target_count` に追従 |
| 5 | `STOPPING` | CONTROLLED_STOP / 期限切れ等で、速度・加速度制限内で減速して停止中。停止後 `HOLDING`。DISARM による減速なら停止後 `DISARMING`、COMMAND_TIMEOUT による減速なら停止後 `FAULT`（この間 `fault_code ≠ NONE`、§3.2） |
| 6 | `DISARMING` | ブレーキ給電 OFF → `BRAKE_APPLY_WAIT_MS` → PWM 0 → 主電源 OFF（電源担当軸のみ）→ `DISARMED` |
| 7 | `FAULT` | 異常ラッチ。PWM 0・ブレーキ作動・主電源 OFF 指令。FAULT_RESET まで解除しない |

PWM = 0 を位置保持と見なさない。`HOLDING` は位置制御で支えている状態。

### 3.2 異常コード `fault_code`（u16）

| 値 | 名前 | 主な発生条件 |
| ---: | --- | --- |
| 0 | `NONE` | |
| 1 | `COMMAND_TIMEOUT` | ARMING/HOLDING/ACTIVE/STOPPING で、最後に有効な SET_POSITION（ARMING 中にまだ無ければ ARM_PREPARE 受理時刻）から `command_timeout_ms` を超えた。ただし STOPPING のうち DISARM による減速と、異常による減速は対象外（`stop_timeout_ms` で抑える）。動いていれば減速停止 → FAULT、止まっていれば即 FAULT |
| 2 | `FOLLOWING_ERROR` | `abs(actual_count − target_count) > following_error_limit_count`（`target_count` は STATUS と同じ、軌道上の目標）。位置制御中（ARMING のブレーキ解除以降・HOLDING・ACTIVE・STOPPING・DISARMING）だけ判定 |
| 3 | `OVERSPEED` | `abs(velocity) > overspeed_count_s`。位置制御中（同上）だけ判定（DISARMED・FAULT で手で動かしても異常にしない） |
| 4 | `SOFT_LIMIT` | 位置制御中に実位置が `[soft_min_count, soft_max_count]` を外れた。ARMING ではブレーキ解除の直前にも確認し、外れていればブレーキを解除せずに FAULT |
| 5 | `ENCODER_FAULT` | PCNT 異常・計数異常 |
| 6 | `CONTROL_OVERRUN` | 制御周期の逸脱 |
| 7 | `CONFIG_INVALID` | 予約（v1 では発生しない）。CONFIG 未受理の ARM_PREPARE は ACK `REJECTED/NOT_CONFIGURED` で拒否し、DISARMED のまま |
| 8 | `STOP_TIMEOUT` | 減速停止が `stop_timeout_ms` 内に完了しない。ただし異常による減速（COMMAND_TIMEOUT）が超えたときは STOP_TIMEOUT ではなく元の異常コードでラッチする。CONTROLLED_STOP の減速中に COMMAND_TIMEOUT になった場合、`stop_timeout_ms` は最初の STOPPING 開始から数える |
| 9 | `POWER_FAULT` | 電源系の異常（入力がある場合） |
| 10 | `HOST_FAULT` | PC から FAULT を受けた |
| 11 | `NOT_REFERENCED` | 予約（v1 では発生しない）。位置基準未確立の ARM_PREPARE は ACK `REJECTED/NOT_REFERENCED` で拒否し、DISARMED のまま |
| 0xFFFF | `UNKNOWN` | |

- STATUS の `fault_code` が `NONE` でないのは、`FAULT` のときと、COMMAND_TIMEOUT による減速中（`mcu_state = STOPPING`、ラッチ前）のとき。後者は停止後に FAULT をラッチし、そのとき FAULT_EVENT（`mcu_state_before = STOPPING`）を送る。減速中に HOST_FAULT を受けたら減速中の異常コードのまま即ラッチする。
- `fault_code ≠ NONE` の間、FAULT_RESET と HOST_FAULT 以外の管理命令は `REJECTED/FAULT_ACTIVE`。PC は `fault_code ≠ NONE` を異常として扱う（`mcu_state` が FAULT になるのを待たない）。

### 3.3 ACK 結果 `result`（u8）と拒否理由 `reason`（u16）

`result`: 0 = `ACCEPTED`、1 = `REJECTED`、2 = `DUPLICATE`（同じ内容の命令を受理済み。再実行しない。`reason` は常に `NONE`）。

MCU が記憶するのは受理（ACCEPTED）した命令だけ。拒否した命令には副作用が無いので記憶せず、再送されたら改めて評価する（そのときの状態で ACCEPTED になることもある）。

`reason`: 0 = なし、1 = `BAD_STATE`、2 = `EPOCH_MISMATCH`、3 = `NOT_CONFIGURED`、4 = `INVALID_PARAM`、5 = `NOT_REFERENCED`、6 = `FAULT_ACTIVE`、7 = `FAULT_CAUSE_PRESENT`。

### 3.4 STATUS `flags`（u16 ビット）

| bit | 名前 |
| ---: | --- |
| 0 | `POWER_ON_CMD`（主電源 ON 指令中。実電圧の確認ではない） |
| 1 | `BRAKE_RELEASE_CMD`（ブレーキ解除給電の指令中） |
| 2 | `POSITION_REFERENCED`（この boot で SET_REFERENCE 済み） |
| 3 | `CONFIGURED`（この boot で CONFIG 受理済み） |
| 4 | `POWER_CONTROLLER`（主電源担当軸として設定されている） |
| 5 | `CURRENT_VALID`（`current_ma` が校正済み実測値。0 のとき `current_ma` は無意味） |
| 6 | `CLOCK_SYNCED`（予約。apply_at 対応時） |

## 4. メッセージ

`request_id` は PC が付ける u32。0 は使わない。`session_epoch` は PC サーバーの起動・再 ARM ごとに変わる u32。MCU は CONFIG で受けた epoch と一致しない ARM_PREPARE / START / SET_POSITION / CONTROLLED_STOP を拒否（SET_POSITION は黙って破棄し `stale_drop_count` を増やす）。

MCU は PC サーバーの再起動を知らない（USB の再オープンでは何も消えない）。そのため PC は **`request_id` と `session_epoch` をサーバーの再起動をまたいで再利用しない**: どちらも起動時に乱数（または時刻）で初期値を決め、そこから増やす。固定値 1 から始めない。

- `request_id` を再利用すると、MCU に残っている受理済み命令と同じ内容の命令（例: DISARM）が DUPLICATE として捨てられる。
- `session_epoch` を再利用すると、同じ epoch の CONFIG では `last_rx_seq` が保持されるため（下記）、新しいサーバーの SET_POSITION が `seq <= last_rx_seq` ですべて破棄され、ARM のたびに COMMAND_TIMEOUT になる（MCU を再起動するまで駆動できない）。v1 の HELLO は保存中の epoch を返さないので、PC からは検出できない。

### PC → MCU

| type | 名前 | payload（オフセット: 型 名前） | 長さ | 応答 |
| --- | --- | --- | ---: | --- |
| `0x01` | HELLO_REQ | 0: u32 request_id | 4 | HELLO |
| `0x02` | CONFIG | 0: u32 request_id / 4: u32 session_epoch / 8: u32 calibration_hash / 12: i32 soft_min_count / 16: i32 soft_max_count / 20: u32 max_vel_count_s / 24: u32 max_acc_count_s2 / 28: u32 following_error_limit_count / 32: u32 overspeed_count_s / 36: u16 command_timeout_ms / 38: u16 stop_timeout_ms / 40: u8 role_flags（bit0 = 主電源担当） / 41: u8 reserved=0 / 42: u16 reserved=0 | 44 | ACK。DISARMED のときだけ受理（それ以外は `REJECTED/BAD_STATE`）。`soft_min_count < soft_max_count`、かつ `max_vel_count_s`・`max_acc_count_s2`・`following_error_limit_count`・`overspeed_count_s`・`command_timeout_ms`・`stop_timeout_ms` がすべて > 0 でなければ `REJECTED/INVALID_PARAM`。v1 ではこれ以外の検査で拒否しない（`role_flags` の bit1〜7 と reserved は無視。ハードウェア上限などで値を拒否しない。上限を超える値の扱いは PC の設定で決める）。受理すると全項目を置き換え、`CONFIGURED` を立て（`POSITION_REFERENCED` と状態は変えない）、HELLO の `calibration_hash` をこの値にする。epoch が変われば `last_rx_seq = 0`（下記）。`role_flags` bit0 は STATUS の `POWER_CONTROLLER` にそのまま出す（PC は設定と違えば ARM しない） |
| `0x03` | ARM_PREPARE | 0: u32 request_id / 4: u32 session_epoch | 8 | ACK、その後 STATUS で ARMING → HOLDING。DISARMED・CONFIGURED・REFERENCED・epoch 一致・実位置が `[soft_min_count, soft_max_count]` 内のときだけ受理。拒否理由はこの順に `BAD_STATE` / `NOT_CONFIGURED` / `NOT_REFERENCED` / `EPOCH_MISMATCH` / `BAD_STATE`（位置が範囲外） |
| `0x04` | START | 0: u32 request_id / 4: u32 session_epoch | 8 | ACK。HOLDING → ACTIVE。HOLDING・epoch 一致に加え、ARM_PREPARE 受理後に受けた有効期限内（`valid_for_ms`）の SET_POSITION があること。無ければ `REJECTED/BAD_STATE`（受理するとすぐ期限切れで STOPPING に落ちるため）。拒否理由はこの順に `BAD_STATE`（HOLDING でない）/ `EPOCH_MISMATCH` / `BAD_STATE`（期限内の SET_POSITION なし）。受理後も、START の後に受けた最初の SET_POSITION までは保持目標のまま（START 前に受けた目標へ飛ばない） |
| `0x05` | SET_POSITION | 0: u32 session_epoch / 4: u32 seq / 8: u32 group_frame_id / 12: i32 target_count / 16: u32 max_vel_count_s / 20: u32 max_acc_count_s2 / 24: u16 valid_for_ms / 26: u16 reserved=0 / 28: u64 apply_at_us（0 = 受信後すぐ） | 36 | なし（STATUS で確認） |
| `0x06` | CONTROLLED_STOP | 0: u32 request_id / 4: u32 session_epoch / 8: u8 reason / 9..11: reserved=0 | 12 | ACK。CONFIG 未受理・epoch 不一致は（どの状態でも）`REJECTED/EPOCH_MISMATCH`。ACTIVE → STOPPING → HOLDING。ACTIVE 以外（DISARMED・ARMING・HOLDING・STOPPING・DISARMING）では何もせず ACCEPTED（PC は HOLDING 中の停止操作でも送る。拒否すると PC は停止失敗として FAULT にする）。`reason` は記録のみ（1 = 管理操作、2 = ゲーム入力の途絶、3 = 中立到達。未知の値も拒否しない） |
| `0x07` | DISARM | 0: u32 request_id | 4 | ACK。管理命令の共通規則（`request_id` = 0・DUPLICATE・`FAULT_ACTIVE`）で弾かれるとき以外は常に ACCEPTED。ARMING / HOLDING → 直ちに DISARMING。ACTIVE → 減速停止 → DISARMING。STOPPING → 減速を続け、停止後 DISARMING。DISARMED / DISARMING → 何もせず ACCEPTED（PC は ARM 途中の停止でも、まだ DISARMED・ARMING の軸に送る。拒否すると PC は停止失敗として FAULT にする） |
| `0x08` | FAULT_RESET | 0: u32 request_id | 4 | ACK。FAULT 以外では `REJECTED/BAD_STATE`。原因（エンコーダ異常・電源異常の入力など、続いている異常）が残っていれば `REJECTED/FAULT_CAUSE_PRESENT`。解除後は DISARMED（自動再 ARM しない）。CONFIG と位置基準は残す（ENCODER_FAULT の解除では `POSITION_REFERENCED` を消す）。実位置がソフトリミット外のままでも解除する（下記） |
| `0x09` | CLOCK_SYNC | 0: u32 request_id / 4: u64 pc_time_us | 12 | CLOCK_SYNC_REPLY |
| `0x0A` | HOST_FAULT | 0: u32 request_id / 4: u16 fault_code / 6: u16 reserved=0 | 8 | ACK（常に ACCEPTED）。どの状態でも即 FAULT ラッチ（PWM 0・ブレーキ作動）。ラッチされるコードは `HOST_FAULT`（既にラッチ済み・減速中の異常コードがあればそれを保つ）。payload の `fault_code` は記録のみ。ACK の後に送る最初の STATUS から `mcu_state = FAULT`、`fault_code ≠ NONE`（PC はこれを確認してから FAULT_RESET の送り先を決める） |
| `0x0B` | SET_REFERENCE | 0: u32 request_id / 4: i32 reference_count | 8 | ACK。DISARMED のときだけ。現在の計数値を `reference_count` と定義し `POSITION_REFERENCED` を立てる |

管理命令の共通規則（ACK を返す命令。MCU はこの順に処理する）:

1. HOST_FAULT は以下をすべて飛ばし、毎回実行して ACCEPTED を返す（`request_id` = 0 でも、同じ内容の再送でも。ラッチは冪等なので再実行が安全側）。記憶もしない。
2. `request_id` = 0 → `REJECTED/INVALID_PARAM`。
3. 直近に受理した 16 件に `msg_type`・`request_id`・payload がすべて同じ命令があれば → `DUPLICATE`、再実行しない。`request_id` だけ同じで内容が違う命令は再送ではなく新しい命令として扱い、実行して記憶を置き換える。
4. `fault_code ≠ NONE` なら FAULT_RESET 以外は `REJECTED/FAULT_ACTIVE`。
5. 命令ごとの条件（上の表）。受理したら記憶する（古いものから捨てる）。

ソフトリミット外からの復帰: FAULT_RESET は実位置がソフトリミット外でも解除する（DISARMED は FAULT と同じくブレーキ作動・PWM 0 で、SET_REFERENCE と CONFIG は DISARMED でしか受けないため、ここで拒否すると MCU の再起動以外に抜け道がない）。代わりに ARM_PREPARE が範囲外を拒否するので、SET_REFERENCE・校正の見直し・手動移動で範囲内に戻すまで再 ARM できない。

SET_POSITION の規則:

- 判定順: CONFIG 未受理・epoch 不一致・`seq <= last_rx_seq` → 破棄して `stale_drop_count`++。次に `apply_at_us ≠ 0` → 破棄して `rx_error_count`++（古くて `apply_at_us ≠ 0` のフレームは `stale_drop_count` だけ数える）。残りが有効フレーム。
- 有効フレームは、どの状態でも（DISARMED・FAULT を含む）`last_rx_seq`・lease・`valid_for_ms` の起点を更新する。目標・フレーム上限・`last_applied_seq` / `last_applied_frame` を更新するのは ACTIVE のときだけ（それ以外では `last_applied_*` は変えない）。
- `seq` は軸ごとに単調増加。`seq <= last_rx_seq` は破棄（`stale_drop_count`++）。
- `seq` の単調性は epoch ごと。受理した CONFIG の epoch が保存中の epoch と違えば `last_rx_seq` を 0 に戻す（PC は新しい epoch で seq を 1 から始めてよい）。同じ epoch の再 CONFIG では保持する（遅れて届いた古いフレームを再生しない）。MCU の再起動でも 0 に戻る。
- `apply_at_us ≠ 0` は v1 では扱わない（`CLOCK_SYNCED` も `capabilities` bit1 も立たない）。そのフレームは破棄して `rx_error_count`++、期限（lease）も延長しない。
- `valid_for_ms`: MCU 受信時刻からこの時間を過ぎた目標は「期限切れ」。新しい SET_POSITION が来なければ `command_timeout_ms` で `COMMAND_TIMEOUT`。PC は通常 10ms 周期で送る。
- `target_count` は MCU 側でも `[soft_min_count, soft_max_count]` に制限する（PC の制限を信用しない最終検査）。
- `max_vel_count_s` / `max_acc_count_s2` はこのフレームの上限。CONFIG の上限より大きい値は CONFIG の値に制限。
- `HOLDING` / `STOPPING` では `target_count` を使わないが、期限監視（lease）としては有効。
- `group_frame_id` は 3 軸に同じ値を配る。MCU は適用した値を `last_applied_frame` に返す。同じ値だから同時に適用されたとは見なさない。

### MCU → PC

| type | 名前 | payload | 長さ |
| --- | --- | --- | ---: |
| `0x81` | HELLO | 0: u32 request_id / 4: u8 protocol_version / 5: u8 axis_id_hint（MCU 側 NVS の値、0 = 未設定。PC は採用しない） / 6: u8 mcu_state / 7: u8 reserved=0 / 8: u64 device_id（eFuse MAC から作る固有 ID） / 16: u32 boot_id（起動ごとに乱数） / 20: u8 fw_major / 21: u8 fw_minor / 22: u8 fw_patch / 23: u8 reserved=0 / 24: u32 capabilities（bit0 = POSITION モード、bit1 = apply_at、bit2 = 電流計測） / 28: u32 calibration_hash（受理済み CONFIG の値、未設定 0） | 32 |
| `0x82` | ACK | 0: u32 request_id / 4: u8 acked_type / 5: u8 result / 6: u16 reason | 8 |
| `0x83` | STATUS | 0: u32 boot_id / 4: u64 mcu_time_us / 12: u8 mcu_state / 13: u8 reserved=0 / 14: u16 fault_code / 16: u16 flags / 18: i16 pwm_permille（符号付き −1000〜1000） / 20: i32 actual_count / 24: i32 target_count（下記） / 28: i32 velocity_count_s / 32: i32 following_error_count（下記） / 36: u32 last_applied_frame / 40: u32 last_applied_seq / 44: u32 last_rx_seq / 48: i32 current_ma / 52: u16 rx_error_count / 54: u16 stale_drop_count | 56 |
| `0x84` | FAULT_EVENT | 0: u32 boot_id / 4: u64 mcu_time_us / 12: u16 fault_code / 14: u8 mcu_state_before / 15: u8 reserved=0 / 16: i32 actual_count | 20 |
| `0x85` | CLOCK_SYNC_REPLY | 0: u32 request_id / 4: u64 pc_time_us（エコー） / 12: u64 mcu_rx_us / 20: u64 mcu_tx_us | 28 |

- STATUS は MCU が自発的に 50Hz（20ms）で送る。状態遷移・FAULT 発生時は即時にも送ってよい。ACK との順序は §1（ACK より後の STATUS は遷移後の状態）。
- STATUS の `target_count` = 位置ループがいま追従している目標（速度・加速度制限を適用済みの軌道上の点）。SET_POSITION で受けた `target_count` そのものではない。HOLDING / STOPPING / DISARMING では保持・減速中の目標。PC は中立への移動（`/v1/center`）の完了を全軸の STATUS `target_count` が中立に達したことで判断し、その後に CONTROLLED_STOP を送る。受けた目標を返すと移動の途中で止められる。
- `following_error_count` = `actual_count − target_count`（符号付き。FOLLOWING_ERROR の判定と同じ値）。`pwm_permille` と `velocity_count_s` は正が count の増える向き。
- FAULT_EVENT は FAULT へ遷移するたびに 1 回送る（原因を問わない。HOST_FAULT・ARMING 中の SOFT_LIMIT も含む）。既に FAULT のときは送らない。`mcu_state_before` は遷移直前の状態。PC は FAULT_EVENT と STATUS のどちらでも異常をラッチする。
- `actual_count` / `target_count` は `SET_REFERENCE` 適用後の座標。基準未確立なら起動時 0 からの相対値で、`POSITION_REFERENCED` = 0。
- `boot_id` が変わったら PC はその軸の校正状態を無効化し、再 ARM を要求する。

## 5. 状態遷移（MCU）

```text
BOOT → DISARMED
DISARMED --ARM_PREPARE(CONFIGURED & REFERENCED & epoch一致 & 実位置がソフトリミット内)--> ARMING --(電源・位置制御・ブレーキ解除完了)--> HOLDING
HOLDING --START(期限内の SET_POSITION あり)--> ACTIVE
ACTIVE --CONTROLLED_STOP / 期限切れ(valid_for_ms)--> STOPPING --(停止確認)--> HOLDING
ARMING / HOLDING --DISARM--> DISARMING → DISARMED
ACTIVE / STOPPING --DISARM--> (減速停止) → DISARMING → DISARMED
任意 --異常 / HOST_FAULT / command_timeout--> FAULT --FAULT_RESET(原因なし)--> DISARMED
```

- 停止確認 = 軌道（STATUS の `target_count`）の速度が 0 になったこと。
- 上の図にない命令と状態の組み合わせ（HOLDING での CONTROLLED_STOP、DISARMED での DISARM など）は §4 の表のとおり何もせず ACCEPTED。
- USB 再接続・CDC の再オープンで自動 ARM・自動 START しない。NVS に ARM 状態を保存しない。
- DISARM による減速（ACTIVE / STOPPING からの DISARM）は lease の対象外。PC が SET_POSITION を止めても COMMAND_TIMEOUT にせず、停止後 DISARMING → DISARMED へ進む（`stop_timeout_ms` を超えたら STOP_TIMEOUT）。
- `valid_for_ms` の期限切れで ACTIVE → STOPPING → HOLDING に落ちた後は、遅れて届いた SET_POSITION では動き出さない。再開には START が要る。
- ARMING の順序: PWM 0 のまま主電源 ON 指令（電源担当軸のみ）→ `POWER_SETTLE_MS` → 実位置がソフトリミット外ならブレーキを解除せず FAULT(SOFT_LIMIT) → 現在位置を保持目標にして位置制御開始 → ブレーキ解除給電 → `BRAKE_RELEASE_WAIT_MS` → HOLDING。
- 主電源（G2）は **電源担当軸だけ** が操作する。それ以外の軸は CONFIG の `role_flags` bit0 = 0 で G2 を操作しない。

## 6. PC 側の期限・周期（既定値、`config/*.toml` で変更）

| 項目 | 既定 | 備考 |
| --- | ---: | --- |
| SET_POSITION 送信周期 | 10ms | プランナー 100Hz |
| `valid_for_ms` | 50 | |
| `command_timeout_ms` | 100 | MCU が自律停止に入るまで |
| STATUS 途絶で軸断とみなす | 200ms | PC 側 |
| ゲーム入力の鮮度期限 | 250ms | 超えたら協調減速 → 保持 |
| `stop_timeout_ms`（CONFIG） | 1500 | MCU 側の減速の上限 |
| ACK の応答期限 | 1000ms | PC 側（固定）。超えると PC は応答なしとして扱い、再送しない（HOST_FAULT と CONFIG だけ再送）。ARM・停止の途中なら FAULT になる。通常は 20ms 以内に返す |
| HELLO_REQ の再送 / 打ち切り | 200ms / 2000ms | どの状態（BOOT を含む）でも HELLO を返す |
| ARM 全体（CONFIG〜全軸 HOLDING）・START の確認 | 5000ms | `POWER_SETTLE_MS + BRAKE_RELEASE_WAIT_MS` はこれより十分短くする |
| 減速 → HOLDING、DISARM → DISARMED | 3000ms | 減速時間と `BRAKE_APPLY_WAIT_MS` の和はこれより短くする |
| 停止後の保持 → DISARM | 1000ms | PC 側。全軸 HOLDING を確認してからこの時間保持し、DISARM を送る |

いずれもベンチ用の候補値で、実機の停止距離・揺れ・PC 負荷を見て確定する（ACK の応答期限以外は `config/*.toml` の `timing` で変更）。

## 7. テストベクタ

`control-server/tests/protocol_vectors.rs` が下記と同じバイト列を検査する。CoreS3 側の単体テストでも同じ値を使う。

<!-- VECTORS:BEGIN -->
`frame` は COBS 前のフレーム（header + payload + crc）、`wire` は COBS 符号化して区切り `0x00` を付けた USB 上のバイト列。いずれも 16 進・空白区切り。「値」は各フィールドの値（JSON、フィールドはレイアウト順、数値は 10 進。`device_id` は 16 桁 hex、`fault_code` / `result` / `reason` は名前で表記）。

### 7.1 HELLO_REQ（`0x01`、PC → MCU）

値: `{"request_id":1}`

```text
frame (10 byte): 01 01 04 00 01 00 00 00 99 BE
wire  (12 byte): 04 01 01 04 02 01 01 01 03 99 BE 00
```

### 7.2 CONFIG（`0x02`、PC → MCU）

値: `{"request_id":2,"session_epoch":3,"calibration_hash":305419896,"soft_min_count":-2700,"soft_max_count":2700,"max_vel_count_s":4000,"max_acc_count_s2":20000,"following_error_limit_count":200,"overspeed_count_s":6000,"command_timeout_ms":100,"stop_timeout_ms":1500,"role_flags":1}`

```text
frame (50 byte): 01 02 2C 00 02 00 00 00 03 00 00 00 78 56 34 12 74 F5 FF FF 8C 0A 00 00 A0 0F 00 00 20 4E 00 00 C8 00 00 00 70 17 00 00 64 00 DC 05 01 00 00 00 D2 48
wire  (52 byte): 04 01 02 2C 02 02 01 01 02 03 01 01 0B 78 56 34 12 74 F5 FF FF 8C 0A 01 03 A0 0F 01 03 20 4E 01 02 C8 01 01 03 70 17 01 02 64 04 DC 05 01 01 01 03 D2 48 00
```

### 7.3 ARM_PREPARE（`0x03`、PC → MCU）

値: `{"request_id":4,"session_epoch":3}`

```text
frame (14 byte): 01 03 08 00 04 00 00 00 03 00 00 00 8C 21
wire  (16 byte): 04 01 03 08 02 04 01 01 02 03 01 01 03 8C 21 00
```

### 7.4 START（`0x04`、PC → MCU）

値: `{"request_id":5,"session_epoch":3}`

```text
frame (14 byte): 01 04 08 00 05 00 00 00 03 00 00 00 E2 4F
wire  (16 byte): 04 01 04 08 02 05 01 01 02 03 01 01 03 E2 4F 00
```

### 7.5 SET_POSITION（`0x05`、PC → MCU）

値: `{"session_epoch":3,"seq":1000,"group_frame_id":1234,"target_count":-1234,"max_vel_count_s":4000,"max_acc_count_s2":20000,"valid_for_ms":50,"apply_at_us":0}`

```text
frame (42 byte): 01 05 24 00 03 00 00 00 E8 03 00 00 D2 04 00 00 2E FB FF FF A0 0F 00 00 20 4E 00 00 32 00 00 00 00 00 00 00 00 00 00 00 1D 07
wire  (44 byte): 04 01 05 24 02 03 01 01 03 E8 03 01 03 D2 04 01 07 2E FB FF FF A0 0F 01 03 20 4E 01 02 32 01 01 01 01 01 01 01 01 01 01 03 1D 07 00
```

### 7.6 CONTROLLED_STOP（`0x06`、PC → MCU）

値: `{"request_id":6,"session_epoch":3,"reason":1}`

```text
frame (18 byte): 01 06 0C 00 06 00 00 00 03 00 00 00 01 00 00 00 74 B7
wire  (20 byte): 04 01 06 0C 02 06 01 01 02 03 01 01 02 01 01 01 03 74 B7 00
```

### 7.7 DISARM（`0x07`、PC → MCU）

値: `{"request_id":7}`

```text
frame (10 byte): 01 07 04 00 07 00 00 00 25 38
wire  (12 byte): 04 01 07 04 02 07 01 01 03 25 38 00
```

### 7.8 FAULT_RESET（`0x08`、PC → MCU）

値: `{"request_id":8}`

```text
frame (10 byte): 01 08 04 00 08 00 00 00 22 66
wire  (12 byte): 04 01 08 04 02 08 01 01 03 22 66 00
```

### 7.9 CLOCK_SYNC（`0x09`、PC → MCU）

値: `{"request_id":9,"pc_time_us":1000000}`

```text
frame (18 byte): 01 09 0C 00 09 00 00 00 40 42 0F 00 00 00 00 00 58 CB
wire  (20 byte): 04 01 09 0C 02 09 01 01 04 40 42 0F 01 01 01 01 03 58 CB 00
```

### 7.10 HOST_FAULT（`0x0A`、PC → MCU）

値: `{"request_id":10,"fault_code":"HOST_FAULT"}`

```text
frame (14 byte): 01 0A 08 00 0A 00 00 00 0A 00 00 00 2D 36
wire  (16 byte): 04 01 0A 08 02 0A 01 01 02 0A 01 01 03 2D 36 00
```

### 7.11 SET_REFERENCE（`0x0B`、PC → MCU）

値: `{"request_id":11,"reference_count":-2770}`

```text
frame (14 byte): 01 0B 08 00 0B 00 00 00 2E F5 FF FF A5 41
wire  (16 byte): 04 01 0B 08 02 0B 01 01 07 2E F5 FF FF A5 41 00
```

### 7.12 HELLO（`0x81`、MCU → PC）

値: `{"request_id":1,"protocol_version":1,"axis_id_hint":0,"mcu_state":1,"device_id":"5131000000000001","boot_id":2309737967,"fw_version":[0,1,0],"capabilities":1,"calibration_hash":0}`

```text
frame (38 byte): 01 81 20 00 01 00 00 00 01 00 01 00 01 00 00 00 00 00 31 51 EF CD AB 89 00 01 00 00 01 00 00 00 00 00 00 00 78 8D
wire  (40 byte): 04 01 81 20 02 01 01 01 02 01 02 01 02 01 01 01 01 01 07 31 51 EF CD AB 89 02 01 01 02 01 01 01 01 01 01 01 03 78 8D 00
```

### 7.13 ACK ACCEPTED（`0x82`、MCU → PC）

値: `{"request_id":2,"acked_type":2,"result":"ACCEPTED","reason":"NONE"}`

```text
frame (14 byte): 01 82 08 00 02 00 00 00 02 00 00 00 10 86
wire  (16 byte): 04 01 82 08 02 02 01 01 02 02 01 01 03 10 86 00
```

### 7.14 ACK REJECTED / EPOCH_MISMATCH（`0x82`、MCU → PC）

値: `{"request_id":4,"acked_type":3,"result":"REJECTED","reason":"EPOCH_MISMATCH"}`

```text
frame (14 byte): 01 82 08 00 04 00 00 00 03 01 02 00 3D 21
wire  (16 byte): 04 01 82 08 02 04 01 01 04 03 01 02 03 3D 21 00
```

### 7.15 STATUS（`0x83`、MCU → PC）

値: `{"boot_id":2309737967,"mcu_time_us":1234567890,"mcu_state":4,"fault_code":"NONE","flags":31,"pwm_permille":-250,"actual_count":-1230,"target_count":-1234,"velocity_count_s":-150,"following_error_count":4,"last_applied_frame":1234,"last_applied_seq":1000,"last_rx_seq":1000,"current_ma":0,"rx_error_count":1,"stale_drop_count":2}`

```text
frame (62 byte): 01 83 38 00 EF CD AB 89 D2 02 96 49 00 00 00 00 04 00 00 00 1F 00 06 FF 32 FB FF FF 2E FB FF FF 6A FF FF FF 04 00 00 00 D2 04 00 00 E8 03 00 00 E8 03 00 00 00 00 00 00 01 00 02 00 C3 5C
wire  (64 byte): 04 01 83 38 09 EF CD AB 89 D2 02 96 49 01 01 01 02 04 01 01 02 1F 10 06 FF 32 FB FF FF 2E FB FF FF 6A FF FF FF 04 01 01 03 D2 04 01 03 E8 03 01 03 E8 03 01 01 01 01 01 02 01 02 02 03 C3 5C 00
```

### 7.16 FAULT_EVENT（`0x84`、MCU → PC）

値: `{"boot_id":2309737967,"mcu_time_us":1234600000,"fault_code":"FOLLOWING_ERROR","mcu_state_before":4,"actual_count":-1500}`

```text
frame (26 byte): 01 84 14 00 EF CD AB 89 40 80 96 49 00 00 00 00 02 00 04 00 24 FA FF FF 3F 2B
wire  (28 byte): 04 01 84 14 09 EF CD AB 89 40 80 96 49 01 01 01 02 02 02 04 07 24 FA FF FF 3F 2B 00
```

### 7.17 CLOCK_SYNC_REPLY（`0x85`、MCU → PC）

値: `{"request_id":9,"pc_time_us":1000000,"mcu_rx_us":5000000,"mcu_tx_us":5000120}`

```text
frame (34 byte): 01 85 1C 00 09 00 00 00 40 42 0F 00 00 00 00 00 40 4B 4C 00 00 00 00 00 B8 4B 4C 00 00 00 00 00 A9 A7
wire  (36 byte): 04 01 85 1C 02 09 01 01 04 40 42 0F 01 01 01 01 04 40 4B 4C 01 01 01 01 04 B8 4B 4C 01 01 01 01 03 A9 A7 00
```
<!-- VECTORS:END -->
