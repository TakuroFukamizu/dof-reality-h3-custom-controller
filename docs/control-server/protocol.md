# PC ⇄ CoreS3 軸コントローラー USB プロトコル v1

状態: **v1 案（PC 側実装済み・CoreS3 側未実装）**。PC 側の参照実装は `control-server/src/protocol/`、MCU 側の参照モデルは `control-server/src/sim_axis.rs`（模擬軸）。CoreS3 ファームウェアはこの文書と末尾のテストベクタに合わせて実装する。

元になった設計: [H3 Control Server 初期アーキテクチャ・互換仕様案 v0.1](architecture.md) §9。

## 1. 基本方針

- 1 本の USB CDC に 1 軸。PC は軸ごとに独立したタスクで読み書きし、1 軸の書き込み待ちで他軸を待たせない。
- PC は **HELLO を受けて軸を決めるまで、その USB ポートに HELLO_REQ 以外を送らない**（同じ PC に別の ESP32-S3 が刺さっていることがある）。軸は HELLO の `selected_axis`（MCU で選択中の系統）で決める（§1.1）。
- 位置制御・PWM・ブレーキシーケンス・期限切れ停止は MCU 側で完結する。PC は位置目標（count）だけを送る。
- 古い目標の再送はしない。SET_POSITION は ACK を返さず、定期 STATUS の `last_applied_*` で適用を確認する。管理命令は `request_id` 付きで ACK を返し、受理済み命令の再送（`msg_type`・`request_id`・payload がすべて同じフレーム）は再実行しない。HOST_FAULT だけは例外で、毎回実行する（§4「管理命令の共通規則」）。
- ACK の `ACCEPTED` は「受理」であって、動作の完了ではない（減速・ARMING の待ち時間などは後で進む。到達は STATUS の `mcu_state` で確認する）。
- **ただし、命令による状態遷移そのものは ACK を送る前に確定させる**（DISARMED→ARMING、HOLDING→ACTIVE、ACTIVE→STOPPING、→DISARMING、任意→FAULT、FAULT→DISARMED、`CONFIGURED`・`POSITION_REFERENCED` の変化）。ACK より後に送る STATUS はすべて遷移後の状態を示す。ACK より前に取ったスナップショットを ACK の後に送らない（USB 送信と制御が別コアでも、STATUS は送信時点の状態から作る）。PC はこれに依存している: ARM_PREPARE の ACK の後に DISARMED の STATUS が来ると軸の状態不一致として FAULT にし、HOST_FAULT の ACK の後の最初の STATUS を見て FAULT_RESET の送り先を決める。

### 1.1 系統（軸）の選択と割当

3 台の CoreS3 は同一ファームウェアで、**どの系統（軸 1〜3）を担当するかは MCU 側で選択する**（電源ボタン操作、NVS に保存）。PC はその選択を受けて軸を割り当てる。PC の設定ファイル `[[axis]].device_id` は任意の「この軸はこの個体のはず」という照合用で、0 なら照合しない。

MCU 側の規則:

- 選択中の系統を HELLO の `selected_axis` と **毎回の STATUS の `selected_axis`** に出す（1〜3、0 = 未選択）。
- 系統の切り替えは **DISARMED のときだけ** 受け付ける。それ以外の状態では操作を無視する（画面に表示してよい）。FAULT 中も切り替えない（まず FAULT_RESET）。万一 DISARMED 以外で選択が変わる実装になっていても、制限値と lease（`command_timeout_ms`）は保ち続けること。PC は切り替えた個体へ何も送らなくなるので、その個体を止めるのは COMMAND_TIMEOUT だけになる（CONFIG を消して lease を止めてはいけない）。
- 切り替えたら、ACK と同じ順序規則（§1）で状態を確定してから、(1) `CONFIGURED` と `POSITION_REFERENCED` を消し、`calibration_hash` を 0 にする（CONFIG の内容も位置基準も軸ごとの値なので、前の系統の値を持ち越さない）、(2) AXIS_SELECT（`0x86`、§4）を 1 回送る、(3) 以後の STATUS は新しい `selected_axis` を載せる。AXIS_SELECT に ACK はない。PC は STATUS の `selected_axis`（1〜3）でも変化を検出するので、1〜3 への切り替えは AXIS_SELECT が壊れて届かなくても追いつく。**0（未選択）への変化は STATUS では検出しない**（旧ファーム互換のため STATUS の 0 は無視する）。N→0 は AXIS_SELECT か次の boot の HELLO でしか PC に伝わらず、AXIS_SELECT が落ちると PC はその個体を軸 N のまま扱う（USB の抜き差しで直る）。したがって、一度系統を選んだ後に「未選択」へ戻す操作はファームの UI に置かないことを推奨する（置くなら、この制約を承知の上で）。
- 系統の選択で `actual_count` の向き・原点・スケールを変えない。count は常に生のエンコーダ計数を SET_REFERENCE で基準化した値で、系統ごとの符号・ストローク・中立位置は PC 側の校正（`[[axis]]`）で扱う。
- 受理済み命令の記憶（DUPLICATE 判定）と `last_rx_seq` は切り替えで消さなくてよい。

PC 側の規則（詳細は `control-server/DESIGN.md` §10）:

- `selected_axis` が 1〜3 ならその軸。0 なら設定に `device_id` が書かれている軸、どれにも無ければ未知の装置としてポートを閉じ、**その USB 装置が消えるまで開き直さない**（閉じた後に MCU で系統を選んで AXIS_SELECT を送っても PC には届かない）。系統を選んでから **USB を抜き差しする**（CoreS3 のリセットでも可）。新品（NVS 未設定で、設定の `device_id` にも無い個体）の初回は必ずこの手順になる。
- 設定の `device_id` が 0 でなく、選択された軸の個体と違えば ARM しない（`axis_device_mismatch`）。
- 同じ系統を 2 台以上が選択していれば（`axis_conflict`）、その軸にはどちらにも CONFIG を送らず、ARM・START・ゲーム入力を拒否する。解消（片方が別の系統へ・切断）で残った 1 台を割り当てる。
- ARMING〜STOPPING の間に担当軸の `selected_axis` が変わったら PC はリグを FAULT にし、接続中の他の軸へ HOST_FAULT を送る（切り替え先の系統を担当していた個体にも、競合で外す前に HOST_FAULT を届けて ACK を待つ）。**切り替えた個体自身には届かない**（競合中・未割当のポートには PC は何も送らない）ので、その個体は lease 切れ（COMMAND_TIMEOUT）で止まる。MCU 側の「DISARMED のときだけ切り替える」規則が最後の砦。駆動中に同じ系統を選んだ 2 台目が現れたら、減速停止 → DISARM してから競合扱いにする。
- 同じ boot のまま系統が変わった個体には、PC 自身が SET_REFERENCE を送り直すまで ARM しない（`reference_stale`。ファームが `POSITION_REFERENCED` を消し忘れても古い基準で動かさない）。

### 1.2 校正（端当てで行程を測る。MCU 主導、PC が開始する）

行程（P 端・N 端）と中立の実測は、従来 env で実機確認済みの端当て（`moter-controler/include/auto_move.h`: P 端当て → 退避 → N 端当て → 保存 → 中立。無パルス・遅すぎ・移動量・時間の守り付き）を **v1 でも MCU が自分で行う**。PC は CALIBRATE（`0x0C`、§4）で開始・中止を指示し、結果を CAL_RESULT（`0x87`）で受ける。

- 受理は **DISARMED のときだけ**（CONFIGURED・REFERENCED は不要）。受理で `mcu_state = CALIBRATING`（8、§3.1）。校正中に ARM_PREPARE / START / CONFIG / SET_REFERENCE は `BAD_STATE`、SET_POSITION は捨てる（数えない）。DISARM は校正の中止（下の終了手順で DISARMED へ。ACCEPTED）。HOST_FAULT は常に FAULT。
- 手順: 主電源 ON 指令（`role_flags` bit0 の軸だけ）→ `CAL_POWER_SETTLE_MS`（5000。2026-10-31 に電源投入 4 s 後は動かず 12 s 後に動いた記録があるため長め。仮値）→ ブレーキ解除給電 → `BRAKE_RELEASE_WAIT_MS` → 端当て校正 → 終了手順。終了手順は DISARMING と同じ（ブレーキ給電 OFF → `BRAKE_APPLY_WAIT_MS` → PWM 0 → 主電源 OFF 指令）→ CAL_RESULT を 1 回 → STATUS（DISARMED）。
- 成功したら MCU は **P 端を 0 とする基準を自分で確立**し（`reference_offset = raw_at_P_end`、以後 `actual_count = raw − reference_offset`）、`POSITION_REFERENCED` を立てる。行程・中立・符号を NVS の系統プロファイルに保存する。失敗・中止では `POSITION_REFERENCED` を消す（原点が信用できない）。FAULT にはしない（結果は CAL_RESULT の `result` / `reason`）。
- CAL_RESULT の座標は `actual_count` と同じ（P 端 = 0。符号は掛けない）。`n_end_count` は N 端の count（系統の符号が +1 なら正、−1 なら負）、`neutral_count` は中立の count（既定は `n_end_count / 2`）。PC はこれから `mechanical_min/max = min/max(0, n_end_count)`、`soft = mechanical ∓ decel_zone`、`count_at_virtual_0 / 512 / 1023 = soft の P 端側 / neutral / soft の N 端側`（仮想位置は P 端 → N 端へ増える約束。向きが逆なら PC の設定で 0 と 1023 を入れ替える）を作る。
- 主電源を持たない軸は、電源担当軸が主電源を入れていなければ 24V が無く、端当ては無パルスで失敗する。PC は校正を **電源担当軸を含む複数軸へ同時に** 指示し、電源担当軸には `KEEP_POWER` を付ける（§4 CALIBRATE）。電源担当軸は自分の校正が終わっても主電源を保ち（POWER_HOLD）、PC は他の軸がすべて CAL_RESULT を返してから電源担当軸へ CALIBRATE(0) を送って主電源を切る。2026-11-01 の実機で、`KEEP_POWER` の無い版では電源担当軸 #1 が先に終わって主電源を切り、#2 / #3 が中立への移動中に無パルスで失敗した（両端の計測は取れていた）。

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
- MCU の送信はブロックしない（Issue #15 M7、ファーム `include/v1_tx_pump.h`）: USB CDC の TX リング（1024 byte）に空きが無いとき、**STATUS は捨てる**（次の 20 ms のものが新しい）、それ以外（ACK・FAULT_EVENT・HELLO・LIMITS・CAL_RESULT・AXIS_SELECT）は **MCU 側で捨てない**: 順序を保って保留し、空きが戻ったら書く。保留が 100 ms 続いたら空きを見ずに書く（HWCDC の待ちは tx timeout 1 ms × 最大 20 回 = 20 ms で有界。未接続なら HWCDC が古い内容を置き換えて即返る）。積まれてから 100 ms 以上経った STATUS は空きがあっても送らない（古い STATUS を順に流さず新しいものだけ）。STATUS 以外に滞在時間の期限は無い。PC が読んでいない間に MCU の受信（SET_POSITION）が止まらないための規則で、STATUS 側の値は PC の STATUS 途絶 200 ms（§6）より短い。詰まったリンクでは古い ACK が遅れて届くことがあり、PC は期限（1000 ms）を過ぎた ACK を無視する。

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
| 8 | `CALIBRATING` | CALIBRATE 受理後、端当て校正の間（§1.2）。MCU 主導。終了で DISARMED |

PWM = 0 を位置保持と見なさない。`HOLDING` は位置制御で支えている状態。

### 3.2 異常コード `fault_code`（u16）

| 値 | 名前 | 主な発生条件 |
| ---: | --- | --- |
| 0 | `NONE` | |
| 1 | `COMMAND_TIMEOUT` | ARMING/HOLDING/ACTIVE/STOPPING で、最後に有効な SET_POSITION（ARMING 中にまだ無ければ ARM_PREPARE 受理時刻）から `command_timeout_ms` を超えた。ただし STOPPING のうち DISARM による減速と、異常による減速は対象外（`stop_timeout_ms` で抑える）。動いていれば減速停止 → FAULT、止まっていれば即 FAULT |
| 2 | `FOLLOWING_ERROR` | `abs(actual_count − target_count) > following_error_limit_count`（`target_count` は STATUS と同じ、軌道上の目標）。位置制御中（ARMING のブレーキ解除以降・HOLDING・ACTIVE・STOPPING・DISARMING）だけ判定 |
| 3 | `OVERSPEED` | `abs(velocity) > overspeed_count_s`。位置制御中（同上）だけ判定（DISARMED・FAULT で手で動かしても異常にしない） |
| 4 | `SOFT_LIMIT` | 位置制御中に実位置が `[soft_min_count, soft_max_count]` を外れた。ARMING ではブレーキ解除の直前にも確認し、外れていればブレーキを解除せずに FAULT |
| 5 | `ENCODER_FAULT` | PCNT 異常・計数異常。ファームの無パルス判定: ブレーキ解除指令中に軌道（`target_count`）が動いている間、`abs(pwm) ≥ max(V1_STALL_DUTY 10 %, duty_max × 75 %)` を 200 ms（`STALL_MS`）出してカウントが変わらない（Issue #15 M2。ループが上限近くまで出しても動かないときだけ。それ未満の無パルスは FOLLOWING_ERROR に任せる）。加えて、目標が止まっているときの停止ブースト（§5）が上限（2 s または `duty_max` で頭打ち）に達しても動かないときもこのコード（詰まった機構と死んだエンコーダは区別できない）。**動けない軸が FOLLOWING_ERROR と ENCODER_FAULT のどちらになるかは PC の `[limits]` で決まる**: `kp × following_error_limit / 100 + FF` がしきい値未満なら FOLLOWING_ERROR（基準は残る）、以上なら ENCODER_FAULT（FAULT_RESET で `POSITION_REFERENCED` が消える）。`duty_max_percent < 10` の SET_LIMITS は受理するが無パルス判定が決して成立しなくなる（画面に「無パルス検出なし」。[limits.md](limits.md) §2.2） |
| 6 | `CONTROL_OVERRUN` | 制御周期の逸脱 |
| 7 | `CONFIG_INVALID` | 予約（v1 では発生しない）。CONFIG 未受理の ARM_PREPARE は ACK `REJECTED/NOT_CONFIGURED` で拒否し、DISARMED のまま |
| 8 | `STOP_TIMEOUT` | 減速停止が `stop_timeout_ms` 内に完了しない。ただし異常による減速（COMMAND_TIMEOUT）が超えたときは STOP_TIMEOUT ではなく元の異常コードでラッチする。CONTROLLED_STOP の減速中に COMMAND_TIMEOUT になった場合、`stop_timeout_ms` は最初の STOPPING 開始から数える |
| 9 | `POWER_FAULT` | 電源系の異常（入力がある場合） |
| 10 | `HOST_FAULT` | PC から FAULT を受けた |
| 11 | `NOT_REFERENCED` | 予約（v1 では発生しない）。位置基準未確立の ARM_PREPARE は ACK `REJECTED/NOT_REFERENCED` で拒否し、DISARMED のまま |
| 12 | `LOCAL_ESTOP` | MCU の画面の非常停止ボタン。ボタンのラッチを解除してから FAULT_RESET（解除前は FAULT_CAUSE_PRESENT） |
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
| `0x02` | CONFIG | 0: u32 request_id / 4: u32 session_epoch / 8: u32 calibration_hash / 12: i32 soft_min_count / 16: i32 soft_max_count / 20: u32 max_vel_count_s / 24: u32 max_acc_count_s2 / 28: u32 following_error_limit_count / 32: u32 overspeed_count_s / 36: u16 command_timeout_ms / 38: u16 stop_timeout_ms / 40: u8 role_flags（bit0 = 主電源担当） / 41: u8 reserved=0 / 42: u16 reserved=0 | 44 | ACK。DISARMED のときだけ受理（それ以外は `REJECTED/BAD_STATE`）。`soft_min_count < soft_max_count`、かつ `max_vel_count_s`・`max_acc_count_s2`・`following_error_limit_count`・`overspeed_count_s`・`command_timeout_ms`・`stop_timeout_ms` がすべて > 0 でなければ `REJECTED/INVALID_PARAM`。v1 ではこれ以外の検査で拒否しない（`role_flags` の bit1〜7 と reserved は無視）。**拒否はしないがクランプはする**: `max_vel_count_s` / `max_acc_count_s2` は絶対上限（`V1_ABS_MAX_VEL_COUNT_S` 6000 / `V1_ABS_MAX_ACC_COUNT_S2` 40000）で切って保存する（PC は CONFIG にこの絶対上限を入れ、実際の上限は毎フレームの SET_POSITION で配る。[limits.md](limits.md) §1）。受理すると全項目を置き換え、`CONFIGURED` を立て（`POSITION_REFERENCED` と状態は変えない）、HELLO の `calibration_hash` をこの値にする。epoch が変われば `last_rx_seq = 0`（下記）。`role_flags` bit0 は STATUS の `POWER_CONTROLLER` にそのまま出す（PC は設定と違えば ARM しない） |
| `0x03` | ARM_PREPARE | 0: u32 request_id / 4: u32 session_epoch | 8 | ACK、その後 STATUS で ARMING → HOLDING。DISARMED・CONFIGURED・REFERENCED・epoch 一致・実位置が `[soft_min_count, soft_max_count]` 内のときだけ受理。拒否理由はこの順に `BAD_STATE` / `NOT_CONFIGURED` / `NOT_REFERENCED` / `EPOCH_MISMATCH` / `BAD_STATE`（位置が範囲外） |
| `0x04` | START | 0: u32 request_id / 4: u32 session_epoch | 8 | ACK。HOLDING → ACTIVE。HOLDING・epoch 一致に加え、ARM_PREPARE 受理後に受けた有効期限内（`valid_for_ms`）の SET_POSITION があること。無ければ `REJECTED/BAD_STATE`（受理するとすぐ期限切れで STOPPING に落ちるため）。拒否理由はこの順に `BAD_STATE`（HOLDING でない）/ `EPOCH_MISMATCH` / `BAD_STATE`（期限内の SET_POSITION なし）。受理後も、START の後に受けた最初の SET_POSITION までは保持目標のまま（START 前に受けた目標へ飛ばない） |
| `0x05` | SET_POSITION | 0: u32 session_epoch / 4: u32 seq / 8: u32 group_frame_id / 12: i32 target_count / 16: u32 max_vel_count_s / 20: u32 max_acc_count_s2 / 24: u16 valid_for_ms / 26: u16 reserved=0 / 28: u64 apply_at_us（0 = 受信後すぐ） | 36 | なし（STATUS で確認） |
| `0x06` | CONTROLLED_STOP | 0: u32 request_id / 4: u32 session_epoch / 8: u8 reason / 9..11: reserved=0 | 12 | ACK。CONFIG 未受理・epoch 不一致は（どの状態でも）`REJECTED/EPOCH_MISMATCH`。ACTIVE → STOPPING → HOLDING。ACTIVE 以外（DISARMED・ARMING・HOLDING・STOPPING・DISARMING）では何もせず ACCEPTED（PC は HOLDING 中の停止操作でも送る。拒否すると PC は停止失敗として FAULT にする）。`reason` は記録のみ（1 = 管理操作、2 = ゲーム入力の途絶、3 = 中立到達。未知の値も拒否しない） |
| `0x07` | DISARM | 0: u32 request_id | 4 | ACK。管理命令の共通規則（`request_id` = 0・DUPLICATE・`FAULT_ACTIVE`）で弾かれるとき以外は常に ACCEPTED。ARMING / HOLDING → 直ちに DISARMING。ACTIVE → 減速停止 → DISARMING。STOPPING → 減速を続け、停止後 DISARMING。DISARMED / DISARMING → 何もせず ACCEPTED（PC は ARM 途中の停止でも、まだ DISARMED・ARMING の軸に送る。拒否すると PC は停止失敗として FAULT にする） |
| `0x08` | FAULT_RESET | 0: u32 request_id | 4 | ACK。FAULT 以外では `REJECTED/BAD_STATE`。原因（エンコーダ異常・電源異常の入力など、続いている異常）が残っていれば `REJECTED/FAULT_CAUSE_PRESENT`。解除後は DISARMED（自動再 ARM しない）。CONFIG と位置基準は残す（ENCODER_FAULT の解除では `POSITION_REFERENCED` を消す）。実位置がソフトリミット外のままでも解除する（下記） |
| `0x09` | CLOCK_SYNC | 0: u32 request_id / 4: u64 pc_time_us | 12 | CLOCK_SYNC_REPLY |
| `0x0A` | HOST_FAULT | 0: u32 request_id / 4: u16 fault_code / 6: u16 reserved=0 | 8 | ACK（常に ACCEPTED）。どの状態でも即 FAULT ラッチ（PWM 0・ブレーキ作動）。ラッチされるコードは `HOST_FAULT`（既にラッチ済み・減速中の異常コードがあればそれを保つ）。payload の `fault_code` は記録のみ。ACK の後に送る最初の STATUS から `mcu_state = FAULT`、`fault_code ≠ NONE`（PC はこれを確認してから FAULT_RESET の送り先を決める） |
| `0x0B` | SET_REFERENCE | 0: u32 request_id / 4: i32 reference_count | 8 | ACK。DISARMED のときだけ。現在の計数値を `reference_count` と定義し `POSITION_REFERENCED` を立てる |
| `0x0C` | CALIBRATE | 0: u32 request_id / 4: u8 mode（1 = 開始、0 = 中止） / 5: u8 flags（bit0 = `KEEP_POWER`。電源担当軸（CONFIG の `role_flags` bit0）だけが解釈し、他の軸は無視する。bit1〜7 は予約 = 0） / 6: u16 reserved=0 | 8 | ACK。`mode = 1`: DISARMED のときだけ（それ以外 `REJECTED/BAD_STATE`）。受理で CALIBRATING（§1.2）。`KEEP_POWER` が立っていれば、校正（成功・失敗・中止を問わず）の終了手順でブレーキを作動させ PWM 0 にした後も **主電源 ON 指令を保ったまま CALIBRATING（CAL_RESULT の `step = 4` POWER_HOLD）に留まる**。`mode = 0` か `CAL_POWER_HOLD_TIMEOUT_MS`（120000、仮値）で主電源 OFF → DISARMED（このとき CAL_RESULT は送らない。結果は校正終了時に送った 1 回だけ）。`mode = 0`: CALIBRATING（POWER_HOLD を含む）なら中止して終了手順 → DISARMED、それ以外は何もせず ACCEPTED。`mode` が 0・1 以外は `REJECTED/INVALID_PARAM` |
| `0x0D` | SET_LIMITS | 0: u32 request_id / 4: u8 duty_max_percent / 5: u8 duty_min_percent / 6: u8 kp_percent_per_100 / 7: u8 reserved=0 / 8: u16 ff_count_s_per_percent / 10: u16 deadband_count / 12: u16 deadband_out_count / 14: u16 reserved=0 | 16 | ACK。位置ループの調整値（PC の `[limits]`。[limits.md](limits.md)）。**どの状態でも受理**（管理命令の共通規則だけ）。`duty_max_percent = 0`、`duty_min_percent > duty_max_percent`、`ff_count_s_per_percent = 0`、`deadband_out_count < deadband_count` は `REJECTED/INVALID_PARAM`。受理したら `duty_max`（と `duty_min`）を絶対上限 `V1_ABS_DUTY_MAX`（60）でクランプして置き換え、**ACK の後に LIMITS を 1 回送る**（REJECTED・DUPLICATE では送らない）。受理前は `config.h` の既定（Duty 7〜20 %）。対応するファームは HELLO の `capabilities` bit3 を立てる |

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
- `max_vel_count_s` / `max_acc_count_s2` はこのフレームの上限。CONFIG の上限より大きい値は CONFIG の値に制限（CONFIG には絶対上限が入っているので、結果として絶対上限）。STOPPING の減速は **最後に適用したフレームの `max_acc_count_s2`** で行う（0 なら CONFIG の値）。CONFIG の値で減速すると絶対上限の急減速になるため。
- `HOLDING` / `STOPPING` では `target_count` を使わないが、期限監視（lease）としては有効。
- `group_frame_id` は 3 軸に同じ値を配る。MCU は適用した値を `last_applied_frame` に返す。同じ値だから同時に適用されたとは見なさない。

### MCU → PC

| type | 名前 | payload | 長さ |
| --- | --- | --- | ---: |
| `0x81` | HELLO | 0: u32 request_id / 4: u8 protocol_version / 5: u8 selected_axis（MCU で選択中の系統 1〜3、0 = 未選択。PC はこれで軸を割り当てる、§1.1） / 6: u8 mcu_state / 7: u8 reserved=0 / 8: u64 device_id（eFuse MAC から作る固有 ID） / 16: u32 boot_id（起動ごとに乱数） / 20: u8 fw_major / 21: u8 fw_minor / 22: u8 fw_patch / 23: u8 reserved=0 / 24: u32 capabilities（bit0 = POSITION モード、bit1 = apply_at、bit2 = 電流計測、bit3 = SET_LIMITS / LIMITS 対応。PC は bit3 の無い軸を `firmware_limits_unsupported` で ARM 阻害にする） / 28: u32 calibration_hash（受理済み CONFIG の値、未設定 0） | 32 |
| `0x82` | ACK | 0: u32 request_id / 4: u8 acked_type / 5: u8 result / 6: u16 reason | 8 |
| `0x83` | STATUS | 0: u32 boot_id / 4: u64 mcu_time_us / 12: u8 mcu_state / 13: u8 selected_axis（HELLO と同じ値。0 = 未選択） / 14: u16 fault_code / 16: u16 flags / 18: i16 pwm_permille（符号付き −1000〜1000） / 20: i32 actual_count / 24: i32 target_count（下記） / 28: i32 velocity_count_s / 32: i32 following_error_count（下記） / 36: u32 last_applied_frame / 40: u32 last_applied_seq / 44: u32 last_rx_seq / 48: i32 current_ma / 52: u16 rx_error_count / 54: u16 stale_drop_count | 56 |
| `0x84` | FAULT_EVENT | 0: u32 boot_id / 4: u64 mcu_time_us / 12: u16 fault_code / 14: u8 mcu_state_before / 15: u8 reserved=0 / 16: i32 actual_count | 20 |
| `0x85` | CLOCK_SYNC_REPLY | 0: u32 request_id / 4: u64 pc_time_us（エコー） / 12: u64 mcu_rx_us / 20: u64 mcu_tx_us | 28 |
| `0x86` | AXIS_SELECT | 0: u32 boot_id / 4: u8 selected_axis（新しい値 1〜3、0 = 未選択） / 5: u8 prev_selected_axis / 6: u8 mcu_state / 7: u8 reserved=0 | 8 |
| `0x88` | LIMITS | 0: u32 boot_id / 4: u8 duty_max_percent / 5: u8 duty_min_percent / 6: u8 kp_percent_per_100 / 7: u8 abs_duty_max_percent / 8: u16 ff_count_s_per_percent / 10: u16 deadband_count / 12: u16 deadband_out_count / 14: u16 reserved=0 / 16: u32 abs_max_vel_count_s / 20: u32 abs_max_acc_count_s2 | 24 |
| `0x87` | CAL_RESULT | 0: u32 boot_id / 4: u8 result（0 = OK、1 = FAILED、2 = CANCELLED） / 5: u8 step（終了時の段階。0 = なし、1 = P 端へ、2 = N 端へ、3 = 中立へ。`KEEP_POWER` で電源を保持する軸は 4 = POWER_HOLD を使わず、校正が終わった段階（通常 3）を入れる。POWER_HOLD は STATUS の `mcu_state = CALIBRATING` と `POWER_ON_CMD` で分かる） / 6: u16 reserved=0 / 8: i32 n_end_count / 12: i32 neutral_count / 16: i8 sign（系統プロファイルの符号 ±1） / 17: u8 reserved=0 / 18: u16 reserved=0 / 20: u8[8] reason（ASCII、NUL 埋め。`ok` / `stall` / `slow` / `travel` / `timeout` / `stop` / `estop` など auto_move の理由） | 28 |

- STATUS は MCU が自発的に 50Hz（20ms）で送る。状態遷移・FAULT 発生時は即時にも送ってよい。ACK との順序は §1（ACK より後の STATUS は遷移後の状態）。
- STATUS の `target_count` = 位置ループがいま追従している目標（速度・加速度制限を適用済みの軌道上の点）。SET_POSITION で受けた `target_count` そのものではない。HOLDING / STOPPING / DISARMING では保持・減速中の目標。PC は中立への移動（`/v1/center`）の完了を、全軸の STATUS `target_count` が中立に達し、**かつ全軸の `actual_count`（実位置）が中立の `[game].center_tolerance_count` 以内に入った**ことで判断し、その後に CONTROLLED_STOP を送る（`target_count` だけで判断すると、自重で遅れた軸を中立の手前で「到達」にしてしまう。2026-11-01 に #2 が 280 count 手前で止まった）。目標が中立に着いてから `[timing].center_reach_timeout_ms` 以内に実位置が入らなければ PC は FAULT にする。受けた目標を返すと移動の途中で止められる。
- `following_error_count` = `actual_count − target_count`（符号付き。FOLLOWING_ERROR の判定と同じ値）。`pwm_permille` と `velocity_count_s` は正が count の増える向き。`pwm_permille` は MCU が実際に PWM に書いている値そのもの（0.1 % 分解能。Issue #25 手順 8 b より前のファームは位置ループが整数 % だったので × 10 の値しか出なかった）。校正中は AutoMachine の整数 % × 10。
- FAULT_EVENT は FAULT へ遷移するたびに 1 回送る（原因を問わない。HOST_FAULT・ARMING 中の SOFT_LIMIT も含む）。既に FAULT のときは送らない。`mcu_state_before` は遷移直前の状態。PC は FAULT_EVENT と STATUS のどちらでも異常をラッチする。
- `actual_count` / `target_count` は `SET_REFERENCE` 適用後の座標。基準未確立なら起動時 0 からの相対値で、`POSITION_REFERENCED` = 0。
- `boot_id` が変わったら PC はその軸の校正状態を無効化し、再 ARM を要求する。
- CAL_RESULT は校正が終わる（成功・失敗・中止）たびに 1 回、終了手順の後・DISARMED を示す STATUS の前に送る（§1.2）。
- LIMITS は SET_LIMITS を受理した直後（ACK の後）に 1 回だけ送る。有効値（クランプ後）と、ファームの絶対上限 3 つ（`abs_*`: Duty 60 %、速度 6000 count/s、加速度 40000 count/s²。`config.h` の `V1_ABS_*`）。PC はこれを `status.axes[].fw_limits` に出し、接続中の軸の最小を実行時の制限値の上限にする。HELLO の直後に自発的には送らない（PC が識別直後に SET_LIMITS を送る）。
- AXIS_SELECT は系統を切り替えたとき（DISARMED のときだけ、§1.1）に 1 回送る。ACK はない。`prev_selected_axis` は切り替え前の値。PC は AXIS_SELECT、または STATUS / HELLO の `selected_axis` の変化のどれでも割当を変える。ただし STATUS の 0 では割当を変えない（旧ファームの reserved=0 と区別できない。§1.1）。

## 5. 状態遷移（MCU）

```text
BOOT → DISARMED
DISARMED --ARM_PREPARE(CONFIGURED & REFERENCED & epoch一致 & 実位置がソフトリミット内)--> ARMING --(電源・位置制御・ブレーキ解除完了)--> HOLDING
HOLDING --START(期限内の SET_POSITION あり)--> ACTIVE
ACTIVE --CONTROLLED_STOP / 期限切れ(valid_for_ms)--> STOPPING --(停止確認)--> HOLDING
ARMING / HOLDING --DISARM--> DISARMING → DISARMED
ACTIVE / STOPPING --DISARM--> (減速停止) → DISARMING → DISARMED
任意 --異常 / HOST_FAULT / command_timeout--> FAULT --FAULT_RESET(原因なし)--> DISARMED
DISARMED --CALIBRATE(1)--> CALIBRATING --(終了手順。成功・失敗・中止、DISARM)--> DISARMED   （§1.2）
```

- 停止確認 = 軌道（STATUS の `target_count`）の速度が 0 になったこと。
- 上の図にない命令と状態の組み合わせ（HOLDING での CONTROLLED_STOP、DISARMED での DISARM など）は §4 の表のとおり何もせず ACCEPTED。
- USB 再接続・CDC の再オープンで自動 ARM・自動 START しない。NVS に ARM 状態を保存しない。
- DISARM による減速（ACTIVE / STOPPING からの DISARM）は lease の対象外。PC が SET_POSITION を止めても COMMAND_TIMEOUT にせず、停止後 DISARMING → DISARMED へ進む（`stop_timeout_ms` を超えたら STOP_TIMEOUT）。
- `valid_for_ms` の期限切れで ACTIVE → STOPPING → HOLDING に落ちた後は、遅れて届いた SET_POSITION では動き出さない。再開には START が要る。
- ARMING の順序: PWM 0 のまま主電源 ON 指令（電源担当軸のみ）→ `POWER_SETTLE_MS` → 実位置がソフトリミット外ならブレーキを解除せず FAULT(SOFT_LIMIT) → 現在位置を保持目標にして位置制御開始 → ブレーキ解除給電 → `BRAKE_RELEASE_WAIT_MS` → HOLDING。
- 主電源（G2）は **電源担当軸だけ** が操作する。それ以外の軸は CONFIG の `role_flags` bit0 = 0 で G2 を操作しない。
- 位置ループ（ファーム `v1_duty_command`、`[limits]` の値で動く。[limits.md](limits.md) §2.2）: `Duty = 軌道速度 / ff + kp × 誤差 / 100`、0 でなければ `|Duty|` を `duty_min`〜`duty_max` に収める。目標が止まっているとき（軌道速度 0）だけ不感帯（入る側 `deadband` / 出る側 `deadband_out`）で Duty 0。積分は無い。**分解能は permille（0.1 %。Issue #25 手順 8 b、2026-11-02）**: FF と P を permille で計算し、和を四捨五入してから下限・上限を掛け、そのまま PWM（10 bit、±1000 permille = フルスケール）に書く。SET_LIMITS / LIMITS の線上は整数 % のままで、`duty_min` / `duty_max` / ENCODER_FAULT のしきい値（`max(10 %, duty_max × 75 %)` を整数 % で切り捨ててから × 10）/ 停止ブーストの刻み（1 % = 10 permille）はファーム内で × 10 して比べる（効き目は整数 % のときと同じ）。旧ファームは `|Duty|` を整数 % に切り捨てていた（5.3 % → 5 %。いまは 53 permille）。2026-11-02 の Issue #15 で足した 2 つの振る舞い（いずれも実機未検証、A/B が要る）:
  - **床（Duty 下限）の適用状態（E、Issue #25 手順 8 c、2026-11-02。`config.h` `V1_FLOOR_OFF_COUNT` = 15）**: 下限 `duty_min`（＋停止ブースト）は、ヒステリシス付きの「床の適用状態」が ON のときだけ掛ける。**ON になる**: |誤差| > `deadband_out`（20）かつ合成（FF + P）の向きが P（誤差）の向きと一致。**OFF になる**: |誤差| ≤ 15、または向きの不一致（即時）。15 < |誤差| ≤ 20 は前の状態を保つ。静止の不感帯に入れば OFF。SET_LIMITS で `deadband_out` を 15 未満にすると ON の判定が先に立つので、OFF の実効しきい値は min(15, `deadband_out`) になりヒステリシスは消える。**OFF のときは有界（`duty_max`）の生の FF + P を permille でそのまま出す**（0 にはしない）。例: 誤差 +50（P +3 %）・軌道速度 −600（FF −6 %）→ 合成 −3 % → 向き不一致 → 生の −3 %（旧 M1 は 0、それより前は −duty_min のバースト）。誤差 0・軌道速度 200（立ち上がり）→ 生の FF 2 %（旧: 下限 9 %）。追従中の小さい誤差では床が消えるので、下限 ≈ 900 count/s が軌道速度を超えるときの「バースト → 停止」は出ない（FF だけで追従する。FF が足りなければ誤差が 20 を超えた所で床が入る）。これは Issue #15 M1（誤差と逆向きの合成を 0 に置換、`V1_FLOOR_SIGN_GATE`）に代わるもので、「誤差と逆向きに床を掛けない」規則は向きの条件として残る。切替フラグは無い（旧動作に戻すには r3 = 914cee8 を焼く）。停止ブースト（下）との関係: 上乗せの対象（軌道速度 0・|誤差| > 20）なら床は必ず ON（静止では合成 = P で向きが一致）。15 < |誤差| ≤ 20 で床が ON のまま残っても上乗せは 0 なので床 = `duty_min` だけ。画面の制御項の行 `FLR` がこの状態。
  - **停止ブースト（H4、`V1_STALL_BOOST_MS` 200 ms）**: 軌道速度 0・**|誤差| > `deadband_out`**（不感帯のヒステリシスの履歴に依らず明示的に見る）・ブレーキ解除指令中に、下限 Duty を出しても 200 ms の間にカウントが 3 未満しか動かなければ、下限を 1 % ずつ上げる（`duty_max` を超えない。SET_LIMITS で `duty_max` が下がればそこまで）。動けば窓を取り直す（上乗せは保つ）。誤差が不感帯に入る・誤差の符号が反転する・軌道が動き出す・ブレーキ作動指令・位置制御の終了で 0 に戻す（その tick の Duty から。上乗せ付きの床を反対向きや軌道の立ち上がりに持ち越さない）。**上限**: 対象になってから `V1_STALL_BOOST_MAX_MS`（2 s）動かない、または上乗せが `duty_max` で頭打ちのまま次の窓も動かなければ **ENCODER_FAULT** をラッチする（詰まった機構を `duty_max` で押し続けない。機構の詰まりとエンコーダの死は区別できず、どちらも位置は信用できないので FAULT_RESET で `POSITION_REFERENCED` が消えるのは §4 のとおり。`duty_max < 10` で無パルス判定を無効にしている構成では FAULT にせず頭打ちのまま）。軌道が動いている間の無パルス判定とは排他で、誤差の上限は従来どおり FOLLOWING_ERROR が守る。上乗せの量は STATUS に載せない（空きフィールドが無い。`pwm_permille` に含まれて見える）。画面の Duty 行に `(+n.n)`（%）、制御項の行に `B`（permille）で出す。

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
`frame` は COBS 前のフレーム（header + payload + crc）、`wire` は COBS 符号化して区切り `0x00` を付けた USB 上のバイト列。いずれも 16 進・空白区切り。「値」は各フィールドの値（JSON、フィールドはレイアウト順、数値は 10 進。`device_id` は 16 桁 hex、`fault_code` / `result` / `reason` は名前で表記。CAL_RESULT の `reason` は NUL を除いた ASCII 文字列）。

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

値: `{"request_id":1,"protocol_version":1,"selected_axis":1,"mcu_state":1,"device_id":"5131000000000001","boot_id":2309737967,"fw_version":[0,1,0],"capabilities":1,"calibration_hash":0}`

```text
frame (38 byte): 01 81 20 00 01 00 00 00 01 01 01 00 01 00 00 00 00 00 31 51 EF CD AB 89 00 01 00 00 01 00 00 00 00 00 00 00 1E 98
wire  (40 byte): 04 01 81 20 02 01 01 01 04 01 01 01 02 01 01 01 01 01 07 31 51 EF CD AB 89 02 01 01 02 01 01 01 01 01 01 01 03 1E 98 00
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

値: `{"boot_id":2309737967,"mcu_time_us":1234567890,"mcu_state":4,"selected_axis":1,"fault_code":"NONE","flags":31,"pwm_permille":-250,"actual_count":-1230,"target_count":-1234,"velocity_count_s":-150,"following_error_count":4,"last_applied_frame":1234,"last_applied_seq":1000,"last_rx_seq":1000,"current_ma":0,"rx_error_count":1,"stale_drop_count":2}`

```text
frame (62 byte): 01 83 38 00 EF CD AB 89 D2 02 96 49 00 00 00 00 04 01 00 00 1F 00 06 FF 32 FB FF FF 2E FB FF FF 6A FF FF FF 04 00 00 00 D2 04 00 00 E8 03 00 00 E8 03 00 00 00 00 00 00 01 00 02 00 49 36
wire  (64 byte): 04 01 83 38 09 EF CD AB 89 D2 02 96 49 01 01 01 03 04 01 01 02 1F 10 06 FF 32 FB FF FF 2E FB FF FF 6A FF FF FF 04 01 01 03 D2 04 01 03 E8 03 01 03 E8 03 01 01 01 01 01 02 01 02 02 03 49 36 00
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

### 7.18 AXIS_SELECT（`0x86`、MCU → PC）

値: `{"boot_id":2309737967,"selected_axis":2,"prev_selected_axis":1,"mcu_state":1}`

```text
frame (14 byte): 01 86 08 00 EF CD AB 89 02 01 01 00 32 C1
wire  (16 byte): 04 01 86 08 08 EF CD AB 89 02 01 01 03 32 C1 00
```

### 7.19 CALIBRATE（`0x0C`、PC → MCU）

値: `{"request_id":7,"mode":1,"flags":0}`

```text
frame (14 byte): 01 0C 08 00 07 00 00 00 01 00 00 00 A2 8F
wire  (16 byte): 04 01 0C 08 02 07 01 01 02 01 01 01 03 A2 8F 00
```

### 7.20 CAL_RESULT（`0x87`、MCU → PC）

値: `{"boot_id":2309737967,"result":"OK","step":3,"n_end_count":5551,"neutral_count":2775,"sign":1,"reason":"ok"}`

```text
frame (34 byte): 01 87 1C 00 EF CD AB 89 00 03 00 00 AF 15 00 00 D7 0A 00 00 01 00 00 00 6F 6B 00 00 00 00 00 00 7A C9
wire  (36 byte): 04 01 87 1C 05 EF CD AB 89 02 03 01 03 AF 15 01 03 D7 0A 01 02 01 01 01 03 6F 6B 01 01 01 01 01 03 7A C9 00
```

### 7.21 SET_LIMITS（`0x0D`、PC → MCU）

値: `{"request_id":13,"duty_max_percent":40,"duty_min_percent":3,"kp_percent_per_100":2,"ff_count_s_per_percent":100,"deadband_count":10,"deadband_out_count":20}`

```text
frame (22 byte): 01 0D 10 00 0D 00 00 00 28 03 02 00 64 00 0A 00 14 00 00 00 40 08
wire  (24 byte): 04 01 0D 10 02 0D 01 01 04 28 03 02 02 64 02 0A 02 14 01 01 03 40 08 00
```

### 7.22 LIMITS（`0x88`、MCU → PC）

値: `{"boot_id":2309737967,"duty_max_percent":40,"duty_min_percent":3,"kp_percent_per_100":2,"abs_duty_max_percent":60,"ff_count_s_per_percent":100,"deadband_count":10,"deadband_out_count":20,"abs_max_vel_count_s":6000,"abs_max_acc_count_s2":40000}`

```text
frame (30 byte): 01 88 18 00 EF CD AB 89 28 03 02 3C 64 00 0A 00 14 00 00 00 70 17 00 00 40 9C 00 00 BF DB
wire  (32 byte): 04 01 88 18 0A EF CD AB 89 28 03 02 3C 64 02 0A 02 14 01 01 03 70 17 01 03 40 9C 01 03 BF DB 00
```
<!-- VECTORS:END -->
