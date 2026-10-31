# ソフトウェア設計

## 全体像

```text
Motion Software / Telemetry
           │
           ▼
   h3-control-server（PC、Rust）
           │  Pitch / Roll / Yaw → 各軸の目標位置（Kinematics）
    ┌──────┼──────┐
    ▼      ▼      ▼
  Axis1  Axis2  Axis3     USB CDC ×3
    │      │      │
 CoreS3 CoreS3 CoreS3     各軸で位置制御ループを完結
```

- 3 台は同一ファームウェア（軸ごとの設定を持たない）。担当する系統（軸 1〜3）は **MCU 側で選択** し（電源ボタン操作、NVS に保存。切り替えは DISARMED のときだけ）、HELLO と毎回の STATUS の `selected_axis` で PC に知らせる。PC はこの値で軸を割り当てる（COM ポート番号に依存しない）。PC の設定ファイルの `device_id`（eFuse MAC 由来）は「この軸はこの個体のはず」という照合用で、違えば ARM しない。同じ系統を 2 台が選ぶと PC はその軸を競合として止める（[control-server/protocol.md](../control-server/protocol.md) §1.1）。制御パラメータ（ソフトリミット・速度・加速度・期限など）は毎 boot、PC の CONFIG で受け取る。
- 通信断時は各軸が自律的に停止する。PC からの指令には seq と group_frame_id を付ける。v1 は適用時刻（`apply_at_us`）を使わず受信時に適用し、3 軸のずれは STATUS の `last_applied_frame` で監視する。USB だけでは厳密な時刻同期にならないので、同期誤差を測定して許容できるか確認する。

## ESP32-S3 のコア分担（案）

| CPU | 担当 | 優先度 |
| --- | --- | --- |
| Core 0 | USB 通信、画面表示、ログ、設定。Core 1 の制御状態と通信タイムアウトの監視 | 通常 |
| Core 1 | エンコーダ計測、位置制御、PWM 生成、異常監視 | 高 |

当初は「ブレーキなど安全系だけを別コアに」という案だったが、位置制御と異常検出を同じ高優先度タスクにまとめた方が停止判断の遅延を管理しやすい。
両コアは電源・メモリ・割り込みを共有するため、ハードウェアのウォッチドッグと独立した駆動遮断手段は別途必要。

USB（Core 0）で受けた管理命令は、制御側（Core 1）の状態遷移を確定させてから ACK を返す。ACK の後に送る STATUS は送信時点の状態から作り、命令前に取ったスナップショットを送らない（[protocol.md](../control-server/protocol.md) §1。PC はこの順序に依存している）。

### タスク構成（案）

| タスク | 周期 | 内容 |
| --- | --- | --- |
| Encoder Acquisition | ハードウェア計数 | PCNT による位置カウント |
| Position Control | 1kHz 目標 | PID（最初は P + 速度制限 + デッドバンド、必要なら D を追加）、速度・加速度制限 |
| Safety Monitor | 1〜10ms | 追従誤差、過速度、通信断、制御周期異常 |
| USB Communication | 100Hz 目標 | 目標位置受信、状態送信 |
| Display | 10〜20Hz | ステータス表示 |
| Logging | 非同期 | 異常履歴、測定データ |

UI や USB の周期で GPIO をポーリングして位置を数えない。`delay()` や全画面描画で制御周期・駆動期限監視をブロックしない。

## エンコーダ計数（PCNT）

- ESP32-S3 の PCNT を 1 unit・2 channel で使い、edge = A / level = B と edge = B / level = A を組み合わせて **方向付き 4 逓倍**（1 電気周期で ±4）。ESP-IDF の `rotary_encoder` 例が基準。
- 白 = A、緑 = B と仮称し、`ENCODER_SIGN = ±1` で表示・計算の方向を統一する。正方向の定義は実機で決める。
- ハードウェアカウンタの上下限（例: ±30000）に watch point を置き、`accum_count` で跨ぎを補償する。アプリ側の累積位置は符号付き 64bit。
- 周期的に PCNT を clear して差分を取る方式は禁止（clear 前後のエッジ欠落）。ZERO は表示オフセットのみ。
- glitch filter は設定項目にし、初期は無効。値と適用状態を画面・ログに出す。計測中は変更禁止。
- PCNT 単体では不正遷移やパルス欠落をすべて検出できない。`ERROR = 0` を正常保証として表示しない。A/B が両方 HIGH でも静止と断線を区別できない。
- 速度は 100ms 程度の窓で `count/s` を算出。RPM・角速度は校正が揃うまで N/A。
- 新旧 PCNT API（`driver/pulse_cnt.h` と旧 API）を混在させない。Arduino-ESP32 3.x / ESP-IDF 5.x 系と M5Unified が両立する環境で依存バージョンを固定する。

## PWM 出力（MD13S）

| 項目 | 値 |
| --- | --- |
| 周波数 | 20kHz（`PWM_FREQ_HZ`） |
| 分解能 | 10bit（0〜1023） |
| 方式 | sign-magnitude。速度 0 で PWM Low |
| DIR | `DIR_POSITIVE_LEVEL = 0`。positive = DIR Low、negative = DIR High |
| 反転 | PWM を 0 にして `DIR_COAST_MS = 5` 待ってから DIR を変える。非ゼロ PWM のまま DIR 反転しない |
| 主電源投入後 | `RELAY_SETTLE_MS = 100` は PWM を出さない |

Duty の意味を曖昧にしない。UI・ログは符号付き Duty [%] に正規化し、LEDC の raw 値も併記する（5% ≒ 51/1023）。初期プログラムで「50〜60 で急に端まで動いた」値は、ブレーキが掛かっていた可能性があるため始動値として流用しない。

## 状態機械

本番の状態機械は [protocol.md](../control-server/protocol.md) §3.1・§5 を正とする（以前の案「DISARMED → READY → ARMED → MOVING → STOPPING → READY」は使わない）:

```text
BOOT → DISARMED → ARMING → HOLDING ⇄ ACTIVE（ACTIVE → STOPPING → HOLDING）
DISARM: → DISARMING → DISARMED
異常はどの状態からでも FAULT ラッチ（PWM 0・ブレーキ作動）→ FAULT_RESET で DISARMED（自動再 ARM しない）
```

`encoder-tester` で実装している単軸試験用の遷移:

```text
DISARMED → POWER_WAIT → BRAKE_WAIT → READY → RUNNING → STOPPING → BRAKE_APPLY → DISARMED
                                                   異常 → FAULT（ラッチ）
```

| 段階 | 内容 |
| --- | --- |
| ARM | 操作者が無人・機構支持・可動余裕・他軸切離し・物理停止手段を確認して送る |
| POWER_WAIT | PWM = 0 のまま主電源リレーの GPIO で ON 指令。`POWER_SETTLE_MS` 待つ。電圧の実測入力はないので「電源確認済み」とは表示しない |
| BRAKE_WAIT | ブレーキ解除リレーの GPIO で解除給電指令。`BRAKE_RELEASE_WAIT_MS` 待つ。ピン番号は [hardware.md](hardware.md) の割当表 |
| READY | `READY_HOLD_MS` 内に RUN / JOG がなければ停止シーケンスへ。ブレーキ解除状態で無期限に待たない |
| RUNNING | 時間制限付き（`MAX_RUN_MS`）、Duty 上限（`MAX_DUTY_PERCENT`）、相対移動上限（`MAX_TRAVEL_COUNTS`）、無変化タイムアウト。自動反復・自動 Duty 増加・自動延長はしない |
| STOPPING | PWM を 0 にし、`count/s` が `STOP_SPEED_THRESHOLD` 以下で `STOP_CONFIRM_MS` 継続したら停止候補。`STOP_TIMEOUT_MS` で異常停止 |
| BRAKE_APPLY | ブレーキ給電 OFF、`BRAKE_APPLY_WAIT_MS` 待ってから主電源 OFF |
| FAULT | PCNT 初期化失敗、通信維持期限切れ、制御周期異常、カウント上限超過などでラッチ。再始動は明示的な FAULT 解除と再 ARM |

NVS に ARM / ON / JOG 状態を保存して自動復帰させない。USB 再接続・リセットで自動運転しない。起動時に `while (!Serial)` で無期限に待たない。

## USB プロトコル

PC（h3-control-server）との本番用プロトコルは [control-server/protocol.md](../control-server/protocol.md)（v1、PC 側実装済み・CoreS3 側未実装）を正とする。下の「本番向け（案）」はそれ以前の案。ファームが実装すべき項目は [control-server/README.md](../control-server/README.md)「CoreS3 側でやること」、参照する挙動は `control-server/src/sim_axis.rs`。

### 本番向け（案）

| メッセージ | 内容 |
| --- | --- |
| HELLO | device_id、選択中の系統（`selected_axis`）、FW バージョン、状態 |
| ARM / DISARM | 駆動準備要求 / 駆動停止 |
| SET_TARGET | 目標位置、速度・加速度制限 |
| STATUS | 実位置、目標位置、PWM、ブレーキ状態 |
| FAULT / RESET_FAULT | 異常コード / 異常解除要求 |

### `encoder-tester` の行コマンド（実装済み・案を含む）

```text
HELP / STATUS / CONFIG
SET <KEY> <VALUE>
ARM / DISARM / STOP / FAULT RESET
RUN duty_pct=<n> direction=positive|negative duration_ms=<ms>
JOG DUTY <signed_percent> DURATION <ms> / KEEPALIVE
ZERO / MARK START / MARK END
CAL ANGLE <deg> / CAL REVS <n>
VISUAL CONFIG ... / VISUAL MARK_START / VISUAL MARK_END / VISUAL CANCEL / VISUAL ACCEPT
STREAM ON|OFF / TRIAL SAVE / TRIAL EXPORT
```

- USB 操作はハートビート（`LEASE_MS`）を必須とし、権限が切れたら駆動を終える。STOP / DISARM は他コマンドより優先し冪等に処理する。
- READY 前に受けた RUN をキューして後から実行しない。過去セッションの ARM / JOG を再接続後に実行しない。
- 応答は機械可読（JSON または明確な接頭辞）。CSV ストリームと `EVT` 行、`#` コメント行を分離する。

### ログ CSV（`encoder-tester`）

```csv
seq,t_us,trial_id,axis,state,power_on_cmd,power_gpio_level,brake_release_cmd,brake_gpio_level,
requested_command,requested_unit,applied_duty_pct,pwm_raw,pwm_full_scale,pwm_hz,dir_level,
count_total_x4,count_trial_x4,velocity_count_s,a_level,b_level,elapsed_ms,stop_reason,log_drop_count
```

既定 50〜100Hz。各エッジを USB へ送らない。時刻は単調増加 64bit µs。リングバッファで欠落数を記録。PC 側は Python（pyserial）でポート列挙・保存・上書き防止・解析（Duty / count/s / count 対時間）を行う。

## 分解能測定モード

[既存構成側の記述](../h3-original/motor-and-feedback-sensor.md#分解能の求め方)の計算を実装する。

| モード | 内容 |
| --- | --- |
| 既知角度 | START → 外部の目印・角度計で測った角度だけ電動で動かす → END → `CAL ANGLE` |
| 既知回転数 | 安全に回せる軸で N 回転 → `CAL REVS` |
| 目視 1 回転（VISUAL_REV） | 停止位置合わせ方式（既定）: 短い電動ジョグで目印を基準線に合わせて開始を記録し、同方向にジョグを繰り返して 1 回転後に終了を記録。通過マーク方式（補助）: 低速回転中に目印の通過で START / END + 停止 |

- 駆動の状態機械と測定の状態機械（IDLE → ALIGN_START → START_MARKED → TRAVERSING → END_MARKED → REVIEW → SAVED）を分離し、ジョグごとの DISARMED で測定開始点を消さない。
- 逆方向で合わせ直した試験は `direction_reversal = true` として自動採用しない。
- PWM と時間だけから PPR は求まらない。外部基準がない試験は「駆動応答試験」として保存し、分解能は UNKNOWN。
- 新しい校正値を位置制御や安全上限に自動適用せず、確認操作を要求する。

## 画面

故障診断用ダッシュボードとして設計する。

```text
┌─────────────────────────┐
│ AXIS 1       READY      │
│ Position      +12345    │
│ Target        +12500    │
│ PWM     32.5%  DIR CW   │
│ Encoder OK  Power ON    │
│ Brake RELEASE(CMD)      │
│ USB OK  Loop 1000Hz     │
│       DISARM            │
└─────────────────────────┘
```

- 電源・ブレーキは実状態を読み返す入力がないため「ON 指令」「解除指令中」と表示し、実状態と区別する。
- 異常時は `ENCODER_FAULT` / `FOLLOWING_ERROR` / `COMMAND_TIMEOUT` / `POWER_FAULT` など、protocol.md §3.2 の異常コード名で原因を明示する（画面上の短い別名を足すのはよい）。
- 試験ツールでは `NO CURRENT MONITORING / POSITION NOT HOMED` を常時表示。

## 現状の実装

### `moter-controler/`

CoreS3 SE 1 台で MD13S・主電源リレー・ブレーキリレーを動かす最初のプログラム（PlatformIO、espressif32 6.12.0、M5Unified 0.2.x）。

- タッチ UI: 電源 入/切、ブレーキ 入/切、速度 −10 / −1 / +1 / +10、停止、非常停止。速度ボタンは電源 ON 時のみ有効。
- シリアルコマンド: `R 0|1`（RELAY）、`B 0|1`（BRAKE）、`S -100..100`（SPEED）、`X`（STOP、リレーは維持）、`E`（ESTOP、速度 0・電源 OFF・ブレーキ OFF）、`?`（STATUS）、`HELP`。
- 起動時: 全出力ピンを安全側（リレー OFF、DIR 正、PWM 0）で駆動してから `M5.begin()`。外部 I²C（G1 / G2）の機器探索と内蔵スピーカー・マイク（G13 / G14 を I²S に使う）を無効化。G2 は主電源リレー、G13 はブレーキ解除リレー。
- ピン割当は `include/config.h`。2026-10-31 に Base15 向けへ変更（ブレーキ G1 → G13、`PIN_VIOUT = 10` を追加。VIOUT の ADC 読み取りは未実装）。ブレーキ解除の論理は `BRAKE_ON_LEVEL`、主電源は `RELAY_ON_LEVEL` と別定数にし、Base15 内の MOSFET が反転段でもブレーキ側だけを変えられるようにした。
- 速度は要求値と適用値を分け、リレー ON 後 100ms は PWM を出さない。

### `encoder-tester/`

Grok Build に指示書（v3）を渡して作成した単軸試験ツール。`moter-controler/` とは独立したフォルダ。`tools/run_once.py` が USB 権限取得 → SET → ARM → READY 確認 → RUN 1 回 → DISARMED まで監視・記録を一続きに行う。実施記録は `encoder-tester/docs/runs/`、生ログは `encoder-tester/logs/`。

### PC 側 `control-server/`

PC 側のコントロールサーバー（Rust）。上の USB プロトコル v1 の PC 側と、MCU 側の参照モデル（模擬軸）を実装済み。CoreS3 ファームウェアのプロトコル部分はまだ無い。詳細は [control-server/README.md](../control-server/README.md)。

## 開発の進め方

- 1 台で単軸の動作を確立してから、同じ構成を残り 2 台へ展開する。
- 工程の目安（当初の 2 週間計画）: I/O 解析 → 単軸ベンチ（PWM/DIR、エンコーダ）→ 閉ループ位置制御・異常停止 → 3 台展開・PC 統合 → 実機の可動範囲・速度・停止試験 → 長時間試験・予備機。
- 単軸の閉ループ制御が安定しない場合、イベントでの有人運転は見送り、無人・低速の限定デモを代替目標にする。
- Kinematics は概念的に `T1 = C + aP + bR`、`T2 = C + aP − bR`、`T3 = C + cY`（P: Pitch、R: Roll、Y: Yaw、C: 中心）。係数は実機のリンク機構から求める。
