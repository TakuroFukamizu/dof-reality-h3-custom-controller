# H3 Control Server — 初期アーキテクチャ・互換仕様案 v0.1

作成日: 2026-10-26
状態: 検討案。コード実装・実機検証・リポジトリ変更は未実施。

## 0. 判断の要約

- PC側のコントロールサーバーを独立プロセス `h3-control-server` とし、初期実装言語はRustを第一候補とする。Goでも同じ仕様を実装できる。言語だけでハードリアルタイム性は保証しない。
- ゲーム、描画、AI、音声、入力機器処理は既存h3game側に残す。サーバーはUSB接続を独占し、3軸協調・校正変換・状態管理・ログを担当する。
- 各CoreS3は位置制御とローカルの停止処理を担当する。PCからPWMをストリーム送信する構成にはしない。
- 既存 `pitch / roll / yaw` の−1〜1、`gain` の0〜1、および既存の軸混合式を互換契約として残す。物理角度やPWM値とは扱わない。
- h3gameは `bridge.py` の機器接続・送信・状態取得を小さいクライアント層へ差し替える。ゲームのシーンは変更しない。ゲームフレームの鮮度確認のため、index.htmlに少量のメタデータ追加を推奨する。
- 接続しただけで原点移動・中央復帰・ブレーキ解除・自動ARMを行わない。これらは元ドライバとの意図的な非互換点。

## 1. 調査した資料と確認範囲

### 実際に取得した既存ゲームのソース

リポジトリ: `TakuroFukamizu/20261004_shi3z_hackathon`、`main`。

| ファイル | 読み取り時のblob SHA | 確認事項 |
|---|---|---|
| h3game/h3_direct.py | 06bc7f0bdda2c5ebff5f9d323074b582629582a0 | クラスAPI、混合式、位置値、レート制限、旧シリアル書式 |
| h3game/bridge.py | 807b4d9c45e6ef04550a64e34ceaf89a01599c18 | HTTP、owner、60Hz再送、入力タイムアウト、再接続、AI等 |
| h3game/web/index.html | 55827600269b6b840b43f864718d19d7ac4b68ca | 30Hz POST、反転、gain、ブラウザー内平滑化 |
| h3game/web/game.js | 0f0621d1bedd76194a46af02da40bb26eb9566f0 | シーンの戻り値とsmooth |
| h3game/README.md | 0cb026450f28c0c87212f2a9a13afb9d175fcf44 | 既存プロセス・ゲーム構成 |

上記はコードから確認した仕様。コードのコメントにある旧H3内部ファームの種類や15秒の挙動は、現行の置換機の保証仕様には採用しない。

### 実機情報

ユーザー提供 `H3_Feedback_Sensor_Results_20261025.md` を使用。対象はモーター#1。モーター軸約400 count/rev（4逓倍）、方向、無負荷可動範囲、給電問題などが記録されている。軸2・3や着座負荷に外挿しない。

`dof-reality-h3-custom-controller` は今回GitHub接続でtree/README取得が404、公開Webでも取得できなかった。したがって現行ファームのソースレビューは未実施であり、本書のPOSITION等は新規要求仕様である。リポジトリの公開・非公開状態や原因は断定しない。

## 2. 既存h3gameの契約

### 2.1 ゲームの値

シーン `update(dt, inp)` は `{pitch, roll, yaw, smooth}` を返す。

- pitch / roll / yaw: −1.0〜+1.0の無次元の動作指令。度、rad、RPMではない。
- gain: 振幅倍率。bridgeが0〜1へ制限。既定0.35。現画面のスライダーは0.1〜0.6。
- smooth: ブラウザー内の平滑化係数。既定0.12。HTTPの席指令には含まれない。
- 画面で軸反転を適用してから送信する。サーバーで同じ反転を二重適用しない。

ブラウザーはHTTP `POST /` を30回/秒程度で送る。

```json
{"owner":1,"pitch":0.3,"roll":-0.2,"yaw":0.0,"gain":0.35}
```

`GET /state` は1秒ごとの画面表示。`GET /reconnect`、`GET /claim`も存在する。`/pads`、`/ai`、`/sync`、`/frame`、`/input.json`、静的配信はゲーム側の責務として残す。

### 2.2 現行変換式

```text
CENTER = 512
g = gain * 511
m1 = 512 + g * (roll + pitch)
m2 = 512 + g * (roll - pitch)
m3 = 512 + g * yaw
```

この和・差を「一般的には逆だから」と変更しない。旧装置のモーター取付方向と対応した既存アプリの契約である。新しいカウント座標との向きは各軸の校正で吸収する。

`set_motors` は各送信で内部curを最大40だけ目標に近づけ、その後 `_send` がint化・0〜1023へのクリップを行う。内部curはクリップ前の値を保持するため、飽和時の戻り方にも影響がある。全入力域での動的な完全一致を不用意に宣言しない。

旧シリアル出力は500000 baud、`[` + A/B/C + 上位byte + 下位byte + `]`。これは旧H3バックエンドのプロトコルであり、新CoreS3へ踏襲する必要はない。

**同じ0〜1023でも、旧APIの値は目標位置であり、MD13Sの10bit PWM Dutyではない。型と名称を明確に分ける。**

### 2.3 互換範囲と非互換範囲

維持: フィールド名、無次元レンジ、gainの意味、混合式、既存画面/シーンの入出力、DRY。

意図的に変更:
- USBを開いた直後の自動centerをしない。
- 再接続しても自動ARM・自動運転再開しない。
- close時に無条件でcenterへ移動しない。
- owner番号だけで実機駆動権限を与えない。
- 非有限値を拒否する。範囲外を互換モードでクリップする場合も制限発生を報告する。
- 旧60Hz senderの再送でゲーム入力の鮮度を延長しない。
- 旧auto_swayは実機既定無効。明示的なデモセッションだけ許可する。

## 3. プロセスと責任分界

### ゲーム側

Chrome/ゲーム、Python bridge.py、joy_reader.pyは既存用途を維持する。AI、音声、画像転送をコントロールサーバーに入れない。

### コントロールサーバー

`h3-control-server`は以下を持つ。

1. Game API: 既存正規化指令の受信、送信元セッション、入力鮮度。
2. Compatibility: h3game-v1混合式、旧0〜1023座標。
3. Calibration: 軸のID対応、向き、中心、ソフトリミット、カウントへの変換。
4. Planner: 3軸共通の時間軸での速度・加速度・必要ならjerk制限。
5. Rig Supervisor: 全軸準備、主電源、開始/停止、異常ラッチ。
6. Axis Links ×3: USBの独占、パケット送受信、最終適用frameの確認。
7. Telemetry/Recorder: 読み取り状態、遅延統計、制限理由、障害前後ログ。
8. Admin API/CLI: ARM、停止、校正、テスト。ゲームのPose APIと分離。

### CoreS3側

PCNT計数、位置制御、PWM生成、局所速度/電流/追従監視、ブレーキシーケンス、期限切れ停止を担当する。通信や描画に位置制御周期を依存させない。

PCサーバー停止時も、各軸が自律的に安全停止へ進む。機械的に結合した3軸の共通停止はソフトだけでは保証せず、物理的な共通停止経路も必要。

## 4. 最小改修でのh3game接続

既存 `bridge.py :8891` を残す。新サーバーは暫定 `127.0.0.1:8892` とし、ポートは設定可能にする。

追加 `motion_client.py` が、WebSocketの持続接続と最新1件の送信スロットを持つ。HTTPハンドラーはUSBやサーバー応答を同期的に待たない。

bridge.pyの改修箇所:

- connect/watcher: 旧USB探索でなくサーバー接続と状態取得。自動ARMしない。
- do_POST: owner検査後に受理したPoseのスナップショットをクライアントへ渡す。gainのみ更新はゲームPoseの生存更新と区別する。
- sender: 旧USBへの60Hz再送はserverバックエンドでは無効。平滑化・タイムアウト・autoの責務を二重化しない。
- /state: 既存のキーを残し、別途armed/calibration_valid/fault等を追加。pitch等は送信指令値であり、実測角度とは表示しない。
- /reconnect: 通信再確立のみ。再運転は明示操作。
- H3_DRY=1: サーバーへの駆動要求を出さない。

`H3_BACKEND=legacy|server|dry` を提案。旧 `h3_direct.py` は削除せずlegacy用に残す。serverモードでimportしただけでシリアルを開かない。

最小修正の代替としてH3クラス互換のRPCプロキシも可能だが、それ単体ではbridgeの古い指令再送と自動center問題を解決しないので第一案としない。

### ゲームフレームの鮮度

index.htmlはsetIntervalで送信し、描画はrequestAnimationFrameで進む。送信タイマーだけが継続した場合、凍結したcurを再送できる。

推奨する小変更:
- ゲーム更新の正常完了時にframe_seqを増加させ、frame時刻を記録。
- POSTにframe_seqと生成からの経過時間を追加。
- 同一frame_seqの再送、/stateのpoll、AI応答、USB keepaliveではゲーム指令の有効期限を更新しない。
- 静止目標でもゲームが正常に更新されればframe_seqは進む。「値が変わらない」を停止検知にしない。
- 再起動時は新しいsource_session。古いepoch/seqは拒否。
- source_ageをキュー待ち後に0へ作り直さない。

既存JSONだけも互換受理できるが、タイマー生存とゲーム描画生存を区別できない制約を明示する。有人運転用では鮮度メタデータを付ける方針。

## 5. ゲームAPI案

ゲーム内部のpitch/roll/yaw/gainは変えず、制御サーバーとの接続でメタデータを追加する。

```json
{
  "type": "pose",
  "protocol": 1,
  "source_session": "server-issued-session",
  "seq": 1250,
  "frame_seq": 2408,
  "source_age_ms": 3,
  "pitch": 0.3,
  "roll": -0.2,
  "yaw": 0.0,
  "gain": 0.35
}
```

seqはパケット順序、frame_seqはゲーム更新順序。異なるプロセスの単調時計を無条件に同じ起点として比較しない。source_ageとサーバー側受信時刻、必要に応じ時刻同期で期限を評価する。

gainはゲームの希望値。機械プロファイルの許可最大値を超えられず、effective_gainと制限理由を返す。旧UI上限0.6を、新装置の検証済み上限として採用しない。

主なAPI:

| API | 意味 |
|---|---|
| WS /v1/motion | Pose、観測用状態。運転許可自体は付与しない |
| GET /v1/status | 接続/準備/運転/異常/校正/実測カウント |
| POST /v1/session | 駆動クライアントの登録、単一所有権 |
| POST /v1/arm | 管理側の明示的な準備要求 |
| POST /v1/stop | 制御停止要求。物理非常停止とは区別 |
| POST /v1/disarm | 停止・保持引継ぎ後に駆動禁止 |
| POST /v1/fault/reset | 原因解消を検査してラッチ解除。自動再ARMなし |
| POST /v1/center | 校正有効・全軸正常時のみ制限付きで中立移動 |

運転入力、管理命令、テレメトリを別キューにする。安全停止を最新Poseによって上書きしてはならない。

## 6. 座標・校正

変換順:

1. 既存の無次元Poseを検査。
2. gainを一度だけ適用し既存混合式で旧仮想位置を得る。
3. 仮想位置を許可範囲へ制限。飽和の扱いはプロファイルで明示。
4. 軸ごとの校正テーブルで実機カウントへ変換。
5. 3軸共通の時間軸で軌道制限。
6. 各MCUへ位置目標を送る。MCUでも最終リミット検査。

仮想位置の0 / 512 / 1023に対する実機カウントを各軸で登録する。512が中立。カウントの増減方向は軸ごとに異なってよい。初期は3点の区分線形補間。必要なら追加校正点へ拡張する。

**0と1023は運用上のソフトリミットであって、実測した機械的端そのものではない。**

機械的端、減速域、運用端、中立を別フィールドにする。軸1の−2742/+2814をそのまま全軸へコピーしない。今回の床からの高さは1軸・1測定点・無負荷条件の結果で、H3全体のpitch/roll角変換には使用しない。

axis_idはUSB列挙順・COM番号でなく、MCU固有IDと物理軸を照合した設定で決める。交換機は手動登録・校正する。重複ID・未知IDがあればARM禁止。

インクリメンタル位置は電源断で基準を失う。NVSに最後のcountを書くだけで校正有効としない。boot_id変更時は校正状態を無効化し、既知基準への復帰を要求する。端当て原点復帰は電流・機構・センサー異常を別途検証するまで標準の自動手順にしない。

## 7. 言語選定とスケジューリング

### 推奨: Rust

GCを持たないこと、状態/単位/所有権を型で分けやすいこと、通信と制御状態の排他をレビューしやすいことを理由に第一候補とする。Rustでもallocation、ロック、OSスケジュール、USB待ちは遅延要因であり、リアルタイムOS相当にはならない。

### Go

100Hz級の目標生成・3USBのI/Oに対し、言語だけを理由に不適としない。goroutine、GC、割り当て量を実測し、事前確保したバッファと上限付きキューで構成する。既存チームの実装速度が大きく有利ならGoも妥当。

### 実行構成

- ネットワーク・管理・ログ: 非同期ランタイム。
- プランナー: 単一所有の状態、固定サイズの3軸目標、単調時計。必要に応じ専用スレッドへ分離。
- USB: 軸別I/O。一軸のwrite待ちで残り全軸の送信を直列に待たせない。
- Pose: 最新値を1件保持。通常目標をFIFOで溜めない。
- 管理/停止: 有界キュー＋応答/期限。FAULTラッチは通常Poseより優先。
- ログ: 有界リング、非同期書込み。落としたログ件数も記録。
- 遅延後に古いtickを高速実行しない。Tokio intervalを使う場合、既定BurstでなくSkip等を明示する。

## 8. 周期と遅延目標（提案値、保証値ではない）

| 区間 | 初期目標 |
|---|---|
| 既存ゲーム→bridge | 既存の約30Hzを維持 |
| PCサーバーの3軸目標更新 | 100Hz / 10ms |
| 各MCUの位置制御 | 初期200Hz / 5ms案。実測で不足なら500〜1000Hzを検討 |
| MCU→PCテレメトリ | 50Hz / 20ms |
| 管理画面 | 10Hz |

ゲーム30Hzの33.3ms刻みを、PCの100Hz化だけで取り戻せるわけではない。既存ブラウザーのsmooth=0.12は60Hz基準で約130msの指数平滑化時定数に相当する。追加平滑化を重ねず、既存の演出と機械保護の制限を分離する。

計測指標:
- source生成からサーバー受信、プランナー採用、USB投入、MCU適用まで。
- p50/p95/p99/max、期限逸脱数、キュー深さ、テレメトリ欠落、3軸適用時刻差。
- MCU/PC時計を同期・評価できない場合、片道遅延をRTTの半分と断定しない。

初期ベンチの評価目標はサーバー受信→MCU適用p99 20ms以内、3軸適用差p99 5ms以内を候補にする。実測がなければ達成済みとしないし、機械安全の根拠にも使わない。

ソース期限250ms、MCU指令期限100ms等はベンチ用の候補に留め、実機停止距離・揺れ・PC負荷を見て確定する。測定ツールのLEASE_MS=2000を本番の停止期限として無条件に継承しない。

## 9. USBプロトコル案

既存のSET/RUNは保守・検証用に残せるが、本番はPOSITIONモードを追加する。モード変更はDISARMED時のみ。

メッセージ候補:
- HELLO / CAPABILITIES / STATUS
- CONFIG / CONFIG_ACK
- ARM_PREPARE / READY / START
- SET_POSITION
- CONTROLLED_STOP / DISARM / FAULT / FAULT_RESET
- CLOCK_SYNC（時刻指定適用を実装する場合）

HELLO: device_id、axis_id、boot_id、fw_version、protocol_version、capabilities、calibration_id/hash、状態。

SET_POSITION: session_epoch、seq、group_frame_id、target_count、速度/加速度の上限、期限、必要なら同期されたMCU時計上のapply_at。

固定長または最大長を定めたバイナリフレーム、長さ、メッセージ種別、CRC、再同期可能なframingを採用する案。数値のサイズ・符号・endianは実装前に固定する。プロトタイプのテキスト通信でも、処理時間・バッファ上限・期限を計測すれば比較できる。

目標の古い再送は禁止。管理命令の再送はrequest_idで重複実行を防ぐ。ACKは「受理」と「適用」を区別し、定期STATUSにlast_applied_frameを含める。USB write成功を実機適用確認としない。

### 3軸同期

初期MVP: 同一group_frame_idを3軸へ配信し、実適用のずれを測る。seqが同じだけで同時適用できたと呼ばない。

改善: PCと各MCUの単調時計のoffset/RTT/不確かさを推定し、未来のapply_atへスケジュール。時刻同期が無効/期限切れなら指定適用を拒否する。

一軸だけ未受信・未適用となることを通信だけで完全排除はできない。全軸のACK待ちで毎フレーム停止する設計も採らない。許容遅延・frame差超過時に全体停止へ遷移し、共通ハードウェア許可信号で補完する。

## 10. 状態・安全契約

全体状態案:

```text
OFFLINE → CONNECTED_DISARMED → CALIBRATION_REQUIRED / READY_TO_ARM
READY_TO_ARM → ARMING → HOLDING → ACTIVE
ACTIVE / HOLDING → STOPPING → BRAKED_DISARMED
異常 → FAULT_LATCHED
```

- 3軸ID、校正、電源状態、停止系がそろわなければARMしない。
- HOLDINGは位置制御で支える状態。PWM=0を位置保持と見なさない。
- 位置制御を確立してブレーキ解除へ進む移行と、ブレーキ保持へ戻す移行は単軸で検証する。
- ブレーキや主電源をフレームごとにON/OFFしない。
- ゲームだけ途絶: 校正・全軸フィードバックが健全なら協調減速・保持・ブレーキへ。既定で無条件の中央復帰を行わない。
- 一軸の追従異常・センサー無効・USB断: 全体FAULT。残軸へ停止を送り、届かない軸はローカル期限切れで停止する。PCだけで同時停止を保証しない。
- サーバークラッシュ・OSスリープ: MCU側期限切れが働く。復帰後は新epoch、再ARM必須。
- 非常停止: 物理経路を優先。サーバーは状態記録と再起動禁止を担う。APIのstopを安全認証済み非常停止と表示しない。
- STOP/FAULT後、ネットワークの古い目標が遅れて届いても駆動を再開しない。

### 共通主電源の所有権

既存のDC_STOP1緑は共通主電源リレーの入力。3台のG2を並列直結しない。

一台をpower-controller役に指定し、他の軸のG2操作を無効化するか、独立した共通許可回路へ集約する。サーバーで3軸準備をまとめても物理インターロックを代替しない。power-controller切断時の全軸ブレーキ/電源シーケンスは実機検証対象。

各軸のG1は軸別ブレーキ解除指令。既存装置の一括ブレーキ指令とは異なる新仕様である。

## 11. 監視・設定

最低限の状態:
- connected / ready / armed / mode / calibration_valid / fault
- requested_pose、effective_gain、virtual_targets、target_counts、actual_counts
- velocity_count_s、following_error、PWM、ブレーキ解除「指令」
- source_age、USB遅延、last_applied_frame、clock_sync品質
- 電流は接続/校正済みのときのみAとして表示。未接続を0Aと表示しない。

実測角度は未校正なのでnull。指令pitch/roll/yawを実測角度と呼ばない。

設定はversion付きTOML/JSON、ハッシュ・機体ID・軸IDを付ける。校正、符号、各リミット変更はDISARMED限定。数値単位はcount、count/s、count/s²、msを名称にも含める。

初期はloopback限定。管理APIにセッショントークン、ブラウザーOrigin検査、入力長制限、rate limitを設ける。ownerの大小は認証ではない。LANの観戦通信に駆動権限を付けない。

## 12. テストと導入順序

### 段階1: 実機なしの互換試験

既存h3_direct.pyを参照モデルに、混合式・量子化・飽和・反転・gainをテストする。

十分に収束したときの旧送信値例（gain=0.35、int化後）:

| pitch | roll | yaw | m1 | m2 | m3 |
|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 0 | 512 | 512 | 512 |
| 1 | 0 | 0 | 690 | 333 | 512 |
| 0 | 1 | 0 | 690 | 690 | 512 |
| 0 | 0 | 1 | 512 | 512 | 690 |

初回送信はmax_stepで異なる。時系列テストも用意する。安全制限で変わる箇所は不一致を隠さず意図的な差分として報告する。

### 段階2: サーバーとゲームの結合

USBの代わりに3軸fake backend。h3gameの全シーン、未接続、DRY、owner切替、ゲームフリーズ、AI待ち、サーバー再起動、ブラウザー休止を検証。

### 段階3: 単軸位置制御

位置目標追従、保持、ブレーキ引継ぎ、ソフトリミット、通信期限切れを無人試験。テスターのRUNが動いたことを位置制御完成と見なさない。

### 段階4: 軸2・3の独立校正

物理方向、中立、制限範囲、速度、保持を各々取得。軸1の値をコピーしない。

### 段階5: 3軸統合

低振幅の単一pitch/roll/yawから、複合姿勢、3軸同期差、CPU/GPU高負荷、USB抜去、遅延、パケット破損、センサーGND断、停止・電源再投入まで確認する。接続順を変えても同じ軸に対応することを試験する。

有人運転の可否は別判定。物理停止、電源/GND、荷重保持、停止距離の確認をソフトの完成で代替しない。

## 13. 推奨配置

```text
dof-reality-h3-custom-controller/
  moter-controler/                 # 既存。維持
  encoder-tester/                  # 測定用途。維持
  axis-controller/                 # 新規: 位置制御・停止状態機械
  control-server/                  # 新規: Rustバイナリ
    src/api/
    src/compat/
    src/motion/
    src/supervisor/
    src/axis_link/
    src/telemetry/
    config/
    tests/
  clients/python/                 # 新規: h3game向けクライアント
  docs/control-server/            # プロトコル・校正・試験結果
```

H3ゲーム側はbridge.py、起動スクリプト、鮮度メタデータ用index.htmlの小変更を中心とし、scene_*.jsの動作指定値を維持する。

## 14. 参照

- https://github.com/TakuroFukamizu/20261004_shi3z_hackathon/blob/main/h3game/h3_direct.py
- https://github.com/TakuroFukamizu/20261004_shi3z_hackathon/blob/main/h3game/bridge.py
- https://github.com/TakuroFukamizu/20261004_shi3z_hackathon/blob/main/h3game/web/index.html
- https://github.com/TakuroFukamizu/20261004_shi3z_hackathon/blob/main/h3game/web/game.js
- H3_Feedback_Sensor_Results_20261025.md（ユーザー提供）
- Rust公式: https://rust-lang.org/
- Go GC Guide: https://go.dev/doc/gc-guide
- Tokio missed ticks: https://docs.rs/tokio/latest/tokio/time/enum.MissedTickBehavior.html
- Tokio latest-value channel: https://docs.rs/tokio/latest/tokio/sync/watch/index.html
- Windows scheduling: https://learn.microsoft.com/en-us/windows/win32/procthread/scheduling-priorities

資料から確認した事実と本書の提案値は区別する。本書の周期・期限・性能目標・API名・ディレクトリ構成は設計案であり、現行実装の仕様や検証済み性能ではない。
