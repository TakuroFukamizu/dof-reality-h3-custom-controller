# 進捗と未確定事項

## 進捗（2026 年 10 月時点）

| 項目 | 状態 |
| --- | --- |
| 既存構成の調査 | 電源基板・制御基板・MD13S・Protection Board・エンコーダ配線・ブレーキ方式を実測で特定（[h3-original](../h3-original/README.md)） |
| CoreS3 1 台で MD13S 駆動 | `moter-controler/` で PWM/DIR 動作確認。ブレーキ未解除で「50〜60 で急に端まで動く」挙動あり |
| 主電源リレー・ブレーキリレーの CoreS3 制御 | DIN Base 構成で G2（主電源）、G1（ブレーキ）で配線・動作確認。24V 給電でブレーキが解除されモーターがスムーズに回ることを確認。Base15 構成ではブレーキを G13 に変更（未確認） |
| エンコーダ入力回路 | DIN Base 構成で AE-LCNV4-MOSFET 経由で Port.C（G17/G18）に接続し計数確認。Base15 構成ではレベル変換を Base15 内に移し XH 8pin 経由にする（未製作） |
| 単軸試験ツール `encoder-tester/` | Grok Build で作成。`run_once.py` で ARM → RUN（Duty 5%、100ms、positive）→ DISARMED まで 1 回実施 |
| Protection Board VIOUT の読み取り | 未実装。G10（XH 8pin #6、Base15 内 10k/10k 分圧）の予定。`PIN_VIOUT` の定数のみ追加 |
| Bottom の Base15 化 | 2026-10-31 に方針決定・ドキュメントとコードを更新。Base15 は未購入・未製作、実機未確認 |
| 分解能測定 | モーター軸 400 count/rev（4 逓倍）を 2026-10-25 に実測。減速比・シート角度換算は未測定 |
| 閉ループ位置制御 | 未実装 |
| 3 台展開・PC 統合 | 2026-10-31 に CoreS3 3 台（Base15）を USB ×3 で PC に接続し、PC からシリアルで 2 軸同時の往復と 3 軸同時の 上限へ → 中立へ を実施（[moter-controler/docs/multi_axis_20261031.md](../../moter-controler/docs/multi_axis_20261031.md)）。制御サーバーとの統合は未実施 |
| 共通非常停止・主電源インターロック | 未設計 |

## 初回試験の結果（2026-10-22、`run_once_positive_20261022_230347`）

設定: `MAX_DUTY_PERCENT=5`, `MAX_RUN_MS=100`, `MAX_TRAVEL_COUNTS=200`, `POWER_SETTLE_MS=1000`, `BRAKE_RELEASE_WAIT_MS=300`, `BRAKE_APPLY_WAIT_MS=300`, `STOP_SPEED_THRESHOLD=0`, `STOP_CONFIRM_MS=200`, `STOP_TIMEOUT_MS=1500`, `READY_HOLD_MS=3000`, `START_GRACE_MS=200`, `MAX_COUNT_PER_S=10000`, `UI_TIMEOUT_MS=1000`, `LEASE_MS=2000`, `JOG_STALE_MS=150`, `JOG_DUTY=5`, `ENCODER_SIGN=1`, `GLITCH_NS=0`

| 時点 | 結果 |
| --- | --- |
| 主電源 ON 指令直後 | カウント 0 → +2 |
| RUNNING（約 101ms、PWM raw 51、DIR Low） | カウント 2 のまま変化なし |
| PWM 停止〜停止確認 | 変化なし |
| ブレーキ給電 OFF 直後（BRAKE_APPLY） | **カウント 2 → −258（差分 −260）** |
| 最終 | DISARMED、停止理由 duration、FAULT なし |

判断:

- ARM の準備・終了シーケンス（主電源 → ブレーキ解除 → READY → 停止 → ブレーキ → 主電源 OFF）はリレーまで届いている。
- 駆動中のエンコーダ応答は得られていない。「PWM が出た」はソフトが出力値を設定したことを示すだけで、MD13S 端子の波形は未確認。
- −260 カウントは RUN 中ではなくブレーキ切替時に発生している。positive で負になったからといって `ENCODER_SIGN` を反転させない。
- 切り分け候補: ブレーキ・リレー切替に伴う電気ノイズや GND 変動、切替時の実際の軸の動き、計数・ログ処理の更新順序。
- 次の試験は RUN なしで ARM → 自動 DISARMED を 1 回行い、PWM の影響を除いてリレー切替だけでカウントが変わるか確認する。glitch filter は現象を消すために最初から変更しない。

## 2026-10-25 の試験結果（`encoder-tester/docs/results_20261025.md`）

- SET 値を決め直し（`encoder-tester/docs/set_proposal.md`）、`tools/session.py` で dry → idle → 正逆駆動 → 1 回転測定を実施。
- フィードバックセンサーは 2 相直交インクリメンタル。**モーター軸 1 回転 = 約 400 カウント（4 逓倍）**。センサーが同じ回転子に直結なら 100 PPR 相当（内部歯車の有無は未確認）。3 回転で 1208 カウント。
- positive（DIR Low）でカウント負・シートが沈む。negative で正・持ち上がる。15 % negative 400 ms の停止差分は 4 回とも 372〜375。
- 10/22 の「ブレーキ切替で −260」は、GND 接続後の ARM のみ ×2 では再現せず（全区間 0）。駆動ありのブレーキ作動時は 0〜−25 が大半、+76 が 1 回。
- 10/22 に RUN 中のカウントが 0 だった件と同種の現象を本日も再現: CoreS3 と H3 側の GND 共通が切れるとリレーもセンサーも反応しない。
- 未解決: DIN Base 12V と USB の同時接続で約 85 秒後に CoreS3 が電源断する（GitHub Issue #1）。本日は USB のみ給電で回避。
- 可動範囲（無負荷）: 全ストローク 5540〜5556 カウント ≈ 13.9 モーター回転（2 回測って一致）。シート右手前の角の高さは下端 306 mm、水平 389 mm、上端 460 mm（154 mm）。0.030 mm/count（下側）〜0.021 mm/count（上端付近）の非線形。目視の水平は範囲中央とほぼ一致。
- 速度: 25 % で約 3000 count/s（モーター軸 7.5 rev/s）まで計数欠けなし。ブレーキは上端で 60 秒保持して滑りなし（無負荷）。
- 未測定: 減速比、シート角度あたりのカウント、着座荷重での中立、端当て時の電流。

## 2026-10-31: Bottom を DIN Base から Base15 に変更する方針

- 理由と Base15 の調査結果は [hardware.md](hardware.md)。Grove 直結できるのが MD13S だけで、リレー用 MOSFET とレベル変換を Bottom 内に収める必要があり、バッテリーと追加 Grove 端子が不要になったため。
- ピン変更: ブレーキ解除リレーを G1（Port.A 白）から **G13**（XH 8pin #5、Base15 内 MOSFET 経由）へ。VIOUT は G10（XH 8pin #6、Base15 内分圧）として `PIN_VIOUT = 10` を追加（ADC 読み取りは未実装）。主電源 G2、MD13S G9/G8、エンコーダ G17/G18 は番号変更なし（原案で空欄だった主電源の G2 と、G17 と重複記載だったエンコーダ B の G18 は、同日に G2 / G18 で確定）。
- `moter-controler` のコードと `docs/cores3-controller/` を更新済み。**Base15 は未製作で実機未確認**。10-22 / 10-25 の試験結果は DIN Base 構成のもの。
- Issue #1（DIN Base の 12V と USB 同時接続で電源断）の前提（バッテリー無しの DIN Base）は Base15 では無くなるが、Base15 の電源経路（MP1584 → M-Bus 5V、逆流防止ダイオード無し）で USB と DC を同時接続したときの挙動は未検証。PR #2 の pmiclog env は DIN Base 向けの調査記録として残す。
- G13 は CoreS3 SE 内部で I²S DOUT（AW88298）と共用。`internal_spk = false`・`external_speaker_value = 0` の維持が前提。ブート時は G1 同様に浮くので、G13 側のゲートにプルダウンが必要（Base15 の MOSFET は非反転で、2026-10-31 に G13 HIGH で解除を実機確認）。moter-controler ではブレーキ用に `BRAKE_ON_LEVEL` を分け、主電源の `RELAY_ON_LEVEL` と独立に反転できるようにした。

## 次の作業

1. Base15 の購入・製作: XH 8pin / 2pin ソケット、リレー駆動用 MOSFET（型番未定）、AE-LCNV4-MOSFET、VIOUT 分圧の実装。M-Bus はシルク番号ではなく位置で配線し、電源 OFF で導通確認。Grove Port.B を GND・5V・G9・G8 に手配線。
2. Base15 構成で G13 ブレーキ・G2 主電源・G17/G18 計数・G9/G8 駆動を単体ずつ再確認。起動中の G13 のレベルを確認。
3. RUN なしの ARM → DISARMED 試験で −260 カウントの再現性を確認。コードで PCNT 取得値・count_total・trial・coast の更新タイミングと状態遷移の順序を確認。
4. 切替時に軸が目視で動いたかを動画で確認。動いていなければセンサー 5V、シフタ 3.3V、GND 配線、白・緑の変換前後の波形を確認。
5. 駆動中にカウントが変化する条件を確認（Duty は結果を見て 1 段階ずつ）。方向（positive / negative）と機構の移動方向の対応を記録。
6. 外部基準（目印・角度計・動画）と対にして分解能を測定。目視 1 回転モードは安全に 360° 回せる軸にだけ使う。
7. VIOUT を XH 8pin #6 → 分圧 → G10 へ接続し、ADC 読み取りを実装して無電流時のゼロ点を記録しテスター値と照合。
8. 単軸の閉ループ位置制御（P + 速度制限 + デッドバンド）と異常停止試験（エンコーダ断線、USB 断線、MCU リセット、電源断）。
9. 残り 2 台へ展開、PC 側の 3 軸統合、共通非常停止・主電源インターロックの設計と検証。

## 未確定事項

### ハードウェア

- エンコーダの分解能、減速比、計測している軸、出力段、最大パルス周波数。
- ブレーキの消費電流、解除・作動時間、保持能力、動的制動の可否、電源遮断時の機構の挙動。
- 主電源リレー駆動（DC_STOP1 緑）の Active High/Low と Q1 の型番。GPIO 未接続時にリレーが OFF を維持するか。
- ACS71240 の感度・ゼロ点、FAULT 出力の使用有無。VIOUT の電圧範囲。
- 24V モーター用電源の仕様と出力コンデンサの残留エネルギー。
- CoreS3 への給電方式と、既存 5V・USB・Base15 DC 入力の間の逆流防止。Base15 の MP1584 出力と M-Bus 5V の間に逆流防止ダイオードは無く、USB と DC の同時接続時の挙動は未検証（Issue #1 は DIN Base 構成での記録）。
- GND 回り込み（USB ×3 で PC 経由）への対策の要否。

### Base15 化に伴う未確定事項（2026-10-31）

- XH 2pin #2 の先が「リレーモジュールの IN」か「電源基板の DC_STOP1 緑（Q1）直結」か。原案は両方を書いており、現行試験構成はリレーモジュール経由。
- Base15 V1.1 と CoreS3 SE の互換性の明示的な公式記載が見つからない（CoreS3 との組合せは Stack Screw Length 表に掲載。バス配列は同一）。
- Base15 のプロト領域の穴数と、本体 15 mm 内の部品高さクリアランス。MOSFET・レベル変換・XH コネクタが収まるか。
- リレー駆動用 MOSFET の型番、プル抵抗の位置・向き・値、XH 8pin #5 に出す信号の形（ドレイン出力かリレーモジュール IN 向けの論理か）。
- G13 のブート中の波形（ROM ブートローダーが一時的に駆動しないか）と、AW88298 DATAI の入力リーク電流（データシートに記載なし）。
- Base15 の Grove J3 / J4 のどちらを Port.B にするか。

### ソフトウェア・運用

- 3 台の時刻同期の誤差と許容範囲。
- 原点復帰の方法と可動範囲の認識。
- Kinematics の係数（実機のリンク機構から）。
- PID ゲイン（無負荷・低速試験から調整）。
- 非常停止時のブレーキ・主電源の作動順序。

## 購入・準備済みの部品

- AE-LCNV4-MOSFET（BSS138）: 使用中。
- AE-TXS0104E-BO: 交換評価用に保留。
- ブレーキ用リレーと駆動回路: 1 軸分準備済み。DIN Base 構成では G1 で制御。Base15 構成では G13 → Base15 内 MOSFET 経由（未製作）。
- 主電源リレー駆動: G2 で制御（試験用）。Base15 では XH 2pin #2 経由。
- Base15 産業用プロト基板モジュール V1.1、XH 8pin / 2pin ソケット、リレー駆動用 MOSFET: 未購入。
