# 配線

この表は、動作確認済みの `moter-controler` と、エンコーダ入力を合わせたもの。画面とログの `POWER ON CMD` / `BRAKE RELEASE CMD` は GPIO への指令であり、接点や電圧を測った結果ではない。

2026-10-31 に Bottom を DIN Base から **Base15 産業用プロト基板モジュール V1.1**（M5Stack K025-B）に変更した。理由は、Grove で直結できるのが MD13S だけで、Feedback Sensor・ブレーキリレー・電源リレーは Grove 以外の接続になること、リレー用 MOSFET とエンコーダ用レベル変換を Bottom 内に収めるとスペースが要りバッテリーを外すことになり、DIN Base のバッテリーと追加 Grove が無用になるため。2026-10-25 までの実測は DIN Base + Grove 配線（ブレーキは G1）で行った。Base15 構成での実機確認はまだ。

Base15 には XH 8pin と XH 2pin のソケットを増設し、それぞれリレー駆動の MOSFET、エンコーダ用レベル変換、または GPIO に接続する。Grove Port.B は MD13S 用に、DIN Base と同じピン配置で Base15 に実装する。

## Base15 について（調査結果）

確認済み（出典: M5Stack 公式 docs と回路図 Sch_BASE_v1.1.pdf）:

- 電源は DC ジャック → PPTC ヒューズ → MP1584EN 降圧（入力 9〜24V、出力 5V・連続 2A）→ M-Bus pos 28 の 5V。CoreS3 へはこの BUS 5V で給電する（DIN Base と同じ経路）。バッテリーと充電回路は無く、pos 30 BAT は素通し。
- M-Bus ヘッダ J1 は 30 ピン全部が受動的に引き出され、各ピンの外側に 1 対 1 のハンダパッドがある。シルクと回路図の番号は初代 Core（ESP32）の GPIO 番号なので、CoreS3 SE では**位置で読む**（下表）。
- 生の入力電圧（9〜24V）が pos 25/27/29（HPWR）と P1 パッド（12V+/12V-）に出ている。CoreS3 SE 側では NC だが、Base15 上は活線パッド。MOSFET・レベル変換・XH の配線時に短絡させない。
- Grove 2 ポート（J3 UART 型 / J4 I2C 型）は回路図上は隣接 4 ピンパッド P3/P4 に繋がるだけで、バスの GPIO・5V・GND には未配線。HY2.0-4P コネクタ 2 個は同梱・未実装。Port.B 相当にするには GND・5V（pos 28）・G9（pos 10、黄）・G8（pos 4、白）を手配線する。
- 公式の Stack Screw Length 表に「Core2/CoreS3 + Module + Base15」の組合せがある。

未確認:

- CoreS3 **SE** と Base15 の互換性の明示的記載は公式ページに見つからなかった（バス配列は CoreS3 と同一）。
- Base15 の 5V と USB の同時給電時の挙動。回路図上、MP1584 出力と BUS 5V の間に逆流防止ダイオードは見えない。
- プロト領域の穴数と、本体 15 mm 内の部品高さクリアランス。実物で確認する。
- 高さの表記は資料間で不一致（28 / 15 / 21.5 mm）。
- Switch Science の製品ページ（products/8661）は取得できず、日本語ページ固有の記載は未確認。

### CoreS3 SE の GPIO と Base15 パッドの対応

| CoreS3 SE GPIO | M-Bus 位置 | Base15 シルク | 用途 |
| --- | --- | --- | --- |
| G10 | 2 | 35 | Protection Board VIOUT（分圧後、ADC1_CH9） |
| G8 | 4 | 36 | MD13S DIR（Port.B 白） |
| G9 | 10 | 26 | MD13S PWM（Port.B 黄） |
| G18 | 15 | 16 | エンコーダ B |
| G17 | 16 | 17 | エンコーダ A |
| G2 | 19 | 2 | 主電源リレー IN |
| G13 | 23 | 15 | ブレーキ解除リレー IN |
| 5V | 28 | 5V | BUS 5V（Base15 の MP1584 出力） |
| 3.3V | 12 | 3V3 | CoreS3 の 3.3V 出力 |

シルクの番号で配線しない。シルク「13」は G7、「1」は G43（TXD0）、「18」は G36（SCK）、「SDA/SCL」は G12/G11（AXP2101 など内部 I2C）なので、使わない。

## XH 8pin ソケット

Feedback Sensor エンコーダー、ブレーキ解除リレー、Protection Board を接続する。#7 / #8 は今後のための拡張用。

| # | 接続先 | Base15 内 | CoreS3 SE | レベル |
| --- | --- | --- | --- | --- |
| 1 | GND | | GND | |
| 2 | エンコーダ A（センサー白） | レベル変換 HV → LV | G17 | 3.3V 直交信号 |
| 3 | エンコーダ B（センサー緑） | レベル変換 HV → LV | G18 | 3.3V 直交信号 |
| 4 | Feedback Sensor 電源 5V | レベル変換の HV 側電源 | N/A | センサーの 5V。CoreS3 の GPIO には入れない |
| 5 | ブレーキ解除リレー IN | リレー駆動 MOSFET（回路構成は未定） | G13 | GPIO は HIGH で解除指令（`BRAKE_RELEASE_LEVEL = 1`）。MOSFET の構成次第で IN の論理が反転する |
| 6 | Protection Board VIOUT | 分圧抵抗 | G10 | ADC 入力。読み取りは未実装 |
| 7 | CoreS3 5V 出力 | | 5V | 予備 |
| 8 | CoreS3 3.3V 出力 | | 3.3V | 予備 |

ユーザー指定の表では #3 も G17 と書かれていたが、A と同じピンにはできないので誤記と判断し、現行の G18 のままにしている。

Base15 内の MOSFET の回路構成（ローサイド / ハイサイド、反転の有無）は未定。DIN Base 時代は G1 をリレーモジュールの IN に直結し、IN HIGH で解除だった。ローサイド 1 段なら G13 HIGH で IN が LOW になり論理が反転するので、実機で極性を確認してから `BRAKE_RELEASE_LEVEL`（encoder-tester）と `BRAKE_ON_LEVEL`（moter-controler）を合わせる。主電源用の `POWER_ON_LEVEL` / `RELAY_ON_LEVEL` は G2 直結なので触らない。[test_checklist.md](test_checklist.md) の最初の項目で見る。

G13 は CoreS3 SE 内部で I2S_DOUT（スピーカーアンプ AW88298 の DATAI）と共用。M-Bus pos 23 に出ている。ファームウェアは `internal_spk = false` と `external_speaker_value = 0` を保つ（後者は Module Display/RCA 検出で M5Unified が G13 を I2S DOUT に取るのを防ぐ）。AW88298 は電源投入既定のスタンバイのままで、DATAI は 3.3V CMOS 入力なので、G13 を GPIO 出力にして問題ない。ESP32-S3 の GPIO13 はストラップピンではなく、リセット中は浮く（G1 と同じ）。リセット中の浮きでリレーが入らないよう、リレー IN がオフ側に保たれるプル抵抗を外付けする（非反転段なら G13 側のゲートにプルダウン。ローサイド反転段ではゲートプルダウンが IN を HIGH に倒すので向きが逆になる。回路構成が決まってから確定する）。M5GFX の表示検出に失敗して board_unknown になったときだけ、M5Unified が G13 を Low→High にマイクロ秒パルスする経路がある。正常に CoreS3 SE と検出されれば通らない。

## XH 2pin ソケット

主電源リレー制御を接続する。3 系統の CoreS3 のうち、実際に接続するのは 1 系統のみの想定。

| # | 接続先 | Base15 内 | CoreS3 SE | レベル |
| --- | --- | --- | --- | --- |
| 1 | GND | | GND | |
| 2 | 主電源リレー IN | GPIO 直結（駆動素子 Q1 は電源基板側。種別は未確認） | G2 | HIGH で給電指令 |

ユーザー指定の表には CoreS3 側のピンが無い。最小変更として、現行の G2 のままにしている。G2 は Core 本体側面の Port.A（SDA）にも出ているので、Port.A には何も挿さない。

## Grove Port.B（MD13S）

HY2.0-4P の色は公式の並び。黒が GND、赤が 5V、黄が IO1、白が IO2。Base15 の Grove は回路図上バスに未配線なので、GND・5V（pos 28）・G9（pos 10）・G8（pos 4）を手配線して DIN Base の Port.B と同じ配置にする。

| 信号 | コネクタ | GPIO | 接続 | レベル |
| --- | --- | --- | --- | --- |
| MD13S PWM | Port.B 黄 | G9 | MD13S PWM | 20 kHz、10 bit。100% は 1023 |
| MD13S DIR | Port.B 白 | G8 | MD13S DIR | 正方向は Low |
| 共通 GND | 黒 | GND | MD13S GND | 3.3 V ロジック |

Port.B の 5V は MD13S に繋がない。

## 駆動

GPIO はリレーのコイルではなく、リレーモジュールの IN（ブレーキは Base15 内の MOSFET 経由、主電源は現行試験構成ではリレーモジュールの IN に直結）を駆動する。リセット中の浮きで入らないよう、リレー IN がオフ側に保たれるプル抵抗を外付けする（向きは回路構成で決まる）。

ブレーキは、24V を供給すると解除、無通電で作動する。これは実測。ソフトの名前は `brake_release` で、`brake_on` とは呼ばない。

電力の経路は `24V 電源 → Protection Board → MD13S → モーター` のまま。G2 の先は、現行試験構成では `XH 2pin #2 → 1ch リレーモジュールの IN`。最終的に電源基板の `DC_STOP1 緑 → R1 15kΩ → Q1 → HRS4H` へ直結するかは要確認（docs/cores3-controller/hardware.md の注 3）。この主リレーは、別体の 24V モーター電源への AC を入切する。AC を切っても 24V がすぐ消えるとは限らない。

G2 は 3 軸共通の AC 給電になり得る。このツールは 1 台の CoreS3 だけで使う。G2 に CoreS3 を並列に繋がない。試験しない軸は、物理的に動かない状態にしておく。

VIOUT は XH 8pin #6 から Base15 内で分圧して G10 へ入れる。Protection Board の Grove は +12V と +5V が共存する特殊なピン配置なので、CoreS3 の Grove には直結しない。このツールに電流 ADC の読み取りは無い。`CURRENT MONITOR: NOT CONNECTED` と出る。Protection Board という名前だけでは、遮断できるとは扱わない。

## エンコーダ

センサーは 6 ピン。赤が +5V、黒が 2 本とも GND、白と緑が直交信号、残りの 1 本は開放。

レベル変換は Base15 内に置く。2026-10-25 までの実測は秋月 AE-LCNV4-MOSFET（BSS138、113837。各側 10kΩ プルアップ）を外付けして行った。Base15 内でも同じ AE-LCNV4-MOSFET を使う前提（docs/cores3-controller/hardware.md のレベルシフタ節）。TXS0104E ではない。Core の内部プルアップとプルダウンは使わない。

| センサー | XH 8pin | 変換基板 | Core |
| --- | --- | --- | --- |
| 白 | #2 | HV1 → LV1 | G17 |
| 緑 | #3 | HV2 → LV2 | G18 |
| 赤 +5V | #4 | HV | センサー側の 5V。Core の信号端子には入れない |
| 黒 GND | #1 | GND | Core GND |

変換基板の LV は Core の 3.3V（M-Bus pos 12）。これは絶縁ではない。センサーの 5V（#4）と CoreS3 の 5V（#7、BUS 5V）を、確認なしで並列にしない。

5V の信号を Core の GPIO に直接入れない。

センサー GND とモーター GND は、既存の Protection Board で既につながっている。モーターの負荷電流を、細い信号 GND に流さない。

停止位置で、緑が約 5V、白が約 100mV だった測定が 1 回ある。位置が変われば値も変わる。毎回そうなる規則ではない。

G17 を A、G18 を B とする。SIGN が +1 のとき、B が Low で A が立ち上がる向きを正とする。機械と逆なら、DISARMED のあいだだけ `SET ENCODER_SIGN -1` とする。この対応は配線表と照合できるようにしてあり、分解能の確定値ではない。

## このソフトが止められないもの

画面の ESTOP と USB の `ESTOP` は、PWM を 0 にし、ブレーキ解除と主電源の指令を落とすソフトのラッチである。物理の非常停止ではない。無人、機構を支えた状態、周囲を隔離した状態で使う。負荷を保持する制御でもない。
