#pragma once

#include <stddef.h>
#include <stdint.h>

// Bottom は Base15 産業用プロト基板モジュール V1.1（M5Stack K025-B）。DIN Base は使わない。
// Base15 に XH 8pin と XH 2pin のソケットを増設し、リレー駆動の MOSFET と
// エンコーダ用のレベル変換を Base15 内に収める。バッテリーは無い。
//
// Grove Port.B（MD13S 用）は DIN Base と同じピン配置で Base15 に実装する。
//   Port.B Yellow (IO1) = G9 = MD13S PWM,  White (IO2) = G8 = MD13S DIR
//   公式 HY2.0-4P の色: Black = GND, Red = 5V, Yellow = IO1, White = IO2
//   M5Unified の port_*_pin1 は「5V に近い端子」で、色では White 側になる。
//   ここでの IO1 / IO2 は上の公式表（Yellow / White）に合わせている。
//
// XH 8pin（Feedback Sensor、ブレーキ解除リレー、Protection Board）
//   #1 GND
//   #2 エンコーダ A（センサー白）→ Base15 内レベル変換 → G17
//   #3 エンコーダ B（センサー緑）→ Base15 内レベル変換 → G18
//   #4 センサー電源 5V（レベル変換の HV 用。CoreS3 の GPIO には入れない）
//   #5 ブレーキ解除リレー IN → Base15 内 MOSFET → G13
//   #6 Protection Board VIOUT → Base15 内で分圧 → G10（ADC1_CH9）
//   #7 CoreS3 5V 出力（予備）
//   #8 CoreS3 3.3V 出力（予備）
// XH 2pin（主電源リレー）
//   #1 GND
//   #2 主電源リレー IN → G2（GPIO 直結。駆動素子 Q1 は電源基板側。種別は未確認）
//
// M-Bus 位置と Base15 のシルク（Base15 のシルクは初代 Core の番号なので位置で読む）:
//   G10 = pos 2（シルク 35）, G8 = pos 4（36）, G9 = pos 10（26）, G18 = pos 15（16）,
//   G17 = pos 16（17）, G2 = pos 19（2）, G13 = pos 23（15）
//
// G13 は CoreS3 SE 内部で I2S_DOUT（スピーカーアンプ AW88298 の DATAI）と共用。
// M5Unified は internal_spk = true のときだけ G13 を I2S にするので、main.cpp の
// internal_spk = false と external_speaker_value = 0（Module Display/RCA 検出時に
// mbus_pin23 = G13 を I2S DOUT として取るのを防ぐ）を保つこと。AW88298 は電源投入既定の
// スタンバイのままで、DATAI は 3.3V CMOS 入力なので GPIO 出力にして問題ない。
//
// MD13S（Cytron、sign-magnitude。RC サーボ用 PWM ではない）
//   PWM Low            → MA Low,  MB Low（停止）
//   PWM High, DIR Low  → MA High, MB Low
//   PWM High, DIR High → MA Low,  MB High
// 論理入力は 3.3 V。GND は Core と共通。Port.B の 5V は MD13S に繋がなくてよい。

constexpr int PIN_MD13S_PWM = 9;    // Port.B IO1 Yellow（Base15 上）
constexpr int PIN_MD13S_DIR = 8;    // Port.B IO2 White（Base15 上）
constexpr int PIN_POWER_RELAY = 2;  // XH 2pin #2  主電源リレー IN（GPIO 直結）
constexpr int PIN_MOTOR_BRAKE = 13; // XH 8pin #5  ブレーキ解除リレー IN（Base15 内 MOSFET 経由）
constexpr int PIN_VIOUT = 10;       // XH 8pin #6  Protection Board VIOUT（Base15 内で分圧、ADC1_CH9）。読み取りは未実装

// 速度 > 0 のときの DIR。真理値表では Low が MA=High。
constexpr int DIR_POSITIVE_LEVEL = 0;

// 主電源リレーの論理（G2 → XH 2pin #2、GPIO 直結）。1 なら HIGH でオン、0 なら LOW でオン。
// DIN Base 構成で HIGH オンを実測。Base15 でも駆動段は Base15 の外なので変えない。
constexpr int RELAY_ON_LEVEL = 1;
// ブレーキ解除リレーの論理（G13 → Base15 内 MOSFET → XH 8pin #5）。1 なら GPIO HIGH で解除指令。
// Base15 内の MOSFET は非反転（2026-10-31 に G13 HIGH で解除を実機確認）。回路を反転段に変えるならここだけを 0 にする。RELAY_ON_LEVEL は触らない。
constexpr int BRAKE_ON_LEVEL = 1;
// GPIO はコイルを直接駆動しない。リセット中に浮いてオンしないよう、リレー IN がオフ側に保たれる
// プル抵抗を外付けする（現構成はどちらも HIGH オンなので GPIO 側にプルダウン）。

constexpr uint32_t PWM_FREQ_HZ = 20000;  // MD13S の上限
constexpr uint8_t PWM_BITS = 10;
constexpr uint8_t PWM_CHANNEL = 4;

constexpr uint32_t RELAY_SETTLE_MS = 100;  // 接点が落ち着くまで PWM を出さない
constexpr uint32_t DIR_COAST_MS = 5;       // 反転前に PWM を落として両出力を Low にする

constexpr int SPEED_MIN = -100;
constexpr int SPEED_MAX = 100;

// --- フィードバックセンサー（encoder-tester の実測に合わせる） ---
// XH 8pin #2 = A（センサー白）→ G17、#3 = B（センサー緑）→ G18。Base15 内のレベル変換後の 3.3V。
constexpr int PIN_ENC_A = 17;
constexpr int PIN_ENC_B = 18;
constexpr int ENCODER_SIGN_DEFAULT = 1;      // 速度 − でカウント増（#1 実測）。系統ごとに校正で判定して上書き
constexpr int32_t COUNTS_PER_MOTOR_REV = 400;  // モーター軸 1 回転の 4 逓倍カウント（表示用）

// --- 自動移動（端当て原点復帰・中立への比例制御） ---
constexpr int HOME_DUTY = 10;                   // 端当ての Duty。10 % 250 ms で異常無く止まった
constexpr int AUTO_DUTY_MAX = 15;               // 自動移動の上限
constexpr int NEUTRAL_DUTY_MIN = 7;             // 7 % で動くことを実測（#3）。持ち上げ側で遅すぎれば HOME_DUTY まで段階的に上げる（#2 は 8 % 台で 150 count/s 未満、2026-10-31）
constexpr int NEUTRAL_KP_PERCENT_PER_100 = 1;   // 誤差 100 カウントあたり 1 %
constexpr int32_t NEUTRAL_BAND = 10;            // 到達とみなす誤差（入口）
constexpr int32_t NEUTRAL_BAND_EXIT = 20;       // Dwell 中はこれを超えるまで帯内とみなす（#3 が 3077〜3080 で往復し収束しなかった 2026-10-31）
constexpr uint32_t STALL_MS = 200;              // 無パルス停止
constexpr int32_t MIN_COUNT_RATE = 150;         // 駆動中にこれ未満の count/s が STALL_MS 続いたら停止判定。10 % の実測 400〜650 の 1/3 以下。片相断線の 65 count/s を弾く
constexpr uint32_t START_GRACE_MS = 300;        // 開始直後の猶予
constexpr int32_t MAX_TRAVEL = 12000;           // 1 回の自動移動の移動量上限（#1 ストローク 5556 の 2 倍強）
constexpr uint32_t MOVE_TIMEOUT_MS = 45000;     // 1 回の自動移動の時間上限
constexpr int32_t SLOW_ZONE = 400;              // 端の手前で HOME_DUTY に落とす距離
constexpr int32_t ARRIVAL_MIN_TRAVEL = 400;     // これ未満の移動で止まったら到達ではなく失敗
constexpr uint32_t DWELL_MS = 200;              // 中立のバンド内に居続ける時間
constexpr int32_t RETREAT_COUNTS = 600;         // 端当てで開始直後に止まったとき、反対へ退避する距離（約 1.5 回転）。退避後に改めて端へ当てる

// --- 手動速度の守り（無パルス／遅すぎで出力 0） ---
constexpr int MANUAL_GUARD_MIN_DUTY = NEUTRAL_DUTY_MIN;  // これ未満は機構が動かないことがある（7 % で動くことを実測）ので監視しない

// --- USB プロトコル v1（MC_PROTO_V1。docs/control-server/protocol.md、設計 docs/superpowers/specs/2026-10-31-moter-controler-v1-protocol-design.md §3） ---
// いずれも実機で詰めるまでの仮値。
// 2026-10-31 の 3 系統同時運転（docs/multi_axis_20261031.md）の実測: Duty 1 % あたりおよそ 100〜130 count/s、始動に 7〜9 %
// （#2 の持ち上げ側は 8 % 台で 200 count/s 未満）。15 % で 1300〜1460 count/s。比例だけでは誤差を数百 count 抱えないと
// 速度が出ないので、軌道の目標速度に応じたフィードフォワードを足し、比例は補正に使う。
constexpr int V1_KP_PERCENT_PER_100 = 2;        // 位置制御の比例ゲイン。誤差 100 カウントあたり 2 %
constexpr int V1_FF_COUNT_S_PER_PERCENT = 100;  // フィードフォワード: 軌道の目標速度 [count/s] ÷ この値 = Duty [%]（実測の下側を取った仮値）
constexpr int32_t V1_DEADBAND = NEUTRAL_BAND;   // 目標が止まっているとき、|誤差| がこれ以下になったら Duty 0（入る側。従来 env の到達帯と同じ 10）
constexpr int32_t V1_DEADBAND_OUT = NEUTRAL_BAND_EXIT;  // Duty 0 のあと、|誤差| がこれを超えるまで再駆動しない（出る側。従来 env の出口帯と同じ 20。到達帯の縁での往復を防ぐ）
constexpr int V1_DUTY_MIN = 7;                  // Duty の下限の初期値（7 %。これ未満では機構が動かないことを実測）。自動移動側の NEUTRAL_DUTY_MIN とは独立で、SET_LIMITS で PC から置き換わる
constexpr int V1_DUTY_MAX = 20;                 // Duty の上限の初期値（実測 20 % ≈ 1900 count/s 前後。overspeed_count_s より十分下の速度に収める）。SET_LIMITS を受けるまでの値
// 絶対上限（docs/superpowers/specs/2026-11-01-motion-limits-design.md §1・§3）。PC から SET_LIMITS / CONFIG / SET_POSITION で
// 受けた値はここで黙ってクランプする（拒否しない）。変えるには焼き直しが要る。HELLO の後、SET_LIMITS の応答 LIMITS で PC に申告する。
// いずれも実機未検証の見積もり: MD13S と 24 V モーターの余裕側で 100 % は使わない。速度 6000 count/s は 15 回転/s で Duty 60 % の
// 外挿（6000〜7800 count/s）より下、加速度 40000 count/s² は 0 → 6000 count/s に 0.15 s。
constexpr int V1_ABS_DUTY_MAX = 60;                     // Duty の絶対上限 [%]
constexpr uint32_t V1_ABS_MAX_VEL_COUNT_S = 6000;       // 速度の絶対上限 [count/s]
constexpr uint32_t V1_ABS_MAX_ACC_COUNT_S2 = 40000;     // 加速度の絶対上限 [count/s²]
constexpr int V1_STALL_DUTY = 10;               // ENCODER_FAULT の判定を始める |Duty| の下限（下限 7 % の押し当てでは #2 の持ち上げ側が動かないことがあるので、下限より上）。ブレーキ解除指令中に目標（軌道）が動いている間だけ、これ以上を STALL_MS 出してカウントが変わらなければ異常（目標が止まっているときの静止やブレーキ作動中は見ない）
// Issue #15 M2: 実際のしきい値は max(V1_STALL_DUTY, duty_max × この割合)。kp を上げると数百 count の誤差だけで 10 % に届き、
// 動けない軸（荷重）が FOLLOWING_ERROR ではなく ENCODER_FAULT になって基準を失うので、ループが上限近くまで出しているときだけ
// 「エンコーダが死んだ」と判定する。それ未満の無パルスは FOLLOWING_ERROR（と下の停止ブースト）に任せる。
// duty_max < V1_STALL_DUTY の SET_LIMITS は受理するが検出が無効になる（画面に警告。docs/control-server/limits.md）。
constexpr int V1_STALL_DUTY_RATIO_PERCENT = 75;
// Issue #15 H4: 停止ブースト。位置ループに積分が無いので、目標が止まっている（軌道速度 0）のに |誤差| が不感帯の外で、
// 下限 Duty を出してもカウントが動かない（荷重とのつり合い）状態が続くと静的誤差を詰められない。auto_move.h の neutral_boost と
// 同じく、この時間カウントが V1_STALL_BOOST_MIN_MOVE 未満しか動かないたびに下限を 1 % ずつ上げる（上限は duty_max。超えない）。
// 誤差が不感帯に入る・誤差の符号が反転する・軌道が動き出す・ブレーキ作動のどれかで 0 に戻す。実機 11-02: kp 6 / duty_min 9 で
// 自重のかかった軸の動き出しは約 9 %、7〜8 % では動かない。
constexpr uint32_t V1_STALL_BOOST_MS = 200;
constexpr int32_t V1_STALL_BOOST_MIN_MOVE = 3;  // 窓の間にこれ未満しか動かなければ「動いていない」（停止中のエンコーダの ±1 のちらつきで窓を取り直さない）
// Issue #15 M1: Duty 下限を「目標へ向かう向き」のときだけ掛ける（1）。FF（軌道速度）と P（誤差）の合成が下限未満で、かつ誤差と
// 逆向きなら、下限まで持ち上げて目標から離れる向きに走らせるのではなく 0（惰行）にする。例: err +50（P +1 %）・vel −200
// （FF −2 %）→ 合成 −1 % → 旧: −duty_min で目標から離れる / 新: 0。0 にすると旧動作（合成値に下限を掛ける）。実機 A/B 用に
// -DV1_FLOOR_SIGN_GATE=0 で切れる。FF が誤差と逆を向くのが正当な場面（減速中に実位置が軌道より先行）があるかは未確定。
#ifndef V1_FLOOR_SIGN_GATE
#define V1_FLOOR_SIGN_GATE 1
#endif
// Issue #15 M7: USB 送信をブロックさせない（include/v1_tx_pump.h）。書く前に TX リングの空きを見て、入らなければ次の poll へ。
constexpr uint32_t V1_TX_STATUS_STALE_MS = 100;  // 積まれてからこれ以上経った STATUS は送らない（20 ms ごとに新しいものが来る。PC の途絶判定 200 ms の半分）
constexpr uint32_t V1_TX_DEFER_MAX_MS = 200;     // STATUS 以外の保留がこれを超えたら捨てる（PC の ACK 期限 1000 ms より十分短い。ここまで詰まったリンクは PC 側でも途絶）
constexpr size_t V1_TX_RING_BYTES = 1024;        // HWCDC の TX リング（既定 256 = STATUS 約 5 本）。Serial.begin() の前に setTxBufferSize で広げる
constexpr uint32_t V1_OVERRUN_MS = 15;          // 制御 tick の間隔がこれを超えたら CONTROL_OVERRUN（周期 5 ms）
constexpr uint32_t POWER_SETTLE_MS = 300;       // ARMING: 主電源 ON 指令からソフトリミット確認・位置制御開始まで
constexpr uint32_t BRAKE_RELEASE_WAIT_MS = 300; // ARMING: ブレーキ解除指令から HOLDING まで（POWER_SETTLE_MS との和を ARM 全体 5000 ms より十分短く）
constexpr uint32_t BRAKE_APPLY_WAIT_MS = 300;   // DISARMING: ブレーキ給電 OFF から PWM 0・主電源 OFF・DISARMED まで
constexpr uint32_t CAL_POWER_HOLD_TIMEOUT_MS = 120000;  // KEEP_POWER: 校正が終わってから主電源を保つ最長時間（protocol.md §4、仮値）
constexpr int V1_CAL_HOME_DUTY = 13;            // v1 の校正（CALIBRATE）の端当て Duty。従来 env の HOME_DUTY（10 %）では #2 が下限から持ち上がる方向で始動しないことがあった（2026-11-01、2 回）。10/31 の実測で 13 % は 920〜985 count/s。端当ての衝撃は 15 %（従来 env の UPPER の速い区間）より小さい。仮値
constexpr uint32_t CAL_POWER_SETTLE_MS = 5000;  // CALIBRATING: 主電源 ON 指令からブレーキ解除指令まで（protocol.md §1.2）。2026-10-31 に電源投入 4 s 後は動かず
                                                // 12 s 後に動いた記録があるため ARMING の POWER_SETTLE_MS より長い。仮値。電源担当でない軸も同じ時間待つ
