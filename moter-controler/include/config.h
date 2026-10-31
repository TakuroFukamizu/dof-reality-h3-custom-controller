#pragma once

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
// Base15 内の MOSFET が反転段（ローサイド 1 段）ならここだけを 0 にする。RELAY_ON_LEVEL は触らない。
constexpr int BRAKE_ON_LEVEL = 1;
// GPIO はコイルを直接駆動しない。リセット中に浮いてオンしないよう、リレー IN がオフ側に保たれる
// プル抵抗を外付けする（非反転なら GPIO にプルダウン。反転段なら向きが逆になるので回路構成で決める）。

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
constexpr int NEUTRAL_DUTY_MIN = 7;             // 7 % で動くことを実測
constexpr int NEUTRAL_KP_PERCENT_PER_100 = 1;   // 誤差 100 カウントあたり 1 %
constexpr int32_t NEUTRAL_BAND = 10;            // 到達とみなす誤差
constexpr uint32_t STALL_MS = 200;              // 無パルス停止
constexpr int32_t MIN_COUNT_RATE = 150;         // 駆動中にこれ未満の count/s が STALL_MS 続いたら停止判定。10 % の実測 400〜650 の 1/3 以下。片相断線の 65 count/s を弾く
constexpr uint32_t START_GRACE_MS = 300;        // 開始直後の猶予
constexpr int32_t MAX_TRAVEL = 12000;           // 1 回の自動移動の移動量上限（#1 ストローク 5556 の 2 倍強）
constexpr uint32_t MOVE_TIMEOUT_MS = 45000;     // 1 回の自動移動の時間上限
constexpr int32_t SLOW_ZONE = 400;              // 端の手前で HOME_DUTY に落とす距離
constexpr int32_t ARRIVAL_MIN_TRAVEL = 400;     // これ未満の移動で止まったら到達ではなく失敗
constexpr uint32_t DWELL_MS = 200;              // 中立のバンド内に居続ける時間
