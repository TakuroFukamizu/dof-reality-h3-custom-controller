#pragma once

#include <stdint.h>

// M5Stack CoreS3 SE の公式 HY2.0-4P 表。
//   Black = GND, Red = 5V, Yellow = IO1, White = IO2
//
// Port A は Core 本体側面の Grove。DIN Base が出すのは Port B と Port C。
//   Port A Yellow G2 = GPIO2 = モーター電源
//   Port A White  G1 = GPIO1 = モーターブレーキ
// Port B は DIN Base の Grove。
//   Port B IO1 Yellow = GPIO9,  IO2 White = GPIO8
//
// M5Unified の port_*_pin1 は「5V に近い端子」で、色では White 側になる。
// ここでの IO1 / IO2 は上の公式表（Yellow / White）に合わせている。
//
// MD13S（Cytron、sign-magnitude。RC サーボ用 PWM ではない）
//   PWM Low            → MA Low,  MB Low（停止）
//   PWM High, DIR Low  → MA High, MB Low
//   PWM High, DIR High → MA Low,  MB High
// 論理入力は 3.3 V。GND は Core と共通。Port B の 5V は MD13S に繋がなくてよい。

constexpr int PIN_MD13S_PWM = 9;    // Port B IO1 Yellow
constexpr int PIN_MD13S_DIR = 8;    // Port B IO2 White
constexpr int PIN_POWER_RELAY = 2;  // Port A Yellow G2  モーター電源
constexpr int PIN_MOTOR_BRAKE = 1;  // Port A White  G1  モーターブレーキ

// 速度 > 0 のときの DIR。真理値表では Low が MA=High。
constexpr int DIR_POSITIVE_LEVEL = 0;

// リレーモジュールの論理入力。1 なら HIGH でオン、0 なら LOW でオン。
// GPIO はコイルを直接駆動しない。モジュールの IN に入れる。
// リセット中に浮いてオンしないよう、オンレベルと逆の抵抗を外付けする
// （HIGH オンならプルダウン、LOW オンならプルアップ）。
constexpr int RELAY_ON_LEVEL = 1;

constexpr uint32_t PWM_FREQ_HZ = 20000;  // MD13S の上限
constexpr uint8_t PWM_BITS = 10;
constexpr uint8_t PWM_CHANNEL = 4;

constexpr uint32_t RELAY_SETTLE_MS = 100;  // 接点が落ち着くまで PWM を出さない
constexpr uint32_t DIR_COAST_MS = 5;       // 反転前に PWM を落として両出力を Low にする

constexpr int SPEED_MIN = -100;
constexpr int SPEED_MAX = 100;

// --- フィードバックセンサー（encoder-tester の実測に合わせる） ---
// Port C: Yellow G17 = A（センサー白）、White G18 = B（センサー緑）。レベル変換後の 3.3V。
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
constexpr uint32_t START_GRACE_MS = 300;        // 開始直後の猶予
constexpr int32_t MAX_TRAVEL = 12000;           // 1 回の自動移動の移動量上限（#1 ストローク 5556 の 2 倍強）
constexpr uint32_t MOVE_TIMEOUT_MS = 45000;     // 1 回の自動移動の時間上限
constexpr int32_t SLOW_ZONE = 400;              // 端の手前で HOME_DUTY に落とす距離
constexpr int32_t ARRIVAL_MIN_TRAVEL = 400;     // これ未満の移動で止まったら到達ではなく失敗
constexpr uint32_t DWELL_MS = 200;              // 中立のバンド内に居続ける時間
