#pragma once

// M5Stack CoreS3 SE の公式 HY2.0-4P 表。
//   Black = GND, Red = 5V, Yellow = IO1, White = IO2
//
// Port A は Core 本体側面の Grove。DIN Base が出すのは Port B と Port C。
//   Port A IO1 Yellow = GPIO2,  IO2 White = GPIO1
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
constexpr int PIN_POWER_RELAY = 1;  // Port A IO2 White

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
