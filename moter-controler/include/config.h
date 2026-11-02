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
// モーター出力の内部分解能は permille（0.1 %。Issue #25 手順 8 b）。従来 env の手動速度・自動移動は整数 % のままで、
// motor_output_write(%) が ×10 して同じ経路に入る。±1000 permille が PWM のフルスケール（(1 << PWM_BITS) − 1）
constexpr int PERMILLE_PER_PERCENT = 10;
constexpr int PERMILLE_MIN = SPEED_MIN * PERMILLE_PER_PERCENT;  // −1000
constexpr int PERMILLE_MAX = SPEED_MAX * PERMILLE_PER_PERCENT;  // +1000

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
constexpr int32_t RETREAT_END_MIN_TRAVEL = 50;  // 退避がこれ以上動いてから止まったら「反対の端に当たった」とみなし、そこから改めて端へ向かう（端の近くから始めた校正が退避で反対の端に当たって失敗した 2026-11-02、Issue #34）。これ未満で止まれば従来どおり失敗

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
// permille 換算（Issue #25 手順 8 b）。位置ループは v1_duty_command から PWM まで permille（0.1 %）で運ぶ。SET_LIMITS / LIMITS の
// 線上は整数 % のまま（V1Tuning の duty_min / duty_max / stall_duty も %）で、しきい値はここの定数と v1_permille() で換算する。
constexpr int V1_PERMILLE_PER_PERCENT = PERMILLE_PER_PERCENT;
constexpr int V1_DUTY_MIN_PERMILLE = V1_DUTY_MIN * V1_PERMILLE_PER_PERCENT;      // 70
constexpr int V1_DUTY_MAX_PERMILLE = V1_DUTY_MAX * V1_PERMILLE_PER_PERCENT;      // 200
constexpr int V1_ABS_DUTY_MAX_PERMILLE = V1_ABS_DUTY_MAX * V1_PERMILLE_PER_PERCENT;  // 600
constexpr int V1_STALL_DUTY_PERMILLE = V1_STALL_DUTY * V1_PERMILLE_PER_PERCENT;  // 100。ENCODER_FAULT しきい値の下限
// Issue #15 H4: 停止ブースト。位置ループに積分が無いので、目標が止まっている（軌道速度 0）のに |誤差| が不感帯の外で、
// 下限 Duty を出してもカウントが動かない（荷重とのつり合い）状態が続くと静的誤差を詰められない。auto_move.h の neutral_boost と
// 同じく、この時間カウントが V1_STALL_BOOST_MIN_MOVE 未満しか動かないたびに下限を 1 % ずつ上げる（上限は duty_max。超えない）。
// 誤差が不感帯に入る・誤差の符号が反転する・軌道が動き出す・ブレーキ作動のどれかで 0 に戻す。実機 11-02: kp 6 / duty_min 9 で
// 自重のかかった軸の動き出しは約 9 %、7〜8 % では動かない。
constexpr uint32_t V1_STALL_BOOST_MS = 200;
constexpr int V1_STALL_BOOST_STEP_PERMILLE = 1 * V1_PERMILLE_PER_PERCENT;  // 窓ごとの上乗せ幅（1 % = 10 permille）
constexpr int32_t V1_STALL_BOOST_MIN_MOVE = 3;  // 窓の間にこれ未満しか動かなければ「動いていない」（停止中のエンコーダの ±1 のちらつきで窓を取り直さない）
// 上乗せの上限: 対象になってからこれだけ動かなければ（または上乗せが duty_max で頭打ちのまま次の窓も動かなければ）ENCODER_FAULT を
// ラッチする（詰まった機構を duty_max で押し続けない。静止目標では無パルス判定が回らないので、ここで閉じる）。0 で時間の上限なし。
constexpr uint32_t V1_STALL_BOOST_MAX_MS = 2000;
// Issue #25 手順 8 (c)「E」: Duty 下限（duty_min ＋ 停止ブースト）はヒステリシス付きの「床の適用状態」が ON のときだけ掛ける。
// ON になる条件: |誤差| > deadband_out（20）かつ P と合成（FF + P）の向きが一致。OFF になる条件: |誤差| ≤ V1_FLOOR_OFF_COUNT（15）
// または向きの不一致（即時）。15 < |誤差| ≤ 20 は前の状態を保つ。OFF のときは有界の生の FF + P を permille でそのまま出す
//（0 にはしない。旧 M1 は誤差と逆向きの合成を 0 に置き換えていた。「誤差と逆向きに床を掛けない」という M1 の規則は向きの条件として残る）。
// 狙い: 追従中の小さい誤差で床（9 % ≈ 900 count/s）が軌道速度を超えて「バースト → 停止」になるのをなくし、FF だけで追従させる。
// 静止の不感帯（10 / 20）と停止ブースト（H4。床が ON のときだけ上乗せが効く）はそのまま。
constexpr int32_t V1_FLOOR_OFF_COUNT = 15;
// Issue #25 手順 9「F」: 有界の積分と保持バイアス。I += Ki × 誤差 × dt [permille] を位置制御中だけ積む。|I| ≤ V1_I_MAX_PERMILLE、
// 変化率 ≤ V1_I_RATE_PERMILLE_PER_S。アンチワインドアップは実出力で見る: 出力が duty_max で飽和、床（duty_min ＋ ブースト／
// 動き出し補助）が合成を持ち上げている、床の向き不一致（E の抑制中）のどれかなら、|I| を増やす向きには積まない（戻す向きは
// 常に許す）。静止の不感帯の中では積まない（凍結）が、学んだ I はそのまま出力に出す（保持バイアス。0 に落とさない。自重で
// 下がる軸を 0 ↔ 9 % の蹴りなしで保持する狙い）。DISARMED / FAULT / SET_REFERENCE / 系統切替 / 位置制御の終了で 0。
// 実機 A/B 用に -DV1_INTEGRAL=0 で切れる（V1Tuning の ki が 0 になる。I は常に 0 で不感帯の中は従来どおり 0）。
#ifndef V1_INTEGRAL
#define V1_INTEGRAL 1
#endif
constexpr double V1_KI_PERMILLE_PER_COUNT_S = 0.1;  // Ki = 0.01 %/(count·s) = 0.1 permille/(count·s)。誤差 20 で 2 permille/s
constexpr int V1_I_MAX_PERMILLE = 30;               // |I| の上限 3 %
constexpr int V1_I_RATE_PERMILLE_PER_S = 10;        // |dI/dt| の上限 1 %/s
// Issue #25 手順 9「T」: 追従中の動き出し補助。|軌道速度| > V1_ASSIST_VEL_COUNT_S なのに |実測速度| < V1_ASSIST_MEAS_VEL_COUNT_S
// のまま |誤差| が増え続けて V1_ASSIST_MS 経ったら、床の適用状態（|誤差| > 20）に依らず床（duty_min、合成の向き）を掛ける。
// エンコーダが V1_ASSIST_RELEASE_MOVE 以上動いたら即解除。停止ブースト（H4）は静止目標だけなので、その補完。保護との関係は
// protocol.md §5（床 9 % は ENCODER_FAULT のしきい値 15 % 未満で、FOLLOWING_ERROR の判定は触らない）。-DV1_TRACK_ASSIST=0 で切れる
#ifndef V1_TRACK_ASSIST
#define V1_TRACK_ASSIST 1
#endif
constexpr int32_t V1_ASSIST_VEL_COUNT_S = 100;       // これより速い軌道速度が対象
constexpr int32_t V1_ASSIST_MEAS_VEL_COUNT_S = 20;   // 実測速度がこれ未満なら「動いていない」（100 ms 窓なので 0 か ±10）
constexpr uint32_t V1_ASSIST_MS = 150;               // この時間続いたら補助 ON
constexpr int32_t V1_ASSIST_RELEASE_MOVE = 3;        // ON の後、これ以上カウントが動けば解除
// Issue #25 手順 10「S」: 整定判定。静止の不感帯（Duty = I の保持）に入るのは「軌道速度 0 かつ |誤差| ≤ deadband かつ
// |実測速度| ≤ V1_SETTLED_MEAS_VEL_COUNT_S」が V1_SETTLED_MS 続いたとき。軌道が動く・|誤差| > deadband_out で即抜ける。
// -DV1_SETTLED_MEASURED=0 で切れる（従来どおり |誤差| ≤ deadband で即入る）
#ifndef V1_SETTLED_MEASURED
#define V1_SETTLED_MEASURED 1
#endif
constexpr int32_t V1_SETTLED_MEAS_VEL_COUNT_S = 10;
constexpr uint32_t V1_SETTLED_MS = 200;
// Issue #25 r6（2026-11-02、Codex 設計コンサル）: 方向別 FF ＋ PI、走行中の床なし。既定 1。-DV1_FF_DIRECTIONAL=0 で r5 までの
// 経路（v/ff の FF、床 E、補助 T、不感帯 10/20、Ki 0.1）に戻る（ホストテストはこの 0 で従来の経路を検証し、r6 は明示的に ON にして検証する）。
// ON のとき SET_LIMITS の ff_count_s_per_percent / duty_min / deadband / deadband_out は受理・LIMITS で返すが位置ループでは使わない。
#ifndef V1_FF_DIRECTIONAL
#define V1_FF_DIRECTIONAL 1
#endif
// 方向別 FF: d = 軌道速度の向き（「上」= カウント増）、w = min(|v| / V1_FF_TAPER_COUNT_S, 1)、u_ff = w × a_d + b_d × v [permille]、
// v = 0 なら 0。a は符号付きの切片（下向きは負の値のまま足す）、b は permille / (count/s)。
// 実測（11-02、#1、r3、一定速度ランプの定常区間の平均 Duty）: 上 99→5.6 %、198→6.2、413→8.9、593→10.5 / 下 −99→−1.8、−198→−2.5、
// −397→−5.3、−592→−7.1。直線回帰: 上 u ≈ 4.6 % + 0.0099 %·v、下 u ≈ −0.74 % + 0.0107 %·v（重力バイアス ≈ +1.9 %、クーロン ≈ 2.7 %）。
constexpr int V1_FF_A_UP_PERMILLE = 46;                        // 上向き（カウント増）の切片 4.6 %
constexpr double V1_FF_B_UP_PERMILLE_PER_COUNT_S = 0.10;       // 上向きの傾き 1.0 % / 100 count/s
constexpr int V1_FF_A_DOWN_PERMILLE = -7;                      // 下向き（カウント減）の符号付き切片 −0.7 %
constexpr double V1_FF_B_DOWN_PERMILLE_PER_COUNT_S = 0.107;    // 下向きの傾き 1.07 % / 100 count/s
constexpr int32_t V1_FF_TAPER_COUNT_S = 30;                    // 反転時に切片を一気に反転させないテーパ幅 [count/s]
// PI: Kp は SET_LIMITS（bringup 6 %/100 count）。Ki は r6 で 0.02 %/(count·s) = 0.2 permille/(count·s)。|I| ≤ V1_I_MAX_PERMILLE、
// |dI/dt| ≤ V1_I_RATE_PERMILLE_PER_S（上と同じ 30 / 10）。アンチワインドアップは「duty_max で飽和した向きへ積み増さない」だけ
//（床・符号不一致の抑制は無い。軌道より先行して逆向きに補正するのは正当）。動き出しパルス中と保持中は凍結、保持に入ったら 0。
constexpr double V1_KI_DIRECTIONAL_PERMILLE_PER_COUNT_S = 0.2;
// 保持: 軌道完了（軌道速度 0）かつ |誤差| ≤ V1_HOLD_ENTER_COUNT かつ |実測速度| ≤ V1_SETTLED_MEAS_VEL_COUNT_S が V1_SETTLED_MS
// 続いたら入る（I と出力を 0。#1 は中立付近で自己保持する）。|誤差| > V1_HOLD_EXIT_COUNT か軌道が動けば出る。10/20 の不感帯に代わる。
constexpr int32_t V1_HOLD_ENTER_COUNT = 2;
constexpr int32_t V1_HOLD_EXIT_COUNT = 4;
// 動き出しパルス（床の唯一の例外）: |誤差| > V1_HOLD_EXIT_COUNT の要求が続いていて、V1_BREAKAWAY_WINDOW_MS の間に指令の向きへ
// V1_BREAKAWAY_MIN_MOVE 未満しか進まなければ、指令の向きに V1_BREAKAWAY_UP/DOWN_PERMILLE（実測の動き出し Duty。合成がそれ以上なら合成）を
// V1_BREAKAWAY_PULSE_MS または 3 count 進むまで出す。進まないままのパルスは V1_BREAKAWAY_MAX_PULSES で打ち切り（連打しない）、
// あとは静止 2 s の監視（ENCODER_FAULT）に任せる。パルスは監視の時計を戻さない。
constexpr int V1_BREAKAWAY_UP_PERMILLE = 90;       // 上向き（カウント増）9 %
constexpr int V1_BREAKAWAY_DOWN_PERMILLE = 60;     // 下向き（カウント減）6 %
constexpr uint32_t V1_BREAKAWAY_WINDOW_MS = 100;   // 「進まない」を判定する窓
constexpr uint32_t V1_BREAKAWAY_PULSE_MS = 100;    // パルスの長さの上限
constexpr int32_t V1_BREAKAWAY_MIN_MOVE = 3;       // 窓／パルスの間にこれ以上進めば「進んだ」
constexpr int V1_BREAKAWAY_MAX_PULSES = 5;         // 進まないまま打てるパルスの数
// 軌道が動いているときの動き出しパルス（PR #33 Codex レビュー P1）: 遅い軌道（30 count/s 前後）に普通に遅れているだけでも
// 100 ms に 3 count が境目になり 9 % を連発しかねない。そこで走行中は「軌道は速い（|v_traj| ≥ MIN_TRAJ）のに軸は止まっている
//（実測 |v| ≤ MAX_VEL）まま MOVING_WINDOW の間 |誤差| が増え続けた」ときだけ打つ。静止目標（軌道速度 0）は従来どおり 100 ms 窓。
// r7: 軌道速度の下限は外す（0）。正弦波の反転付近（|v_traj| < 100 が周期の 1/4）で FF がテーパして止まったままになるのが r6 の追従誤差の
// 主因だったため。遅い追従（25〜30 count/s）は実測 |v| > 10 なので対象外のまま
constexpr int32_t V1_BREAKAWAY_MOVING_MIN_TRAJ_COUNT_S = 0;
constexpr int32_t V1_BREAKAWAY_MOVING_MAX_VEL_COUNT_S = 10;
constexpr uint32_t V1_BREAKAWAY_MOVING_WINDOW_MS = 200;
// r8（Issue #25、Codex 案 d）: 軌道速度がこの帯の間は「引っかかった時だけ短く押す」。100 ms 向きへ 1 count も進まず |誤差| が
// 増えていれば上 90 / 下 60 permille を最初の 1 count か 50 ms まで、その後 200 ms は押さない（回数の上限は V1_BREAKAWAY_MAX_PULSES）。
// 実機 11-02: 50〜150 count/s の速度ムラが r3 の 1.3〜1.9 倍（静止摩擦 ≈ 9 % と動摩擦 ≈ 5 % の差）。HI = 0 で無効。
constexpr int32_t V1_BREAKAWAY_BAND_LO_COUNT_S = 50;
constexpr int32_t V1_BREAKAWAY_BAND_HI_COUNT_S = 150;
constexpr uint32_t V1_BREAKAWAY_BAND_WINDOW_MS = 100;
constexpr uint32_t V1_BREAKAWAY_BAND_PULSE_MS = 50;
constexpr uint32_t V1_BREAKAWAY_BAND_COOLDOWN_MS = 200;
constexpr int32_t V1_BREAKAWAY_BAND_MIN_MOVE = 1;
// r9（Issue #35、Codex 案）: 速度誤差の減衰項。方向別の経路（V1_FF_DIRECTIONAL）だけで効く。既定 1、-DV1_VEL_DAMPING=0 で r8 と同じ出力。
// 実機 11-02（#1、r8 kp 30）: FE rms 3.6 は良いが、軌道 50〜150 count/s の区間で 100 ms 窓の速度が約 3 Hz・±60 count/s で波打つ
//（P だけの減衰不足と読んでいる）。そこで 40 ms の滑り窓の速い速度推定 v_fast を tick（5 ms）ごとに更新し、
//   u_d = clamp(Kd × (v_traj − v_fast), −D_MAX, +D_MAX) [permille]、u = clamp(FF + P + I + u_d, ±duty_max)
// を出す（v_traj は FF に使っている軌道の目標速度）。飽和・アンチワインドアップの判定は u_d 込みの合成で見る。保持（HOLD）の出力 0、
// 動き出しパルス、保護（FOLLOWING_ERROR・走行中の無パルス・静止 2 s の監視・ソフトリミット）は変えない。
// v_fast = (いまの生カウント − 約 40 ms 前の生カウント) / 実際の経過秒。40 ms 以上前のサンプルのうち最も新しいものを使う。
// 起動直後（位置制御の開始・FAULT の後）で 40 ms 分のサンプルが無ければ、最も古いサンプルが窓の半分（20 ms）以上前なら
// それで割り、それも無ければ推定なし（u_d = 0）。tick の間隔が 2 × 窓（80 ms）を超えて空いたらサンプルを捨てて取り直す。
// 量子化: 40 ms に 1 count = 25 count/s → u_d は 1.25 permille 刻み。
#ifndef V1_VEL_DAMPING
#define V1_VEL_DAMPING 1
#endif
constexpr double V1_VEL_KD_PERMILLE_PER_COUNT_S = 0.05;  // Kd: 速度誤差 1 count/s あたり 0.05 permille（200 count/s で上限 10）
constexpr int V1_VEL_D_MAX_PERMILLE = 10;                // |u_d| の上限 1 %
constexpr uint32_t V1_V_FAST_WINDOW_MS = 40;             // 速い速度推定の窓
// r10（Issue #44・#46）: 保護の監視 V1Supervisor（include/v1_supervisor.h）。方向別の経路（V1_FF_DIRECTIONAL）だけで効く。
// 「進まない → ENCODER_FAULT」の 2 つの判定（走行中の無パルス・静止 2 s の監視）を置き換える。電流検出が無いので「エンコーダは
// 生きているが完全に止まっている（乗員の荷重）」と「エンコーダ断」はファームでは区別できない。そこで ENCODER_FAULT はラッチせず、
// 「見えないまま動けた距離（見えない走行）」を実際に出した PWM から積算して上限で止める（FEEDBACK_UNCERTAIN）。過負荷（生きているのに
// 進まない）は飽和 1 s で LOAD_LIMITED。どちらも **PWM 0 ＋ ブレーキ作動（主電源は保つ）の持続する停止**で、自動では戻らない
//（DISARM → 再 ARM の通常の ARMING 手順だけが戻す。PR #48 の Codex 事前レビュー P0）。いずれも仮値で、実機で詰める。
// -DV1_SUPERVISOR=0 で切れる（r9 と同じ判定）。
#ifndef V1_SUPERVISOR
#define V1_SUPERVISOR 1
#endif
// 見えない走行の包絡線速度: 既定は Issue #44 の仕様どおり |Duty| > 0 なら一律 V1_SUP_ENV_COUNT_S（2500 count/s）。
// -DV1_SUP_ENV_PROPORTIONAL=1 で |Duty| に比例（V1_SUP_ENV_PROP_REF_PERMILLE = 20 % で 2500。上限なし）に切り替える（初期の検証では使わない）。
// 一律の注意: 保持に入る前の整定（P + I が数 permille で動かない）や 1 count だけの補正も 2500 count/s で積むので、確かな動き（2 count）が
// 無いまま小さな補正が続くと中央付近でも 400 ms 分で止まる（実機で頻度を見る）。
#ifndef V1_SUP_ENV_PROPORTIONAL
#define V1_SUP_ENV_PROPORTIONAL 0
#endif
constexpr int32_t V1_SUP_ENV_COUNT_S = 2500;
constexpr int V1_SUP_ENV_PROP_REF_PERMILLE = 200;
constexpr int V1_SUP_ENV_REF_PERMILLE = V1_SUP_ENV_PROPORTIONAL ? V1_SUP_ENV_PROP_REF_PERMILLE : 0;  // 0 = 一律
// 予算（count）= min(V1_SUP_BUDGET_MAX_COUNT, max(0, D − V1_SUP_END_RESERVE_COUNT))。D = 最後に信用できた位置から駆動の向きの機械端
// までの距離。機械端は CONFIG のソフトリミットから V1_MECH_DECEL_ZONE_COUNT だけ外（PC の calibration.rs が soft ≥ mechanical +
// decel_zone を検証している。bringup.toml の decel_zone は 50。PC がこれより小さい decel_zone を使うとここは楽観側になる）。
// 2500 count/s なら中央付近 400 ms、D = 650 で 100 ms、D ≤ 400 で 0（先読みは最悪の tick 間隔 V1_OVERRUN_MS で見るので実際は 1 tick 弱短い）。
// 運用の注意: 既定では端から 400 count 以内（機械端基準）で端へ向かう出力は出せない → PC の目標は機械端から 400 より内側に置く。
constexpr int32_t V1_SUP_BUDGET_MAX_COUNT = 1000;
constexpr int32_t V1_SUP_END_RESERVE_COUNT = 400;
constexpr int32_t V1_MECH_DECEL_ZONE_COUNT = 50;
// 再生バッファ（Issue #45）の参照が端の手前で止まる位置に残す、見えない走行の予算の余裕 [count]。参照の止まる位置 =
// ソフトリミットから V1_SUP_END_RESERVE_COUNT − V1_MECH_DECEL_ZONE_COUNT + これ（V1_SOFT_RESERVE_COUNT、下の再生バッファの節）。
// 最悪の tick の先読み（2500 count/s × V1_OVERRUN_MS 15 ms = 37.5）以上が要る（static_assert）。さらに、参照の寄せの終わり（減速の尾。
// 参照速度が数十 count/s の間は FF の切片が端の向きに出るのに、軸は 2 count 確かに動くまで時間が掛かる）の包絡線もここから払う。
// ホストの PWM プラント（test_r10_integrated_playout_stops_outside_no_drive_zone）では余裕 100 だと寄せの終わりで尽き、150 で残り 86、
// 250 で残り 125 以上だったので 250 にした（仮値。実機の慣性・遅れで要確認）。代償: 参照はソフトリミットから 600（機械端から 650）の
// 手前で止まる（ストローク 5551 なら使えるのは約 4250）。一律の包絡線と予算 D − 400 のまま範囲を広げるには、比例の包絡線
//（V1_SUP_ENV_PROPORTIONAL）か最小予算（V1_SUP_MIN_BUDGET）の検証が要る
constexpr int32_t V1_SUP_END_MARGIN_COUNT = 250;
// 確認のための最小の予算（仕様からの逸脱。既定 0 = 仕様どおり）: -DV1_SUP_MIN_BUDGET=1 で min(50, max(0, D − V1_END_HARD_MARGIN_COUNT))
// を下限にする（端から 400 以内でも 20 ms 分は端へ出せる）
#ifndef V1_SUP_MIN_BUDGET
#define V1_SUP_MIN_BUDGET 0
#endif
constexpr int32_t V1_SUP_MIN_BUDGET_VARIANT_COUNT = 50;
constexpr int32_t V1_SUP_MIN_BUDGET_COUNT = V1_SUP_MIN_BUDGET ? V1_SUP_MIN_BUDGET_VARIANT_COUNT : 0;
// 「エンコーダが確かに動いた」（予算を戻す唯一の根拠）: 出している Duty の向きへ、窓の中の最も戻った所から V1_SUP_RENEW_MOVE_COUNT 以上の
// 正味の移動。±1 のちらつき・片相のトグル（n ↔ n+1）は 1 にしかならないので戻さない。窓は半分（100 ms）ずつ 2 つを持つ滑り窓の近似
// （実効 100〜200 ms）。Duty 0・向きの反転で窓を取り直す。既知の穴: ±2 以上の雑音の往復は戻してしまう（PCNT のフィルタが無効）
constexpr int32_t V1_SUP_RENEW_MOVE_COUNT = 2;
constexpr uint32_t V1_SUP_RENEW_WINDOW_MS = 200;
// 過負荷（LOAD_LIMITED）: |出力| ≥ duty_max × 99 % で誤差の向きに出している（飽和）のに、100 ms の窓ごとに向きへ 5 count 進む・
// |誤差| が 3 減る、のどちらも無いまま 1 s → PWM 0 ＋ ブレーキ作動の持続する停止（自動では戻らない。安全に保てる通電の保持 Duty が
// 分からないので機械のブレーキで保つ。人が乗った状態の保持 Duty と発熱を測ってから見直す）
constexpr int V1_LOAD_SAT_RATIO_PERMILLE = 990;
constexpr uint32_t V1_LOAD_SAT_MAX_MS = 1000;
constexpr uint32_t V1_LOAD_WINDOW_MS = 100;
constexpr int32_t V1_LOAD_PROGRESS_COUNT = 5;
constexpr int32_t V1_LOAD_ERR_DROP_COUNT = 3;
// Issue #46: 実測速度を見た端の手前の制動（END_BRAKING、ラッチしない）。実測速度（40 ms 窓の v_fast、無ければ 100 ms 窓）の向きの
// ソフトリミットまでの残り d に対し、停止距離 v² / (2 × V1_BRAKE_GUARD_ACC_COUNT_S2) > d なら反対向きに duty_max を出す。
// 減速度 1000 count/s² は実機が確実に止まれる値の仮置き（要計測）。|v| < V1_BRAKE_GUARD_MIN_VEL_COUNT_S は見ない。
// 解除: |v| < 最小速度 / 2、または停止距離 ≤ d − V1_BRAKE_GUARD_HYST_COUNT。この出力も見えない走行の予算に積み、尽きれば停止（P1a）
constexpr int32_t V1_BRAKE_GUARD_ACC_COUNT_S2 = 1000;
constexpr int32_t V1_BRAKE_GUARD_MIN_VEL_COUNT_S = 50;
constexpr int32_t V1_BRAKE_GUARD_HYST_COUNT = 20;
// Issue #46: SOFT_LIMIT の猶予（既定 OFF = 外れたら即ラッチ。-DV1_SOFT_GRACE=1 で ON）。ON のとき、少し越えただけではラッチせず内側へ
// 戻す（外向きの出力は 0）。ラッチ: 機械端（上の推定）から V1_END_HARD_MARGIN_COUNT 以内、外向きに V1_SOFT_OUT_VEL_COUNT_S を超える
// 速度で V1_SOFT_OUTWARD_GRACE_MS 動き続けた、外に V1_SOFT_RECOVER_MAX_MS 居続けた、目標が外、のどれか。速度と遅れに応じた停止余裕を
// 入れるまでは既定で使わない（PR #48 の Codex 事前レビュー P1c）
#ifndef V1_SOFT_GRACE
#define V1_SOFT_GRACE 0
#endif
constexpr int32_t V1_END_HARD_MARGIN_COUNT = 20;
constexpr int32_t V1_SOFT_OUT_VEL_COUNT_S = 20;
constexpr uint32_t V1_SOFT_OUTWARD_GRACE_MS = 200;
constexpr uint32_t V1_SOFT_RECOVER_MAX_MS = 2000;
// Issue #46: FOLLOWING_ERROR の猶予（既定 OFF = 従来どおり超えたら即ラッチ。-DV1_FE_GRACE=1 で ON）。ON のとき、上限そのものは変えず、
// 超えていても「エンコーダが生きている（直近 V1_FE_ACTIVITY_MS にカウント変化）」かつ「超えてから V1_FE_TURN_MS 以内か、|誤差| が
// 減っている（V1_FE_WINDOW_MS ごとに V1_FE_MIN_DROP_COUNT 以上）」間は最長 V1_FE_GRACE_MAX_MS ラッチしない（LIMIT_RECOVERY）。
// 戻りの動特性を実機で確かめるまでは既定で使わない（P1c）
#ifndef V1_FE_GRACE
#define V1_FE_GRACE 0
#endif
constexpr uint32_t V1_FE_ACTIVITY_MS = 100;
constexpr uint32_t V1_FE_WINDOW_MS = 100;
constexpr int32_t V1_FE_MIN_DROP_COUNT = 2;
constexpr uint32_t V1_FE_GRACE_MAX_MS = V1_FE_GRACE ? 2000 : 0;  // 0 = 猶予なし
constexpr uint32_t V1_FE_GRACE_VARIANT_MS = 2000;
constexpr uint32_t V1_FE_TURN_MS = 200;             // 超えた直後に減り始めるのを待つ時間（超えた瞬間の |誤差| は必ず増えている）
constexpr int V1_FE_HARD_RATIO_PERCENT = 150;       // |誤差| が上限の 1.5 倍以上なら猶予なしでラッチ（猶予中の行き過ぎの上限）
// r10（Issue #44 の 14:22 の実測、PR #48）: 静止目標での力の段階的な引き上げと、小さな残り誤差の受け入れ。監視（V1_SUPERVISOR）が ON の
// 方向別の経路だけ。どちらの出力も見えない走行の予算に積む。
// A. 引き上げ: 軌道が止まっていて保持でなく |誤差| > V1_RESIDUAL_BAND_COUNT のまま、V1_ESCALATE_WINDOW_MS の窓の間に誤差の向きへ
//    breakaway_min_move（3）進まなければ、誤差の向きの |出力| を max(動き出し Duty, 合成) + V1_ESCALATE_STEP_PERMILLE から窓ごとに
//    V1_ESCALATE_STEP_PERMILLE ずつ duty_max まで上げる（#1 は最下端で 26 count 行き過ぎ、P 78 + I 30 = 10.8 % のまま 2 s 動けず
//    ENCODER_FAULT になった）。進んだ窓では上げずに保つ（下げると止まってまた上げ直す鋸歯になる）。誤差の向きの反転・保持・
//    |誤差| ≤ 帯・軌道が動く・停止で 0 に戻す。duty_max でも進まなければ LOAD_LIMITED（飽和 1 s）か、カウントが全く動かなければ
//    その前に見えない走行の予算（中央付近 400 ms）で停止（どちらも PWM 0 ＋ ブレーキ）。ENCODER_FAULT にはしない。
//    刻みは 50 ms ごとに 3 %: 一律の包絡線の予算（1000 count = 400 ms）の中で、再生バッファ（Issue #45）の参照が目標に寄り切るまでの
//    出力の分を引いても duty_max 20 % に届く（止まってから約 150〜200 ms）ように選んだ（100 ms ごとに 1 % だと 20 % まで 1 s 掛かり、
//    予算が先に尽きる。r10 統合の前は 2 % だったが、参照の寄せの分で 19 % 止まりになった）
constexpr int V1_ESCALATE_STEP_PERMILLE = 30;
constexpr uint32_t V1_ESCALATE_WINDOW_MS = 50;
// B. 小さな残り誤差: 軌道が止まっていて保持でなく |誤差| ≤ V1_RESIDUAL_BAND_COUNT のときは、出力は動き出しパルスだけ（その間の P + I は
//    0。数 permille の P + I は静止摩擦に届かず、熱と見えない走行の予算を使うだけ）。進まないパルスが V1_RESIDUAL_MAX_PULSES 回に
//    なったら残り誤差を受け入れて出力 0（パルスも打たない）。|誤差| > V1_RESIDUAL_EXIT_COUNT か軌道が動くまで続ける。
//    トレードオフ: 静止の精度は ±6 count まで落ちうる（#2 は 6 % で下がらず、0.2〜0.3 s ごとのパルスが体感のガクガクだった）。
//    r6 の「4 で止まったまま P + I が流れ続ける」は起きない（受け入れ中・パルスの合間は出力 0）
constexpr int32_t V1_RESIDUAL_BAND_COUNT = 6;
constexpr int32_t V1_RESIDUAL_EXIT_COUNT = 12;
constexpr int V1_RESIDUAL_MAX_PULSES = 3;
// Issue #45: 再生バッファ（playout）で SET_POSITION の列から参照（位置・速度）を作る。既定 1、-DV1_PLAYOUT=0 で従来の
// 「目標ごとに行って止まる」軌道（profile_step）に戻る（V1Tuning::playout_delay_ms = 0 と同じ）。
// 従来の欠陥: 100 Hz の整数の目標を毎回の移動先として扱い、追いつくと速度を加速度制限を迂回して 0 にしていた → ゆっくり
//（≲ 70 count/s）では FF（切片 4.6 % を |v| < 30 でテーパ）と r9 の減衰が 1 tick で約 50 permille 抜ける。
// 新しい経路: 受信時刻つきでフレームをリングに積み、playout 時刻 = いま − DELAY で、その前後 WINDOW の最小二乗直線から
// 位置・速度を出し、速度・加速度の制限（フレームの max_vel / max_acc。CONFIG で頭打ち）の中で参照を追従させる。
// 新しいフレームが途切れたら EXTRAP_MS までは直線で外挿、それを越えたら max_acc で減速する（期限 valid_for の扱いは従来どおり）。
#ifndef V1_PLAYOUT
#define V1_PLAYOUT 1
#endif
#ifndef V1_PLAYOUT_DELAY_MS
#define V1_PLAYOUT_DELAY_MS 40                            // 再生の遅れ（20 / 40 / 60 を実機で比べる。Issue #45）
#endif
constexpr uint32_t V1_PLAYOUT_WINDOW_MS = 80;             // 軌道の傾きを推定する窓（playout 時刻を中心に ±窓/2）
constexpr uint32_t V1_PLAYOUT_EXTRAP_MS = 40;             // 最新フレームを越えて直線で外挿してよい時間。越えたら減速
constexpr double V1_PLAYOUT_KP_PER_S = 15.0;              // 参照の位置の追従ゲイン [1/s]（推定位置 − 参照 → 速度の補正）。目標が動いている間
constexpr double V1_PLAYOUT_KP_FLAT_PER_S = 40.0;         // 同、目標が止まっている（窓の目標がすべて同じ）とき
// PR #49 レビュー P1a: フレームが途切れても参照が「受けた最新の目標」をこれ以上越えないよう、最新の目標 + これまでに止まれる速さに抑える
constexpr int32_t V1_PLAYOUT_LOSS_MARGIN_COUNT = 5;
// PR #49 レビュー P1b: 減速（再生バッファの制動・途切れ・STOPPING）に使う加速度の下限 [count/s²]。フレームの max_acc が 0・ごく小さくても止まる
constexpr uint32_t V1_MIN_DECEL_COUNT_S2 = 500;
constexpr double V1_PLAYOUT_SNAP_COUNT = 0.5;             // 目標が止まっていて参照がここまで近く、1 tick で止まれるなら速度を 0 に
// Issue #46: ソフトリミットに向かう参照速度の上限 v ≤ sqrt(2 × A_BRAKE × max(0, d − RESERVE))（d = 参照位置から進む向きの
// ソフトリミットまでの距離）。参照は端の RESERVE 手前で止まる。保護（SOFT_LIMIT / FOLLOWING_ERROR）は変えない。
// !!! A_BRAKE・RESERVE は仮値。実機が実際に止まれる減速度と、全速（600 count/s）から止めたときの行き過ぎを計測して決めること。
// r10 統合（PR #48 × #49）: どちらも保護の監視の値から作る（一元化。上の V1_SUP_END_RESERVE_COUNT / V1_MECH_DECEL_ZONE_COUNT /
// V1_SUP_END_MARGIN_COUNT / V1_BRAKE_GUARD_ACC_COUNT_S2 を変えれば両方が動く）。
//  - A_BRAKE = 端の手前の制動（END_BRAKING）の減速度。参照の減速度 ≤ 制動の減速度なので、参照どおりに追従している限り制動は掛からない
//  - RESERVE = 監視の「端へ出せない帯」（機械端から V1_SUP_END_RESERVE_COUNT = ソフトリミットから 400 − 50 = 350）の外に、
//    V1_SUP_END_MARGIN_COUNT（250）だけ見えない走行の予算を残す所 = ソフトリミットから 600。最後の寄せ（ゆっくり近づく間は確かな動き
//    2 count までの包絡線を積む）と最悪の tick（15 ms = 37.5 count）の先読みを、その予算で賄う
constexpr double V1_SOFT_BRAKE_ACC_COUNT_S2 = static_cast<double>(V1_BRAKE_GUARD_ACC_COUNT_S2);
constexpr int32_t V1_SOFT_RESERVE_COUNT = V1_SUP_END_RESERVE_COUNT - V1_MECH_DECEL_ZONE_COUNT + V1_SUP_END_MARGIN_COUNT;
// Issue #15 M7: USB 送信をブロックさせない（include/v1_tx_pump.h）。書く前に TX リングの空きを見て、入らなければ次の poll へ。
constexpr uint32_t V1_TX_STATUS_STALE_MS = 100;  // 積まれてからこれ以上経った STATUS は送らない（20 ms ごとに新しいものが来る。PC の途絶判定 200 ms の半分）
constexpr uint32_t V1_TX_HOLD_WRITE_THROUGH_MS = 100;  // STATUS 以外（ACK など。捨てない）を空き不足で保留してこれだけ経ったら、空きを見ずに書く（HWCDC の待ちは tx timeout 1 ms × 20 = 20 ms で有界。availableForWrite が 0 を返し続ける状態での退路）
constexpr uint32_t V1_TX_TIMEOUT_MS = 1;         // v1 の HWCDC tx timeout（従来 env は 20 ms）。空きを見てから書くので通常は待たない。WriteThrough のときの上限が 20 × これ
constexpr size_t V1_TX_RING_BYTES = 1024;        // HWCDC の TX リング（既定 256 = STATUS 約 5 本）。Serial.begin() の前に setTxBufferSize で広げる
constexpr uint32_t V1_OVERRUN_MS = 15;
// r10 統合: 参照の止まる位置の余裕は、最悪の tick の先読みの包絡線より大きくなければならない
static_assert(static_cast<int64_t>(V1_SUP_END_MARGIN_COUNT) * 1000 >= static_cast<int64_t>(V1_SUP_ENV_COUNT_S) * V1_OVERRUN_MS,
              "V1_SUP_END_MARGIN_COUNT must cover one worst-case tick of the blind-drive envelope");          // 制御 tick の間隔がこれを超えたら CONTROL_OVERRUN（周期 5 ms）
constexpr uint32_t POWER_SETTLE_MS = 300;       // ARMING: 主電源 ON 指令からソフトリミット確認・位置制御開始まで
constexpr uint32_t BRAKE_RELEASE_WAIT_MS = 300; // ARMING: ブレーキ解除指令から HOLDING まで（POWER_SETTLE_MS との和を ARM 全体 5000 ms より十分短く）
constexpr uint32_t BRAKE_APPLY_WAIT_MS = 300;   // DISARMING: ブレーキ給電 OFF から PWM 0・主電源 OFF・DISARMED まで
constexpr uint32_t CAL_POWER_HOLD_TIMEOUT_MS = 120000;  // KEEP_POWER: 校正が終わってから主電源を保つ最長時間（protocol.md §4、仮値）
constexpr int V1_CAL_HOME_DUTY = 13;            // v1 の校正（CALIBRATE）の端当て Duty。従来 env の HOME_DUTY（10 %）では #2 が下限から持ち上がる方向で始動しないことがあった（2026-11-01、2 回）。10/31 の実測で 13 % は 920〜985 count/s。端当ての衝撃は 15 %（従来 env の UPPER の速い区間）より小さい。仮値
constexpr uint32_t CAL_POWER_SETTLE_MS = 5000;  // CALIBRATING: 主電源 ON 指令からブレーキ解除指令まで（protocol.md §1.2）。2026-10-31 に電源投入 4 s 後は動かず
                                                // 12 s 後に動いた記録があるため ARMING の POWER_SETTLE_MS より長い。仮値。電源担当でない軸も同じ時間待つ
