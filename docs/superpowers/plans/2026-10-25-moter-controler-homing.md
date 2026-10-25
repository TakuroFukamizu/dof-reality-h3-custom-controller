# moter-controler 系統選択・原点復帰・定位置移動 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** moter-controler に、系統 #1〜#3 の選択と NVS 保存、フィードバックセンサーの 4 逓倍計数と表示、P 端・N 端への端当て原点復帰、中立への比例制御移動、校正シーケンスを追加する。

**Architecture:** GPIO と時計を持たない純粋な状態機械（`auto_move.h`）をホストテストで固め、5 ms 周期の制御タスク（`control_task.cpp`）がエンコーダ読み取り・状態機械・PWM 出力を回す。`main.cpp` は UI とシリアルだけを担当し、要求と状態を critical section 越しの構造体コピーで受け渡す。系統プロファイルは Arduino `Preferences`（NVS）に校正完了時だけ書く。

**Tech Stack:** PlatformIO espressif32@6.12.0（Arduino-ESP32 2.x / ESP-IDF 4.4）、M5Unified 0.2.x、`driver/pcnt.h`（legacy）、`Preferences.h`、ホストテストは `c++ -std=c++17`。

**Spec:** `docs/superpowers/specs/2026-10-25-moter-controler-homing-design.md`

## Global Constraints

- プラットフォームは `espressif32@6.12.0` のまま。pioarduino に移さない。ビルドは `/Users/takuro/.platformio/penv/bin/pio run`（PATH の pio は壊れている）。
- ピン: PWM 9、DIR 8、主電源リレー 2、ブレーキリレー 1、エンコーダ A 17、B 18。`DIR_POSITIVE_LEVEL = 0`、`RELAY_ON_LEVEL = 1`。PWM 20 kHz 10 bit、100 % = 1023。
- `brake_on == true` は GPIO1 HIGH ＝ ブレーキ**解除**。UI ラベルは「ブレーキ 解除/作動」。
- 座標: P 端（速度 + で当たる端）が原点 0、N 端へ向かって増える。`ENCODER_SIGN` の既定 +1（速度 − でカウント増）。系統ごとに符号を自動判定して保存する。
- 定数（`config.h`）: `HOME_DUTY=10`, `AUTO_DUTY_MAX=15`, `NEUTRAL_DUTY_MIN=7`, `NEUTRAL_KP_PERCENT_PER_100=1`, `NEUTRAL_BAND=10`, `STALL_MS=200`, `START_GRACE_MS=300`, `MAX_TRAVEL=12000`, `MOVE_TIMEOUT_MS=45000`, `SLOW_ZONE=400`, `ARRIVAL_MIN_TRAVEL=400`, `DWELL_MS=200`, `COUNTS_PER_MOTOR_REV=400`。
- NVS 名前空間 `h3`、キー `motor`, `m{n}_stroke`, `m{n}_neutral`, `m{n}_cal`, `m{n}_sign`。書き込みは校正完了時と系統切り替え時だけ。
- 自動移動の開始条件: 主電源リレー ON かつ settle 済み（`outputLive()`）、ブレーキ解除、手動速度 0、自動移動中でない。中止条件: STOP、非常停止、主電源 OFF、ブレーキ作動、手動速度操作。
- 原点を無効にするのは、系統切り替え、再起動、非常停止の 3 つだけ。
- 既存コマンド `R` `B` `S` `X` `E` `?` `HELP` と既存ボタンは残す。
- コミットメッセージは日本語、末尾に `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` と `Claude-Session: https://claude.ai/code/session_01M5Ahf2987coXT4qNpgXnvh`。

## Review Focus

1. 端当て中にセンサーが止まった（断線・GND 欠落）とき、「到達」と誤判定して原点を取らないこと。開始から `ARRIVAL_MIN_TRAVEL` 未満の移動で止まったら失敗。ただし原点有効で端まで `SLOW_ZONE` 以内に居たときは到達。→ Task 2 のテストで固定。
2. 中立への比例制御で、惰性でバンドを出入りしても PWM の正負が暴れず、`DWELL_MS` 連続でバンド内に居たときだけ完了すること。→ Task 2。
3. 符号が逆の系統（#2・#3 の可能性）で、原点復帰後の「N 端へ」が反対側へ走らないこと。P 端行程でカウントが増えたら `working_sign = −1` に切り替え、以後の position が N 端へ向かって増えること。→ Task 2。
4. 自動移動中の STOP / 非常停止 / 主電源 OFF / ブレーキ作動 / 手動速度が、次の 5 ms tick で duty 0 になること。非常停止では原点も無効になること。→ Task 2（状態機械）と Task 5（制御タスクの配線）。
5. 16 bit PCNT の ±20000 リミット跨ぎで int64 累積が飛ばないこと（ストローク 5556 は 1 回の移動で跨がないが、累積は跨ぐ）。→ Task 3 は ISR なのでホストテストできない。実機で 30000 カウント以上往復して `?` の count が連続することを Task 8 で確認する。

---

## File Structure

- Modify `moter-controler/include/config.h`: エンコーダピンと自動移動の定数を追加。
- Create `moter-controler/include/motor_profile.h`: `MotorProfile` 構造体、端のラベル、中立の既定値。純粋。
- Create `moter-controler/include/auto_move.h`: `AutoMachine` 状態機械。GPIO・時計・NVS に依存しない。純粋。
- Create `moter-controler/include/encoder_pcnt.h`, `moter-controler/src/encoder_pcnt.cpp`: legacy PCNT の 4 逓倍、リミット ISR、int64 累積。
- Create `moter-controler/include/profile_store.h`, `moter-controler/src/profile_store.cpp`: `Preferences` での読み書き。
- Create `moter-controler/include/motor_output.h`, `moter-controler/src/motor_output.cpp`: 既存 `writePwm` / `setupPwm` / `writeMotor` を main.cpp から移す。
- Create `moter-controler/include/control_task.h`, `moter-controler/src/control_task.cpp`: 5 ms タスク、速度窓、共有スナップショット。
- Modify `moter-controler/src/main.cpp`: UI・シリアルの拡張。PWM 書き込みは control_task 経由にする。
- Create `moter-controler/tests/test_host.cpp`, `moter-controler/tests/run_tests.sh`: ホストテスト（encoder-tester と同じ形）。
- Modify `moter-controler/platformio.ini`: `env:m5stack-cores3-usbpower` を追加（開発時は USB のみ給電。Issue #1 回避）。
- Create `moter-controler/docs/acceptance_homing.md`: 実機の受け入れ確認手順と結果欄。

---

### Task 1: ホストテスト基盤と系統プロファイル

**Files:**
- Create: `moter-controler/include/motor_profile.h`
- Create: `moter-controler/tests/test_host.cpp`
- Create: `moter-controler/tests/run_tests.sh`
- Modify: `moter-controler/include/config.h`

**Interfaces:**
- Produces:
  ```cpp
  struct MotorProfile {
    uint8_t motor = 1;       // 1..3
    int32_t stroke = 0;      // P 端から N 端まで。未校正は 0
    int32_t neutral = 0;     // 原点からのカウント
    int8_t sign = 1;         // +1 / -1
    bool calibrated = false;
  };
  const char* p_end_label(uint8_t motor);   // 1,2 → "下限", 3 → "右端"
  const char* n_end_label(uint8_t motor);   // 1,2 → "上限", 3 → "左端"
  int32_t default_neutral(int32_t stroke);  // stroke / 2
  bool profile_motor_ok(uint8_t motor);     // 1..3
  ```
- `config.h` に Global Constraints の定数と `PIN_ENC_A = 17`, `PIN_ENC_B = 18`, `ENCODER_SIGN_DEFAULT = 1` を `constexpr` で追加。

- [ ] **Step 1: `tests/run_tests.sh` を encoder-tester と同じ形で作る**

```sh
#!/bin/sh
set -eu
cd "$(dirname "$0")"
c++ -std=c++17 -Wall -Wextra -I../include test_host.cpp -o /tmp/moter_controler_host
/tmp/moter_controler_host
```
`chmod +x` する。

- [ ] **Step 2: 失敗するテストを書く（`tests/test_host.cpp`）**

最小の `CHECK(cond)` マクロと `main()` を用意し、次を書く。

```cpp
CHECK(std::string(p_end_label(1)) == "下限");
CHECK(std::string(p_end_label(3)) == "右端");
CHECK(std::string(n_end_label(2)) == "上限");
CHECK(std::string(n_end_label(3)) == "左端");
CHECK(default_neutral(5556) == 2778);
CHECK(profile_motor_ok(0) == false && profile_motor_ok(3) == true && profile_motor_ok(4) == false);
MotorProfile p; CHECK(p.sign == 1 && !p.calibrated && p.stroke == 0 && p.motor == 1);
```

- [ ] **Step 3: 実行して失敗を確認**

Run: `moter-controler/tests/run_tests.sh`
Expected: コンパイルエラー（`motor_profile.h` が無い）

- [ ] **Step 4: `motor_profile.h` と `config.h` の定数を実装**

- [ ] **Step 5: 実行して成功を確認**

Run: `moter-controler/tests/run_tests.sh`
Expected: `N passed` と出て終了コード 0

- [ ] **Step 6: Commit**

```bash
git add moter-controler/include/motor_profile.h moter-controler/include/config.h moter-controler/tests
git commit -m "moter-controler に系統プロファイルとホストテスト基盤を足す"
```

---

### Task 2: 自動移動の状態機械（純粋ロジック）

**Files:**
- Create: `moter-controler/include/auto_move.h`
- Modify: `moter-controler/tests/test_host.cpp`

**Interfaces:**
- Consumes: `MotorProfile`, `config.h` の定数。
- Produces:
  ```cpp
  enum class AutoKind : uint8_t { Idle, HomeP, HomeN, Neutral, Cal };
  enum class AutoPhase : uint8_t { Idle, Moving, Dwell, Done, Cancelled, Failed };
  enum class CalStep : uint8_t { None, ToP, ToN, ToNeutral };

  struct AutoConfig {   // 既定値は config.h の定数
    int home_duty, auto_duty_max, neutral_duty_min, kp_percent_per_100, neutral_band;
    uint32_t stall_ms, start_grace_ms, move_timeout_ms, dwell_ms;
    int32_t max_travel, slow_zone, arrival_min_travel;
  };
  struct AutoInputs {
    uint32_t now_ms;
    int64_t raw_count;          // ISR 累積。符号を掛ける前
    bool relay_live;            // outputLive()
    bool brake_released;        // brake_on
    bool manual_speed_nonzero;  // requested_speed != 0
    bool stop;                  // この tick に STOP が来た
    bool estop;                 // この tick に非常停止が来た
  };
  struct AutoMachine {
    AutoConfig cfg{};
    MotorProfile profile{};
    bool origin_valid = false;
    int64_t origin_raw = 0;
    int8_t working_sign = 1;      // 判定前は profile.sign
    AutoKind kind = AutoKind::Idle;
    AutoPhase phase = AutoPhase::Idle;
    CalStep cal_step = CalStep::None;
    int duty_signed = 0;          // 出力。-100..100。+ が P 端向き
    int32_t last_measured_stroke = 0;  // N 端到達時の position
    bool profile_dirty = false;   // 校正完了で true。保存したら呼び手が false に戻す
    char reason[24] = "";         // "stall", "travel", "timeout", "stop", "estop", "relay", "brake", "manual", "ok"

    int32_t position(int64_t raw) const;   // (raw - origin_raw) * working_sign
    bool can_start(AutoKind k, const AutoInputs& in, const char** err) const;
    bool request(AutoKind k, const AutoInputs& in, const char** err);  // can_start → 開始
    void tick(const AutoInputs& in);        // duty_signed を更新
    void invalidate_origin();
    void set_profile(const MotorProfile& p); // 系統切り替え。origin も無効にする
  };
  ```
- 内部規則（実装者が従う。テストが固定する）:
  - 開始時に `start_raw`, `start_ms`, `last_change_ms`, `last_change_raw` を記録。
  - 無パルス: `now - start_ms >= start_grace_ms` かつ `now - last_change_ms >= stall_ms` で停止判定。
  - 端当て（HomeP/HomeN）での停止判定: `|position(now) - position(start)| >= arrival_min_travel` なら到達。または開始時に原点有効で「その端までの距離 <= slow_zone」なら到達。それ以外は `Failed "stall"`。
  - HomeP 到達: `origin_raw = raw`, `origin_valid = true`。到達前に `raw` が開始から **400 以上増えて**いたら（速度 + で増えた）`working_sign = -1`、400 以上減っていたら `+1`。
  - HomeN 到達: `last_measured_stroke = position(raw)`。Cal 中なら `profile.stroke = それ`, `profile.neutral = default_neutral`, `profile.sign = working_sign`, `profile.calibrated = true`, `profile_dirty = true`。
  - 端当ての duty: 原点有効で端までの距離 > slow_zone なら `auto_duty_max`、それ以外 `home_duty`。P 端向きは `+duty`、N 端向きは `-duty`。
  - Neutral: `err = profile.neutral - position`。`|err| <= neutral_band` なら duty 0 で Dwell、`dwell_ms` 連続で Done。それ以外 `duty = clamp(|err| * kp / 100, neutral_duty_min, auto_duty_max)`、符号は `err > 0 → -duty`（N 端向き）、`err < 0 → +duty`。Neutral 中の無パルスは `Failed "stall"`。
  - 移動量上限: `|position(now) - position(start)| > max_travel` で `Failed "travel"`。時間上限: `now - start_ms > move_timeout_ms` で `Failed "timeout"`。
  - 中止: `stop`→"stop", `estop`→"estop"（原点も無効）, `!relay_live`→"relay", `!brake_released`→"brake", `manual_speed_nonzero`→"manual"。`Cancelled`、duty 0。
  - Cal: `ToP` → HomeP 到達 → `ToN` → HomeN 到達 → `ToNeutral` → Neutral Done → Done。途中の失敗・中止で `profile_dirty` は立てない。
  - `can_start`: Idle/Done/Cancelled/Failed のときだけ。`relay_live && brake_released && !manual_speed_nonzero` が必要。HomeN は `origin_valid`（Cal 中を除く）。Neutral は `origin_valid && profile.calibrated`。失敗理由は `err` に "busy" / "not_live" / "brake" / "manual" / "no_origin" / "not_calibrated"。

- [ ] **Step 1: 失敗するテストを書く。1 本の時刻を進めるヘルパーを作る**

```cpp
struct Sim { AutoMachine m; AutoInputs in{}; int64_t raw = 0; 
  Sim() { in.relay_live = true; in.brake_released = true; }
  void step(uint32_t dt_ms, int64_t draw) { in.now_ms += dt_ms; raw += draw; in.raw_count = raw; m.tick(in); in.stop = in.estop = false; } };
```
テスト（名前と主張）:
- `test_home_p_arrives`: `request(HomeP)` → duty_signed == +10。20 ms ごとに raw −12 を 50 回（−600）→ phase Moving。その後 raw 変化 0 を 250 ms → phase Done、`origin_valid`、`position(raw) == 0`、duty 0。
- `test_home_p_stall_without_travel_fails`: request(HomeP) → raw 変化 0 を 600 ms → phase Failed、reason "stall"、`origin_valid == false`。
- `test_home_p_near_end_short_travel_is_arrival`: 先に原点を取り（上のシーケンス）、position が 100（N 端側へ 100 進めた状態）から request(HomeP) → raw 変化 −100 で止まる（stall）→ Done。
- `test_sign_autodetect`: request(HomeP) で raw が **増える**（+12 ×50）→ 到達後 `working_sign == -1`、以後 `position` は raw 減で増える。
- `test_home_n_measures_stroke_and_cal_saves`: Cal を request → HomeP 到達 → 自動で HomeN（duty_signed == −10 か −15）→ raw +5556 相当で到達 → `profile.stroke == 5556`, `neutral == 2778`, `calibrated`, `profile_dirty` → Neutral に入り、position を 2778 に寄せると Dwell → 200 ms で Done。
- `test_home_n_rejected_without_origin`: 新品の machine で request(HomeN) → false、err == "no_origin"。
- `test_neutral_rejected_uncalibrated`: 原点ありでも `profile.calibrated == false` なら err "not_calibrated"。
- `test_neutral_p_control`: calibrated, stroke 5556, neutral 2778, position 0 から request(Neutral) → duty_signed == −15（上限クランプ）。position 2700（err 78 → 0.78 % → 下限 7）→ duty_signed == −7。position 2800（err −22）→ +7。position 2778 → 0、Dwell。Dwell 中に 2790 へ出る → Moving に戻る。2775 で 200 ms → Done。
- `test_slow_zone`: 原点あり、位置 1000 から request(HomeP) → duty +15。位置 300 → +10。
- `test_cancel_reasons`: Moving 中に `in.stop = true` → Cancelled "stop" duty 0。別インスタンスで `relay_live=false` → "relay"、`brake_released=false` → "brake"、`manual_speed_nonzero=true` → "manual"、`estop=true` → "estop" かつ `origin_valid == false`。
- `test_travel_and_timeout`: request(HomeP) で raw が止まらず −12001 → Failed "travel"。別インスタンスで 45 001 ms 動き続ける（毎 tick 変化あり）→ Failed "timeout"。
- `test_can_start_conditions`: `relay_live=false` → "not_live"。Moving 中の request → "busy"。
- `test_set_profile_invalidates_origin`: 原点取得後 `set_profile(別系統)` → `origin_valid == false`。

- [ ] **Step 2: 実行して失敗を確認**

Run: `moter-controler/tests/run_tests.sh`
Expected: コンパイルエラー（`auto_move.h` が無い）

- [ ] **Step 3: `auto_move.h` を実装**

ヘッダのみ（`inline`）。`cfg` の既定値は `config.h` の定数から初期化する。

- [ ] **Step 4: 実行して成功を確認**

Run: `moter-controler/tests/run_tests.sh`
Expected: 全件 passed

- [ ] **Step 5: Commit**

```bash
git add moter-controler/include/auto_move.h moter-controler/tests/test_host.cpp
git commit -m "自動移動の状態機械を追加する（端当て原点復帰、中立への比例制御、校正）"
```

---

### Task 3: PCNT エンコーダ読み取りと USB 給電ビルド環境

**Files:**
- Create: `moter-controler/include/encoder_pcnt.h`, `moter-controler/src/encoder_pcnt.cpp`
- Modify: `moter-controler/platformio.ini`
- Modify: `moter-controler/src/main.cpp`（`cfg.output_power` と `setUsbOutput` を `MC_USB_POWER` で切替。encoder-tester の `ENCODER_TESTER_USB_POWER` と同じ形）

**Interfaces:**
- Produces:
  ```cpp
  bool encoder_begin();          // PCNT unit 0 を 4 逓倍で起動。失敗で false
  int64_t encoder_count();       // ISR 累積 + 現在値。符号は生（速度 − で増）
  int encoder_a_level();         // digitalRead(PIN_ENC_A)
  int encoder_b_level();
  ```
- 実装の決め: `pcnt_unit_config_t` ×2。ch0: `pulse_gpio_num=PIN_ENC_A, ctrl_gpio_num=PIN_ENC_B, pos_mode=PCNT_COUNT_DEC, neg_mode=PCNT_COUNT_INC, lctrl_mode=PCNT_MODE_REVERSE, hctrl_mode=PCNT_MODE_KEEP`。ch1: `pulse=PIN_ENC_B, ctrl=PIN_ENC_A, pos_mode=PCNT_COUNT_INC, neg_mode=PCNT_COUNT_DEC, lctrl=REVERSE, hctrl=KEEP`。`counter_h_lim=20000, counter_l_lim=-20000`。`PCNT_EVT_H_LIM` / `PCNT_EVT_L_LIM` の ISR（`pcnt_isr_service_install` + `pcnt_isr_handler_add`）で `volatile int64_t overflow` に ±20000 を足す。フィルタ無効（`pcnt_filter_disable`）。`gpio_pullup_dis` / `gpio_pulldown_dis` を両ピンに。読み出しは「counter → overflow → counter」で一致するまで最大 3 回。

- [ ] **Step 1: `platformio.ini` に `env:m5stack-cores3-usbpower` を追加**

既存 env に `-DMC_USB_POWER=0` を足し、新 env は `extends = env:m5stack-cores3` で `-DMC_USB_POWER=1`。コメントに「DIN 12V と USB の同時接続で電源断（Issue #1）。開発時は USB のみ」と書く。

- [ ] **Step 2: `main.cpp` の電源設定を `#if MC_USB_POWER` で切替**

`MC_USB_POWER=1` のとき `cfg.output_power = true` で `setUsbOutput(false)` を呼ばない。既定（0）は現状どおり。

- [ ] **Step 3: `encoder_pcnt.{h,cpp}` を実装**

- [ ] **Step 4: `main.cpp` の `setup()` で `encoder_begin()` を呼び、失敗なら `Serial.println("ERR pcnt")`。`publish()` の行末に ` ENC=<count> A=<a> B=<b>` を仮に足す**

- [ ] **Step 5: 両環境でビルド**

Run: `cd moter-controler && /Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3 && /Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-usbpower`
Expected: 両方 SUCCESS

- [ ] **Step 6: 実機で計数を確認（USB のみ給電、GND 共通を確認）**

Run: `usbpower` を書き込み、`?` を送り `ENC=` が出ること。手動 `S -10` → 1 秒 → `S 0` → `?` で ENC が増えていること（速度 − で増）。`S 10` で減ること。
Expected: 符号が Global Constraints どおり。異なれば `encoder_pcnt.cpp` の pos/neg を見直す（`ENCODER_SIGN_DEFAULT` は変えない）。

- [ ] **Step 7: Commit**

```bash
git add moter-controler/include/encoder_pcnt.h moter-controler/src/encoder_pcnt.cpp moter-controler/platformio.ini moter-controler/src/main.cpp
git commit -m "PCNT でフィードバックセンサーを 4 逓倍計数し、USB 給電ビルドを足す"
```

---

### Task 4: NVS プロファイル保存

**Files:**
- Create: `moter-controler/include/profile_store.h`, `moter-controler/src/profile_store.cpp`

**Interfaces:**
- Consumes: `MotorProfile`。
- Produces:
  ```cpp
  bool profile_store_begin();                    // Preferences.begin("h3", false)
  uint8_t profile_store_selected_motor();        // 無ければ 1
  void profile_store_select_motor(uint8_t m);    // "motor" を書く
  MotorProfile profile_store_load(uint8_t m);    // 無ければ既定（stroke 0, calibrated false, sign +1）
  bool profile_store_save(const MotorProfile& p);// m{n}_stroke / neutral / cal / sign を書く
  ```
- キー名は `m1_stroke` のように `snprintf("m%u_%s")` で作る。

- [ ] **Step 1: 実装**

- [ ] **Step 2: ビルド**

Run: `/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-usbpower`
Expected: SUCCESS

- [ ] **Step 3: Commit**

```bash
git add moter-controler/include/profile_store.h moter-controler/src/profile_store.cpp
git commit -m "系統プロファイルを NVS に保存する"
```

---

### Task 5: 制御タスクと PWM 出力の集約

**Files:**
- Create: `moter-controler/include/motor_output.h`, `moter-controler/src/motor_output.cpp`（既存 `writePwm`, `setupPwm`, `writeMotor`, `applied_speed` を移す）
- Create: `moter-controler/include/control_task.h`, `moter-controler/src/control_task.cpp`
- Modify: `moter-controler/src/main.cpp`（`writeMotor` 直接呼び出しを無くし、要求を control task に渡す）

**Interfaces:**
- `motor_output.h`:
  ```cpp
  void motor_output_begin();        // setupPwm
  void motor_output_write(int percent);  // 既存 writeMotor（反転時 PWM 0 → 5 ms → DIR）
  int motor_output_applied();
  ```
- `control_task.h`:
  ```cpp
  struct ControlInputs {            // main → task（critical section で丸ごとコピー）
    bool relay_live; bool brake_released; int manual_speed;
    bool stop_pulse; bool estop_pulse;       // task が読んだら消す
    AutoKind request; bool request_pulse;    // 自動移動の開始要求
    MotorProfile profile; bool profile_pulse; // 系統切り替え（set_profile）
  };
  struct ControlSnapshot {          // task → main
    int64_t raw_count; int32_t position; bool origin_valid; int count_per_s;
    int a_level, b_level; int applied_speed;
    AutoKind kind; AutoPhase phase; CalStep cal_step; char reason[24];
    MotorProfile profile; bool profile_dirty; int32_t last_measured_stroke; int8_t working_sign;
    char request_error[16];
  };
  void control_begin(const MotorProfile& initial);
  void control_set_inputs(const ControlInputs& in);   // 部分更新用に個別 setter を作ってもよい
  void control_request(AutoKind k);
  void control_pulse_stop(); void control_pulse_estop();
  void control_set_profile(const MotorProfile& p);
  void control_clear_profile_dirty();
  ControlSnapshot control_snapshot();
  ```
- タスクの tick（5 ms、core 1、優先度 5）: `encoder_count()` → `AutoInputs` を組む → 要求があれば `request()`（err は snapshot に）→ `tick()` → 出力: 自動移動 Moving/Dwell 中は `duty_signed`、それ以外は `relay_live ? manual_speed : 0` を `motor_output_write` に渡す → 100 ms 窓の count/s → snapshot。
- 手動速度が 0 以外になった tick は `AutoInputs.manual_speed_nonzero = true` になり、状態機械が自分で Cancelled になる。

- [ ] **Step 1: `motor_output.{h,cpp}` を切り出し、main.cpp をそれを使う形に直してビルド**

Run: `/Users/takuro/.platformio/penv/bin/pio run -e m5stack-cores3-usbpower`
Expected: SUCCESS。動作は変わらない。

- [ ] **Step 2: `control_task.{h,cpp}` を実装。main.cpp の `setSpeed` / `estop` / `setRelay` / `setBrake` は control の入力を更新するだけにし、PWM は task だけが書く**

`applyOutput()` と `was_live` の処理は task に移る（`relay_live` を毎 loop で渡す）。

- [ ] **Step 3: ビルドし、実機で既存の手動操作が変わらないことを確認**

Run: 書き込み → `R 1` → `B 1` → `S 10` → `S 0` → `S -10` → `X` → `E`。
Expected: 従来どおり動き、`?` の `ENC=` が追従する。

- [ ] **Step 4: Commit**

```bash
git add moter-controler/include moter-controler/src
git commit -m "制御タスクを追加し、PWM 出力を 5 ms 周期の 1 箇所に集約する"
```

---

### Task 6: シリアルコマンドと ENC / AUTO 行

**Files:**
- Modify: `moter-controler/src/main.cpp`

**Interfaces:**
- 追加コマンド: `M 1|2|3`（relay OFF かつ自動移動中でないときだけ。`profile_store_select_motor` → `profile_store_load` → `control_set_profile`。それ以外は `ERR busy`）、`HOME` → `control_request(HomeP)`、`UPPER` → `HomeN`、`NEUTRAL` → `Neutral`、`CAL` → `Cal`。
- `?` の出力に 1 行追加:
  `ENC motor=%u count=%lld pos=%ld cps=%d a=%d b=%d origin=%d cal=%d stroke=%ld neutral=%ld sign=%d auto=%s phase=%s reason=%s`
  `auto` は `idle|home|upper|neutral|cal`、`phase` は `idle|moving|dwell|done|cancelled|failed`。
- 自動移動が Moving/Dwell のあいだ 500 ms ごとに `ENC` 行を出す。`phase` が変わった tick に `AUTO <auto> <phase> <reason> pos=<pos>` を出す。
- `profile_dirty` を見たら `profile_store_save` → `control_clear_profile_dirty` → `AUTO saved motor=<n> stroke=<s> neutral=<n> sign=<±1>`。
- 警告（仕様の ±5 % / ±10 %）: `upper done` のとき `profile.calibrated` なら `|last_measured_stroke - profile.stroke| > profile.stroke / 20` で `AUTO warn stroke measured=<m> saved=<s>`。`home done` のとき開始前に原点が有効だったなら、到達時の旧座標での position の絶対値が `profile.stroke / 10` を超えていれば `AUTO warn origin_shift=<pos>`（旧原点との差）。どちらも動作は止めない。
- HELP に新コマンドを足す。

- [ ] **Step 1: 実装**

- [ ] **Step 2: ビルドして実機でシリアルから一巡**

Run: `M 1` → `R 1` → `B 1` → `HOME` → `AUTO home done ok` を待つ → `UPPER` → `AUTO upper done ok`（`ENC` の `stroke` はまだ保存値）→ `CAL` → `AUTO saved ...` → `NEUTRAL` → `AUTO neutral done ok`。
Expected: 途中の `ENC` 行が 500 ms ごとに出る。`X` で `AUTO ... cancelled stop`。

- [ ] **Step 3: Commit**

```bash
git add moter-controler/src/main.cpp
git commit -m "系統選択と自動移動のシリアルコマンド、ENC / AUTO 行を足す"
```

---

### Task 7: 画面

**Files:**
- Modify: `moter-controler/src/main.cpp`（`layoutButtons`, `render`, `onButton`, `Action`）

**Interfaces:**
- `Action` に `Motor, HomeP, HomeN, Neutral, Cal` を追加。`buttons` は 13 個。
- レイアウト（320×240、こちらの判断。実装者が微調整してよい）: ヘッダー y 0–26（左に `#1` のタップ領域 48 px、右に状態）、電源/ブレーキ y 30 h 34、センサー行 y 68 h 30（カウント `Font4`、右に `cps`、`A B`、`原点`/`校正` の有無）、速度 y 102 h 34（数字は `-10 -1 [n] +1 +10` のように中央の数字を小さく）、自動 y 140 h 34（4 つ）、停止/非常停止 y 178 h 38、フッター y 222。
- ボタンの有効条件は `control_snapshot()` と `AutoMachine::can_start` と同じ規則で決め、無効は灰色。系統ボタンは relay OFF かつ phase が Moving/Dwell でないときだけ。
- ラベル: ブレーキは「ブレーキ 解除」「ブレーキ 作動」。自動は `p_end_label` / `n_end_label` / 「中立へ」 / 「校正」。
- 状態欄: 既存の 停止/待機/運転 に加え、自動移動中は `下限へ` `上限へ` `中立へ` `校正中`、終了後 1.2 秒は `到達` / `中止` / `失敗:<reason>`、未校正の系統なら常に「未校正」を右端に出す。
- 描画は snapshot を 50 ms ごとに取り、値が変わったら `ui_dirty`。

- [ ] **Step 1: 実装**

- [ ] **Step 2: ビルドして実機でタッチ操作を一巡**

Run: 系統タップで #1→#2→#3→#1。電源 → ブレーキ解除 → 下限へ → 上限へ → 校正 → 中立へ。各ボタンの灰色/有効が条件どおり。
Expected: 速度ボタンを押すと自動移動が中止になり状態欄に「中止」。

- [ ] **Step 3: Commit**

```bash
git add moter-controler/src/main.cpp
git commit -m "系統選択、センサー表示、自動移動ボタンを画面に足す"
```

---

### Task 8: 実機の受け入れ確認と記録

**Files:**
- Create: `moter-controler/docs/acceptance_homing.md`
- Modify: `moter-controler/README.md`（あれば。無ければ作る: ビルド環境 2 つ、コマンド一覧、ホストテストの実行）

- [ ] **Step 1: 仕様の「受け入れ確認（実機）」1〜9 を表にした `acceptance_homing.md` を作る。列は 手順 / 期待 / 結果 / 日時**

- [ ] **Step 2: USB のみ給電・GND 共通を確認して、#1 で 1〜8 を実施し、結果を書く**

Review Focus 5 の確認もここで: `S -15` → 20 秒 → `X` → `S 15` → 20 秒 → `X` を 2 往復（約 ±30000 カウント相当は端で止まるので、代わりに「上限へ」「下限へ」を 6 往復して `?` の `count` が毎回連続していること）。

- [ ] **Step 3: 項目 7（センサー 1 本外し）は、ブレーキ解除・電源 ON のまま線を外さない。電源 OFF で外し、`HOME` → `START_GRACE_MS` 内に `AUTO home failed stall` が出ることを確認**

- [ ] **Step 4: #2・#3 は、ケーブルを差し替えたあと `M 2` / `M 3` → 「校正」。止まりに当たることを目視。符号判定の結果（`sign=`）を記録**

- [ ] **Step 5: `encoder-tester/docs/results_20261025.md` の「未確定」に、#2・#3 のストロークと符号を追記**

- [ ] **Step 6: Commit**

```bash
git add moter-controler/docs moter-controler/README.md encoder-tester/docs/results_20261025.md
git commit -m "原点復帰と定位置移動の実機受け入れ結果を記録する"
```
