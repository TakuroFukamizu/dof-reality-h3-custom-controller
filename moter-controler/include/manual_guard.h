#pragma once

// 手動速度（±1 / ±10、S コマンド）の出力中に、フィードバックセンサーの計数が止まった、
// または遅すぎたら出力を 0 にラッチする。自動移動の無パルス停止・遅すぎ停止と同じ窓を使う。
// センサーのケーブルが抜けたまま手動で動かし続けるのを防ぐための守り。
// GPIO・時計を持たない。制御タスクが tick ごとに呼び、戻り値を PWM に出す。
//
// - 速度 0 または主電源リレーが live でないときは計測を止め、ラッチも解除する。
// - |速度| が MANUAL_GUARD_MIN_DUTY（7 %）未満は監視しない（機構が動かないことがあり、+1 で徐々に上げる操作を
//   無パルスで止めてしまうため）。7 % に達した時点から計測を始める。
// - 無パルス: 計測開始から START_GRACE_MS 経過後、カウントが STALL_MS 変わらなければ "stall"。
// - 遅すぎ: |速度| が HOME_DUTY 以上のときだけ、STALL_MS ごとの窓で MIN_COUNT_RATE 未満なら "slow"。
// - ラッチ中は速度指令が残っていても 0 を返す。指令が 0 に戻ったら解除し、次の指令で最初から計測する。

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "auto_move.h"

struct ManualGuard {
  AutoConfig cfg{};
  bool latched = false;
  uint32_t fault_seq = 0;  // ラッチするたびに +1。main が通知に使う
  char reason[8] = "";

  int tick(uint32_t now_ms, int64_t raw, int manual_speed, bool relay_live) {
    if (manual_speed == 0 || !relay_live) {
      running_ = false;
      latched = false;
      return 0;
    }
    if (latched) {
      return 0;
    }
    const int mag = manual_speed < 0 ? -manual_speed : manual_speed;
    if (mag < cfg.manual_min_duty) {
      running_ = false;  // 監視範囲に戻ったら最初から計測する
      return manual_speed;
    }
    if (!running_) {
      running_ = true;
      start_ms_ = now_ms;
      last_raw_ = raw;
      last_change_ms_ = now_ms;
      anchor_raw_ = raw;
      anchor_ms_ = now_ms;
      return manual_speed;
    }
    if (raw != last_raw_) {
      last_raw_ = raw;
      last_change_ms_ = now_ms;
    }
    const bool past_grace = now_ms - start_ms_ >= cfg.start_grace_ms;
    if (past_grace && now_ms - last_change_ms_ >= cfg.stall_ms) {
      latch("stall");
      return 0;
    }
    if (now_ms - anchor_ms_ >= cfg.stall_ms) {
      const int64_t delta = llabs(raw - anchor_raw_);
      const int64_t rate = (delta * 1000) / static_cast<int64_t>(now_ms - anchor_ms_);
      anchor_raw_ = raw;
      anchor_ms_ = now_ms;
      if (past_grace && mag >= cfg.home_duty && rate < cfg.min_count_rate) {
        latch("slow");
        return 0;
      }
    }
    return manual_speed;
  }

 private:
  bool running_ = false;
  uint32_t start_ms_ = 0;
  int64_t last_raw_ = 0;
  uint32_t last_change_ms_ = 0;
  int64_t anchor_raw_ = 0;
  uint32_t anchor_ms_ = 0;

  void latch(const char* why) {
    latched = true;
    fault_seq += 1;
    strncpy(reason, why, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
  }
};
