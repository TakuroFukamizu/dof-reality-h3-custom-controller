#pragma once

#include <stdint.h>

// 反転の順序（PWM 0 → coast_us 待つ → DIR 切替 → PWM）を、待ちでブロックせずに守る状態機械（ホストテスト用に純粋）。
// motor_output.cpp がピンへの書き込みに使う。以前は制御 tick の中で delay(DIR_COAST_MS) していて、反転のたびに
// tick の処理時間が 5 ms 余計にかかっていた（実機の画面で exe 6.3 ms。2026-11-02）。
//
// 規則: DIR ピンの向きと逆向きの出力は、PWM が 0 になってから coast_us 経つまで出さない（その間は PWM 0。DIR は変えない）。
// 直接の反転（+70 → −70）だけでなく、0 を挟んだ反転（+70 → 0 → −70）でも 0 の時間が coast_us に満たなければ待つ。
// 同じ向きの変化・0 への変化・起動後最初の出力は待たない。
// 呼び出し側は毎 tick（5 ms）step を呼ぶ: 待っている間に要求を覚えておくのではなく、次の tick の要求で判断し直す
//（その間に要求が 0 や元の向きに戻れば、それに従う）。tick が 5 ms ちょうどなら反転は 1 tick の PWM 0 の後に出る。
struct MotorDirCoast {
  struct Out {
    int permille = 0;       // いま出す値（符号付き。反転待ちの間は 0）
    bool write_dir = false; // DIR ピンを dir に書き換える（PWM を書く前に）
    int8_t dir = 0;         // 書く向き（+1 / −1）
  };

  int8_t dir = 0;              // DIR ピンに最後に書いた向き（+1 / −1。0 = まだ書いていない）
  bool pwm_zero = true;        // いま PWM が 0
  uint64_t zero_since_us = 0;  // PWM が 0 になった時刻

  Out step(int target_permille, uint64_t now_us, uint32_t coast_us) {
    Out o;
    o.dir = dir;
    if (target_permille == 0) {
      to_zero(now_us);
      return o;
    }
    const int8_t want = target_permille > 0 ? 1 : -1;
    if (dir != 0 && want != dir) {
      if (!pwm_zero) {  // 反転の 1 tick 目: PWM を落として待ち始める
        to_zero(now_us);
        return o;
      }
      // 時刻が戻ったら（起こらないはず）経過 0 として待つ側に倒す
      const uint64_t elapsed = now_us > zero_since_us ? now_us - zero_since_us : 0;
      if (elapsed < coast_us) return o;
    }
    if (want != dir) {
      o.write_dir = true;
      dir = want;
    }
    o.dir = dir;
    o.permille = target_permille;
    pwm_zero = false;
    return o;
  }

  // 反転待ちで PWM 0 にしているか（要求は 0 でないのに出していない）
  static bool coasting(int target_permille, const Out& o) { return target_permille != 0 && o.permille == 0; }

 private:
  void to_zero(uint64_t now_us) {
    if (!pwm_zero) {
      pwm_zero = true;
      zero_since_us = now_us;
    }
  }
};
