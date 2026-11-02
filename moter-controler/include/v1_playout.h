#pragma once

// Issue #45: SET_POSITION の再生バッファ（playout）。GPIO・時計を持たない純粋ロジック（V1Axis が使う）。
//
// 時刻の付け直し（V1PlayoutClock）: 受信時刻そのものは USB のぶれ（±数 ms）と、UI ループが止まった間に溜まって同じ時刻に
// まとめて届くフレームで乱れる（まとめて届いた数フレームが同じ時刻に縦に並ぶと傾きの推定が大きく崩れる。試験で 25 count/s が
// 一瞬 −1 count/s まで落ちた）。サーバーの planner は一定周期で seq を 1 つずつ進める（supervisor.rs）ので、直近 kHist 個の
// （seq, 受信時刻）に最小二乗直線「受信時刻 = a + P × seq」を当て、残差の最小値まで a を下げた（＝いちばん遅れの少なかった
// フレームに合わせた）直線をフレームの時刻とする。ぶれ・まとめ届き・欠落（seq が飛ぶ）が時刻に出ない。点が少ない・周期が
// おかしい（0.5〜200 ms の外）ときは受信時刻をそのまま使う。
//  - 取り直し: seq が戻った（epoch が変わった）・seq が 1000 以上飛んだ・受信が 500 ms 以上空いた。V1Axis は CONFIG の受理・
//    系統の切り替えでも reset() する。
//  - 周期の変化（PR #49 レビュー P2a）: 直近 4 間隔の (受信時刻の差 / seq の差) が推定周期の 0.6 倍未満か 1.6 倍超で
//    2 フレーム続いたら、直近 kKeep 個だけ残して取り直す。間隔がほぼ 0（推定周期の 0.3 倍未満）の点を含む間は数えない
//   （40 ms の途切れの後のまとめ届き。途切れの直後の 1 間隔は長いが、続く 0 の間隔で数え直す）。100 → 50 Hz は約 4 フレーム
//   （80 ms）、50 → 100 Hz は約 5 フレーム（50 ms）で取り直す。取り直す前のフレームは前の直線の時刻のまま（time_of）。
//    取り直した直後で点が足りない間は受信時刻のまま。その間も starved は受信時刻で決めるので、届いている限り参照は止まらない。
//  - 計算量（P2c）: push は O(1)（周期の変化の判定だけ）。直線の当てはめは refit_if_dirty() で 1 tick に 1 回まで（kHist 点 ×
//    3 パス、単精度）。時刻・seq は最新を原点にした差で float にする（320 ms 分の µs は float の仮数 24 bit に収まる）。
//
// 再生（V1PlayoutBuffer）: ACTIVE で受けた目標（整数 count、ソフトリミットで clamp 済み）をリングに積み、playout 時刻 tp
//（= いま − 遅れ）での軌道の位置と速度を、tp を中心とする窓のフレームの三角重み付き最小二乗直線で推定する（単精度。時刻は tp
// からの秒、位置は最新の目標からの差）。
//  - 隣り合う整数の差ではなく窓全体の傾きなので、ゆっくり（1 フレームの進みが 0〜1 count）でも速度が 0 と数十 count/s を
//    行き来しない。直線の軌道では位置・速度とも一致する。角（止まる・反転）は窓の幅で丸まる。
//  - 窓の中の目標がすべて同じ（整数で比べる）なら傾きはちょうど 0、位置はその目標そのもの。V1Axis はこれを「目標が止まった」
//    として参照を止める（保持・整定の判定は setpoint_vel_ == 0.0 を見る）。
//  - tp より後のフレームがまだ届いていなければ（遅れ・途切れ）窓を過去へ広げ（幅は常に窓以上）、直線を外挿する。
//  - starved（新しい目標が途切れた）は**受信時刻**で決める: 最新フレームの受信から 遅れ + extrap を越えた（tp > 最新の受信 +
//    extrap）。付け直した時刻では決めない（周期の変化などで付け直しが狂っても、届いている間は止めない）。
//  - まだ tp に届くフレームが無い（開始直後）なら has = false。

#include <math.h>
#include <stddef.h>
#include <stdint.h>

class V1PlayoutClock {
 public:
  static constexpr size_t kHist = 32;     // 10 ms のフレームで 320 ms 分（40 ms の途切れが 3 回入っても下側の包絡が取れる）
  static constexpr size_t kMinPoints = 5;
  static constexpr size_t kKeep = 5;      // 周期の変化で取り直すとき残す点（直近 4 間隔 = 新しい周期の分）
#ifndef V1_PLAYOUT_CADENCE_SPAN
#define V1_PLAYOUT_CADENCE_SPAN 4
#endif
#ifndef V1_PLAYOUT_CADENCE_RUNS
#define V1_PLAYOUT_CADENCE_RUNS 2
#endif
  static constexpr size_t kSpan = V1_PLAYOUT_CADENCE_SPAN;      // 周期の変化を見る間隔の数（まとめ届きは和では周期どおり。40 ms の途切れでも比 ≤ 約 1.45）
  static constexpr int kCadenceRuns = V1_PLAYOUT_CADENCE_RUNS;  // 外れが続いたら取り直す数

  void reset() {
    head_ = 0;
    count_ = 0;
    valid_ = false;
    dirty_ = false;
    off_runs_ = 0;
    has_prev_ = false;
  }
  bool valid() const { return valid_; }
  float period_us() const { return period_us_; }
  uint32_t reseeds() const { return reseeds_; }  // 周期の変化で取り直した回数（計器・試験用）

  // O(1)。当てはめは refit_if_dirty() まで遅らせる
  void push(uint32_t seq, uint64_t rx_us) {
    if (count_ > 0) {
      const Entry& last = at(count_ - 1);
      if (seq <= last.seq || seq - last.seq > 1000u || rx_us < last.rx || rx_us - last.rx > 500000ull) reset();
    }
    append(seq, rx_us);
    dirty_ = true;
    // 周期の変化（直近 kSpan 間隔の平均周期と推定周期の比）
    if (valid_ && count_ > kSpan) {
      const Entry& a = at(count_ - 1 - kSpan);
      const Entry& b = at(count_ - 1);
      const float p = static_cast<float>(b.rx - a.rx) / static_cast<float>(b.seq - a.seq);
      const float ratio = p / period_us_;
      // まとめ届き（間隔がほぼ 0 の点を含む）は周期の変化ではない（途切れの後）: 数え直す
      bool batched = false;
      for (size_t i = count_ - kSpan; i < count_; ++i) {
        const Entry& e0 = at(i - 1);
        const Entry& e1 = at(i);
        if (static_cast<float>(e1.rx - e0.rx) < 0.3f * period_us_ * static_cast<float>(e1.seq - e0.seq)) batched = true;
      }
      if (batched) {
        off_runs_ = 0;
      } else if (ratio < 0.6f || ratio > 1.6f) {
        if (++off_runs_ >= kCadenceRuns) {
          reseed();
          ++reseeds_;
        }
      } else {
        off_runs_ = 0;
      }
    }
  }

  // 直線を当て直す（push の後、1 tick に 1 回まで）
  void refit_if_dirty() {
    if (!dirty_) return;
    dirty_ = false;
    refit();
  }

  // seq のフレームの時刻（付け直し）。使えなければ受信時刻 rx_us のまま
  // 周期の変化で取り直す前のフレーム（seq < 境界）は、取り直す前の直線のまま（新しい周期で付け直すと昔のフレームの間隔が狂う）
  uint64_t time_of(uint32_t seq, uint64_t rx_us) const {
    if (has_prev_ && static_cast<int32_t>(seq - prev_boundary_seq_) < 0) return prev_.at(seq);
    if (!valid_) return rx_us;
    return line_.at(seq);
  }

 private:
  struct Entry {
    uint32_t seq;
    uint64_t rx;
  };
  // 時刻の直線: t = ref_rx + base_off + P × (seq − ref_seq)
  struct Line {
    float period_us = 0.0f;
    float base_off_us = 0.0f;
    uint32_t ref_seq = 0;
    uint64_t ref_rx = 0;
    uint64_t at(uint32_t seq) const {
      const float off = base_off_us + period_us * static_cast<float>(static_cast<int32_t>(seq - ref_seq));
      const int64_t t = static_cast<int64_t>(ref_rx) + static_cast<int64_t>(lroundf(off));
      return t <= 0 ? 0 : static_cast<uint64_t>(t);
    }
  };
  const Entry& at(size_t i) const { return buf_[(head_ + i) % kHist]; }  // 0 = 最古
  void append(uint32_t seq, uint64_t rx_us) {
    buf_[(head_ + count_) % kHist] = Entry{seq, rx_us};
    if (count_ < kHist) {
      ++count_;
    } else {
      head_ = (head_ + 1) % kHist;
    }
  }
  void reseed() {
    Entry keep[kKeep];
    const size_t n = count_ < kKeep ? count_ : kKeep;
    for (size_t i = 0; i < n; ++i) keep[i] = at(count_ - n + i);
    if (valid_ && n > 0) {
      prev_ = line_;
      prev_boundary_seq_ = keep[0].seq;
      has_prev_ = true;
    }
    head_ = 0;
    count_ = 0;
    for (size_t i = 0; i < n; ++i) append(keep[i].seq, keep[i].rx);
    valid_ = false;
    dirty_ = true;
    off_runs_ = 0;
  }

  void refit() {
    valid_ = false;
    if (count_ < kMinPoints) return;
    const Entry& nw = at(count_ - 1);
    // 最新を原点に: x = seq − seq_new（≤ 0）、y = rx − rx_new [us]（≤ 0）
    float sx = 0.0f, sy = 0.0f;
    for (size_t i = 0; i < count_; ++i) {
      sx -= static_cast<float>(nw.seq - at(i).seq);
      sy -= static_cast<float>(nw.rx - at(i).rx);
    }
    const float n = static_cast<float>(count_);
    const float xm = sx / n, ym = sy / n;
    float sxx = 0.0f, sxy = 0.0f;
    for (size_t i = 0; i < count_; ++i) {
      const float x = -static_cast<float>(nw.seq - at(i).seq) - xm;
      const float y = -static_cast<float>(nw.rx - at(i).rx) - ym;
      sxx += x * x;
      sxy += x * y;
    }
    if (sxx <= 0.0f) return;
    const float p = sxy / sxx;
    if (!(p >= 500.0f && p <= 200000.0f)) return;  // 0.5〜200 ms の外は信用しない
    const float a = ym - p * xm;  // x = 0（最新の seq）での直線の値
    float rmin = 0.0f;
    for (size_t i = 0; i < count_; ++i) {
      const float x = -static_cast<float>(nw.seq - at(i).seq);
      const float y = -static_cast<float>(nw.rx - at(i).rx);
      const float r = y - (a + p * x);
      if (i == 0 || r < rmin) rmin = r;
    }
    period_us_ = p;
    line_.period_us = p;
    line_.ref_seq = nw.seq;
    line_.ref_rx = nw.rx;
    line_.base_off_us = a + rmin;  // 下側の包絡（いちばん遅れの少ないフレーム）に合わせる
    valid_ = true;
  }

  Entry buf_[kHist]{};
  size_t head_ = 0;
  size_t count_ = 0;
  bool valid_ = false;
  bool dirty_ = false;
  int off_runs_ = 0;
  uint32_t reseeds_ = 0;
  float period_us_ = 0.0f;
  Line line_{};
  Line prev_{};
  bool has_prev_ = false;
  uint32_t prev_boundary_seq_ = 0;
};

struct V1PlayoutFit {
  bool has = false;      // tp 以前のフレームがある
  bool starved = false;  // 最新フレームの受信から 遅れ + extrap を越えた（新しい目標が途切れている）
  double pos = 0.0;      // 推定位置 [count]
  double vel = 0.0;      // 推定速度 [count/s]（窓の目標がすべて同じならちょうど 0）
  uint8_t n = 0;         // 使った点の数（計器用）
};

class V1PlayoutBuffer {
 public:
  static constexpr size_t kCap = 32;  // 10 ms のフレームで 320 ms 分（窓 80 ＋ 遅れ 40 ＋ 途切れの余裕）

  void clear() {
    head_ = 0;
    count_ = 0;
  }
  size_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  int32_t newest_pos() const { return count_ ? at(count_ - 1).pos : 0; }

  // seq・受信時刻 rx_us の目標 pos を積む（受信順）。満杯なら最古を捨てる。O(1)
  void push(uint32_t seq, uint64_t rx_us, int32_t pos) {
    buf_[(head_ + count_) % kCap] = Sample{seq, rx_us, pos};
    if (count_ < kCap) {
      ++count_;
    } else {
      head_ = (head_ + 1) % kCap;
    }
  }

  V1PlayoutFit fit(uint64_t tp, uint64_t window_us, uint64_t extrap_us, const V1PlayoutClock& clk) const {
    V1PlayoutFit f;
    if (count_ == 0) return f;
    // フレームの時刻（付け直し。受信順に単調にする）を tp からの秒（float）で
    float r[kCap];
    uint64_t prev = 0;
    for (size_t i = 0; i < count_; ++i) {
      uint64_t t = clk.time_of(at(i).seq, at(i).rx);
      if (i > 0 && t < prev) t = prev;
      prev = t;
      r[i] = t >= tp ? static_cast<float>(t - tp) * 1e-6f : -static_cast<float>(tp - t) * 1e-6f;
    }
    if (r[0] > 0.0f) return f;
    f.has = true;
    const uint64_t rx_new = at(count_ - 1).rx;
    if (tp > rx_new && tp - rx_new > extrap_us) f.starved = true;
    // 窓: tp を中心に ±窓/2。tp より後のフレームがまだ届いていなければ（遅れ・途切れ）その分だけ過去へ広げ、幅は常に窓以上
    //（片側の短い窓だと、ゆっくりの軌道で同じ整数の目標しか入らず傾きがちょうど 0 になり「止まった」と誤認する）。
    // 重みは窓の中心で 1・両端で 0 の三角（点が窓に出入りしても推定が段にならない）。窓の外でも tp を挟む前後の 1 点ずつは
    // 小さな重みで必ず入れる（欠落で窓が空にならないように）。
    size_t before = 0;  // tp 以前で最新
    for (size_t i = 0; i < count_; ++i) {
      if (r[i] <= 0.0f) before = i;
    }
    const size_t after = before + 1;  // tp より後で最古（count_ なら無し）
    const float half = static_cast<float>(window_us) * 0.5e-6f;  // [s]
    float w_hi = half;
    const float r_new = r[count_ - 1];
    if (r_new < w_hi) w_hi = r_new > 0.0f ? r_new : 0.0f;
    const float w_lo = w_hi - 2.0f * half;
    const float c = 0.5f * (w_lo + w_hi);
    const float hw = half + 1e-4f;  // 端の点も重み ≈ 0（0 で割らない）
    const int32_t y_ref = at(count_ - 1).pos;  // 位置は最新の目標からの差で float にする
    float wt[kCap];
    float sw = 0.0f, st = 0.0f, sy = 0.0f;
    bool flat = true;
    bool first = true;
    int32_t y0 = 0;
    uint8_t n = 0;
    for (size_t i = 0; i < count_; ++i) {
      float w = 0.0f;
      if (r[i] >= w_lo && r[i] <= w_hi) w = 1.0f - fabsf(r[i] - c) / hw;
      if ((i == before || i == after) && w < 0.05f) w = 0.05f;
      wt[i] = w;
      if (w <= 0.0f) continue;
      const int32_t y = at(i).pos;
      if (first) {
        y0 = y;
        first = false;
      } else if (y != y0) {
        flat = false;
      }
      sw += w;
      st += w * r[i];
      sy += w * static_cast<float>(y - y_ref);
      ++n;
    }
    f.n = n;
    if (flat) {  // 窓の中の目標がすべて同じ: 傾きちょうど 0、位置はその目標（重み付きの平均の丸めを持ち込まない）
      f.vel = 0.0;
      f.pos = static_cast<double>(y0);
      return f;
    }
    const float tm = st / sw;
    const float ym = sy / sw;
    float stt = 0.0f, sty = 0.0f;
    for (size_t i = 0; i < count_; ++i) {
      if (wt[i] <= 0.0f) continue;
      const float dt = r[i] - tm;
      stt += wt[i] * dt * dt;
      sty += wt[i] * dt * (static_cast<float>(at(i).pos - y_ref) - ym);
    }
    const float b = stt > 1e-12f ? sty / stt : 0.0f;
    f.vel = static_cast<double>(b);
    f.pos = static_cast<double>(y_ref) + static_cast<double>(ym - b * tm);  // tp（相対 0）での直線の値
    return f;
  }

 private:
  struct Sample {
    uint32_t seq;
    uint64_t rx;
    int32_t pos;
  };
  const Sample& at(size_t i) const { return buf_[(head_ + i) % kCap]; }  // 0 = 最古

  Sample buf_[kCap]{};
  size_t head_ = 0;
  size_t count_ = 0;
};
