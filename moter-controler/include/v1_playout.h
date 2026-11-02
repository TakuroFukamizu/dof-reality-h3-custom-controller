#pragma once

// Issue #45: SET_POSITION の再生バッファ（playout）。GPIO・時計を持たない純粋ロジック（V1Axis が使う）。
//
// 時刻の付け直し（V1PlayoutClock）: 受信時刻そのものは USB のぶれ（±数 ms）と、UI ループが止まった間に溜まって同じ時刻に
// まとめて届くフレームで乱れる（まとめて届いた数フレームが同じ時刻に縦に並ぶと傾きの推定が大きく崩れる。試験で 25 count/s が
// 一瞬 −1 count/s まで落ちた）。サーバーの planner は一定周期で seq を 1 つずつ進める（supervisor.rs）ので、直近 kHist 個の
// （seq, 受信時刻）に最小二乗直線「受信時刻 = a + P × seq」を当て、残差の最小値まで a を下げた（＝いちばん遅れの少なかった
// フレームに合わせた）直線をフレームの時刻とする。ぶれ・まとめ届き・欠落（seq が飛ぶ）が時刻に出ない。点が少ない・周期が
// おかしい（0.5〜200 ms の外）ときは受信時刻をそのまま使う。seq が戻った（epoch が変わった）・受信が 500 ms 以上空いたら取り直す。
//
// 再生（V1PlayoutBuffer）: ACTIVE で受けた目標（整数 count、ソフトリミットで clamp 済み）をリングに積み、playout 時刻 tp
//（= いま − 遅れ）での軌道の位置と速度を、tp を中心とする窓 [tp − 窓/2, tp + 窓/2] のフレームの最小二乗直線で推定する。
//  - 隣り合う整数の差ではなく窓全体の傾きなので、ゆっくり（1 フレームの進みが 0〜1 count）でも速度が 0 と数十 count/s を
//    行き来しない。直線の軌道では位置・速度とも厳密に一致する。角（止まる・反転）は窓の幅で丸まる（行き過ぎない向き）。
//  - 窓の中の目標がすべて同じなら傾きは厳密に 0.0、位置はその目標そのもの（Σ(y − ȳ) = 0 が厳密に成り立つ）。
//    V1Axis はこれを「目標が止まった」として参照を止める（保持・整定の判定は setpoint_vel_ == 0.0 を見る）。
//  - tp より新しいフレームがまだ届いていない（遅れ・途切れ）ときは窓が片側になり、直線を外挿する。最新フレームより
//    extrap 以上先なら starved（V1Axis は減速して止める）。
//  - まだ tp に届くフレームが無い（開始直後）なら has = false。
// 窓に点が 1 つしか無い・時刻が全部同じなら傾き 0 で、その平均の位置。窓の外でも tp を挟む前後の 1 点ずつは必ず含める
//（欠落で窓が空にならないように）。

#include <math.h>
#include <stddef.h>
#include <stdint.h>

class V1PlayoutClock {
 public:
  static constexpr size_t kHist = 32;     // 10 ms のフレームで 320 ms 分（40 ms の途切れが 3 回入っても下側の包絡が取れる）
  static constexpr size_t kMinPoints = 6;

  void reset() {
    head_ = 0;
    count_ = 0;
    valid_ = false;
  }
  bool valid() const { return valid_; }
  double period_us() const { return period_us_; }

  void push(uint32_t seq, uint64_t rx_us) {
    if (count_ > 0) {
      const Entry& last = at(count_ - 1);
      if (seq <= last.seq || seq - last.seq > 1000u || rx_us < last.rx || rx_us - last.rx > 500000ull) reset();
    }
    buf_[(head_ + count_) % kHist] = Entry{seq, rx_us};
    if (count_ < kHist) {
      ++count_;
    } else {
      head_ = (head_ + 1) % kHist;
    }
    refit();
  }

  // seq のフレームの時刻（付け直し）。使えなければ受信時刻 rx_us のまま
  uint64_t time_of(uint32_t seq, uint64_t rx_us) const {
    if (!valid_) return rx_us;
    const double t = base_us_ + period_us_ * (static_cast<double>(seq) - static_cast<double>(ref_seq_));
    return t <= 0.0 ? 0 : static_cast<uint64_t>(llround(t));
  }

 private:
  struct Entry {
    uint32_t seq;
    uint64_t rx;
  };
  const Entry& at(size_t i) const { return buf_[(head_ + i) % kHist]; }  // 0 = 最古

  void refit() {
    valid_ = false;
    if (count_ < kMinPoints) return;
    const Entry& nw = at(count_ - 1);
    // 最新を原点に: x = seq − seq_new（≤ 0）、y = rx − rx_new [us]
    double sx = 0.0, sy = 0.0;
    for (size_t i = 0; i < count_; ++i) {
      sx += -static_cast<double>(nw.seq - at(i).seq);
      sy += -static_cast<double>(nw.rx - at(i).rx);
    }
    const double n = static_cast<double>(count_);
    const double xm = sx / n, ym = sy / n;
    double sxx = 0.0, sxy = 0.0;
    for (size_t i = 0; i < count_; ++i) {
      const double x = -static_cast<double>(nw.seq - at(i).seq) - xm;
      const double y = -static_cast<double>(nw.rx - at(i).rx) - ym;
      sxx += x * x;
      sxy += x * y;
    }
    if (sxx <= 0.0) return;
    const double p = sxy / sxx;
    if (!(p >= 500.0 && p <= 200000.0)) return;  // 0.5〜200 ms の外は信用しない
    const double a = ym - p * xm;  // x = 0（最新の seq）での直線の値
    double rmin = 0.0;
    for (size_t i = 0; i < count_; ++i) {
      const double x = -static_cast<double>(nw.seq - at(i).seq);
      const double y = -static_cast<double>(nw.rx - at(i).rx);
      const double r = y - (a + p * x);
      if (i == 0 || r < rmin) rmin = r;
    }
    period_us_ = p;
    ref_seq_ = nw.seq;
    base_us_ = static_cast<double>(nw.rx) + a + rmin;  // 下側の包絡（いちばん遅れの少ないフレーム）に合わせる
    valid_ = true;
  }

  Entry buf_[kHist]{};
  size_t head_ = 0;
  size_t count_ = 0;
  bool valid_ = false;
  double period_us_ = 0.0;
  double base_us_ = 0.0;
  uint32_t ref_seq_ = 0;
};

struct V1PlayoutFit {
  bool has = false;      // tp 以前のフレームがある
  bool starved = false;  // tp が最新フレームより extrap を越えて先（新しい目標が途切れている）
  double pos = 0.0;      // 推定位置 [count]
  double vel = 0.0;      // 推定速度 [count/s]
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

  // seq・受信時刻 rx_us の目標 pos を積む（受信順）。満杯なら最古を捨てる。
  void push(uint32_t seq, uint64_t rx_us, double pos) {
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
    // フレームの時刻（付け直し。受信順に単調にする）
    uint64_t t[kCap];
    for (size_t i = 0; i < count_; ++i) {
      t[i] = clk.time_of(at(i).seq, at(i).rx);
      if (i > 0 && t[i] < t[i - 1]) t[i] = t[i - 1];
    }
    if (t[0] > tp) return f;
    f.has = true;
    if (t[count_ - 1] < tp && tp - t[count_ - 1] > extrap_us) f.starved = true;
    // 窓: tp を中心に ±窓/2。tp より後のフレームがまだ届いていなければ（遅れ・途切れ）その分だけ過去へ広げ、幅は常に窓以上
    //（片側の短い窓だと、ゆっくりの軌道で同じ整数の目標しか入らず傾きがちょうど 0 になり「止まった」と誤認する）。
    // 重みは窓の中心で 1・両端で 0 の三角（点が窓に出入りしても推定が段にならない）。窓の外でも tp を挟む前後の 1 点ずつは
    // 小さな重みで必ず入れる（欠落で窓が空にならないように）。
    size_t before = 0;  // tp 以前で最新
    for (size_t i = 0; i < count_; ++i) {
      if (t[i] <= tp) before = i;
    }
    const size_t after = before + 1;  // tp より後で最古（count_ なら無し）
    const double half = static_cast<double>(window_us) / 2e6;  // [s]
    double w_hi = half;
    const double r_new = rel_s(t[count_ - 1], tp);
    if (r_new < w_hi) w_hi = r_new > 0.0 ? r_new : 0.0;
    const double w_lo = w_hi - 2.0 * half;
    const double c = (w_lo + w_hi) / 2.0;
    const double hw = half + 1e-4;  // 端の点も重み ≈ 0（0 で割らない）
    double sw = 0.0, st = 0.0, sy = 0.0;
    double wt[kCap];
    size_t n = 0;
    for (size_t i = 0; i < count_; ++i) {
      const double r = rel_s(t[i], tp);
      double w = 0.0;
      if (r >= w_lo && r <= w_hi) w = 1.0 - fabs(r - c) / hw;
      if ((i == before || i == after) && w < 0.05) w = 0.05;
      wt[i] = w;
      if (w <= 0.0) continue;
      sw += w;
      st += w * r;
      sy += w * at(i).pos;
      ++n;
    }
    const double tm = st / sw;
    const double ym = sy / sw;
    double stt = 0.0, sty = 0.0;
    for (size_t i = 0; i < count_; ++i) {
      if (wt[i] <= 0.0) continue;
      const double dt = rel_s(t[i], tp) - tm;
      stt += wt[i] * dt * dt;
      sty += wt[i] * dt * (at(i).pos - ym);
    }
    // 窓の中の目標がすべて同じなら傾きはちょうど 0、位置はその目標そのもの（重み付きの平均の丸めを持ち込まない）
    bool flat = true;
    double y0 = 0.0;
    bool first = true;
    for (size_t i = 0; i < count_; ++i) {
      if (wt[i] <= 0.0) continue;
      if (first) {
        y0 = at(i).pos;
        first = false;
      } else if (at(i).pos != y0) {
        flat = false;
      }
    }
    if (flat) {
      f.vel = 0.0;
      f.pos = y0;
    } else {
      const double b = stt > 1e-12 ? sty / stt : 0.0;
      f.vel = b;
      f.pos = ym - b * tm;  // tp（相対 0）での直線の値
    }
    f.n = static_cast<uint8_t>(n);
    return f;
  }

 private:
  struct Sample {
    uint32_t seq;
    uint64_t rx;
    double pos;
  };
  const Sample& at(size_t i) const { return buf_[(head_ + i) % kCap]; }  // 0 = 最古
  static double rel_s(uint64_t t, uint64_t tp) {
    return t >= tp ? static_cast<double>(t - tp) / 1e6 : -static_cast<double>(tp - t) / 1e6;
  }

  Sample buf_[kCap]{};
  size_t head_ = 0;
  size_t count_ = 0;
};
