// moter-controler のホストテスト。GPIO・時計・NVS に依存しない純粋ロジックだけを対象にする。
#include <cstdio>
#include <string>

#include "motor_profile.h"

namespace {

int g_pass = 0;
int g_fail = 0;

#define CHECK(cond)                                                      \
  do {                                                                   \
    if (cond) {                                                          \
      ++g_pass;                                                          \
    } else {                                                             \
      ++g_fail;                                                          \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
    }                                                                    \
  } while (0)

void test_motor_profile() {
  CHECK(std::string(p_end_label(1)) == "下限");
  CHECK(std::string(p_end_label(2)) == "下限");
  CHECK(std::string(p_end_label(3)) == "右端");
  CHECK(std::string(n_end_label(2)) == "上限");
  CHECK(std::string(n_end_label(3)) == "左端");
  CHECK(default_neutral(5556) == 2778);
  CHECK(profile_motor_ok(0) == false);
  CHECK(profile_motor_ok(1) == true);
  CHECK(profile_motor_ok(3) == true);
  CHECK(profile_motor_ok(4) == false);
  MotorProfile p;
  CHECK(p.sign == 1);
  CHECK(!p.calibrated);
  CHECK(p.stroke == 0);
  CHECK(p.motor == 1);
}

}  // namespace

int main() {
  test_motor_profile();
  std::printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
