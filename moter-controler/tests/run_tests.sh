#!/bin/sh
set -eu
cd "$(dirname "$0")"
# Issue #25 r6: 位置ループの既定（config.h V1_FF_DIRECTIONAL=1）を 0 にして組む。従来の経路（床 E・補助 T・不感帯 10/20）のテストは
# 既定の V1Tuning のまま、r6 の経路は V1Tuning(true) で明示的に検証する。ファームの既定は pio のビルドが見る。
# Issue #35 r9: 減衰項（config.h V1_VEL_DAMPING=1）も 0 にして組み、r6〜r8 のテストは r8 と同じ出力で検証する。r9 は
# vel_kd_permille_per_count_s を明示的に入れて検証する（test_r9_*）。
c++ -std=c++17 -Wall -Wextra -DV1_FF_DIRECTIONAL=0 -DV1_VEL_DAMPING=0 -I../include test_host.cpp -o /tmp/moter_controler_host
/tmp/moter_controler_host
# 焼く既定（V1_VEL_DAMPING=1）でも同じテストが通ること（減衰を前提にしない試験は Kd を明示的に 0 にしている）
c++ -std=c++17 -Wall -Wextra -DV1_FF_DIRECTIONAL=0 -I../include test_host.cpp -o /tmp/moter_controler_host_damping
/tmp/moter_controler_host_damping
