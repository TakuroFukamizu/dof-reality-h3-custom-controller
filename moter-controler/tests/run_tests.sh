#!/bin/sh
set -eu
cd "$(dirname "$0")"
# Issue #25 r6: 位置ループの既定（config.h V1_FF_DIRECTIONAL=1）を 0 にして組む。従来の経路（床 E・補助 T・不感帯 10/20）のテストは
# 既定の V1Tuning のまま、r6 の経路は V1Tuning(true) で明示的に検証する。ファームの既定は pio のビルドが見る。
c++ -std=c++17 -Wall -Wextra -DV1_FF_DIRECTIONAL=0 -I../include test_host.cpp -o /tmp/moter_controler_host
/tmp/moter_controler_host
