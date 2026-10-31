#!/bin/sh
set -eu
cd "$(dirname "$0")"
c++ -std=c++17 -Wall -Wextra -I../include test_host.cpp -o /tmp/moter_controler_host
/tmp/moter_controler_host
