#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT"

./build/refuel_mode0_tests
./build/refuel_mode1_tests
./build/refuel_stage_tests
./build/refuel_demo demo/input/0210 build/test_outputs/0210
./build/refuel_demo demo/input/0306_many_to_many build/test_outputs/0306_many_to_many
