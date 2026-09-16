#!/bin/bash
# 全量回归：15 个既有组合（单指）+ 4 个多指组合
source /opt/ros/jazzy/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate dex
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FAIL=0
run() {
  desc="$*"
  out=$(timeout 500 python -u "$(dirname "${BASH_SOURCE[0]}")/regression.py" "$@" 2>&1 | grep -av "leaked\|^ - \|nanobind\|root link\|kdl_parser\|^ *at line")
  res=$(echo "$out" | grep "^RESULT" | tail -1)
  echo "$res"
  if ! echo "$res" | grep -q "ALL PASS"; then
    FAIL=1
    echo "---- 失败详情 ($desc) ----"
    echo "$out" | grep -a "FAIL\|Error\|assert" | head -10
  fi
}

echo "== 既有 15 组合（单指）=="
run right full
run right fixed
run right tcp
run left full
run left fixed
run left tcp
run both full
run both fixed
run both tcp
run right full free
run right fixed free
run left full free
run left fixed free
run both full free
run both fixed free

echo "== B2 多指组合 =="
run right full fixed index,thumb
run right fixed fixed index,thumb
run right full free index,thumb
run both fixed fixed index,thumb

echo
if [ $FAIL -eq 0 ]; then echo "==== 总体: ALL PASS ===="; else echo "==== 总体: 有失败 ===="; fi
