#!/bin/bash
# 全量回归：9 个无腰组合 + 3 个腰部参与组合 + 3 个多指组合
# 腰动必须 --arm both（腰是共享关节，单臂+腰动已被 configure 拒绝）；both fixed all 待协商
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

echo "== 无腰 9 组合 =="
run right full
run right fixed
run right tcp
run left full
run left fixed
run left tcp
run both full
run both fixed
run both tcp

echo "== 腰部参与（必须 --arm both：腰是共享关节）=="
run both full free
run both fixed free
run both full all
# both fixed all：右臂解出的腰把 roll 顶到 ±25° 限位、左臂在该腰下不可达
# （W2 实测点 49/72）→ 已由腰协商修复（修 23 帧），见 docs/实施计划 W2。
run both fixed all

echo "== B2 多指组合 =="
run right full fixed index,thumb
run right fixed fixed index,thumb
run both fixed fixed index,thumb

echo
if [ $FAIL -eq 0 ]; then echo "==== 总体: ALL PASS ===="; else echo "==== 总体: 有失败 ===="; fi
