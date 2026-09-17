#!/bin/bash
# golden 基线捕捉：16 组合全量实跑（W2 后：腰动必须 --arm both，单臂+腰动被拒），
# 产出 tests/golden/{*.npz, entries/*.json, manifest.json}
source /opt/ros/jazzy/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate dex
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FAIL=0
run() {
  desc="$*"
  out=$(timeout 500 python -u tests/golden_capture.py "$@" 2>&1 | grep -av "leaked\|^ - \|nanobind\|root link\|kdl_parser\|^ *at line")
  res=$(echo "$out" | grep "^GOLDEN" | tail -1)
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

echo "== 腰部参与（必须 --arm both：腰是共享关节，单臂+腰动已被 configure 拒绝）=="
run both full free
run both fixed free
run both full all
run both fixed all

echo "== B2 多指组合 =="
run right full fixed index,thumb
run right fixed fixed index,thumb
run both fixed fixed index,thumb

python - <<'EOF'
import json, glob, os
d = "tests/golden"
entries = {}
for p in sorted(glob.glob(os.path.join(d, "entries", "*.json"))):
    e = json.load(open(p))
    entries[e["label"]] = e
manifest = {
    "date": "2026-09-17",
    "thresholds": entries and next(iter(entries.values()))["thresholds"],
    "acm_hash_by_combo": {k: v["acm_hash"] for k, v in entries.items()},
    "versions": {k: {"nanobind": v["nanobind"], "numpy": v["numpy"]}
                 for k, v in entries.items()},
    "entries": entries,
}
json.dump(manifest, open(os.path.join(d, "manifest.json"), "w"),
          ensure_ascii=False, indent=1)
print(f"manifest.json: {len(entries)} combos, "
      f"ALL PASS = {sum(1 for v in entries.values() if v['result'] == 'ALL PASS')}")
EOF

if [ $FAIL -eq 0 ]; then echo "==== golden 捕捉: ALL PASS ===="; else echo "==== golden 捕捉: 有失败 ===="; fi
