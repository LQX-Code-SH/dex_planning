#!/bin/bash
# golden 基线捕捉：26 组合全量实跑（W1 后：原 free 改名 all + 新增 yaw-only free），
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
run right full all
run right fixed all
run left full all
run left fixed all
run both full all
run both fixed all

echo "== W1 yaw-only（--waist free = 只放 yaw）=="
run right full free
run right fixed free
run left full free
run left fixed free
run both full free
run both fixed free

echo "== B2 多指组合 =="
run right full fixed index,thumb
run right fixed fixed index,thumb
run right full all index,thumb
run right full free index,thumb
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
