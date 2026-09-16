# §8.4 CI 第 1 项（N9 红线）：未装 tesseract 的裸 venv 子进程 import 必须成功。
# eager import 检查：contracts 包 import 零第三方依赖（numpy 延迟到调用期）；
# 顶层包 __init__ 为空。tesseract 只允许出现在 core/tess.py（第 2 项，另行 grep）。
set -euo pipefail
cd "$(dirname "$0")/.."

VENV=$(mktemp -d)
python -m venv "$VENV" --without-pip 2>/dev/null || python -m venv "$VENV"
"$VENV/bin/python" -c "import tienkung_planning.contracts; print('contracts OK')"
"$VENV/bin/python" -c "import tienkung_planning; print('top-level OK')"
rm -rf "$VENV"
echo "BARE VENV IMPORT: PASS"
