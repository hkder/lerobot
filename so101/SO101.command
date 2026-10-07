#!/usr/bin/env bash
# macOS: double-click in Finder to open the SO-101 operator console.
cd "$(dirname "$0")/.." || exit 1
[ -d .venv ] || uv sync --locked --python 3.12 --extra dataset --extra feetech --extra viz --extra async --extra hardware
# PySide6 goes into the project venv once (uv sync removes it, so check every time).
.venv/bin/python -c "import PySide6" 2>/dev/null || uv pip install --python .venv PySide6
exec .venv/bin/python so101/app/main.py
