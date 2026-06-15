#!/usr/bin/env bash
set -euo pipefail

root=/workspace
build_dir=/tmp/virtdrone-build
report_dir="$root/reports"
scenario_dir="$report_dir/scenarios"
chart_dir="$root/docs/tutorials/charts"

mkdir -p "$report_dir" "$scenario_dir"

cmake -S "$root" -B "$build_dir" -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build "$build_dir" -j

ctest_out="$report_dir/ctest-output.txt"
ctest --test-dir "$build_dir" --output-on-failure 2>&1 | tee "$ctest_out"

summary_html="$report_dir/refactor-validation-summary.html"
if command -v python3 >/dev/null 2>&1; then
python3 - <<'PY'
from pathlib import Path
from html import escape
from datetime import datetime

root = Path('/workspace')
report_dir = root / 'reports'
chart_dir = root / 'docs/tutorials/charts'
ctest_path = report_dir / 'ctest-output.txt'
ctest_text = ctest_path.read_text(errors='replace') if ctest_path.exists() else ''

charts = []
if chart_dir.exists():
    for path in sorted(chart_dir.iterdir()):
        if path.suffix.lower() not in {'.png', '.jpg', '.jpeg', '.webp'}:
            continue
        rel = path.relative_to(root).as_posix()
        charts.append(
            f'<figure style="margin:16px 0;"><figcaption><strong>{escape(path.name)}</strong></figcaption><img src="../{escape(rel)}" alt="{escape(path.name)}" style="max-width:100%;height:auto;border:1px solid #ccc;" /></figure>'
        )

chart_html = ''.join(charts) if charts else '<p>No chart images found in docs/tutorials/charts.</p>'
html = f"""<!doctype html>
<html>
<head>
  <meta charset='utf-8' />
  <title>virtDrone Refactor Validation Summary</title>
  <style>
    body {{ font-family: Segoe UI, Arial, sans-serif; margin: 24px; line-height: 1.4; }}
    h1, h2 {{ margin: 0 0 12px; }}
    section {{ margin: 24px 0; }}
    pre {{ background: #111; color: #eee; padding: 12px; overflow: auto; max-height: 420px; }}
  </style>
</head>
<body>
  <h1>virtDrone Refactor Validation Summary</h1>
  <p>Generated: {escape(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))}</p>
  <section>
    <h2>CTest Output</h2>
    <pre>{escape(ctest_text)}</pre>
  </section>
  <section>
    <h2>Charts</h2>
    {chart_html}
  </section>
</body>
</html>
"""
(report_dir / 'refactor-validation-summary.html').write_text(html, encoding='utf-8')
print(f'Summary written to: {report_dir / "refactor-validation-summary.html"}')
PY
else
  echo "python3 not found in container, skipping HTML summary generation"
fi
