#!/usr/bin/env bash
# Build docs/report.pdf from docs/report.md (pandoc + headless Chromium). Needs: pandoc, chromium/chrome.
# Usage: bash tools/build_report_pdf.sh   (override the browser with CHROME=/path/to/chrome)
set -euo pipefail
cd "$(dirname "$0")/../docs"
CHROME="${CHROME:-$(command -v chromium chromium-browser google-chrome 2>/dev/null | head -1 || true)}"
[ -n "$CHROME" ] || CHROME="$(ls /opt/pw-browsers/chromium-*/chrome-linux/chrome 2>/dev/null | head -1 || true)"
[ -x "$CHROME" ] || { echo "no Chromium/Chrome found (set CHROME=...)"; exit 1; }
pandoc report.md -f gfm -t html5 -s --metadata title="Autonomous Navigation of TurtleBot3 Burger - IE4060 Assignment 02" \
  --css report.css --embed-resources --resource-path=. -o report.html
# pandoc's own title block would duplicate the H1 in the markdown
sed -i '/<header id="title-block-header">/,/<\/header>/d' report.html
"$CHROME" --headless --no-sandbox --disable-gpu --no-pdf-header-footer --print-to-pdf=report.pdf "file://$PWD/report.html" 2>/dev/null
rm -f report.html
command -v pdfinfo >/dev/null && pdfinfo report.pdf | grep -E "Pages|Page size"
echo "wrote docs/report.pdf"
