#!/bin/sh
# Offline storyboard run. Records nothing by itself.
# Run it in a visible terminal while you screen-record. See demo-script.md.
# Hermes chat + the Robot tab is a separate recording; this script does not start Hermes.
set -eu
cd "$(dirname "$0")/.."
OUT="${1:-/tmp/hermes-ros-demo}"
exec python3 demo/mock_demo.py --present --out "$OUT"
