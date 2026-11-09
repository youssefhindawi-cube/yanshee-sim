#!/usr/bin/env bash
# Build robot-slideshow.mp4 from a folder of images (run on the laptop; needs ffmpeg).
# Images are ordered by any leading number in the filename (e.g. -30,-15,0,15,30).
# Usage: build/build-slideshow.sh <images-folder> [seconds-per-slide] [output.mp4]
set -euo pipefail
DIR=${1:?usage: build-slideshow.sh <images-folder> [secs] [out.mp4]}
SECS=${2:-5}
OUT=${3:-$HOME/robot-slideshow.mp4}
HERE=$(cd "$(dirname "$0")/.." && pwd)
python3 "$HERE/laptop/tv-slides" "$DIR" --secs "$SECS" --build-only --out "$OUT"
echo "built: $OUT"
