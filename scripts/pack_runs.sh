#!/bin/bash
# Pack the runs a new machine needs (runs/ is not in git):
#   scripts/pack_runs.sh [OUTPUT_DIR]      default ..
# Writes two archives:
#   pen_art_final_runs.tar.zst      the final runs and a runs/viewer_runs.json
#                                   that lists them
#   pen_art_motion_library.tar.zst  the motion library frame strokes are
#                                   adapted from (without its attempts/ folder,
#                                   a record of past misses that planning does
#                                   not read) and the cell profiles
# Unpack both in a clone's root with tar --zstd -xf.
set -euo pipefail
cd "$(dirname "$0")/.."
output=$(realpath "${1:-..}")

final=(
  geomfirst_gf94_liberty_detail_r14_w24
  geomfirst_gf98_ghost_detail_r14_w24
  geomfirst_gf98_pumpkin_detail_r24_w24
  geomfirst_gf100_palace_detail_r24_w24
  geomfirst_gf101_palace2_w24
)
for run in "${final[@]}" motion_library_phase1_upright27_v1 geomfirst_profiles; do
  [ -e "runs/$run" ] || { echo "missing: runs/$run" >&2; exit 1; }
done

stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
mkdir "$stage/runs"
printf '%s\n' "${final[@]}" | python3 -c \
  "import json, sys; json.dump(sys.stdin.read().split(), sys.stdout, indent=1)" \
  > "$stage/runs/viewer_runs.json"
for run in "${final[@]}"; do ln -s "$PWD/runs/$run" "$stage/runs/$run"; done
tar --zstd -h -C "$stage" -cf "$output/pen_art_final_runs.tar.zst" runs
tar --zstd --exclude=runs/motion_library_phase1_upright27_v1/attempts \
  -cf "$output/pen_art_motion_library.tar.zst" \
  runs/motion_library_phase1_upright27_v1 runs/geomfirst_profiles
ls -la "$output"/pen_art_*.tar.zst
