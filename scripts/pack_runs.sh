#!/bin/bash
# Pack the runs a new machine needs into one archive (runs/ is not in git):
#   scripts/pack_runs.sh [OUTPUT]          default ../3d_art_runs.tar.zst
# Unpack in the new clone's root with:
#   tar --zstd -xf 3d_art_runs.tar.zst
# Contents: the cell profiles, the frame and detail sources the current
# results were made from, the verified frame and detail runs (everything the
# viewer lists), their detail reports, runs/viewer_runs.json, and the motion
# library that frame strokes are adapted from (without its attempts/ folder, a
# record of past misses that planning does not read). Intermediate planning
# rounds and old experiments are left out.
set -euo pipefail
cd "$(dirname "$0")/.."
output=${1:-../3d_art_runs.tar.zst}

items=(
  runs/viewer_runs.json
  runs/motion_library_phase1_upright27_v1
  runs/geomfirst_profiles
  # Frame sources (frame_source.py) and the old Palace frame.
  runs/geomfirst_libertystatue_c1_source
  runs/geomfirst_ghost_b10_source
  runs/geomfirst_pumpkin_b10_source
  runs/geomfirst_palace2_source
  runs/geomfirst_palace_b10_source
  # Detail sources (detail_tours.py) behind the final detail runs.
  runs/geomfirst_liberty_detail_source
  runs/geomfirst_ghost_detail4_source
  runs/geomfirst_pumpkin_detail4_source
  runs/geomfirst_palace_detail_source
  # Detail reports and the final round's source.
  runs/geomfirst_gf94_liberty_detail_detail.json
  runs/geomfirst_gf98_ghost_detail_detail.json
  runs/geomfirst_gf98_pumpkin_detail_detail.json
  runs/geomfirst_gf100_palace_detail_detail.json
  runs/geomfirst_gf94_liberty_detail_r14_source
  runs/geomfirst_gf98_ghost_detail_r14_source
  runs/geomfirst_gf98_pumpkin_detail_r24_source
  runs/geomfirst_gf100_palace_detail_r24_source
)
# Every run the viewer lists (verified frames, detail runs, Liberty studies).
while read -r run; do items+=("runs/$run"); done < <(
  .venv/bin/python -c "import json; print('\n'.join(json.load(open('runs/viewer_runs.json'))))"
)
missing=0
for item in "${items[@]}"; do
  [ -e "$item" ] || { echo "missing: $item" >&2; missing=1; }
done
[ "$missing" = 0 ] || exit 1
du -shc --exclude=attempts "${items[@]}" | tail -1
tar --zstd --exclude=runs/motion_library_phase1_upright27_v1/attempts -cf "$output" "${items[@]}"
ls -la "$output"
