#!/bin/bash
# Plan and verify a frame (geometry-first, 24 workers). Run from the repo root:
#   TAG=gfNN scripts/plan_frame.sh NAME SOURCE PROFILE
# Writes runs/geomfirst_${TAG}_NAME (the run), _benchmark.json/.out, and appends
# one line to runs/geomfirst_${TAG}_progress.txt ending in FULL_BUILD_VERIFIED
# when the plan passed.
set -u
cd "$(dirname "$0")/.."
: "${TAG:?set TAG, e.g. TAG=gf102}"
.venv/bin/python scripts/benchmark_parallel_build.py --source-run "$2" --profile "$3" \
  --output "runs/geomfirst_${TAG}_$1" --workers "${WORKERS:-24}" --maximum-seconds 7200 \
  --verification-option=--accept-geometry-proofs > "runs/geomfirst_${TAG}_$1_benchmark.out" 2>&1
echo "$1 exit $? $(date +%T) $(tail -1 "runs/geomfirst_${TAG}_$1_benchmark.out" | cut -c1-80)" \
  >> "runs/geomfirst_${TAG}_progress.txt"
