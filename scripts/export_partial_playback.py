"""Export and freshly replay an exact retained forward prefix for the viewer."""

import argparse
from pathlib import Path

from pen_art.planning.cell_replay import replay_cell
from pen_art.planning.partial_export import export_forward_prefix

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source", type=Path, required=True)
parser.add_argument("--destination", type=Path, required=True)
parser.add_argument("--maximum-seconds", type=float, default=300)
args = parser.parse_args()
plan = export_forward_prefix(args.source, args.destination)
print(f"Exported {plan['completed_strokes']}/{plan['requested_strokes']} sections", flush=True)
result = replay_cell(
    args.destination,
    args.destination / "profile.yaml",
    args.maximum_seconds,
    "offline_dual_arm_plan",
)
print(result["status"], result.get("completed_strokes"), flush=True)
