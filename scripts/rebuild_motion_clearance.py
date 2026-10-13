"""Populate and exercise current-mount robot/tool self/static clearance bounds."""

import argparse
import json
from pathlib import Path

from pen_art.planning.motion_cache_precompute import rebuild_clearance_atlas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-seconds", type=float, default=60)
    parser.add_argument("--maximum-samples", type=int, default=512)
    parser.add_argument("--reference-library", type=Path)
    parser.add_argument("--filter-profile-context", action="store_true",
                        help="For a shared library, use only matching profile/process records; "
                             "record skipped IDs without importing their clearance bounds")
    args = parser.parse_args()
    if args.maximum_seconds <= 0 or args.maximum_samples < 1:
        parser.error("Time and sample budgets must be positive")
    result = rebuild_clearance_atlas(
        args.library, args.profile, args.output, maximum_seconds=args.maximum_seconds,
        maximum_samples=args.maximum_samples, reference_library=args.reference_library,
        filter_profile_context=args.filter_profile_context,
    )
    print(json.dumps({"status": result["status"], "seconds": result["seconds"],
                      "configuration_samples": result["configuration_samples"]}))


if __name__ == "__main__":
    main()
