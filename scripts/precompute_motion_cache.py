"""Generate whole zigzag seeds checked against their growing deposited prefix."""

import argparse
import json
from pathlib import Path

from pen_art.planning.motion_cache_precompute import precompute, promote_catalogue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-build", type=Path, action="append", default=[])
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--catalogue", type=Path, help="Activate an already generated catalogue")
    parser.add_argument("--publish-library", type=Path,
                        help="Idempotently activate independently admitted seeds in this library")
    parser.add_argument("--profile", type=Path,
                        default=Path("research/planning_cell/cell_turntable.yaml"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--maximum-seconds", type=float, default=600)
    parser.add_argument("--seconds-per-request", type=float, default=90)
    parser.add_argument("--max-families", type=int,
                        help="Explicitly limit source geometries; otherwise include every zigzag")
    parser.add_argument("--variants-per-family", type=int, default=2)
    parser.add_argument("--indices", type=int, nargs="+", default=[0, 1, 2, 3])
    parser.add_argument("--gpu", action="store_true")
    parser.add_argument("--self-static-only", action="store_true",
                        help="Legacy limited seeds; omit own growing-prefix admission")
    args = parser.parse_args()
    if args.catalogue is not None:
        if args.publish_library is None:
            parser.error("--catalogue requires --publish-library")
        print(json.dumps(promote_catalogue(args.catalogue, args.publish_library, args.profile)))
        return
    if not args.source_build and args.manifest is None:
        parser.error("Supply --source-build or --manifest")
    if args.output is None:
        parser.error("Generation requires --output")
    if (args.maximum_seconds <= 0 or args.seconds_per_request <= 0
            or (args.max_families is not None and args.max_families < 1)
            or args.variants_per_family < 1 or any(i not in range(4) for i in args.indices)):
        parser.error("Budgets/counts must be positive and indices must be 0, 1, 2 or 3")
    report = precompute(
        args.source_build, args.profile, args.output, maximum_seconds=args.maximum_seconds,
        seconds_per_request=args.seconds_per_request, maximum_families=args.max_families,
        variants_per_family=args.variants_per_family, indices=args.indices, gpu=args.gpu,
        manifest=args.manifest, growing_prefix=not args.self_static_only,
    )
    if args.publish_library is not None:
        promote_catalogue(args.output, args.publish_library, args.profile)
    print(json.dumps({"status": report["status"], "records": report["records"],
                      "seconds": report["seconds"]}))


if __name__ == "__main__":
    main()
