"""Offline dataset and model commands: python -m pen_art.learning --help."""

import argparse
import importlib
import json
from pathlib import Path

from .data import curate


def callback(value):
    if value is None:
        return None
    module, name = value.split(":", 1)
    return getattr(importlib.import_module(module), name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    data = sub.add_parser("curate")
    data.add_argument(
        "episodes",
        nargs="+",
        help="JSONL episodes, checksummed attempt JSON, or motion-library/attempt directories",
    )
    data.add_argument("--output", required=True)
    data.add_argument("--profile", required=True)
    data.add_argument("--knots", type=int, default=32)
    data.add_argument("--probes", type=int, default=32)
    data.add_argument("--seed", type=int, default=17)
    data.add_argument("--fit-validator", help="module:function(episode,q,source) -> bool")
    training = sub.add_parser("train")
    training.add_argument("dataset")
    training.add_argument("--output", required=True)
    training.add_argument("--profile", required=True)
    training.add_argument("--steps", type=int, default=2000)
    training.add_argument("--batch-size", type=int, default=32)
    training.add_argument("--hidden", type=int, default=256)
    training.add_argument("--seed", type=int, default=17)
    training.add_argument("--device", default="cpu")
    evaluation = sub.add_parser("evaluate")
    evaluation.add_argument("dataset")
    evaluation.add_argument("checkpoint")
    evaluation.add_argument("--output", required=True)
    evaluation.add_argument("--profile", required=True)
    evaluation.add_argument("--validator", help="module:function(episode,q,source) -> status")
    evaluation.add_argument("--split", choices=("validation", "test"), default="test")
    evaluation.add_argument("--count", type=int, default=4)
    evaluation.add_argument("--device", default="cpu")
    inspect = sub.add_parser("inspect")
    inspect.add_argument("dataset")
    args = parser.parse_args()
    if args.command == "inspect":
        print((Path(args.dataset) / "manifest.json").read_text())
        return
    from .oracle import CellSignatureOracle

    oracle = CellSignatureOracle(args.profile)
    if args.command == "curate":
        report = curate(
            args.episodes,
            args.output,
            knots=args.knots,
            probes=args.probes,
            seed=args.seed,
            signature_fn=oracle,
            fit_validator=callback(args.fit_validator),
        )
    elif args.command == "train":
        from .diffusion import train
        from .geometry import metadata_from_arm

        manifest = json.loads((Path(args.dataset) / "manifest.json").read_text())
        if manifest["context_hash"] != oracle.context_hash:
            raise ValueError("Dataset model/static context differs from the loaded profile")
        report = train(
            args.dataset,
            args.output,
            steps=args.steps,
            batch_size=args.batch_size,
            hidden=args.hidden,
            seed=args.seed,
            device=args.device,
            kinematics=metadata_from_arm(oracle.arm),
        )
    else:
        from .diffusion import MotionDiffusionProvider
        from .evaluation import evaluate

        manifest = json.loads((Path(args.dataset) / "manifest.json").read_text())
        provider = MotionDiffusionProvider(
            args.checkpoint, oracle.context_hash, signature_fn=oracle, device=args.device
        )
        report = evaluate(
            provider,
            args.dataset,
            split=args.split,
            count=args.count,
            validator=callback(args.validator),
        )
        Path(args.output).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
