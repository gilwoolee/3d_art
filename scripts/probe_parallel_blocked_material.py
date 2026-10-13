"""Show that a predicted, never-validated predecessor deposit blocks a later action.

Reads a finished parallel run. One late action is rechecked twice by the real
planning job: against the selected predecessor deposits, then against a chain
in which one predecessor's journal has an extra capsule placed on the late
action's own path. That altered predecessor deposit is never collision-checked
itself, which is the situation a parallel worker is in. Nothing is published.
"""

import argparse
import pickle
from pathlib import Path
from time import perf_counter

import numpy as np

from pen_art.io import read_json, write_json
from pen_art.planning import parallel_build
from pen_art.planning.section_build import SectionBuild


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--action-index", type=int, required=True)
    parser.add_argument("--predecessor-index", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--action-budget", type=float, default=240.0)
    parser.add_argument(
        "--cold-proposal",
        action="store_true",
        help="Also run the cache-miss fallback proposal for the probed action",
    )
    args = parser.parse_args()
    summary = read_json(args.run / "parallel_build.json")
    actions = summary["actions"]
    late, earlier = args.action_index, args.predecessor_index
    if not 0 <= earlier < late < len(actions):
        parser.error("The predecessor must come before the probed action")
    build = SectionBuild(args.run, args.profile, 3600)
    context = parallel_build.prepare_context(build, args.action_budget)
    root = context["root"]
    chain = [[row["deposit_key"], row["table_index"]] for row in _selected(actions[:late])]
    spec = {
        "kind": "check",
        "index": late,
        "candidate": actions[late]["candidate_key"],
        "deposit_key": actions[late]["deposit_key"],
        "chain": chain,
        "key": "probe-clear",
    }
    started = perf_counter()
    clear = parallel_build.run_job(spec)
    clear_seconds = perf_counter() - started
    # Put one short strand at the middle of the late action's own deposit.
    own = parallel_build._deposits(root, actions[late]['deposit_key'])
    rows = [row for operation in own if operation[0] == "add" for row in operation[1]]
    target = rows[len(rows) // 2]
    obstruction = {
        "start": np.asarray(target["start"]).tolist(),
        "end": (np.asarray(target["start"]) + [0.0, 0.0, 0.004]).tolist(),
        "radius_m": 0.0005,
        "name": "probe/injected-predecessor-obstruction",
    }
    journal = parallel_build._deposits(root, actions[earlier]['deposit_key'])
    altered = [*journal, ("add", [obstruction])]
    key = parallel_build.deposit_key(altered)
    (root / "deposits" / f"{key}.pkl").write_bytes(pickle.dumps(altered))
    blocked_chain = [list(row) for row in chain]
    blocked_chain[earlier][0] = key
    started = perf_counter()
    blocked = parallel_build.run_job({**spec, "chain": blocked_chain, "key": "probe-blocked"})
    blocked_seconds = perf_counter() - started
    cold = None
    if args.cold_proposal:
        # An actual cache-miss proposal for the base action; the later blocked
        # probe remains separate so a difficult cold zigzag cannot mask it.
        started = perf_counter()
        cold = parallel_build.run_job(
            {
                "kind": "propose",
                "index": 0,
                "mode": "cold",
                "ordinal": 0,
                "chain": [],
                "key": "probe-cold",
            }
        )
        cold_check = None
        if cold["status"] == "OK":
            cold_check = parallel_build.run_job({
                "kind": "check", "index": 0, "candidate": "probe-cold",
                "deposit_key": cold["deposit_key"], "chain": [], "key": "probe-cold-check",
            })
        full_check = None
        if cold_check is not None and cold_check["status"] == "OK":
            # Independently recheck the selected cold local clock with the
            # complete robot. This probe certifies no connecting/home motion.
            build.planner.verification_raw = True
            build.planner.optimize_contact_geometry = True
            build.planner.path_certificates.enabled = False
            for identity, arm in build.planner.arms.items():
                arm.collision = getattr(arm.collision, "model", arm.collision)
                tip = build.planner.tip_models[identity]
                build.planner.tip_models[identity] = getattr(tip, "model", tip)
            full_check = parallel_build.run_job({
                "kind": "check", "index": 0, "candidate": "probe-cold",
                "deposit_key": cold["deposit_key"], "chain": [],
                "key": "probe-cold-full-robot-check",
            })
        cold = {
            "status": cold["status"],
            "full_robot_check_status": None if full_check is None else full_check["status"],
            "full_robot_failure": None if full_check is None else full_check.get("failure"),
            "progressive_check_status": None if cold_check is None else cold_check["status"],
            "progressive_check_failure": None if cold_check is None else cold_check.get("failure"),
            "action_index": 0,
            "seconds": perf_counter() - started,
            "variant": cold.get("variant"),
            "capsules": cold.get("capsules"),
            "failure": cold.get("failure"),
        }
    for name in ("probe-clear", "probe-blocked", "probe-cold", "probe-cold-check",
                 "probe-cold-full-robot-check"):
        (root / "jobs" / f"{name}.pkl").unlink(missing_ok=True)
    (root / "deposits" / f"{key}.pkl").unlink()
    passed = (clear["status"] == "OK" and blocked["status"] == "REJECTED"
              and (cold is None or (cold["status"] == "OK"
                                    and cold["progressive_check_status"] == "OK"
                                    and cold["full_robot_check_status"] == "OK")))
    write_json(
        args.output,
        {
            "scope": "One real late-action planning check, with and without an obstructing "
            "deposit injected into an earlier action's predicted journal. The altered "
            "predecessor is never validated. Pen-candidate collision scope.",
            "run": str(args.run),
            "action": actions[late]["action"],
            "predecessor": actions[earlier]["action"],
            "obstruction": obstruction,
            "clear": {"status": clear["status"], "seconds": clear_seconds},
            "blocked": {
                "status": blocked["status"],
                "seconds": blocked_seconds,
                "failure": blocked.get("failure"),
            },
            "cold_fallback_proposal": cold,
            "passed": passed,
        },
    )
    print("PASSED" if passed else "FAILED", clear["status"], blocked["status"])
    return 0 if passed else 1


def _selected(actions):
    return [{"deposit_key": row["deposit_key"], **row["selected"]} for row in actions]


if __name__ == "__main__":
    raise SystemExit(main())
