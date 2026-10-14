"""Add surface detail to a WirePrint frame as a few long strokes drawn after it.

    .venv/bin/python scripts/detail_tours.py FRAME_SOURCE TARGET [options]

FRAME_SOURCE is a source run whose ``framework_graph.json`` is the frame (for
example one written by ``prepare_framework``). TARGET is written as a new
source run: the frame plus detail tours, with ``stroke_graph.json``,
``construction_plan.json``, ``structural_screen.json`` and ``detail_tours.json``.

Rules the detail follows (see .claude/skills/detail-tours/SKILL.md):

* It lies on the frame: lines sit 1 mm (one wire) outside the surface, so they
  rest on the frame wire the way a coil turn rests on the turn below. Points
  where they touch are recorded in ``fused_turns``; there are no struts.
* It is drawn after the frame. The planner lets the pen tip run along existing
  wire; only the nose behind the tip (estimated tapered nose, 3 mm and up)
  must keep ``--margin`` from it, and must stay above the bed.
* One stroke keeps one turntable position (90 degree steps; the pen body leans
  toward the robot), so a stroke stays on one face of the object. Its tilt
  may change along the stroke (0-60 degrees here); the chosen tilt profile
  changes as little as possible.
* Print time is dominated by strokes, so lines on one face are joined end to
  end through short connecting runs on the surface into long tours.

Detail lines come from slice creases (ridges and valleys found by slicing the
mesh along x, y and z), an optional finer pass inside ``--fine-region``, and,
with ``--liberty``, the Statue of Liberty's crown rays, crown band, torch flame
and balcony and the book outline, which keep their designed shape.
"""

import argparse
import shutil
from copy import deepcopy
from pathlib import Path

import numpy as np
import trimesh
import yaml
from scipy.cluster.hierarchy import fcluster, linkage
from scipy.spatial import cKDTree
from shapely.geometry import MultiPoint, Point, Polygon

from pen_art.curves import curve_interval, curve_length
from pen_art.io import read_json, write_json
from pen_art.planning.section_build import write_section_artifacts
from pen_art.planning.sections import section_actions
from pen_art.structure import analyze_structure

WIRE_R = 0.0005
TILTS = (0, 15, 30, 45, 60)
# Measured on planned runs (cube, vase, palace, liberty): about 6 s per stroke
# (pen start and stop waits, approach, withdraw, travel), drawing about
# 20 mm/s on curvy paths. A 90 degree turntable step takes about 8 s.
SECONDS_PER_STROKE, DRAW_SPEED, SECONDS_PER_TURN = 6.0, 0.020, 8.0


# --- geometry helpers ------------------------------------------------------------
def length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def resample(p, step):
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    u = np.linspace(0, s[-1], max(2, int(np.ceil(s[-1] / step)) + 1))
    return np.c_[[np.interp(u, s, p[:, k]) for k in range(3)]].T


def smooth(p, passes):
    p = p.copy()
    for _ in range(passes):
        p[1:-1] = 0.5 * p[1:-1] + 0.25 * (p[:-2] + p[2:])
    return p


def dense(p, step=0.0005):
    out = [p[0]]
    for a, b in zip(p[:-1], p[1:], strict=True):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(out)


def nose_radius(dz):
    """Radius of the estimated tapered nose dz behind the tip.

    From configs/cell/one_flexiv_straight.yaml; the tip itself is 0.5 mm.
    """
    return np.minimum(np.where(dz < 0.003, WIRE_R, 0.0008 + (dz - 0.003) * (0.0112 / 0.027)), 0.012)


class Surface:
    def __init__(self, mesh, offset):
        self.mesh, self.offset = mesh, offset

    def push_out(self, p):
        """Move samples inside the mesh or nearer than ``offset`` out along the surface normal."""
        closest, _, tri = trimesh.proximity.closest_point(self.mesh, p)
        outside = -trimesh.proximity.signed_distance(self.mesh, p)
        move = outside < self.offset
        q = p.copy()
        q[move] = closest[move] + self.mesh.face_normals[tri[move]] * self.offset
        q[:, 2] = np.maximum(q[:, 2], 0.0)
        return q

    def lay(self, p, push=True):
        """1 mm samples resting on the surface, and the azimuth each sample faces."""
        p = resample(p, 0.001)
        if push:
            for passes in (2, 1, 1):
                p = smooth(self.push_out(p), passes)
            p = resample(self.push_out(p), 0.001)
        closest, _, tri = trimesh.proximity.closest_point(self.mesh, p)
        out = p - closest
        flat = np.linalg.norm(out[:, :2], axis=1) < 1e-5
        out[flat] = self.mesh.face_normals[tri[flat]]
        return p, np.degrees(np.arctan2(out[:, 1], out[:, 0]))


# --- detail lines ------------------------------------------------------------------
def slice_creases(mesh, threshold_mm, arc=1.5, sample=0.25, step=0.75, reach=1.3, min_len=6.0):
    """Ridge and valley lines (mm, with signed depth) from slices along x, y and z.

    A slice-outline sample is a crease where it bulges out of (ridge) or dents
    into (valley) the chord to its neighbours ``arc`` mm away; crease samples
    are chained from slice to slice.
    """
    lines = []
    for axis in (0, 1, 2):
        u, v = [k for k in range(3) if k != axis]
        lo, hi = mesh.bounds[:, axis] * 1000
        slices = []
        for h in np.arange(lo + 0.5, hi - 0.5, step):
            section = mesh.section(
                plane_origin=np.eye(3)[axis] * h / 1000, plane_normal=np.eye(3)[axis]
            )
            pts = []
            for ring in [] if section is None else section.discrete:
                ring = np.asarray(ring) * 1000
                L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(ring, axis=0), axis=1))]
                if len(ring) < 9 or L[-1] < 4 * arc:
                    continue
                closed = np.linalg.norm(ring[0] - ring[-1]) < 1e-6
                t = np.arange(0, L[-1], sample)
                p = np.c_[[np.interp(t, L, ring[:, k]) for k in range(3)]].T
                k, n = int(arc / sample), len(p)
                idx = np.arange(n)
                a = p[(idx - k) % n] if closed else p[np.clip(idx - k, 0, n - 1)]
                b = p[(idx + k) % n] if closed else p[np.clip(idx + k, 0, n - 1)]
                q = p[:, [u, v]]
                area = 0.5 * np.sum(q[:, 0] * np.roll(q[:, 1], -1) - np.roll(q[:, 0], -1) * q[:, 1])
                normal = np.c_[(b - a)[:, v], -(b - a)[:, u]] * (1 if area > 0 else -1)
                normal /= np.linalg.norm(normal, axis=1, keepdims=True) + 1e-12
                sag = np.sum((q - ((a + b) / 2)[:, [u, v]]) * normal, axis=1)
                for i in range(n):
                    if not closed and (i < k or i >= n - k):
                        continue
                    w = sag[max(0, i - k) : i + k + 1]
                    if abs(sag[i]) >= threshold_mm and sag[i] == (
                        w.max() if sag[i] > 0 else w.min()
                    ):
                        pts.append([*p[i], sag[i]])
            slices.append((h, np.array(pts)))
        tracks, live = [], []
        for h, pts in slices:
            used, keep = set(), []
            for tr in live:
                last = tr[-1]
                if h - last[axis] > 2 * step + 1e-6:
                    tracks.append(tr)
                    continue
                if len(pts):
                    d = np.linalg.norm(pts[:, :3] - last[:3], axis=1)
                    d[np.sign(pts[:, 3]) != np.sign(last[3])] = np.inf
                    for i in np.argsort(d):
                        if d[i] > reach * (h - last[axis]) / step or i in used:
                            break
                        used.add(i)
                        tr.append(pts[i])
                        break
                keep.append(tr)
            live = keep + [[pts[i]] for i in range(len(pts)) if i not in used]
        for tr in tracks + live:
            tr = np.array(tr)
            if len(tr) > 1 and length(tr[:, :3]) >= min_len:
                lines.append((axis, tr))
    return lines


def liberty_features(mesh):
    """Statue of Liberty crown, torch and book, in mm, keeping their designed shape."""
    V = mesh.vertices * 1000

    def rings(z):
        s = mesh.section(plane_origin=[0, 0, z / 1000], plane_normal=[0, 0, 1])
        return (
            []
            if s is None
            else [Polygon(np.asarray(p)[:, :2] * 1000) for p in s.discrete if len(p) > 3]
        )

    def ring_near(z, xy):
        return min(rings(z), key=lambda poly: poly.distance(Point(xy)) - poly.area * 1e-6)

    def loop(poly, z, offset):
        ring = np.asarray(poly.buffer(offset, quad_segs=4).simplify(0.3).exterior.coords)
        return np.c_[ring, np.full(len(ring), z)]

    out = []
    head = ring_near(108, (-4.5, 0.5))
    c = np.array(head.centroid.coords[0])
    band = V[(V[:, 2] > 107) & (V[:, 2] < 125)]
    r = np.linalg.norm(band[:, :2] - c, axis=1)
    # Each crown ray crosses a shell around the head as a small blob; the
    # raised arm passing the head crosses it as a big one.
    shell = band[(r > 11) & (r < 12.5)]
    labels = fcluster(linkage(shell, "single"), 1.0, "distance")
    for k in np.unique(labels):
        blob = shell[labels == k]
        if len(blob) < 10 or np.ptp(blob, axis=0).max() > 4.0:
            continue
        mid = blob.mean(0)
        d0 = (mid[:2] - c) / np.linalg.norm(mid[:2] - c)
        rel = band[:, :2] - c
        along = rel @ d0
        side = np.linalg.norm(rel - along[:, None] * d0, axis=1)
        ray = band[
            (along > 11)
            & (along < 17)
            & (side < 2.5)
            & (np.abs(band[:, 2] - mid[2]) < 0.6 * (along - 11) + 2.5)
        ]
        tip = ray[np.argmax(ray[:, :2] @ d0)]
        d = (tip[:2] - c) / np.linalg.norm(tip[:2] - c)
        out.append((f"crown-ray-{len(out)}", np.linspace(np.r_[c + d * 6.5, 112.0], tip, 8), False))
    out.append(("crown-band", loop(ring_near(112, tuple(c)), 112.0, 1.5), False))
    flame = V[V[:, 2] > 140.5]
    xz = np.asarray(
        MultiPoint(flame[:, [0, 2]])
        .convex_hull.buffer(1.5, quad_segs=4)
        .simplify(0.3)
        .exterior.coords
    )
    out.append(
        ("torch-flame", np.c_[xz[:, 0], np.full(len(xz), np.median(flame[:, 1])), xz[:, 1]], False)
    )
    out.append(("torch-balcony", loop(ring_near(138.0, (-19, -19)), 138.0, 2.0), False))
    P = V[(V[:, 0] > 14) & (V[:, 2] > 56) & (V[:, 2] < 83)]
    mean = P.mean(0)
    _, _, W = np.linalg.svd(P - mean, full_matrices=False)
    rect = np.asarray(
        MultiPoint(
            np.c_[(P - mean) @ W[0], (P - mean) @ W[1]]
        ).minimum_rotated_rectangle.exterior.coords
    )
    h = (P - mean) @ W[2]
    side = np.sign(W[2][0]) if abs(W[2][0]) > 0.2 else 1.0
    lift = (h.max() if side > 0 else h.min()) + side * 1.0
    corners = mean + rect[:, :1] * W[0] + rect[:, 1:] * W[1] + lift * W[2]
    edges = [
        np.linspace(a, b, 12, endpoint=False)
        for a, b in zip(corners[:-1], corners[1:], strict=True)
    ]
    out.append(("book", np.vstack(edges + [corners[:1]]), False))
    return out


def dip(p, frame, tree, reach, spacing=0.006, width=0.002):
    """Bend a laid line onto nearby frame wire so it rests there.

    Where the line passes within ``reach`` of frame wire (a local minimum of the
    distance, at most every ``spacing``), move it toward the wire until the two
    wires touch, blending the move over about ``width`` either side.
    """
    d, idx = tree.query(p)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    shift, taken = np.zeros_like(p), []
    for i in np.argsort(d):
        if d[i] > reach:
            break
        touching = d[i] <= 2 * WIRE_R + 0.0002
        if touching or d[i] != d[max(0, i - 3) : i + 4].min():
            continue
        if any(abs(s[i] - t) < spacing for t in taken):
            continue
        move = (frame[idx[i]] - p[i]) * (1 - 2 * WIRE_R / d[i])
        shift += np.exp(-(((s - s[i]) / width) ** 2))[:, None] * move
        taken.append(s[i])
    q = p + shift
    q[:, 2] = np.maximum(q[:, 2], 0.0)
    return q


def candidate_lines(mesh, args):
    """(name, points m, importance, lay on surface) for every candidate detail line."""
    out = []
    if args.liberty:
        out += [(name, p / 1000, np.inf, push) for name, p, push in liberty_features(mesh)]
    # Shallower creases come later in importance; they fill in smooth objects.
    found = [line for th in args.crease_threshold for line in slice_creases(mesh, th)]
    for axis, t in found:
        out.append(
            (
                f"crease-{'xyz'[axis]}",
                smooth(t[:, :3], 3) / 1000,
                length(t[:, :3]) * np.abs(t[:, 3]).mean(),
                True,
            )
        )
    if args.fine_region:
        box = np.asarray(args.fine_region, float).reshape(3, 2)
        for axis, t in slice_creases(
            mesh, 0.12, arc=0.7, sample=0.12, step=0.35, reach=1.0, min_len=2.5
        ):
            if np.all((t[:, :3] >= box[:, 0]) & (t[:, :3] <= box[:, 1]), axis=1).mean() > 0.8:
                out.append(
                    (
                        f"fine-{'xyz'[axis]}",
                        t[:, :3] / 1000,
                        4 * length(t[:, :3]) * np.abs(t[:, 3]).mean(),
                        True,
                    )
                )
    return out


def detail_profile(base, path, nose_clearance=0.0003):
    """The frame's profile, plus pen tilts for detail and the nose clearance."""
    profile = yaml.safe_load(Path(base).read_text())
    profile["drawing_pen_tilt_candidates_deg"] = [0.0, 60.0, 45.0, 30.0, 15.0]
    profile["environment"]["tool_clearance_m"] = {"tapered_nose": nose_clearance}
    profile["setup_description"] = (
        profile.get("setup_description", "")
        + " Detail strokes after the frame may tilt the pen toward the robot; the nose"
        + f" keeps {nose_clearance * 1000:g} mm from wire."
    )
    Path(path).write_text(yaml.safe_dump(profile, sort_keys=False))
    return Path(path)


def planner_tilts(graph, frame_source, profile_path, cell_dir, seconds=5.0):
    """Tilts the planner's own sampler can draw each tour with, best first.

    At the tour's turntable position, each tilt must pass the planner's pen
    screen against the frame (start, middle, end) and give a drawing path
    (IK along the whole tour). This is the check the planner's cold sampler
    makes; tours with no tilt left would only cost a planning round.
    """
    from time import monotonic

    from pen_art.curves import flatten_curve, transform_points
    from pen_art.planning.axis_corridor import pen_clear
    from pen_art.planning.cell_motion import drawing_alternatives, fixed_rotation_families
    from pen_art.planning.cell_planner import CellPlanner
    from pen_art.planning.scene import MaterialScene
    from pen_art.planning.turntable import IndexedTurntable

    settings = yaml.safe_load(Path("configs/dual_arm.yaml").read_text())
    settings.update(maximum_subdivision_depth=20)
    from pen_art.cell import prepare_cell
    from pen_art.planning.section_build import prepare_candidates

    shutil.rmtree(cell_dir, ignore_errors=True)
    prepare_cell(profile_path, cell_dir, frame_source)
    prepare_candidates(cell_dir)
    planner = CellPlanner(cell_dir, profile_path, settings, float("inf"))
    arm = planner.collision_arm(next(iter(planner.arms)))
    table = IndexedTurntable.from_profile(planner.profile)
    actions = section_actions(graph, group_sources=True)["actions"]
    frame = [
        np.asarray(seg)[[0, -1]]
        for a in actions
        if a["stage"] != "artistic"
        for seg in a["segments"]
    ]
    hints = {
        e["parent_curve"]: e["pen_tilt_candidates_deg"]
        for e in graph["edges"]
        if "pen_tilt_candidates_deg" in e
    }
    scenes, out = {}, {}
    for action in actions:
        tour = action.get("parent_curve")
        if action["stage"] != "artistic" or tour in out:
            continue
        index = action["turntable_index"]
        world = table.build_transform(index)
        if index not in scenes:
            scenes[index] = MaterialScene()
            for k, seg in enumerate(frame):
                a, b = transform_points(seg, world)
                scenes[index].add(a, b, WIRE_R, f"frame/{k}")
        points = transform_points(flatten_curve(action, 0.00005, 0.003)[0], world)
        tried = [*hints.get(tour, []), *[float(t) for t in sorted(TILTS, reverse=True)]]
        works = []
        for tilt in dict.fromkeys(tried):
            screened = [
                f
                for f in fixed_rotation_families(arm, world, [tilt])
                if all(
                    pen_clear(
                        planner,
                        arm,
                        points[i],
                        f["rotations"],
                        scenes[index],
                        planner.margin + 2e-6,
                    )
                    for i in (0, len(points) // 2, -1)
                )
            ]
            if not screened:
                continue
            for drawing in drawing_alternatives(
                arm,
                action,
                {action["start"], action["end"]},
                world,
                planner.obstacles,
                deadline=monotonic() + seconds,
                seeds=[arm.initial_positions],
                settings={"maximum_subdivision_depth": 20},
                clearance_m=planner.margin,
                rotations=screened,
                orientation_policy="fixed",
            ):
                if drawing["status"] == "SAMPLED_DRAWING_PATH":
                    works.append(tilt)
                    break
            if len(works) >= 2:
                break
        out[tour] = works
    return out


# --- main ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("frame_source", type=Path)
    parser.add_argument("target", type=Path)
    parser.add_argument(
        "--profile",
        type=Path,
        default=Path("runs/geomfirst_profiles/liberty_statue.yaml"),
        help="cell profile: build placement, robot base and turntable",
    )
    parser.add_argument("--max-strokes", type=int, default=80)
    parser.add_argument(
        "--crease-threshold",
        type=float,
        nargs="+",
        default=[0.12, 0.08, 0.05],
        help="mm a crease bulges or dents; several values add shallower creases",
    )
    parser.add_argument(
        "--min-line", type=float, default=0.008, help="m; shorter crease lines are ignored"
    )
    parser.add_argument(
        "--offset", type=float, default=0.001, help="m outside the surface (one wire)"
    )
    parser.add_argument("--bridge", type=float, default=0.022, help="m; longest connecting run")
    parser.add_argument(
        "--margin", type=float, default=0.001, help="m nose clearance (planner clearance_m)"
    )
    parser.add_argument(
        "--check-with-planner",
        action="store_true",
        help="keep only tours the planner's sampler can draw, with the tilts it can use",
    )
    parser.add_argument("--detail", type=float, default=5.0, help="m of detail to aim for")
    parser.add_argument("--minutes", type=float, default=15.0, help="added print-time limit")
    parser.add_argument(
        "--collect", type=float, default=1.6, help="collect this x --detail of reachable line"
    )
    parser.add_argument(
        "--dip", type=float, default=0.0035, help="m; bend lines onto frame wire this near"
    )
    parser.add_argument(
        "--fine-region",
        type=float,
        nargs=6,
        metavar=("X0", "X1", "Y0", "Y1", "Z0", "Z1"),
        help="mm box for a finer crease pass (Liberty face: -11 2 -15 15 95 111)",
    )
    parser.add_argument("--liberty", action="store_true", help="add Liberty crown, torch and book")
    args = parser.parse_args()

    source, target = args.frame_source, args.target
    mesh = trimesh.load(source / "normalized.glb", force="mesh")
    surface = Surface(mesh, args.offset)
    profile = yaml.safe_load(args.profile.read_text())
    build = np.asarray(profile["world_from_build"])[:2, 3]
    base = np.asarray(profile["arms"][0]["world_from_base"])[:2, 3]
    robot_az = np.degrees(np.arctan2(*(base - build)[::-1]))
    # Build azimuth of the face that looks at the robot at each turntable position.
    sectors = [(robot_az - 90 * k + 180) % 360 - 180 for k in range(4)]

    def sector_of(az):
        return np.argmin(np.abs((az[:, None] - np.array(sectors)[None] + 180) % 360 - 180), axis=1)

    def pose_axis(sec, tilt):
        h, t = np.radians(sectors[sec]), np.radians(tilt)
        return np.array([np.sin(t) * np.cos(h), np.sin(t) * np.sin(h), np.cos(t)])

    def nose_hits(x, a, pts, tree):
        """Wire points inside the nose (+margin) at tip x, axis a; None if it meets the bed."""
        s = np.linspace(0.003, 0.03, 10)
        if np.any(x[2] + s * a[2] - nose_radius(s) * np.sqrt(max(0.0, 1 - a[2] ** 2)) <= 0):
            return None
        idx = tree.query_ball_point(x + a * 0.1, r=0.1 + 0.012 + WIRE_R + args.margin)
        if not idx:
            return np.zeros(0, int)
        idx = np.asarray(idx)
        d = pts[idx] - x
        s = d @ a
        rho = np.linalg.norm(d - s[:, None] * a, axis=1)
        return idx[(s > 0.003) & (rho < nose_radius(s) + WIRE_R + args.margin)]

    framework = read_json(source / "framework_graph.json")
    frame = np.vstack([dense(np.asarray(e["segments"][0])[[0, -1]]) for e in framework["edges"]])
    frame_tree = cKDTree(frame)

    def clear_sets(p, sec):
        out = []
        for x in p:
            ok = set()
            for t in TILTS:
                h = nose_hits(x, pose_axis(sec, t), frame, frame_tree)
                if h is not None and not len(h):
                    ok.add(t)
            out.append(ok)
        return out

    # 1-2. take lines by importance, skipping near-duplicates; lay each on the
    #      surface and cut it where it turns to another face or where no tilt
    #      clears the frame; stop once enough reachable line is collected
    pieces, stages = [], {"laid": 0.0, "one face": 0.0}
    chosen, taken, used, reachable = [], [], 0.0, 0.0
    for name, p, importance, push in sorted(candidate_lines(mesh, args), key=lambda c: -c[2]):
        if reachable >= args.collect * args.detail:
            break
        if importance != np.inf and length(p) < args.min_line:
            continue
        q = resample(p, 0.0005)
        if taken and (cKDTree(np.vstack(taken)).query(q)[0] < 0.0015).mean() > 0.5:
            continue
        chosen.append((name, q, push))
        taken.append(q)
        used += length(q)
        p, az = surface.lay(q, push)
        stages["laid"] += length(p)
        p = dip(p, frame, frame_tree, args.dip)
        sec = sector_of(az)
        for i in range(1, len(sec) - 1):
            if sec[i - 1] == sec[i + 1] != sec[i]:
                sec[i] = sec[i - 1]
        cuts = np.flatnonzero(np.diff(sec)) + 1
        for a, b in zip(np.r_[0, cuts], np.r_[cuts, len(p)], strict=True):
            if b - a < 6:
                continue
            part, s = p[a:b], int(sec[a])
            stages["one face"] += length(part)
            sets = clear_sets(part, s)
            ok = np.array([bool(x) for x in sets], int)
            for run in np.split(np.arange(len(part)), np.flatnonzero(np.diff(ok)) + 1):
                if ok[run[0]] and len(run) >= 8:
                    pieces.append((part[run], s, [sets[i] for i in run], [name]))
                    reachable += length(part[run])
    stages["tilt clears frame"] = sum(length(r[0]) for r in pieces)

    # 3. join pieces of one face end to end into tours
    def bridge(a, b):
        p = surface.push_out(
            np.linspace(a, b, max(2, int(np.ceil(np.linalg.norm(b - a) / 0.001)) + 1))
        )
        return surface.push_out(smooth(p, 1))

    tours = []
    for sec in range(4):
        left = [i for i, r in enumerate(pieces) if r[1] == sec]
        while left:
            first = min(left, key=lambda i: pieces[i][0][:, 2].min())
            left.remove(first)
            p, _, sets, names = pieces[first]
            if p[0, 2] > p[-1, 2]:
                p, sets = p[::-1], sets[::-1]
            path, tilts, names, tried = [p], list(sets), list(names), set()
            while True:
                end = path[-1][-1]
                options = [
                    (np.linalg.norm((pieces[i][0][-1] if rev else pieces[i][0][0]) - end), i, rev)
                    for i in left
                    if i not in tried
                    for rev in (False, True)
                ]
                options = [o for o in options if o[0] <= args.bridge]
                if not options:
                    break
                _, i, rev = min(options)
                q, qs = pieces[i][0], pieces[i][2]
                if rev:
                    q, qs = q[::-1], qs[::-1]
                b = bridge(end, q[0])[1:-1]
                bs = clear_sets(b, sec) if len(b) else []
                if any(not x for x in bs):
                    tried.add(i)
                    continue
                path += [b, q]
                tilts += bs + list(qs)
                names += pieces[i][3]
                left.remove(i)
            tours.append((np.vstack(path), sec, tilts, names))
    stages["joined (with connecting runs)"] = sum(length(t[0]) for t in tours)
    tours = sorted((t for t in tours if length(t[0]) >= 0.008), key=lambda t: -length(t[0]))[
        : args.max_strokes
    ]
    stages["8 mm or longer, stroke cap"] = sum(length(t[0]) for t in tours)

    # 4. order tours so no nose meets detail drawn earlier. Peel from the end:
    #    the last tour must clear all others. When none can, cut the conflicting
    #    samples out of the tour that conflicts least.
    allp = np.vstack([t[0] for t in tours])
    tree = cKDTree(allp)
    start = np.cumsum([0] + [len(t[0]) for t in tours])
    owner = np.concatenate([np.full(len(t[0]), k) for k, t in enumerate(tours)])
    items = {}
    for k, (_, sec, tilts, _) in enumerate(tours):
        g = np.arange(start[k], start[k + 1])
        items[k] = (
            g,
            sec,
            [
                {t: nose_hits(allp[i], pose_axis(sec, t), allp, tree) for t in tilts[j]}
                for j, i in enumerate(g)
            ],
        )
    next_id, remaining, order, allowed_at, cut_out = len(tours), set(items), [], {}, 0

    def allowed(k):
        """Clear tilts per sample: the nose misses every other remaining tour and
        this tour's own wire more than 4 mm along it (either drawing direction)."""
        others = remaining - {k}
        return [
            {
                t
                for t, h in hs.items()
                if not any(owner[j] in others or (owner[j] == k and abs(j - g) > 4) for j in h)
            }
            for g, hs in zip(items[k][0], items[k][2], strict=True)
        ]

    while remaining:
        pick = next(
            (
                (k, a)
                for k in sorted(
                    remaining, key=lambda k: (-items[k][1], -allp[items[k][0], 2].min())
                )
                for a in [allowed(k)]
                if all(a)
            ),
            None,
        )
        if pick:
            remaining.discard(pick[0])
            order.append(pick[0])
            allowed_at[pick[0]] = pick[1]
            continue
        k, a = min(
            ((k, allowed(k)) for k in remaining), key=lambda r: sum(1 for x in r[1] if not x)
        )
        bad = np.array([not x for x in a])
        g, sec, hits = items.pop(k)
        remaining.discard(k)
        owner[g[bad]] = -1
        cut_out += int(bad.sum())
        good = np.flatnonzero(~bad)
        for run in np.split(good, np.flatnonzero(np.diff(good) > 1) + 1):
            if len(run) >= 8:
                items[next_id] = (g[run], sec, [hits[i] for i in run])
                owner[g[run]] = next_id
                remaining.add(next_id)
                next_id += 1
            elif len(run):
                owner[g[run]] = -1
                cut_out += len(run)
    order.reverse()
    stages["ordered"] = sum(length(allp[items[k][0]]) for k in order)

    def tilt_profile(sets):
        """A clear tilt per sample that changes as little as possible (Viterbi)."""
        T = np.array(TILTS, float)
        unary = np.array([[0.01 * abs(t - 45) if t in s else np.inf for t in TILTS] for s in sets])
        cost, back = unary[0].copy(), np.zeros((len(sets), len(T)), int)
        for i in range(1, len(sets)):
            total = cost[:, None] + np.abs(T[:, None] - T[None, :])
            back[i] = np.argmin(total, axis=0)
            cost = total[back[i], np.arange(len(T))] + unary[i]
        path = [int(np.argmin(cost))]
        for i in range(len(sets) - 1, 0, -1):
            path.append(back[i][path[-1]])
        return T[path[::-1]]

    # 5. graph: tours on the frame, fused where they touch
    graph = deepcopy(framework)
    nodes = {n["id"]: n for n in graph["nodes"]}
    edges = {e["id"]: e for e in graph["edges"]}
    cuts = {key: {0.0: e["start"], 1.0: e["end"]} for key, e in edges.items()}
    keys, fracs, pts = [], [], []
    for key, e in edges.items():
        a, b = np.asarray(e["segments"][0][0]), np.asarray(e["segments"][0][-1])
        for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / 0.0005))):
            keys.append(key)
            fracs.append(t)
            pts.append(a + t * (b - a))
    ftree = cKDTree(np.asarray(pts))
    touch = WIRE_R * 2 + 0.0004

    def new_node(point, prefix):
        nid = f"{prefix}{len(nodes):05d}"
        nodes[nid] = {
            "id": nid,
            "position": np.asarray(point).tolist(),
            "ground": bool(point[2] <= 1e-7),
        }
        return nid

    fused, art_edges, rows, rank, dropped = [], [], [], -1, 0
    # Trim each tour to start on a contact and overhang at most 8 mm; leave out
    # tours that never touch the frame. Then keep the longest tours (most detail
    # per stroke) within the detail length and the added print time.
    ready = {}
    for k in order:
        p, tilt = allp[items[k][0]], tilt_profile(allowed_at[k])
        d, _ = ftree.query(p)
        on = np.flatnonzero(d <= touch)
        if not len(on):
            dropped += 1
            continue
        # Start on a contact; of the two directions keep the one that keeps
        # more line, and when they keep about the same, the one that ends
        # going up, so the pen backs away from wire it has just laid.
        options = []
        for flip in (False, True):
            q, t, o = (p[::-1], tilt[::-1], len(p) - 1 - on[::-1]) if flip else (p, tilt, on)
            q, t = q[o[0] : min(len(q), o[-1] + 9)], t[o[0] : min(len(q), o[-1] + 9)]
            rising = len(q) > 5 and q[-1, 2] > q[-6, 2]
            options.append((round(length(q) / 0.003), rising, q, t))
        _, _, p, tilt = max(options, key=lambda r: (r[0], r[1]))
        if length(p) >= 0.006:
            ready[k] = (p, tilt)
        else:
            dropped += 1
    keep, total = set(), 0.0
    for k in sorted(ready, key=lambda k: -length(ready[k][0])):
        L = length(ready[k][0])
        strokes = len(keep) + 1
        minutes = (
            (total + L) / DRAW_SPEED
            + SECONDS_PER_STROKE * strokes
            + SECONDS_PER_TURN * min(strokes, 12)
        ) / 60
        if total + L <= args.detail and minutes <= args.minutes:
            keep.add(k)
            total += L
    stages["within detail and time"] = total
    for k in [k for k in order if k in keep]:
        (p, tilt), sec = ready[k], items[k][1]
        # The planner holds one tilt per stroke: offer the tilts clear along the
        # whole tour first (steepest first), then the tour's most common ones.
        whole = sorted(set.intersection(*allowed_at[k]), reverse=True)
        common = [
            t
            for t, _ in sorted(
                zip(*np.unique(tilt, return_counts=True), strict=True), key=lambda r: -r[1]
            )
        ]
        hints = [float(t) for t in dict.fromkeys([*whole, *common])]
        rank += 1
        # The planner starts a stroke on a built node, so the tour begins on
        # the frame wire itself, at a node it shares with the frame.
        _, first = ftree.query(p[0])
        key0, t0 = keys[first], fracs[first]
        near = [u for u in cuts[key0] if abs(u - t0) < 1e-3]
        t0 = near[0] if near else t0
        if t0 not in cuts[key0]:
            e = edges[key0]
            ea, eb = np.asarray(e["segments"][0][0]), np.asarray(e["segments"][0][-1])
            cuts[key0][t0] = new_node(ea + t0 * (eb - ea), "cut-n")
        p = np.vstack([nodes[cuts[key0][t0]]["position"], p[1:]])
        knots = resample(p, 0.003)
        kk = np.vstack([2 * knots[0] - knots[1], knots, 2 * knots[-1] - knots[-2]])
        segs = np.array(
            [
                [
                    kk[i],
                    kk[i] + (kk[i + 1] - kk[i - 1]) / 6,
                    kk[i + 1] - (kk[i + 2] - kk[i]) / 6,
                    kk[i + 1],
                ]
                for i in range(1, len(kk) - 2)
            ]
        )
        segs[..., 2] = np.maximum(segs[..., 2], 0.0)
        d, idx = ftree.query(knots)
        s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(knots, axis=0), axis=1))]
        contacts, last = [], -1.0
        for i in range(len(knots)):
            if d[i] <= touch and d[i] == d[max(0, i - 2) : i + 3].min() and s[i] - last >= 0.004:
                contacts.append(i)
                last = s[i]
        contacts = [i for i in contacts if s[i] >= 0.002]
        bounds = sorted(set([0, len(knots) - 1] + contacts))
        tid = f"tour-{rank:03d}"
        knot_node = {i: new_node(knots[i], "tour-n") for i in bounds}
        knot_node[0] = cuts[key0][t0]
        for i in contacts:
            key, t = keys[idx[i]], fracs[idx[i]]
            near = [u for u in cuts[key] if abs(u - t) < 1e-3]
            t = near[0] if near else t
            if t not in cuts[key]:
                e = edges[key]
                ea, eb = np.asarray(e["segments"][0][0]), np.asarray(e["segments"][0][-1])
                cuts[key][t] = new_node(ea + t * (eb - ea), "cut-n")
            fused.append([knot_node[i], cuts[key][t]])
        for j, (a, b) in enumerate(zip(bounds[:-1], bounds[1:], strict=True)):
            art_edges.append(
                {
                    "id": f"{tid}-m{j:03d}",
                    "start": knot_node[a],
                    "end": knot_node[b],
                    "role": "artistic",
                    "origin": "detail_tour",
                    "required": True,
                    "diameter_m": 2 * WIRE_R,
                    "segments": segs[a:b].tolist(),
                    "process_status": "not_checked",
                    "parent_curve": tid,
                    "source_interval": [a, b],
                    "stage": "artistic",
                    "order_hint": 20000 + rank,
                    "turntable_index": int(sec),
                    "pen_tilt_candidates_deg": hints,
                }
            )
        rows.append(
            {
                "id": tid,
                "length_m": length(p),
                "turntable_index": int(sec),
                "contacts": len(contacts),
                "tilt_deg": tilt.tolist(),
                "lines": names_of(tours, start, items[k][0][0]),
            }
        )

    split = []
    for key, e in edges.items():
        ordered = sorted(cuts[key])
        lo, hi = e["source_interval"]
        for i, (a, b) in enumerate(zip(ordered[:-1], ordered[1:], strict=True)):
            split.append(
                {
                    **e,
                    "id": f"{key}-part{i}" if len(ordered) > 2 else key,
                    "start": cuts[key][a],
                    "end": cuts[key][b],
                    "segments": curve_interval(e, a, b),
                    "source_interval": [lo + a * (hi - lo), lo + b * (hi - lo)],
                }
            )
    for e in split + art_edges:
        e["length_m"] = curve_length(e)
    graph.update(
        nodes=list(nodes.values()),
        edges=split + art_edges,
        fused_turns=graph.get("fused_turns", []) + fused,
        method="wireprint_with_detail_tours",
    )

    if args.check_with_planner:
        profile_path = detail_profile(
            args.profile, args.profile.parent / f"{target.name}_check_profile.yaml"
        )
        cell_dir = target.parent / f"{target.name}_check_cell"
        works = planner_tilts(graph, source, profile_path, cell_dir)
        gone = {tour for tour, tilts in works.items() if not tilts}
        for e in graph["edges"]:
            if works.get(e.get("parent_curve")):
                e["pen_tilt_candidates_deg"] = works[e["parent_curve"]]
        graph["edges"] = [e for e in graph["edges"] if e.get("parent_curve") not in gone]
        used_nodes = {n for e in graph["edges"] for n in (e["start"], e["end"])}
        graph["nodes"] = [n for n in graph["nodes"] if n["id"] in used_nodes]
        graph["fused_turns"] = [
            p for p in graph["fused_turns"] if p[0] in used_nodes and p[1] in used_nodes
        ]
        rows = [r for r in rows if r["id"] not in gone]
        stages["arm can draw it"] = sum(r["length_m"] for r in rows)
        print(f"planner sampler: {len(gone)} of {len(works)} tours have no drawable tilt")

    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(source, target)
    for stale in ("style_selection.json", "style_attachments.json"):
        (target / stale).unlink(missing_ok=True)
    construction = tilt_hints(section_actions(graph, group_sources=True), graph)
    write_json(target / "stroke_graph.json", graph)
    write_json(target / "construction_plan.json", construction)
    write_section_artifacts(target, graph, construction)
    screen = analyze_structure(graph, read_json(source / "fabrication_config.json")["structure"])
    write_json(target / "structural_screen.json", screen)
    detail = sum(r["length_m"] for r in rows)
    turns = sum(
        1
        for a, b in zip(rows[:-1], rows[1:], strict=True)
        if a["turntable_index"] != b["turntable_index"]
    )
    minutes = (
        detail / DRAW_SPEED + SECONDS_PER_STROKE * len(rows) + SECONDS_PER_TURN * (turns + 1)
    ) / 60
    write_json(
        target / "detail_tours.json",
        {
            "tours": rows,
            "turntable_sector_azimuth_deg": sectors,
            "estimated_added_minutes": minutes,
            "settings": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()},
        },
    )
    run = read_json(target / "run.json")
    run["id"] = target.name
    run["name"] = f"{run.get('name', target.name).split(' + ')[0]} + detail tours (not planned)"
    write_json(target / "run.json", run)
    tilts = np.concatenate([r["tilt_deg"] for r in rows])
    median = 1e3 * np.median([r["length_m"] for r in rows])
    counts = dict(zip(*[x.tolist() for x in np.unique(tilts, return_counts=True)], strict=True))
    steady = sum(1 for r in rows if len(set(r["tilt_deg"])) == 1)
    sag = 1e3 * screen.get("maximum_displacement_m", np.nan)
    stages["touches frame, trimmed"] = detail
    print("length by stage (m): " + ", ".join(f"{k} {v:.2f}" for k, v in stages.items()))
    print(f"lines {len(chosen)} ({used:.2f} m) -> {len(pieces)} pieces -> {len(tours)} tours")
    print(f"ordering cut {cut_out / 1000:.2f} m; {dropped} tours never touch the frame")
    print(f"detail {detail:.2f} m in {len(rows)} strokes (median {median:.0f} mm)")
    print(f"{len(fused)} fused contacts, {turns} turntable changes, about {minutes:.1f} min more")
    print(f"tilt samples {counts}; {steady}/{len(rows)} strokes keep one tilt")
    print(f"structure pass {screen['modeled_pass']} sag {sag:.3f} mm {screen.get('failure', '')}")


def tilt_hints(construction, graph):
    """Copy each tour's tilt candidates onto its construction action."""
    hints = {
        e["parent_curve"]: e["pen_tilt_candidates_deg"]
        for e in graph["edges"]
        if "pen_tilt_candidates_deg" in e
    }
    for action in construction["actions"]:
        if action.get("parent_curve") in hints:
            action["pen_tilt_candidates_deg"] = hints[action["parent_curve"]]
    return construction


def names_of(tours, start, sample):
    return tours[int(np.searchsorted(start, sample, side="right") - 1)][3]


if __name__ == "__main__":
    main()
