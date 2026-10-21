"""Add surface detail to a WirePrint frame as a few long strokes drawn after it.

``pen-art detail NAME`` calls ``generate_detail`` for an object.

FRAME_SOURCE is the folder ``pen-art frame`` wrote; its ``framework_graph.json``
is the frame. TARGET references its assets, with the detail tours added to
``stroke_graph.json``, ``construction_plan.json`` and ``structural_screen.json``,
and listed in ``detail_tours.json``.

Rules the detail follows:

* It lies on the frame: lines sit 1 mm (one wire) outside the surface, so they
  rest on the frame wire the way a coil turn rests on the turn below. Points
  where they touch are recorded in ``fused_turns``; there are no struts.
* It is drawn after the frame. The planner lets the pen tip run along existing
  wire; only the nose behind the tip (estimated tapered nose, 3 mm and up)
  must keep MARGIN from it, and must stay above the bed.
* One stroke keeps one side of the object (one of four), and the pen body
  leans toward that side: toward the robot for the side facing it, away from
  it for the far side. The planner draws a stroke at one tilt (0-60 degrees);
  the tilts offered are those clear along the whole stroke first.
* Print time is dominated by strokes, so lines on one face are joined end to
  end through short connecting runs on the surface into long tours.

Detail lines come from slice creases: ridges and valleys found by slicing the
mesh along x, y and z.
"""

import shutil
from copy import deepcopy
from functools import partial
from pathlib import Path

import numpy as np
import trimesh
import yaml
from scipy.spatial import cKDTree

from pen_art.curves import curve_interval, curve_length
from pen_art.frame import write_strokes
from pen_art.io import asset_path, read_json, reference_assets, write_json
from pen_art.planning.sections import section_actions

WIRE_R = 0.0005
TILTS = (0, 15, 30, 45, 60)
# Measured on planned detail runs (pumpkin, ghost, 2026-10-14): each detail
# tour costs about 20 s besides drawing (travel into a tilted pose 9-11 s,
# approach and withdrawal 6-7 s, waits and join about 4 s); detail draws at
# about 10 mm/s.
SECONDS_PER_STROKE, DRAW_SPEED = 20.0, 0.010
# Exit status when no drawable detail was found; nothing is written.
class NoDetail(ValueError):
    """No drawable detail was found; the frame can still be planned."""
# Lines sit OFFSET (one wire) outside the surface; crease lines shorter than
# MIN_LINE are ignored; line is collected until COLLECT times the detail wanted
# is reachable; a line bends onto frame wire within DIP; pieces of one face are
# joined by connecting runs up to BRIDGE long into at most MAX_STROKES tours;
# the nose keeps MARGIN (the planner's clearance) from wire. All in metres.
OFFSET, MIN_LINE, COLLECT, DIP, BRIDGE, MAX_STROKES, MARGIN = (
    0.001, 0.008, 1.6, 0.0035, 0.022, 80, 0.001,
)
# Lines and frame edges are sampled every STEP; a candidate line is skipped as
# a near-duplicate when more than half of it lies within DUPLICATE of lines
# already taken. Laid lines have 1 mm samples: a part on one face needs
# FACE_SAMPLES of them, and a run some tilt clears (or a piece left by a cut)
# needs MIN_SAMPLES. Connecting runs are sampled every BRIDGE_STEP; tours
# shorter than MIN_TOUR are dropped.
STEP = 0.0005  # m
DUPLICATE = 0.0015  # m
FACE_SAMPLES = 6
MIN_SAMPLES = 8
BRIDGE_STEP = 0.001  # m
MIN_TOUR = 0.008  # m
# Ordering: the nose may touch a tour's own wire within OWN_WIRE samples of the tip.
OWN_WIRE = 4
# Tilt per sample: changing tilt costs its degrees; each degree away from
# LIKED_TILT costs TILT_COST.
LIKED_TILT, TILT_COST = 45, 0.01
# A tour touches frame wire within TOUCH; it is trimmed to start on a contact
# and run at most OVERHANG samples (mm) past its last one; it ends going up
# when it rose over its last RISE samples; two directions keep the same line
# when they round to the same SAME_LENGTH; a trimmed tour shorter than
# MIN_SUPPORTED is dropped.
TOUCH = WIRE_R * 2 + 0.0004  # m
OVERHANG, RISE = 8, 5
SAME_LENGTH, MIN_SUPPORTED = 0.003, 0.006  # m
# Planner check: a tour is cut at most MAX_CUTS times; END_TRIM samples come
# off an end that cannot be approached or left; a cut at a failing sample
# removes CUT_BEFORE samples before it and CUT_AFTER from it on; pieces
# shorter than MIN_CHECKED are dropped.
MAX_CUTS, END_TRIM, CUT_BEFORE, CUT_AFTER = 4, 5, 2, 3
MIN_CHECKED = 0.010  # m
# Stroke graph: cubic knots every KNOT_SPACING; a contact is a knot within
# TOUCH that is nearest the frame among CONTACT_WINDOW knots either side, at
# least CONTACT_SPACING after the last contact and FIRST_CONTACT from the
# start; a frame cut within CUT_SNAP (edge fraction) of another reuses it;
# detail tours are ordered after the frame from DETAIL_ORDER.
KNOT_SPACING = 0.003  # m
CONTACT_WINDOW = 2
CONTACT_SPACING, FIRST_CONTACT = 0.004, 0.002  # m
CUT_SNAP = 1e-3
DETAIL_ORDER = 20000
# Nose screen: NOSE_SAMPLES points from NOSE_TIP (where the nose starts behind
# the tip) to NOSE_LENGTH are kept off the bed; wire is searched in a ball
# NOSE_BALL past the widest nose radius NOSE_MAX_R, centred NOSE_BALL up the pen.
NOSE_TIP, NOSE_LENGTH, NOSE_SAMPLES = 0.003, 0.03, 10
NOSE_BALL, NOSE_MAX_R = 0.1, 0.012  # m


def estimated_minutes(length_m, strokes):
    """Print minutes that detail adds: drawing at DRAW_SPEED plus each stroke's approach."""
    return (length_m / DRAW_SPEED + SECONDS_PER_STROKE * strokes) / 60


def choose_detail(lines, detail_m, minutes, keep=(), dropped=(), most=None):
    """The ids of the detail strokes to draw.

    ``lines`` is (id, metres) pairs. The strokes in ``keep`` stay, except those
    in ``dropped``; then, longest first, up to ``most`` more lines are added
    while the total stays within ``detail_m`` and the estimated added
    ``minutes``.
    """
    metres = dict(lines)
    chosen = [name for name in keep if name not in dropped]
    total, added = sum(metres[name] for name in chosen), 0
    for name, length in sorted(lines, key=lambda line: -line[1]):
        if most is not None and added >= most:
            break
        if name in dropped or name in chosen:
            continue
        if total + length <= detail_m + 1e-9 and (
            estimated_minutes(total + length, len(chosen) + 1) <= minutes + 1e-9
        ):
            chosen.append(name)
            total, added = total + length, added + 1
    return chosen


def no_detail(why):
    raise NoDetail(why)


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

    The modelled pen's nose (src/pen_art/cell_template.yaml); the tip itself is 0.5 mm.
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

    def lay(self, p):
        """1 mm samples resting on the surface, and the azimuth each sample faces."""
        p = resample(p, 0.001)
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


def candidate_lines(mesh, thresholds):
    """(name, points m, importance) for every candidate detail line."""
    # Shallower creases come later in importance; they fill in smooth objects.
    found = [line for th in thresholds for line in slice_creases(mesh, th)]
    return [
        (
            f"crease-{'xyz'[axis]}",
            smooth(t[:, :3], 3) / 1000,
            length(t[:, :3]) * np.abs(t[:, 3]).mean(),
        )
        for axis, t in found
    ]


def detail_profile(base, path, nose_clearance=0.0003):
    """The frame's profile, plus pen tilts for detail and the nose clearance."""
    profile = yaml.safe_load(Path(base).read_text())
    profile["drawing_pen_tilt_candidates_deg"] = [0.0, 60.0, 45.0, 30.0, 15.0]
    profile["environment"]["tool_clearance_m"] = {"tapered_nose": nose_clearance}
    # Tilted poses can fold the arm into itself: planning checks the arm too.
    profile["planning_checks_robot"] = True
    # Detail lies on the outside; back away 15 mm rather than 30 mm.
    profile["transit"] = {"withdraw_distance_m": 0.015}
    Path(path).write_text(yaml.safe_dump(profile, sort_keys=False))
    return Path(path)


def cubic_pieces(p, spacing=KNOT_SPACING):
    """C1 cubic pieces through knots every ``spacing`` along a polyline."""
    knots = resample(p, spacing)
    if len(knots) == 2:
        return np.array([np.linspace(knots[0], knots[1], 4)])
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
    return segs


class PlannerCheck:
    """Ask the planner's own samplers whether the arm can draw a line.

    On the line's face, with the pen leaning toward it, a tilt must pass the planner's pen
    screen against the frame (start, middle, end), give a drawing path with
    the whole arm (IK along the line, self and fixed obstacles), keep the
    planner's posture limit, and allow an approach and a withdrawal. When no
    tilt works, ``check`` says where to cut: at the failing point, or a few
    millimetres off the end that cannot be approached or left.
    """

    def __init__(self, frame_source, framework, profile_path, cell_dir, faces, seconds=5.0):
        from pen_art.cell import load_frame_config, prepare_cell
        from pen_art.planning.cell_planner import CellPlanner
        from pen_art.planning.scene import FixedPlate

        settings = dict(load_frame_config()["planning"])
        shutil.rmtree(cell_dir, ignore_errors=True)
        prepare_cell(profile_path, cell_dir, frame_source)
        self.planner = CellPlanner(cell_dir, profile_path, settings, float("inf"))
        self.identity = self.planner.arm.identity
        # The whole arm, not only the pen: planning screens the pen, and the
        # final replay rejects tilted poses where the arm folds into itself.
        self.arm = self.planner.arm
        self.table = FixedPlate.from_profile(self.planner.profile)
        # World azimuth each face of the object looks toward; the pen leans
        # toward the face it draws on.
        self.faces = faces
        self.frame = [np.asarray(e["segments"][0])[[0, -1]] for e in framework["edges"]]
        self.scenes, self.seconds = {}, seconds

    def scene(self):
        from pen_art.curves import transform_points
        from pen_art.planning.scene import MaterialScene

        if "frame" not in self.scenes:
            world = self.table.build_transform()
            scene = MaterialScene()
            for k, seg in enumerate(self.frame):
                a, b = transform_points(seg, world)
                scene.add(a, b, WIRE_R, f"frame/{k}")
            self.scenes["frame"] = scene
        return self.scenes["frame"]

    def check(self, p, index, hints, clear=None):
        """(tilts that work, where to cut: None, "start", "end" or a sample index).

        ``clear(p, index)`` gives the tilts clear of the frame at each sample;
        only tilts clear along the whole line are tried (the planner checks the
        nose at every pose). Where no tilt stays clear, cut there.
        """
        from time import monotonic

        from pen_art.curves import flatten_curve, transform_points
        from pen_art.planning.cell_motion import (
            drawing_alternatives,
            fixed_rotation_families,
            pen_clear,
        )

        planner, arm = self.planner, self.arm
        world = self.table.build_transform()
        lean = float(self.faces[index])
        stroke = {
            "id": "check",
            "start": "s",
            "end": "e",
            "closed": False,
            "junction_visits": [],
            "members": ["m"],
            "segments": cubic_pieces(p).tolist(),
            "length_m": length(p),
        }
        points = transform_points(flatten_curve(stroke, 0.00005, 0.003)[0], world)
        tried = list(dict.fromkeys([*hints, *[float(t) for t in sorted(TILTS, reverse=True)]]))
        if clear is not None:
            sets = clear(p, index)
            common, end = set(sets[0]), 0
            while end + 1 < len(sets) and common & sets[end + 1]:
                end += 1
                common &= sets[end]
            if end + 1 < len(sets):
                return [], max(1, end)  # no single tilt clears the whole line
            tried = [t for t in tried if t in common]
        works, cut = [], None
        for tilt in tried:
            screened = [
                f
                for f in fixed_rotation_families(arm, world, [tilt], lean_azimuth_deg=lean)
                if all(
                    pen_clear(
                        planner,
                        arm,
                        points[i],
                        f["rotations"],
                        self.scene(),
                        planner.margin + 2e-6,
                    )
                    for i in (0, len(points) // 2, -1)
                )
            ]
            if not screened:
                continue
            for drawing in drawing_alternatives(
                arm,
                stroke,
                {"s"},
                world,
                planner.obstacles,
                deadline=monotonic() + self.seconds,
                seeds=[arm.initial_positions],
                settings={"maximum_subdivision_depth": 20},
                clearance_m=planner.margin,
                rotations=screened,
                orientation_policy="fixed",
            ):
                if drawing["status"] != "SAMPLED_DRAWING_PATH":
                    failed = drawing.get("failed_sample")
                    if cut is None and failed is not None:
                        cut = int(round(failed / max(1, len(points) - 1) * (len(p) - 1)))
                    continue
                result = self.in_and_out(drawing)
                if result is True:
                    works.append(tilt)
                    break
                if cut is None:
                    cut = (
                        result
                        if isinstance(result, str)
                        else int(
                            round(result / max(1, len(drawing["positions"]) - 1) * (len(p) - 1))
                        )
                    )
            if len(works) >= 2:
                break
        return works, (None if works else cut)

    def in_and_out(self, drawing, posture_deg_per_mm=0.8):
        """True, or what failed: a sample index (posture), "start" or "end"."""
        from time import monotonic

        from pen_art.planning.transit import find_departure

        planner = self.planner
        arm = planner.collision_arm(self.identity)
        q = np.asarray(drawing["positions"], float)
        tips = np.array([arm.robot.forward(x, arm.flange_to_tcp)[:3, 3] for x in q])
        step = np.linalg.norm(np.diff(tips, axis=0), axis=1)
        swing = np.degrees(np.abs(np.diff(q, axis=0))).max(axis=1)
        over = swing - posture_deg_per_mm * np.maximum(step * 1000, 0.1)
        if np.any(over > 0):
            return int(np.argmax(over))
        if np.degrees(np.abs(q[-1] - q[0])).max() > posture_deg_per_mm * (step.sum() * 1000 + 1):
            return int(np.argmax(swing))
        scene = self.scene()
        after = scene.copy()
        for k, (a, b) in enumerate(zip(tips[:-1], tips[1:], strict=True)):
            after.add(a, b, WIRE_R, f"own/{k}")
        for end, start, material in (("start", q[0], scene), ("end", q[-1], after)):
            axis = (-arm.robot.forward(start, arm.flange_to_tcp)[:3, 2]).tolist()
            lift = find_departure(
                arm.robot,
                arm.flange_to_tcp,
                start,
                {
                    **planner.transit,
                    "departure_directions": [axis, *planner.transit["departure_directions"]],
                },
                planner.lift_check(self.identity, material, start),
                monotonic() + self.seconds,
            )
            if lift["status"] != "SAMPLED_DRAWING_PATH":
                return end
        return True


# --- main ---------------------------------------------------------------------------
def generate_detail(source, target, profile, *, detail=2.0, minutes=15.0,
                    crease_threshold=(0.12, 0.08, 0.05)):
    """Generate detail over a frame; return its tours or an explicit no-detail reason."""
    settings = {"detail": detail, "minutes": minutes, "crease_threshold": crease_threshold}
    try:
        return _generate_detail(Path(source), Path(target), Path(profile), settings)
    except NoDetail as error:
        print(f"detail 0.00 m in 0 strokes: {error}")
        return {"tours": [], "spares": [], "reason": str(error)}


def _generate_detail(source, target, profile, settings):
    """Find detail lines, make drawable tours and write them over the frame."""
    mesh = trimesh.load(asset_path(source, "normalized.glb"), force="mesh")
    surface = Surface(mesh, OFFSET)
    faces = face_azimuths(profile)
    framework = read_json(source / "framework_graph.json")
    frame = np.vstack([dense(np.asarray(e["segments"][0])[[0, -1]]) for e in framework["edges"]])
    frame_tree = cKDTree(frame)
    clear = partial(clear_sets, faces=faces, frame=frame, tree=frame_tree)
    samples = frame_samples(framework)
    funnel = {}

    candidates = find_lines(mesh, settings["crease_threshold"])
    pieces, taken = lay_and_cut(
        candidates, surface, frame, frame_tree, faces, clear, COLLECT * settings["detail"], funnel
    )
    tours = join_tours(pieces, surface, clear, funnel)
    if not tours:
        no_detail("no crease lines long enough that a pen tilt clears the frame")
    ordered, cut_out = order_tours(tours, faces, funnel)
    ready, dropped = choose_tilts(ordered, samples[2])
    ready = planner_filter(
        ready, profile, source, target, framework, faces, clear, samples[2], funnel
    )
    pick_strokes(ready, settings["detail"], settings["minutes"], funnel)
    graph, rows, fused = build_graph(framework, ready, faces, samples)
    rows, screen, minutes = write(source, target, graph, rows, faces, settings)
    report(funnel, taken, pieces, tours, cut_out, dropped, rows, fused, screen, minutes)
    return read_json(target / "detail_tours.json")


# --- faces and the nose screen -------------------------------------------------------
def face_azimuths(profile_path):
    """World azimuth of each of the object's four faces, the one facing the robot first."""
    profile = yaml.safe_load(profile_path.read_text())
    build = np.asarray(profile["world_from_build"])[:2, 3]
    base = np.asarray(profile["arm"]["world_from_base"])[:2, 3]
    robot_az = np.degrees(np.arctan2(*(base - build)[::-1]))
    # Four faces of the object: the one looking at the robot, then each
    # quarter turn round. A line keeps to one face; the pen leans toward it.
    return [(robot_az - 90 * k + 180) % 360 - 180 for k in range(4)]


def sector_of(az, faces):
    """The face nearest each azimuth."""
    return np.argmin(np.abs((az[:, None] - np.array(faces)[None] + 180) % 360 - 180), axis=1)


def pen_axis(azimuth, tilt):
    """Unit pen axis, tip to body, leaning ``tilt`` degrees toward ``azimuth``."""
    h, t = np.radians(azimuth), np.radians(tilt)
    return np.array([np.sin(t) * np.cos(h), np.sin(t) * np.sin(h), np.cos(t)])


def nose_hits(x, a, pts, tree):
    """Wire points inside the nose (+margin) at tip x, axis a; None if it meets the bed."""
    s = np.linspace(NOSE_TIP, NOSE_LENGTH, NOSE_SAMPLES)
    if np.any(x[2] + s * a[2] - nose_radius(s) * np.sqrt(max(0.0, 1 - a[2] ** 2)) <= 0):
        return None
    idx = tree.query_ball_point(x + a * NOSE_BALL, r=NOSE_BALL + NOSE_MAX_R + WIRE_R + MARGIN)
    if not idx:
        return np.zeros(0, int)
    idx = np.asarray(idx)
    d = pts[idx] - x
    s = d @ a
    rho = np.linalg.norm(d - s[:, None] * a, axis=1)
    return idx[(s > NOSE_TIP) & (rho < nose_radius(s) + WIRE_R + MARGIN)]


def clear_sets(p, sec, faces, frame, tree):
    """The tilts whose nose clears frame and bed at each sample, leaning toward face ``sec``."""
    out = []
    for x in p:
        ok = set()
        for t in TILTS:
            h = nose_hits(x, pen_axis(faces[sec], t), frame, tree)
            if h is not None and not len(h):
                ok.add(t)
        out.append(ok)
    return out


# --- stages --------------------------------------------------------------------------
def find_lines(mesh, thresholds):
    """Candidate detail lines, most important first."""
    return sorted(candidate_lines(mesh, thresholds), key=lambda c: -c[2])


def lay_and_cut(candidates, surface, frame, frame_tree, faces, clear, enough, funnel):
    """Pieces of laid line that keep one face and that some tilt clears, and the lines taken.

    Lines are taken by importance, skipping near-duplicates, until ``enough``
    metres are reachable. Each is laid on the surface and cut where it turns to
    another face or where no tilt clears the frame. A piece is (points, face,
    clear tilts per sample, line names).
    """
    pieces, taken, reachable = [], [], 0.0
    funnel.update({"laid": 0.0, "one face": 0.0})
    for name, p, _ in candidates:
        if reachable >= enough:
            break
        if length(p) < MIN_LINE:
            continue
        q = resample(p, STEP)
        if taken and (cKDTree(np.vstack(taken)).query(q)[0] < DUPLICATE).mean() > 0.5:
            continue
        taken.append(q)
        p, az = surface.lay(q)
        funnel["laid"] += length(p)
        p = dip(p, frame, frame_tree, DIP)
        sec = sector_of(az, faces)
        for i in range(1, len(sec) - 1):
            if sec[i - 1] == sec[i + 1] != sec[i]:
                sec[i] = sec[i - 1]
        cuts = np.flatnonzero(np.diff(sec)) + 1
        for a, b in zip(np.r_[0, cuts], np.r_[cuts, len(p)], strict=True):
            if b - a < FACE_SAMPLES:
                continue
            part, s = p[a:b], int(sec[a])
            funnel["one face"] += length(part)
            sets = clear(part, s)
            ok = np.array([bool(x) for x in sets], int)
            for run in np.split(np.arange(len(part)), np.flatnonzero(np.diff(ok)) + 1):
                if ok[run[0]] and len(run) >= MIN_SAMPLES:
                    pieces.append((part[run], s, [sets[i] for i in run], [name]))
                    reachable += length(part[run])
    funnel["tilt clears frame"] = sum(length(r[0]) for r in pieces)
    return pieces, taken


def bridge(surface, a, b):
    """A connecting run on the surface from a to b."""
    p = surface.push_out(
        np.linspace(a, b, max(2, int(np.ceil(np.linalg.norm(b - a) / BRIDGE_STEP)) + 1))
    )
    return surface.push_out(smooth(p, 1))


def join_tours(pieces, surface, clear, funnel):
    """Tours (points, face, clear tilts per sample, line names): pieces of one face joined up.

    Each tour starts at the lowest piece left and takes the nearest piece end
    within BRIDGE whose connecting run a tilt clears; the longest MAX_STROKES
    tours of MIN_TOUR or more are kept.
    """
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
                options = [o for o in options if o[0] <= BRIDGE]
                if not options:
                    break
                _, i, rev = min(options)
                q, qs = pieces[i][0], pieces[i][2]
                if rev:
                    q, qs = q[::-1], qs[::-1]
                b = bridge(surface, end, q[0])[1:-1]
                bs = clear(b, sec) if len(b) else []
                if any(not x for x in bs):
                    tried.add(i)
                    continue
                path += [b, q]
                tilts += bs + list(qs)
                names += pieces[i][3]
                left.remove(i)
            tours.append((np.vstack(path), sec, tilts, names))
    funnel["joined (with connecting runs)"] = sum(length(t[0]) for t in tours)
    tours = sorted((t for t in tours if length(t[0]) >= MIN_TOUR), key=lambda t: -length(t[0]))[
        :MAX_STROKES
    ]
    funnel["8 mm or longer, stroke cap"] = sum(length(t[0]) for t in tours)
    return tours


def allowed(k, items, owner, remaining):
    """Clear tilts per sample: the nose misses every other remaining tour and
    this tour's own wire more than OWN_WIRE samples along it (either drawing direction)."""
    others = remaining - {k}
    return [
        {
            t
            for t, h in hs.items()
            if not any(owner[j] in others or (owner[j] == k and abs(j - g) > OWN_WIRE) for j in h)
        }
        for g, hs in zip(items[k][0], items[k][2], strict=True)
    ]


def cut_conflicts(k, a, items, owner, remaining, next_id):
    """Replace tour ``k`` by its runs where ``a`` has a clear tilt; the next id and samples cut.

    Runs shorter than MIN_SAMPLES are cut too.
    """
    bad = np.array([not x for x in a])
    g, sec, hits = items.pop(k)
    remaining.discard(k)
    owner[g[bad]] = -1
    cut_out = int(bad.sum())
    good = np.flatnonzero(~bad)
    for run in np.split(good, np.flatnonzero(np.diff(good) > 1) + 1):
        if len(run) >= MIN_SAMPLES:
            items[next_id] = (g[run], sec, [hits[i] for i in run])
            owner[g[run]] = next_id
            remaining.add(next_id)
            next_id += 1
        elif len(run):
            owner[g[run]] = -1
            cut_out += len(run)
    return next_id, cut_out


def order_tours(tours, faces, funnel):
    """Tours (points, face, clear tilts per sample, line names) in drawing order, and samples cut.

    No nose meets detail drawn earlier. Peel from the end: the last tour must
    clear all others. When none can, cut the conflicting samples out of the
    tour that conflicts least.
    """
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
                {t: nose_hits(allp[i], pen_axis(faces[sec], t), allp, tree) for t in tilts[j]}
                for j, i in enumerate(g)
            ],
        )
    next_id, remaining, order, allowed_at, cut_out = len(tours), set(items), [], {}, 0
    while remaining:
        pick = next(
            (
                (k, a)
                # Stay on the face of the tour picked last, so the arm moves
                # round the object as little as possible.
                for k in sorted(
                    remaining,
                    key=lambda k: (
                        items[k][1] != (items[order[-1]][1] if order else items[k][1]),
                        -items[k][1],
                        -allp[items[k][0], 2].min(),
                    ),
                )
                for a in [allowed(k, items, owner, remaining)]
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
            ((k, allowed(k, items, owner, remaining)) for k in remaining),
            key=lambda r: sum(1 for x in r[1] if not x),
        )
        next_id, cut = cut_conflicts(k, a, items, owner, remaining, next_id)
        cut_out += cut
    order.reverse()
    ordered = [
        (allp[items[k][0]], items[k][1], allowed_at[k], names_of(tours, start, items[k][0][0]))
        for k in order
    ]
    funnel["ordered"] = sum(length(t[0]) for t in ordered)
    return ordered, cut_out


def tilt_profile(sets):
    """A clear tilt per sample that changes as little as possible (Viterbi)."""
    T = np.array(TILTS, float)
    unary = np.array(
        [[TILT_COST * abs(t - LIKED_TILT) if t in s else np.inf for t in TILTS] for s in sets]
    )
    cost, back = unary[0].copy(), np.zeros((len(sets), len(T)), int)
    for i in range(1, len(sets)):
        total = cost[:, None] + np.abs(T[:, None] - T[None, :])
        back[i] = np.argmin(total, axis=0)
        cost = total[back[i], np.arange(len(T))] + unary[i]
    path = [int(np.argmin(cost))]
    for i in range(len(sets) - 1, 0, -1):
        path.append(back[i][path[-1]])
    return T[path[::-1]]


def trim_overhang(p, tilt, ftree):
    """Start on a contact, overhang at most OVERHANG mm; None if it never touches.

    Of the two directions keep the one that keeps more line, and when they
    keep about the same, the one that ends going up, so the pen backs away
    from wire it has just laid.
    """
    d, _ = ftree.query(p)
    on = np.flatnonzero(d <= TOUCH)
    if not len(on):
        return None
    options = []
    for flip in (False, True):
        q, t, o = (p[::-1], tilt[::-1], len(p) - 1 - on[::-1]) if flip else (p, tilt, on)
        end = min(len(q), o[-1] + OVERHANG + 1)
        q, t = q[o[0] : end], t[o[0] : end]
        rising = len(q) > RISE and q[-1, 2] > q[-RISE - 1, 2]
        options.append((round(length(q) / SAME_LENGTH), rising, q, t))
    _, _, q, t = max(options, key=lambda r: (r[0], r[1]))
    return (q, t) if length(q) >= MIN_SUPPORTED else None


def choose_tilts(ordered, ftree):
    """Ready entries for the ordered tours that touch the frame, and how many never do.

    Each tour gets its smoothest clear tilt per sample, is trimmed to start on
    a contact, and offers the planner its tilt hints.
    """
    ready, dropped = [], 0
    for position, (p, sec, allowed_here, names) in enumerate(ordered):
        tilt = tilt_profile(allowed_here)
        kept = trim_overhang(p, tilt, ftree)
        if kept is None:
            dropped += 1
            continue
        # The planner holds one tilt per stroke: offer the tilts clear along the
        # whole tour first (steepest first), then the tour's most common ones.
        whole = sorted(set.intersection(*allowed_here), reverse=True)
        common = [
            t
            for t, _ in sorted(
                zip(*np.unique(tilt, return_counts=True), strict=True), key=lambda r: -r[1]
            )
        ]
        hints = [float(t) for t in dict.fromkeys([*whole, *common])]
        ready.append(
            {
                "position": position,
                "p": kept[0],
                "tilt": kept[1],
                "hints": hints,
                "face": sec,
                "lines": names,
            }
        )
    return ready, dropped


def cut_parts(p, tilt, cut):
    """What is left of a line (and its tilts) after cutting at ``cut``."""
    if cut == "start":
        return [(p[END_TRIM:], tilt[END_TRIM:])]
    if cut == "end":
        return [(p[:-END_TRIM], tilt[:-END_TRIM])]
    before, after = max(0, cut - CUT_BEFORE), cut + CUT_AFTER
    return [(p[:before], tilt[:before]), (p[after:], tilt[after:])]


def planner_filter(ready, profile, source, target, framework, faces, clear, ftree, funnel):
    """The ready entries the arm can draw, each with the tilts that work.

    A failing tour is cut at the failing point, or trimmed at the end the arm
    cannot reach, and the pieces are checked again, at most MAX_CUTS deep.
    """
    # Scratch for the check, named for the object (its profile sits beside
    # the cell profile, whose robot paths are relative to it); both go
    # when the check is done.
    scratch = f"{target.parent.name}_{target.name}_check"
    check_profile = detail_profile(profile, profile.parent / f"{scratch}.yaml")
    check_cell = target.parent / ".work" / scratch
    checker = PlannerCheck(source, framework, check_profile, check_cell, faces)
    queue, ready, checks = list(ready), [], 0
    while queue:
        entry = queue.pop(0)
        p, tilt = entry["p"], entry["tilt"]
        works, cut = checker.check(p, entry["face"], entry["hints"], clear)
        checks += 1
        if works:
            ready.append({**entry, "hints": works})
            continue
        if cut is None or entry.get("depth", 0) >= MAX_CUTS:
            continue
        for q, t in cut_parts(p, tilt, cut):
            kept = trim_overhang(q, t, ftree) if len(q) >= MIN_SAMPLES else None
            if kept is not None and length(kept[0]) >= MIN_CHECKED:
                queue.append(
                    {**entry, "p": kept[0], "tilt": kept[1], "depth": entry.get("depth", 0) + 1}
                )
    shutil.rmtree(check_cell, ignore_errors=True)
    check_profile.unlink(missing_ok=True)
    funnel["arm can draw it"] = sum(length(e["p"]) for e in ready)
    print(f"planner check: {checks} lines checked, {len(ready)} drawable")
    return ready


def pick_strokes(ready, detail_m, minutes, funnel):
    """Mark each ready entry ``spare`` unless it is among the longest within budget.

    Spares are drawn by plan-geometry instead of any the arm cannot draw after all.
    """
    lines = [(i, length(e["p"])) for i, e in enumerate(ready)]
    picked = set(choose_detail(lines, detail_m, minutes))
    for i, entry in enumerate(ready):
        entry["spare"] = i not in picked
    funnel["within detail and time"] = sum(length(e["p"]) for e in ready if not e["spare"])


# --- the stroke graph ----------------------------------------------------------------
def frame_samples(framework):
    """Points every STEP along the frame edges: edge id and fraction of each, and their tree."""
    keys, fracs, pts = [], [], []
    for key, e in {e["id"]: e for e in framework["edges"]}.items():
        a, b = np.asarray(e["segments"][0][0]), np.asarray(e["segments"][0][-1])
        for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / STEP))):
            keys.append(key)
            fracs.append(t)
            pts.append(a + t * (b - a))
    return keys, fracs, cKDTree(np.asarray(pts))


def new_node(nodes, point, prefix):
    """Add a node at ``point`` to ``nodes``; its id."""
    nid = f"{prefix}{len(nodes):05d}"
    nodes[nid] = {
        "id": nid,
        "position": np.asarray(point).tolist(),
        "ground": bool(point[2] <= 1e-7),
    }
    return nid


def frame_node(nodes, edges, cuts, key, t):
    """The node splitting frame edge ``key`` at fraction ``t`` (or a cut within CUT_SNAP)."""
    near = [u for u in cuts[key] if abs(u - t) < CUT_SNAP]
    t = near[0] if near else t
    if t not in cuts[key]:
        e = edges[key]
        ea, eb = np.asarray(e["segments"][0][0]), np.asarray(e["segments"][0][-1])
        cuts[key][t] = new_node(nodes, ea + t * (eb - ea), "cut-n")
    return cuts[key][t]


def add_tour(entry, rank, faces, samples, nodes, edges, cuts, fused):
    """The graph edges and the detail row of one tour; adds its nodes, frame cuts and contacts."""
    keys, fracs, ftree = samples
    p, tilt, hints, sec = entry["p"], entry["tilt"], entry["hints"], entry["face"]
    # The planner starts a stroke on a built node, so the tour begins on
    # the frame wire itself, at a node it shares with the frame.
    _, first = ftree.query(p[0])
    start = frame_node(nodes, edges, cuts, keys[first], fracs[first])
    p = np.vstack([nodes[start]["position"], p[1:]])
    knots, segs = resample(p, KNOT_SPACING), cubic_pieces(p, KNOT_SPACING)
    d, idx = ftree.query(knots)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(knots, axis=0), axis=1))]
    contacts, last = [], -1.0
    for i in range(len(knots)):
        w = d[max(0, i - CONTACT_WINDOW) : i + CONTACT_WINDOW + 1]
        if d[i] <= TOUCH and d[i] == w.min() and s[i] - last >= CONTACT_SPACING:
            contacts.append(i)
            last = s[i]
    contacts = [i for i in contacts if s[i] >= FIRST_CONTACT]
    bounds = sorted(set([0, len(knots) - 1] + contacts))
    tid = f"tour-{rank:03d}"
    knot_node = {i: new_node(nodes, knots[i], "tour-n") for i in bounds}
    knot_node[0] = start
    for i in contacts:
        fused.append([knot_node[i], frame_node(nodes, edges, cuts, keys[idx[i]], fracs[idx[i]])])
    tour_edges = [
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
            "order_hint": DETAIL_ORDER + rank,
            "face": int(sec),
            "pen_lean_azimuth_deg": float(faces[sec]),
            "pen_tilt_candidates_deg": hints,
        }
        for j, (a, b) in enumerate(zip(bounds[:-1], bounds[1:], strict=True))
    ]
    row = {
        "id": tid,
        "length_m": length(p),
        "face": int(sec),
        "contacts": len(contacts),
        "tilt_deg": tilt.tolist(),
        "lines": entry["lines"],
        "spare": entry["spare"],
    }
    return tour_edges, row


def split_frame(edges, cuts):
    """The frame edges split at their cuts."""
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
    return split


def build_graph(framework, ready, faces, samples):
    """The stroke graph: tours on the frame, fused where they touch; a row per tour; contacts."""
    graph = deepcopy(framework)
    nodes = {n["id"]: n for n in graph["nodes"]}
    edges = {e["id"]: e for e in graph["edges"]}
    cuts = {key: {0.0: e["start"], 1.0: e["end"]} for key, e in edges.items()}
    fused, art_edges, rows = [], [], []
    for rank, entry in enumerate(sorted(ready, key=lambda e: e["position"])):
        tour_edges, row = add_tour(entry, rank, faces, samples, nodes, edges, cuts, fused)
        art_edges += tour_edges
        rows.append(row)
    split = split_frame(edges, cuts)
    for e in split + art_edges:
        e["length_m"] = curve_length(e)
    graph.update(
        nodes=list(nodes.values()),
        edges=split + art_edges,
        fused_turns=graph.get("fused_turns", []) + fused,
        method="wireprint_with_detail_tours",
    )
    return graph, rows, len(fused)


# --- output --------------------------------------------------------------------------
def write(source, target, graph, rows, faces, settings):
    """Write the target run and ``detail_tours.json``; the drawn rows, screen and minutes."""
    spares = [r for r in rows if r.pop("spare")]
    rows = [r for r in rows if r not in spares]
    if not rows:
        no_detail("the arm can draw none of the detail lines")
    screen = write_detail_source(source, target, graph)
    minutes = estimated_minutes(sum(r["length_m"] for r in rows), len(rows))
    write_json(
        target / "detail_tours.json",
        {
            "tours": rows,
            # In the stroke graph too; plan-geometry leaves them out until needed.
            "spares": spares,
            "face_azimuth_deg": faces,
            "estimated_added_minutes": minutes,
            "settings": settings,
        },
    )
    return rows, screen, minutes


def report(funnel, taken, pieces, tours, cut_out, dropped, rows, fused, screen, minutes):
    """Print the metres kept by each stage and a summary of the detail."""
    detail = sum(r["length_m"] for r in rows)
    tilts = np.concatenate([r["tilt_deg"] for r in rows])
    median = 1e3 * np.median([r["length_m"] for r in rows])
    counts = dict(zip(*[x.tolist() for x in np.unique(tilts, return_counts=True)], strict=True))
    steady = sum(1 for r in rows if len(set(r["tilt_deg"])) == 1)
    sag = 1e3 * screen.get("maximum_displacement_m", np.nan)
    used = sum(length(q) for q in taken)
    funnel["touches frame, trimmed"] = detail
    print("length by stage (m): " + ", ".join(f"{k} {v:.2f}" for k, v in funnel.items()))
    print(f"lines {len(taken)} ({used:.2f} m) -> {len(pieces)} pieces -> {len(tours)} tours")
    print(f"ordering cut {cut_out / 1000:.2f} m; {dropped} tours never touch the frame")
    print(f"detail {detail:.2f} m in {len(rows)} strokes (median {median:.0f} mm)")
    print(f"{fused} fused contacts, about {minutes:.1f} min more")
    print(f"tilt samples {counts}; {steady}/{len(rows)} strokes keep one tilt")
    print(f"structure pass {screen['modeled_pass']} sag {sag:.3f} mm {screen.get('failure', '')}")


def write_detail_source(source, target, graph):
    """``target``: shared source assets with the selected stroke graph
    ``graph``, its strokes and its structural screen, which is returned."""
    if target.exists():
        shutil.rmtree(target)
    target.mkdir(parents=True)
    reference_assets(source, target)
    construction = tilt_hints(section_actions(graph), graph)
    material = read_json(asset_path(source, "fabrication_config.json"))["structure"]
    return write_strokes(target, graph, construction, material)


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
