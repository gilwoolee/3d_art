"""Generate deterministic reference fixtures; these do not test automatic abstraction.

Run from the repository: .venv/bin/python scripts/generate_primitive_fixtures.py
STL coordinates are millimeters (STL itself has no unit metadata). Graphs use meters.
"""

import argparse
import tempfile
from pathlib import Path

import networkx as nx
import numpy as np
import trimesh
import yaml

from pen_art.assets import prepare_asset
from pen_art.curves import curve_length, sample_curve
from pen_art.io import artifact, sha256, write_json
from pen_art.planning.build_order_preflight import preflight
from pen_art.strokes import pen_strokes


class Reference:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def node(self, point):
        point = np.asarray(point, dtype=float)
        point[np.abs(point) < 1e-14] = 0
        for node in self.nodes:
            if np.linalg.norm(point - node["position"]) < 1e-12:
                return node["id"]
        identity = f"n{len(self.nodes):03}"
        self.nodes.append(dict(id=identity, position=point.tolist(), ground=bool(point[2] == 0)))
        return identity

    def curve(self, identity, pieces):
        for index, control in enumerate(pieces):
            edge = dict(
                id=f"{identity}-{index}",
                start=self.node(control[0]),
                end=self.node(control[-1]),
                role="artistic",
                origin="reference_fixture",
                required=True,
                diameter_m=0.001,
                parent_curve=identity,
                source_interval=[index, index + 1],
                segments=[np.asarray(control).tolist()],
                process_status="not_checked",
            )
            edge["length_m"] = curve_length(edge)
            self.edges.append(edge)

    def line(self, identity, points):
        self.curve(
            identity,
            [
                [a, a + (b - a) / 3, a + 2 * (b - a) / 3, b]
                for a, b in zip(np.asarray(points)[:-1], np.asarray(points)[1:], strict=True)
            ],
        )

    def circle(self, identity, center, x, y):
        center, x, y = map(np.asarray, (center, x, y))
        pieces = []
        k = 4 / 3 * np.tan(np.pi / 8)
        for i in range(4):
            a, b = i * np.pi / 2, (i + 1) * np.pi / 2
            p = center + np.cos(a) * x + np.sin(a) * y
            q = center + np.cos(b) * x + np.sin(b) * y
            pieces.append(
                [
                    p,
                    p + k * (-np.sin(a) * x + np.cos(a) * y),
                    q - k * (-np.sin(b) * x + np.cos(b) * y),
                    q,
                ]
            )
        self.curve(identity, pieces)

    def graph(self):
        return artifact(
            "stroke_graph",
            frame="build",
            units="m",
            nodes=self.nodes,
            edges=self.edges,
            method="explicit_reference_fixture",
            limitations=["Hand-specified reference topology; not automatic mesh abstraction"],
        )


def polyhedron(sides):
    ref = Reference()
    if sides == 4:
        xy = np.array([[-0.04, -0.04], [0.04, -0.04], [0.04, 0.04], [-0.04, 0.04]])
    else:
        xy = np.array([[-0.04, -0.03], [0.04, -0.03], [0, 0.03]])
    vertices = np.array([[x, y, z] for z in [0, 0.08] for x, y in xy])
    for i in range(sides):
        j = (i + 1) % sides
        for level in range(2):
            ref.line(f"rim{level}-{i}", vertices[[level * sides + i, level * sides + j]])
        ref.line(f"upright{i}", vertices[[i, sides + i]])
    return trimesh.convex.convex_hull(vertices), ref


def bottle():
    ref = Reference()
    levels = [(0, 0.025), (0.075, 0.025), (0.10, 0.012), (0.12, 0.012)]
    for i, (z, r) in enumerate(levels):
        ref.circle(f"ring{i}", [0, 0, z], [r, 0, 0], [0, r, 0])
    for i in range(4):
        a = i * np.pi / 2
        ref.line(f"rib{i}", [[r * np.cos(a), r * np.sin(a), z] for z, r in levels])
    profile = [[0, 0]] + [[r, z] for z, r in levels] + [[0, 0.12]]
    return trimesh.creation.revolve(profile, sections=64), ref


def torus():
    ref = Reference()
    major, minor = 0.045, 0.015
    for i in range(4):
        a = i * np.pi / 2
        radius = major + minor * np.cos(a)
        ref.circle(f"ring{i}", [0, 0, minor + minor * np.sin(a)], [radius, 0, 0], [0, radius, 0])
        radial = np.array([np.cos(a), np.sin(a), 0])
        ref.circle(f"meridian{i}", major * radial + [0, 0, minor], minor * radial, [0, 0, minor])
    mesh = trimesh.creation.torus(major, minor, major_sections=64, minor_sections=32)
    mesh.apply_translation([0, 0, minor])
    return mesh, ref


def generate(destination):
    config = yaml.safe_load(Path("configs/fabrication.yaml").read_text())
    rows, panels = [], []
    for name, factory, counts in [
        ("cube", lambda: polyhedron(4), (8, 12, 12)),
        ("triangular_prism", lambda: polyhedron(3), (6, 9, 9)),
        ("bottle", bottle, (16, 28, 8)),
        ("torus", torus, (16, 32, 8)),
    ]:
        run = destination / name
        run.mkdir(parents=True, exist_ok=True)
        mesh, reference = factory()
        assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0
        assert mesh.extents.max() <= 0.15 and abs(mesh.bounds[0, 2]) < 1e-12
        mm_mesh = mesh.copy()
        mm_mesh.apply_scale(1000)
        source = run / "model_mm.stl"
        mm_mesh.export(source)
        loaded = trimesh.load_mesh(source, process=True)
        assert loaded.is_watertight and loaded.is_winding_consistent
        assert np.allclose(loaded.bounds / 1000, mesh.bounds, atol=1e-8)
        graph = reference.graph()
        strokes = pen_strokes(graph)
        assert (len(graph["nodes"]), len(graph["edges"]), len(strokes["curves"])) == counts
        topology = nx.Graph([(e["start"], e["end"]) for e in graph["edges"]])
        assert nx.is_connected(topology)
        if name in {"torus", "bottle"}:
            assert all(topology.degree(n["id"]) >= 3 for n in graph["nodes"])
        positions = {n["id"]: n["position"] for n in graph["nodes"]}
        for edge in graph["edges"]:
            assert np.allclose(edge["segments"][0][0], positions[edge["start"]], atol=1e-12)
            assert np.allclose(edge["segments"][-1][-1], positions[edge["end"]], atol=1e-12)
        write_json(run / "stroke_graph.json", graph)
        write_json(run / "pen_strokes.json", strokes)
        write_json(
            run / "reference_curves.json",
            artifact(
                "curves",
                frame="build",
                units="m",
                method="explicit_reference_fixture",
                curves=strokes["curves"],
                limitations=graph["limitations"],
            ),
        )
        write_json(run / "fabrication_config.json", config)
        report = preflight(run)
        write_json(run / "build_order/preflight.json", report)
        # Exercise the actual pipeline importer, including STL unit normalization.
        with tempfile.TemporaryDirectory(prefix="primitive-import-") as tmp:
            imported = prepare_asset(
                source,
                Path(tmp),
                dict(source_up="z", max_extent_m=float(mesh.extents.max()), geometry_faces=10000),
            )
            assert imported["diagnostics"]["watertight"]
            assert np.allclose(imported["bounds_m"], mesh.bounds, atol=1e-8)
        row = dict(
            name=name,
            source="model_mm.stl",
            source_units="mm",
            source_up="z",
            max_extent_m=float(mesh.extents.max()),
            bounds_m=mesh.bounds.tolist(),
            stl_sha256=sha256(source),
            graph_sha256=sha256(run / "stroke_graph.json"),
            vertices=counts[0],
            material_members=counts[1],
            reference_curves=counts[2],
            total_length_m=strokes["total_length_m"],
            preflight_status=report["status"],
            max_displacement_m=report["final_structure"].get("maximum_displacement_m"),
            mesh_watertight=True,
            importer_roundtrip_verified=True,
            automatic_abstraction=False,
            robot_feasible=None,
            pen_feasible=None,
        )
        write_json(run / "manifest.json", row)
        rows.append(row)
        paths = []
        for curve in strokes["curves"]:
            points = sample_curve(curve)
            projected = np.column_stack(
                (
                    points[:, 0] - 0.55 * points[:, 1],
                    -0.3 * points[:, 0] - 0.3 * points[:, 1] - points[:, 2],
                )
            )
            paths.append(" ".join(f"{150 + 900 * x:.2f},{175 + 900 * y:.2f}" for x, y in projected))
        panels.append(
            f'<g transform="translate({len(panels) * 300},0)"><text x="15" y="25">'
            f"{name}</text>" + "".join(f'<polyline points="{p}"/>' for p in paths) + "</g>"
        )
        print(f"{name}: {counts[1]} members / {counts[2]} curves; {report['status']}", flush=True)
    write_json(
        destination / "manifest.json",
        dict(
            fixtures=rows,
            source_units="mm",
            graph_units="m",
            generator="scripts/generate_primitive_fixtures.py",
            assumptions=[
                "Explicit reference curves, not automatic mesh abstraction.",
                "STL carries no units: coordinates are millimeters; source_up=z.",
                "Circular curves use four cubic quarter-circle approximations.",
                "Ground nodes are clamped to substrate; rigid welded joints assumed.",
                "Uncalibrated 1 mm strand, cooled linear elastic frame under self-weight.",
                "Final-frame preflight does not validate construction order or robot motion.",
                "No motion planning, pen calibration or physical fabrication performed.",
            ],
        ),
    )
    (destination / "preview.svg").write_text(
        '<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="240" viewBox="0 0 1200 240">'
        '<rect width="1200" height="240" fill="white"/><style>polyline{fill:none;stroke:#245;'
        "stroke-width:1.5}text{font:16px sans-serif;fill:#123}</style>"
        + "".join(panels)
        + '<text x="15" y="227">Reference wireframe fixtures only — '
        'no automatic abstraction or robot feasibility claim.</text></svg>'
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("research/primitive_fixtures"))
    generate(parser.parse_args().output)
