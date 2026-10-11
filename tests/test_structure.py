import numpy as np
import pytest

from pen_art.graph import straight_edge
from pen_art.structure import analyze_structure


@pytest.fixture
def material():
    return {
        "calibrated": False,
        "young_modulus_pa": 1e9,
        "poisson_ratio": 0.35,
        "density_kg_m3": 1240.0,
        "element_length_m": 0.005,
        "maximum_linear_deflection_ratio": 0.1,
        "maximum_displacement_m": 0.005,
        "yield_stress_pa": 15e6,
    }


def cantilever():
    nodes = {
        "a": {"id": "a", "position": [0.0, 0, 0], "ground": True},
        "b": {"id": "b", "position": [0.04, 0, 0], "ground": False},
    }
    edge = straight_edge("a", "b", nodes, "beam", "artistic", 0.001)
    return {"nodes": list(nodes.values()), "edges": [edge]}


def test_cantilever_uniform_gravity_matches_closed_form(material):
    graph = cantilever()
    result = analyze_structure(graph, material)
    inertia, area = np.pi * 0.001**4 / 64, np.pi * 0.001**2 / 4
    load = material["density_kg_m3"] * area * 9.81
    expected = load * 0.04**4 / (8 * material["young_modulus_pa"] * inertia)
    tip = next(n for n in result["nodes"] if n["id"] == "b")
    assert tip["displacement_m"][2] == pytest.approx(-expected, rel=1e-6)
    assert result["relative_residual"] < 1e-8
    assert result["physical_status"] == "unverified"
    assert len(result["nodes"]) == 2  # numerical nodes never become physical anchors


def test_cantilever_tip_force_and_torsion(material):
    material["density_kg_m3"] = 0
    result = analyze_structure(
        cantilever(), material, nodal_loads={"b": [0.0, 0.001, 0, 0.000001, 0, 0]}
    )
    inertia = np.pi * 0.001**4 / 64
    expected = 0.001 * 0.04**3 / (3 * material["young_modulus_pa"] * inertia)
    tip = next(n for n in result["nodes"] if n["id"] == "b")
    assert tip["displacement_m"][1] == pytest.approx(expected, rel=1e-6)
    assert result["maximum_stress_pa"] > 0


def test_floating_component_is_rejected(material):
    graph = cantilever()
    graph["nodes"][0]["ground"] = False
    result = analyze_structure(graph, material)
    assert result["failure"] == "UNSUPPORTED_COMPONENT"
    assert not result["modeled_pass"]
