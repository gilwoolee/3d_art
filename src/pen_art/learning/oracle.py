"""Offline exact configuration collision signatures for fixed cell geometry."""

from dataclasses import replace
from functools import lru_cache
from pathlib import Path

import numpy as np

from ..cell import load_cell
from ..planning.motion_library import (
    CachedCollisionModel,
    MaterialOnlyCollisionModel,
    collision_context_hash,
    digest,
)
from ..planning.motion_runtime import static_shapes
from ..planning.scene import MaterialScene
from ..planning.turntable import IndexedTurntable


def material_scene(scene):
    material = MaterialScene()
    for i, capsule in enumerate(scene.get("capsules", [])):
        if isinstance(capsule, dict):
            material.add(capsule["start"], capsule["end"], capsule["radius_m"], str(i))
        else:
            material.add(capsule[:3], capsule[3:6], capsule[6], str(i))
    return material


@lru_cache(maxsize=4)
def load_profile(profile):
    config, arms, obstacles = load_cell(profile)
    return config, next(iter(arms.values())), obstacles


class CellSignatureOracle:
    def __init__(self, profile, *, verification_directory=None, verification_raw=False):
        self.config, loaded, self.obstacles = load_profile(str(profile))
        model = loaded.collision
        while isinstance(model, (CachedCollisionModel, MaterialOnlyCollisionModel)):
            model = model.model
        # Profile loader entries are shared; per-oracle acceleration must not
        # mutate another oracle's collision model or raw baseline.
        self.arm = replace(loaded, collision=model)
        self.verification_raw = verification_raw
        self.context_hash = digest(
            {
                "model": collision_context_hash(self.arm.collision, self.arm.tool.metadata),
                "profile": self.config,
            }
        )
        if verification_directory is not None and not verification_raw:
            table = (
                IndexedTurntable.from_profile(self.config) if "turntable" in self.config else None
            )
            self.arm.collision = CachedCollisionModel(
                model,
                Path(verification_directory),
                geometry_context=self.arm.tool.metadata,
                static_geometry=static_shapes(self.config, table),
            )

    def __call__(self, episode, probe_q):
        context = episode.get("validation", {}).get("context_hash")
        if context is not None and context != self.context_hash:
            raise ValueError("Episode model/static context differs from the loaded profile")
        material = material_scene(episode["scene"])
        material.clearance_query_enabled = not self.verification_raw
        margin = self.config["environment"]["clearance_m"]
        try:
            return np.asarray(
                [
                    bool(self.arm.collision.check(q, self.obstacles, margin, material))
                    for q in probe_q
                ],
                dtype=np.float32,
            )
        finally:
            if isinstance(self.arm.collision, CachedCollisionModel) and self.arm.collision._pending:
                self.arm.collision.flush()
