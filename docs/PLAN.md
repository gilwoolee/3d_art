# Single-arm wire sculpture — active plan

Updated: 2026-10-11. Target: one Flexiv + one 3D pen, objects within 15 cm per side.

## Goal and approach

Produce a complete build order and executable simulation from an empty base.
Follow [Caelan’s extrusion formulation](https://arxiv.org/html/2002.02360): search
backward over material states and robot configurations, checking structural
stability, extrusion access, and pen-off travel. Stop at the first feasible plan.
Pen stops, repositioning, and rejoining are normal actions. Keep the original
shape; introduce explicit supports only when needed. No timing target or two-arm work.

## Work next

1. **Small fixtures — ready:** cube, triangular prism, torus, and bottle STLs with
   reference wireframes. Test order planning separately from automatic abstraction.
2. **Reliable graph:** preserve curve intervals and real junctions; split into
   joinable extrusion sections. Check the final structure before searching order.
   Resolve numerical failures and record material assumptions; they are not proof
   of physical impossibility. Support changes create a new graph revision.
3. **Build-order search:** combine backward search with forward stiffness guidance.
   Evaluate robot motions lazily, allow alternate starts/directions and pen-off
   transitions, and backtrack when a choice blocks later work.
4. **GPU integration:** planning now runs on Blondie; add cuRobo batched IK and
   pen-off motion candidates to order search. Keep stiffness/search logic and
   precise path validation separate. Benchmark on the small fixtures first.
5. **Complete and replay:** demonstrate cube/prism, then torus/bottle, then Palace.
   Add extrusion timing and cooling; replay every section, transition, and return
   home. Show the entire build in the viewer at 10×. Hardware validation follows
   measured pen calibration.

## Current state

- Implemented: mesh import, curve/graph stages, structural screening, robot/tool
  models, collision checks, section restarts, timing, and partial-build viewer.
- Missing: integrated Caelan-style order search and a verified complete build.
- Existing Palace full-graph structural solve is numerically unresolved; existing
  Torus graph fails the provisional structural model. Investigate on clean fixtures.
- Blondie driver and CUDA work; Linux planning, fresh Flexiv assets, and the viewer
  are operational. All four fixtures pass the provisional final-frame screen.
  cuRobo passes a GPU smoke check; Flexiv and order-search integration remain open.

## Acceptance

Every source section is covered once; each start/rejoin has real support; selected
release states pass the stated structural model; the robot can extrude and travel
without collision and return home. Retain current 1 mm clearance and 0.3 mm tracking
limits. Unknown material/process behavior stays explicitly unverified.
Use a small set of meaningful regression cases, not an expanding test suite.

## Reference

[Detailed review and integration notes](archive/2026_1011_caelan_build_order_plan.md)
· [Historical implementation log](archive/2026_1010_mesh_to_robot_pen_plan.md)

Existing reusable code: `structure.py`, `planning/sequence.py`, `cell_planner.py`,
`stroke_restarts.py`, `full_build_export.py`, and `build_order_preflight.py`.
Use `scripts/generate_primitive_fixtures.py` and `scripts/prepare_single_arm_cell.py`
on Blondie; run instructions are in [README](../README.md).
