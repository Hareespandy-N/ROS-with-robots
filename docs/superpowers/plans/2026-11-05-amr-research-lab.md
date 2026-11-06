# AMR Research Lab Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans inline, as approved
> by the user. A fresh whole-project reviewer checks the result before upload.

**Goal:** Publish an original, runnable AMR simulation/research repository.
**Architecture:** A standard-library Python core provides grid planning,
tracking, deterministic simulation, docking, and SVG/CSV/JSON reporting.
A thin ROS 2 Humble adapter exposes simulation-only topics.
**Tech Stack:** Python 3.10+, unittest, setuptools, ROS 2 Humble, GitHub Actions.
**Spec:** ../specs/2026-11-05-amr-research-lab-design.md

## Global Constraints
- Original code, maps, prose, and SVGs; no third-party code/media redistribution.
- No ROS dependency in the offline core; no PC software installation.
- No live hardware access; topics under `/amr_lab/sim/` only.
- Simulation metrics are not physical accuracy evidence.
- Preserve existing repository history; publish only this project's files.

## Review Focus
- Non-finite coordinates and zero/negative limits fail explicitly.
- Occupied/outside endpoints and corner-cutting never yield an unsafe path.
- Collision prediction and braking limits are shared in both comparisons.
- Goal heading wrapping, blocked-path waiting, and timeout terminate correctly.
- ROS packaging and topic/frame wiring are checked without fake ROS stubs.

## Task 1: Planner and simulated motion
**Files:** `amr_research_lab/navigation.py`, `tests/test_navigation.py`.
**Interfaces:** `Pose(x,y,yaw)`, `World(width,height,obstacles,resolution)`,
`plan(world,start,goal,radius) -> list[tuple[float,float]]`,
`integrate(pose,v,w,dt) -> Pose`.
- [ ] Write tests for obstacle detours, unreachable/occupied goals, corner
  cutting, input validation, and exact differential-drive integration.
- [ ] Run `python -m unittest discover -s tests -v`; observe missing feature.
- [ ] Implement the core and run the same checks to green.

## Task 2: Controller, experiment runner, and docking
**Files:** `amr_research_lab/simulation.py`, `amr_research_lab/docking.py`,
`amr_research_lab/reporting.py`, `amr_research_lab/__main__.py`,
`tests/test_simulation.py`, `tests/test_docking.py`, `tests/test_reporting.py`.
**Consumes:** Task 1 geometry/planning interfaces.
**Produces:** `Simulation(scenario,controller).step()`, `.run()`, `.metrics()`,
`DockingController.update(pose,dt,obstructed=False) -> tuple[float,float]`,
`write_report(simulation,output_dir)`.
- [ ] Write failing behavior tests for shared limits, goal+yaw convergence,
  obstacle wait/resume, collision/timeout failures, docking failure/settling,
  and valid reproducible CSV/JSON/SVG outputs.
- [ ] Implement only these behaviors; run the full suite to green.
- [ ] Run both controllers on all three scenarios; retain honest results.

## Task 3: ROS adapter and publication
**Files:** `amr_research_lab/ros_node.py`, `launch/simulation.launch.py`,
`config/simulation.rviz`, `package.xml`, `setup.py`, `setup.cfg`,
`resource/amr_research_lab`, `tests/test_ros_smoke.py`, `.github/workflows/ci.yml`,
`README.md`, `docs/RESEARCH.md`, `docs/PROVENANCE.md`, `docs/QUICKSTART_TA.md`,
`docs/architecture.svg`, `LICENSE`, `.gitignore`.
**Consumes:** Task 2 simulation step and metrics interfaces.
- [ ] Define ROS smoke-test expectations before adapter implementation.
- [ ] Add simulated odom/path/obstacle/velocity topics with private frames.
- [ ] Add Python and ROS container CI; syntax/package-check locally and
  explicitly record local ROS smoke test as skipped when ROS is absent.
- [ ] Write original docs and diagrams, cite all five papers, and explain
  which ideas are implemented versus background-only references.
- [ ] Review full project, fix significant issues with regression tests,
  run suite+benchmarks+publication scan, upload, and verify public visibility.

## Verification commands
`python -m unittest discover -s tests -v`
`python -m compileall -q amr_research_lab tests launch`
`python -m amr_research_lab benchmark --output examples/results`
`python setup.py check --strict`
ROS CI: `colcon build`, source install, then run the ROS topic smoke test.

## Execution record
Task progress is recorded in the native checklist and the local ledger.
Remote publication uses one coherent commit after all local verification.
