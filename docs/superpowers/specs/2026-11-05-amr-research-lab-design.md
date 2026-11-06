# AMR Research Lab design

## Intent
Create an original, reproducible educational AMR simulation and ROS 2 research
project in `Hareespandy-N/ROS-with-robots`. The user selected simulation plus
research and authorized uploading the result and making this repository public.

## Approach and choices
The selected approach is a Python simulation with a small ROS 2 Humble adapter.
A ROS-only starter would be less useful on this Windows PC, which has Python
3.12 but no working WSL/ROS installation. A production robot integration would
need hardware-specific requirements and is outside this request. These are
implementation choices by the author, not claims about an existing robot.

Use Python 3.10+ standard library for the core; ROS 2 Humble / Ubuntu 22.04 for
the optional adapter. No new PC software installation is required.

## Deliverable
- Footprint-aware A* grid planning with no diagonal corner cutting.
- Independently written fixed-lookahead pure pursuit and an educational
  clearance/curvature-regulated variant inspired by the cited RPP paper.
- Bounded differential-drive integration, speed/acceleration limits, sampled
  trajectory collision checking, goal heading control, and failure states.
- Deterministic open-floor, warehouse, and temporary-blockage experiments.
- A separate simulated docking state machine: ALIGN, APPROACH, SETTLE, DOCKED,
  and FAILED. It never controls a charger, lift, motor driver, or PLC.
- CSV traces, JSON metrics, original SVG paths/plots, and a reproducible CLI.
- A ROS package with a simulation node, launch file, RViz configuration, and
  topics under `/amr_lab/sim/`. No global `/cmd_vel` publishing or hardware
  driver integration. Frames are `amr_lab/map` and `amr_lab/base_link`.
- English README and Tamil quick-start notes, architecture diagram, source
  bibliography, paper-to-component mapping, license/provenance notes, and CI.

## Research and publication boundaries
Use the five supplied papers as attributed conceptual references, not as
redistributed documents. Upstream repositories are linked, not vendored.
Write new code, prose, maps, plots, and diagrams. Keep third-party figures,
screenshots, PDFs, and copied implementation code out of this publication.
License the newly authored project with MIT; do not relicense external works.
This is an educational implementation, not a claimed new scientific method,
an exact reproduction of Nav2/RPP, or an endorsed ROS/Nav2 distribution.

## Metrics and correctness
Author defaults: circular footprint radius 0.22 m, map grid 0.10 m,
integration timestep 0.05 s, speed limits 0.60 m/s and 1.5 rad/s,
acceleration limits 0.50 m/s^2 and 2.0 rad/s^2, XY tolerance 0.04 m,
yaw tolerance 0.04 rad, timeout 120 s. Both controllers share collision
checking, acceleration limits, and goal settling. Temporary blockage is a
scripted rectangle across the open corridor from simulation time 3 to 7 s.
Brake within the stated limits; if the swept footprint collides, report
collision failure. Do not turn an emergency instantaneous stop into a
falsely claimed acceleration-limited success.

Measure completion, simulated duration, path length, minimum obstacle
clearance, blocked time, speed/acceleration maxima, and final XY/yaw errors.
Simulation ground truth is identified explicitly. A simulator's final error
is not independent physical metrology; real-robot +/-1 mm accuracy is
"not proven yet". Report real observed outcomes, including unsuccessful runs.
Compare controllers using the same map, start, goal, footprint, and limits.

Validate blocked start/goal, unreachable paths, diagonal corner cutting,
non-finite inputs, invalid limits, goal heading wrap, swept collision checks,
rate limits, timeout, dynamic obstacle stop/resume, and docking transitions.
Run Python unit/integration tests, syntax and package checks, deterministic
benchmarks, output-schema and SVG checks, and publication secret/provenance
review. Test the ROS adapter in a real ROS environment if available; otherwise
label it unexecuted locally and include a Linux ROS CI smoke test.

## Structure
`amr_research_lab/` contains geometry/planning, controller/simulation, docking,
reporting/CLI, and the optional ROS node. `tests/` covers behavior, not copied
implementation details. `docs/` holds the original diagrams and research
notes. `examples/results/` holds reproducible simulation outputs.

## Completion
Publish one reviewed commit, retain existing repository history, verify its
files and visibility, and provide the repository link and tested boundaries.
Do not access or modify any live robot, production map, credentials, fleet
source, or industrial network as part of this project.

The user approved this design and inline implementation/publication on
2026-11-05. A fresh builder check found no blocking decision; its suggested
author defaults are specified above.
