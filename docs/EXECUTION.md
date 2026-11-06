# Execution evidence

Design approved for inline implementation and publication on 2026-11-05.
The initial remote repository contained only its generated README and one commit.

Pre-flight interfaces: simulation consumes Pose/World/plan/integrate; reporting
and ROS consume Simulation.step/trace/metrics. These are defined in the plan.

Author decision: keep the initial scope compact and standard-library only.
ROS 2 is optional at runtime and is tested separately in Linux CI. No live
hardware integration is present.

The local Windows environment provides Python 3.12.10 and Git 2.53.0.
WSL and Docker are not available. Test evidence is filled in after execution;
the presence of a CI definition is never treated as an observed CI pass.

## Observed local checks (2026-11-06)

- Tests were written before the relevant modules; the initial runs failed
  because navigation, simulation/docking, and reporting were not implemented.
- `python -m unittest discover -s tests -v`: 21 discovered, 20 passed,
  1 explicitly skipped because real ROS `rclpy` is absent; exit 0.
- `python -m compileall -q amr_research_lab tests launch`: exit 0.
- `python setup.py check --strict`: exit 0.
- `python -m amr_research_lab benchmark --output examples/results`: six
  navigation cases completed, with honest metrics retained; exit 0.
- The docking demo reached its simulated DOCKED state; no charge command.
- Generated report tests parse CSV/JSON/SVG and compare repeated outputs.

ROS runtime execution, visible RViz operation, physical robot behavior, and
physical positioning accuracy are not established by these Windows checks.

## Fresh code review and regression fixes

A separate read-only reviewer found two correctness defects: obstacle events
inside an integration step could be missed, and invalid blockage windows
were accepted. Both were reproduced with failing regression tests before
the fixes. Scenario construction now rejects malformed/nonfinite/reversed
windows; the numerical step splits collision checks at scripted event times
and checks end-of-step occupancy before classifying success.

`test_new_obstacle_cannot_win_a_race_with_completion` and
`test_invalid_blockage_windows_fail_at_construction` failed before the change
and passed afterward. The author reran the full suite and six benchmarks.
The fixes were not represented as a second independent review.

A no-index, no-dependency wheel build/install into a separate task-local
directory succeeded, and an experiment imported from that installed package
completed. Publication review checked committed files for selected credential
patterns/private addresses, bundled binary/media, broken relative links,
invalid JSON/SVG, and benchmark limit violations; this is a bounded scan,
not a guarantee of exhaustive legal or security clearance.

## First Linux CI run

[Run 42219520133](https://github.com/Hareespandy-N/ROS-with-robots/actions/runs/42219520133)
passed the Python 3.10 and 3.12 jobs, and built the ROS Humble package with
colcon. Its real ROS smoke test failed on a node-exit graph lookup race:
queued messages remained after the bounded simulator's graph entry vanished.
The test now handles that specific documented graph exception while retaining
all message/frame/exit checks and requiring all four simulated publisher
topics. The process output stream is also closed during cleanup.
The failed CI run supplies the regression baseline; the next run, rather
than this source edit, determines whether the ROS check passes.
