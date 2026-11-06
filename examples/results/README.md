# Reproducible ideal-simulation results

These are toy-map experiments, not a Nav2 benchmark or physical accuracy evidence.

| Scenario | Controller | Outcome | Duration (s) | XY error (mm) | Min clearance (m) | Blocked (s) |
|---|---|---|---:|---:|---:|---:|
| open_floor | baseline | completed | 20.45 | 1.23 | 0.580 | 0.00 |
| open_floor | regulated | completed | 20.40 | 0.98 | 0.580 | 0.00 |
| warehouse | baseline | completed | 30.60 | 1.24 | 0.153 | 0.60 |
| warehouse | regulated | completed | 62.00 | 1.23 | 0.162 | 0.00 |
| temporary_blockage | baseline | completed | 18.85 | 0.55 | 0.022 | 2.60 |
| temporary_blockage | regulated | completed | 18.90 | 0.23 | 0.032 | 0.75 |
