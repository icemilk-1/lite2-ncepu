# Monte Carlo navigation report

- generated_at_utc: `2026-11-12T08:54:17+00:00`
- scene: `ruins_reference`
- mode: `course`
- profile: `ideal`
- seeds: `42`
- jobs: `1`
- min_pass_rate: `0.0`

Command template:

```text
F:\airace\lite2_sim\.venv-mujoco\Scripts\python.exe F:\airace\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --seed 42 --randomize ideal
```

## Summary

| metric | value |
| --- | --- |
| runs | 1 |
| passed | 0 |
| failed | 1 |
| success_rate | 0.0% |
| error_runs | 0 |
| truth_error_m mean | 2.5190 |
| truth_error_m std | 0.0000 |
| truth_error_m min | 2.5190 |
| truth_error_m max | 2.5190 |
| collisions_total | 0 |
| recoveries_total | 0 |
| mean_elapsed_simulation_s | 123.8500 |
| mean_wall_duration_s | 121.0140 |
| randomization_profiles | ideal |
| threshold_met | yes |

## Per-run results

| seed | passed | result | truth error m | collisions | recoveries | sim time s | run dir |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | no | goal_blocked | 2.5190 | 0 | 0 | 123.8500 | runs/mujoco_20261112_165216_67e955 |

## Stage pass rates

| stage | passed | total | rate |
| --- | --- | --- | --- |
| ramp | 1 | 1 | 100.0% |
| s_bend | 1 | 1 | 100.0% |
| dead_end_exit | 0 | 1 | 0.0% |
| south_rubble_route | 0 | 1 | 0.0% |
| rubble_return | 0 | 1 | 0.0% |
| round_pipe | 0 | 1 | 0.0% |
| narrow_passage_rejection | 0 | 1 | 0.0% |
| square_culvert | 0 | 1 | 0.0% |
| finish | 0 | 1 | 0.0% |

## Failure reasons

| reason | count |
| --- | --- |
| goal_blocked | 1 |

## Sampled randomization

| seed | profile | lidar noise m | linear scale | yaw scale | command latency s | dx m | dy m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | ideal | 0.0000 | 1.0000 | 1.0000 | 0.0000 | 0.0000 | 0.0000 |

