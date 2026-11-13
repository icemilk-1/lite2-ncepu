# Monte Carlo navigation report

- generated_at_utc: `2026-11-12T08:56:01+00:00`
- scene: `ruins_reference`
- mode: `course`
- profile: `none`
- seeds: `42`
- jobs: `1`
- min_pass_rate: `0.0`

Command template:

```text
F:\airace\lite2_sim\.venv-mujoco\Scripts\python.exe F:\airace\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --seed 42
```

## Summary

| metric | value |
| --- | --- |
| runs | 1 |
| passed | 0 |
| failed | 1 |
| success_rate | 0.0% |
| error_runs | 0 |
| truth_error_m mean | 8.4121 |
| truth_error_m std | 0.0000 |
| truth_error_m min | 8.4121 |
| truth_error_m max | 8.4121 |
| collisions_total | 0 |
| recoveries_total | 0 |
| mean_elapsed_simulation_s | 3.0500 |
| mean_wall_duration_s | 5.3030 |
| randomization_profiles | none |
| threshold_met | yes |

## Per-run results

| seed | passed | result | truth error m | collisions | recoveries | sim time s | run dir |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165556_2a20df |

## Stage pass rates

| stage | passed | total | rate |
| --- | --- | --- | --- |
| ramp | 0 | 1 | 0.0% |
| s_bend | 0 | 1 | 0.0% |
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
| insufficient_observations | 1 |

