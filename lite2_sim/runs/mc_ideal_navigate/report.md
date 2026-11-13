# Monte Carlo navigation report

- generated_at_utc: `2026-11-12T09:04:26+00:00`
- scene: `ruins_reference`
- mode: `navigate`
- profile: `ideal`
- seeds: `42-43`
- jobs: `2`
- min_pass_rate: `not gated`

Command template:

```text
F:\airace\lite2_sim\.venv-mujoco\Scripts\python.exe F:\airace\lite2_sim\run_mujoco.py --scene ruins_reference --navigate --headless --seed 42 --randomize ideal
```

## Summary

| metric | value |
| --- | --- |
| runs | 2 |
| passed | 0 |
| failed | 2 |
| success_rate | 0.0% |
| error_runs | 0 |
| truth_error_m mean | 12.7764 |
| truth_error_m std | 0.0000 |
| truth_error_m min | 12.7764 |
| truth_error_m max | 12.7764 |
| collisions_total | 0 |
| recoveries_total | 0 |
| mean_elapsed_simulation_s | 62.8000 |
| mean_wall_duration_s | 51.1755 |
| randomization_profiles | ideal |
| threshold_met | yes |

## Per-run results

| seed | passed | result | truth error m | collisions | recoveries | sim time s | run dir |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | no | stalled | 12.7764 | 0 | 0 | 62.8000 | runs/mujoco_20261112_170335_06ed7a |
| 43 | no | stalled | 12.7764 | 0 | 0 | 62.8000 | runs/mujoco_20261112_170335_5ff2ba |

## Failure reasons

| reason | count |
| --- | --- |
| stalled | 2 |

## Sampled randomization

| seed | profile | lidar noise m | linear scale | yaw scale | command latency s | dx m | dy m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | ideal | 0.0000 | 1.0000 | 1.0000 | 0.0000 | 0.0000 | 0.0000 |
| 43 | ideal | 0.0000 | 1.0000 | 1.0000 | 0.0000 | 0.0000 | 0.0000 |

