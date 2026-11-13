# Monte Carlo navigation report

- generated_at_utc: `2026-11-12T08:53:03+00:00`
- scene: `sensor_lab`
- mode: `navigate`
- profile: `ideal`
- seeds: `42-43`
- jobs: `2`
- min_pass_rate: `not gated`

Command template:

```text
F:\airace\lite2_sim\.venv-mujoco\Scripts\python.exe F:\airace\lite2_sim\run_mujoco.py --scene sensor_lab --navigate --headless --seed 42 --randomize ideal
```

## Summary

| metric | value |
| --- | --- |
| runs | 2 |
| passed | 2 |
| failed | 0 |
| success_rate | 100.0% |
| error_runs | 0 |
| truth_error_m mean | 0.0930 |
| truth_error_m std | 0.0000 |
| truth_error_m min | 0.0930 |
| truth_error_m max | 0.0930 |
| collisions_total | 0 |
| recoveries_total | 0 |
| mean_elapsed_simulation_s | 45.1000 |
| mean_wall_duration_s | 35.7525 |
| randomization_profiles | ideal |
| threshold_met | yes |

## Per-run results

| seed | passed | result | truth error m | collisions | recoveries | sim time s | run dir |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | yes | goal_reached | 0.0930 | 0 | 0 | 45.1000 | runs/mujoco_20261112_165228_b6c869 |
| 43 | yes | goal_reached | 0.0930 | 0 | 0 | 45.1000 | runs/mujoco_20261112_165228_d0c902 |

## Failure reasons

| reason | count |
| --- | --- |
| none | 0 |

## Sampled randomization

| seed | profile | lidar noise m | linear scale | yaw scale | command latency s | dx m | dy m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | ideal | 0.0000 | 1.0000 | 1.0000 | 0.0000 | 0.0000 | 0.0000 |
| 43 | ideal | 0.0000 | 1.0000 | 1.0000 | 0.0000 | 0.0000 | 0.0000 |

