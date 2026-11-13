# Monte Carlo navigation report

- generated_at_utc: `2026-11-13T06:29:40+00:00`
- scene: `ruins_reference`
- mode: `course`
- profile: `sim2real_v1`
- seeds: `42-61`
- jobs: `3`
- min_pass_rate: `0.8`

Command template:

```text
F:\airace\lite2_sim\.venv-mujoco\Scripts\python.exe F:\airace\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --seed 42 --randomize sim2real_v1
```

## Summary

| metric | value |
| --- | --- |
| runs | 20 |
| passed | 0 |
| failed | 20 |
| success_rate | 0.0% |
| error_runs | 2 |
| truth_error_m mean | 7.7078 |
| truth_error_m std | 1.9328 |
| truth_error_m min | 0.1622 |
| truth_error_m max | 8.6112 |
| collisions_total | 3 |
| recoveries_total | 0 |
| mean_elapsed_simulation_s | 40.8667 |
| mean_wall_duration_s | 56.0489 |
| randomization_profiles | sim2real_v1 |
| threshold_met | no |

## Per-run results

| seed | passed | result | truth error m | collisions | recoveries | sim time s | run dir |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | no | start_blocked | 8.4121 | 0 | 0 | 17.3500 | runs/mujoco_20261113_142143_375650 |
| 43 | no | insufficient_observations | 8.4121 | 0 | 0 | 10.6500 | runs/mujoco_20261113_142143_815e21 |
| 44 | no | timeout | 7.4251 | 0 | 0 | 121.3000 | runs/mujoco_20261113_142143_4a2512 |
| 45 | no | insufficient_observations | 8.4121 | 0 | 0 | 10.6500 | runs/mujoco_20261113_142156_dbb0b8 |
| 46 | no | timeout | 6.8145 | 0 | 0 | 121.0500 | runs/mujoco_20261113_142203_912dac |
| 47 | no | insufficient_observations | 8.4121 | 0 | 0 | 10.6500 | runs/mujoco_20261113_142210_d763ba |
| 48 | no | timeout | 6.2819 | 0 | 0 | 121.4500 | runs/mujoco_20261113_142224_26ca63 |
| 49 | no | insufficient_observations | 8.4152 | 0 | 0 | 11.6500 | runs/mujoco_20261113_142448_b7565d |
| 50 | no | insufficient_observations | 8.6112 | 0 | 0 | 23.3500 | runs/mujoco_20261113_142504_cff05e |
| 51 | no | insufficient_observations | 8.4121 | 0 | 0 | 10.6500 | runs/mujoco_20261113_142532_116596 |
| 52 | no | collision | 0.1622 | 3 | 0 | 167.8000 | runs/mujoco_20261113_142535_e9b637 |
| 53 | no | insufficient_observations | 8.4971 | 0 | 0 | 11.6500 | runs/mujoco_20261113_142546_a4df67 |
| 54 | no | start_blocked | 8.4121 | 0 | 0 | 17.3500 | runs/mujoco_20261113_142552_960809 |
| 55 | no | start_blocked | 8.4121 | 0 | 0 | 17.3500 | runs/mujoco_20261113_142603_45a188 |
| 56 | no | start_blocked | 8.4121 | 0 | 0 | 17.3500 | runs/mujoco_20261113_142613_e37130 |
| 57 | no | start_blocked | 8.4121 | 0 | 0 | 17.3500 | runs/mujoco_20261113_142625_9e7852 |
| 58 | no | runtime_error | - | 0 | 0 | - | runs/mujoco_20261113_142635_cac573 |
| 59 | no | start_blocked | 8.4121 | 0 | 0 | 17.3500 | runs/mujoco_20261113_142639_7c58ef |
| 60 | no | runtime_error | - | 0 | 0 | - | runs/mujoco_20261113_142648_74d803 |
| 61 | no | insufficient_observations | 8.4125 | 0 | 0 | 10.6500 | runs/mujoco_20261113_142653_4ae89e |

## Stage pass rates

| stage | passed | total | rate |
| --- | --- | --- | --- |
| ramp | 1 | 18 | 5.6% |
| s_bend | 0 | 18 | 0.0% |
| dead_end_exit | 0 | 18 | 0.0% |
| obstacle_bypass | 0 | 18 | 0.0% |
| narrow_passage_rejection | 0 | 18 | 0.0% |
| square_culvert | 0 | 18 | 0.0% |
| finish | 0 | 18 | 0.0% |

## Failure reasons

| reason | count |
| --- | --- |
| insufficient_observations | 8 |
| start_blocked | 6 |
| timeout | 3 |
| run_error | 2 |
| collision | 1 |

## Sampled randomization

| seed | profile | lidar noise m | linear scale | yaw scale | command latency s | dx m | dy m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | sim2real_v1 | 0.0278 | 0.9887 | 1.0211 | 0.0658 | -0.0649 | 0.0761 |
| 43 | sim2real_v1 | 0.0268 | 0.9844 | 1.0056 | 0.0066 | 0.0139 | -0.0440 |
| 44 | sim2real_v1 | 0.0123 | 1.0431 | 0.9515 | 0.0387 | -0.0540 | 0.0572 |
| 45 | sim2real_v1 | 0.0252 | 1.0928 | 0.9275 | 0.0793 | 0.0016 | 0.0447 |
| 46 | sim2real_v1 | 0.0150 | 1.0461 | 0.9722 | 0.0116 | 0.0694 | -0.0664 |
| 47 | sim2real_v1 | 0.0238 | 1.0222 | 0.9510 | 0.1131 | 0.0747 | -0.0287 |
| 48 | sim2real_v1 | 0.0026 | 0.9359 | 0.9688 | 0.0894 | 0.0244 | 0.0581 |
| 49 | sim2real_v1 | 0.0188 | 1.0867 | 0.9264 | 0.0890 | 0.0249 | -0.0778 |
| 50 | sim2real_v1 | 0.0119 | 0.9223 | 1.0160 | 0.1251 | -0.0421 | 0.0235 |
| 51 | sim2real_v1 | 0.0237 | 0.9806 | 1.0128 | 0.0461 | 0.0458 | -0.0453 |
| 52 | sim2real_v1 | 0.0142 | 1.0966 | 0.9316 | 0.0861 | 0.0080 | 0.0749 |
| 53 | sim2real_v1 | 0.0090 | 0.9364 | 1.0489 | 0.1078 | -0.0632 | 0.0266 |
| 54 | sim2real_v1 | 0.0214 | 1.0538 | 1.0261 | 0.0778 | -0.0662 | -0.0072 |
| 55 | sim2real_v1 | 0.0181 | 0.9163 | 0.9793 | 0.1306 | -0.0012 | 0.0405 |
| 56 | sim2real_v1 | 0.0178 | 1.0700 | 1.0346 | 0.0025 | 0.0071 | -0.0423 |
| 57 | sim2real_v1 | 0.0145 | 1.0100 | 0.9989 | 0.0702 | -0.0170 | -0.0550 |
| 58 | sim2real_v1 | 0.0081 | 0.9522 | 0.9530 | 0.1320 | 0.0553 | -0.0575 |
| 59 | sim2real_v1 | 0.0212 | 1.0607 | 0.9761 | 0.1239 | -0.0502 | -0.0544 |
| 60 | sim2real_v1 | 0.0006 | 1.0206 | 0.9378 | 0.0041 | 0.0581 | 0.0081 |
| 61 | sim2real_v1 | 0.0181 | 0.9478 | 0.9236 | 0.1158 | -0.0106 | 0.0796 |

## Errors

| seed | error | stderr tail |
| --- | --- | --- |
| 58 | cannot access local variable 'standstill_escapes' where it is not associated with a value | Failed run: F:\airace\lite2_sim\runs\mujoco_20261113_142635_cac573 |
| 60 | cannot access local variable 'standstill_escapes' where it is not associated with a value | Failed run: F:\airace\lite2_sim\runs\mujoco_20261113_142648_74d803 |

