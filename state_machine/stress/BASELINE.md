# Baseline memory report

Measured 2026-10-06 on `main` at de18584 with `python3 -m state_machine.stress --suite full`
(Python 3.13, x86-64 Linux, 8 GB). Memory sizes carry over to the Jetson Nano closely;
ms/tick will be several times higher on the Nano's ARM cores.

Re-run after adding a module and compare against this table.

RAM budget for our code: 3072 MB (4096 MB total - 1024 MB reserved for OS/GPU; change with --total-mb / --reserve-mb)

| case | sim | ticks | ms/tick (mean / max) | start MB | peak MB | growth MB | leak MB/h | 8 h proj. MB | % budget | |
|---|---|---|---|---|---|---|---|---|---|---|
| sm_scenarios_x200 | 3.6 h | 131,000 | 0.02 / 0.1 | 15 | 15 | +0.1 | +0.01 | 15 | 0.5% | ok |
| sm_patrol_1h | 1.0 h | 35,999 | 0.01 / 6.0 | 16 | 16 | +0.1 | +0.05 | 16 | 0.5% | ok |
| sm_fuzz_1h | 1.0 h | 35,999 | 0.01 / 0.7 | 23 | 25 | +2.0 | +0.35 | 28 | 0.9% | ok |
| sm_flapping_1h | 1.0 h | 35,999 | 0.02 / 0.9 | 19 | 28 | +3.6 | +2.73 | 42 | 1.4% | ok |
| sm_payload_64kb | 10 min | 5,999 | 7.11 / 20.6 | 16 | 17 | +1.3 | +0.11 | 18 | 0.6% | ok |
| sm_history_all_10min | 10 min | 5,999 | 0.02 / 4.4 | 15 | 24 | +8.5 | +50.40 | 419 | 13.6% | ok |
| sm_history_ring_10min | 10 min | 5,999 | 0.02 / 0.1 | 15 | 16 | +1.0 | +0.11 | 17 | 0.6% | ok |
| pipeline_10min | 10 min | 6,000 | 2.33 / 31.9 | 54 | 59 | +5.4 | +7.63 | 119 | 3.9% | ok |
| sm_patrol_8h | 8.0 h | 287,999 | 0.02 / 2.7 | 18 | 19 | +0.9 | +0.05 | 19 | 0.6% | ok |
| sm_fuzz_8h | 8.0 h | 287,999 | 0.02 / 213.6 | 254 | 299 | +9.4 | +0.65 | 264 | 9.7% | ok |
| sm_flapping_8h | 8.0 h | 287,999 | 0.02 / 45.2 | 44 | 114 | +6.8 | +3.93 | 51 | 3.7% | ok |
| sm_payload_256kb | 3 min | 1,799 | 29.55 / 58.4 | 17 | 22 | +3.7 | - | - | 0.7% | ok |
| sm_history_all_1h | 1.0 h | 35,999 | 0.02 / 20.8 | 16 | 66 | +50.2 | +49.96 | 415 | 13.5% | ok |
| pipeline_1h | 1.0 h | 36,000 | 2.30 / 51.1 | 54 | 69 | +14.4 | +11.31 | 148 | 4.8% | ok |
| pipeline_1h_nulllog | 1.0 h | 36,000 | 2.28 / 14.7 | 54 | 62 | +7.8 | +4.41 | 93 | 3.0% | ok |
| pipeline_2cam_720p_buf2s | 20 min | 12,000 | 14.05 / 43.3 | 54 | 273 | +219.3 | +10.43 | 353 | 11.5% | ok |
| pipeline_2cam_1080p_buf5s | 10 min | 6,000 | 31.59 / 322.5 | 54 | 1252 | +1197.5 | +10.43 | 1333 | 43.4% | ok |
| pipeline_model_512mb | 10 min | 6,000 | 2.38 / 10.7 | 566 | 571 | +5.3 | +7.59 | 631 | 20.5% | ok |

Cases:
- **sm_scenarios_x200**: All named demo scenarios, 200x: baseline loop cost. {'ticks_over_budget': 0, 'transitions': 15800}
- **sm_patrol_1h**: 60 back-to-back missions; flat memory = no leak in the loop. {'ticks_over_budget': 0, 'transitions': 420, 'events': 420}
- **sm_fuzz_1h**: Random module outputs, 5 events/s. {'ticks_over_budget': 0, 'transitions': 4482, 'events': 18000}
- **sm_flapping_1h**: Bird toggles every tick: transitions list growth. {'ticks_over_budget': 0, 'transitions': 35999, 'events': 36000}
- **sm_payload_64kb**: Observation +64 KB for each of 4 future modules (deepcopy cost). {'ticks_over_budget': 0, 'transitions': 70, 'observation_json_kb': 265.2}
- **sm_history_all_10min**: Keeping every tick's snapshot in memory. {'ticks_over_budget': 0, 'kept_snapshots': 6000}
- **sm_history_ring_10min**: Keeping only the last 60 s of snapshots. {'ticks_over_budget': 0, 'kept_snapshots': 600}
- **pipeline_10min**: Real CV + Safety + Docking + SM + Motion, 1 camera 640x480 @15 fps. {'ticks_over_budget': 0, 'transitions': 70, 'frames': 9000, 'frame_mb': 0.88, 'motion_history_len': 1860, 'motion_log_records': 1890}
- **sm_patrol_8h**: An 8-hour day of missions, state machine only. {'ticks_over_budget': 0, 'transitions': 3360, 'events': 3360}
- **sm_fuzz_8h**: 8 h of random module outputs at 20 events/s. {'ticks_over_budget': 1, 'transitions': 61198, 'events': 576000}
- **sm_flapping_8h**: 8 h of a bird flickering every tick. {'ticks_over_budget': 0, 'transitions': 287999, 'events': 288000}
- **sm_payload_256kb**: Observation +256 KB per module (1 MB world deep-copied every tick). {'ticks_over_budget': 0, 'transitions': 21, 'observation_json_kb': 1060.0}
- **sm_history_all_1h**: 1 h of every snapshot kept in memory. {'ticks_over_budget': 0, 'kept_snapshots': 36000}
- **pipeline_1h**: Full pipeline 1 h, motion logs kept in memory (MemoryLogSink). {'ticks_over_budget': 0, 'transitions': 420, 'frames': 54000, 'frame_mb': 0.88, 'motion_history_len': 11160, 'motion_log_records': 11340}
- **pipeline_1h_nulllog**: Full pipeline 1 h, motion logs serialised then dropped. {'ticks_over_budget': 0, 'transitions': 420, 'frames': 54000, 'frame_mb': 0.88, 'motion_history_len': 11160, 'motion_log_records': None}
- **pipeline_2cam_720p_buf2s**: Front + downward camera at 1280x720, keeping 2 s of frames. {'ticks_over_budget': 0, 'transitions': 140, 'frames': 36000, 'frame_mb': 2.64, 'motion_history_len': 3720, 'motion_log_records': 3780}
- **pipeline_2cam_1080p_buf5s**: Worst case: 2x 1080p cameras, 5 s frame buffer each. {'ticks_over_budget': 2, 'transitions': 70, 'frames': 18000, 'frame_mb': 5.93, 'motion_history_len': 1860, 'motion_log_records': 1890}
- **pipeline_model_512mb**: Pipeline plus a 512 MB stand-in for a future detector model. {'ticks_over_budget': 0, 'transitions': 70, 'frames': 9000, 'frame_mb': 0.88, 'motion_history_len': 1860, 'motion_log_records': 1890}
