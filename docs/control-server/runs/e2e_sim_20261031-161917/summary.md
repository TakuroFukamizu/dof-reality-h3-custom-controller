# e2e_sim 結果

- 結果: **PASS**（チェック 120 / 120 通過）
- 実行: 2026-10-31 16:19:42（ローカル時刻）、所要 23.0 s
- 構成: h3-control-server（`control-server/config/sim.toml`、sim backend）＋ パッチ後の h3game `bridge.py`（`H3_BACKEND=server`）。ブラウザーは driver が模擬（30 Hz POST、frame_age_ms = 4）
- 環境: macOS-26.5-arm64-arm-64bit-Mach-O / Python 3.14.4
- リポジトリ: `55f024e (worktree; uncommitted changes not identified)`
- h3game: `649ab66 + 0001-h3game-add-H3_BACKEND-server-h3-control-server-and-f.patch `
- 模擬軸での結果。実機（CoreS3）・実ブラウザーでの値ではない。

## 省略

- 模擬軸の異常注入（stall / freeze_status / force_fault / reboot）: サーバーのバイナリに HTTP の入口がないので e2e では行わない。Rust の tests/server.rs・supervisor.rs・sim_axis.rs で確認。e2e の FAULT は管理 API の /v1/fault（operator）

## 遷移・応答時間（driver 側で計測、status を 10 ms ごとに poll）

| 項目 | ms |
| --- | ---: |
| server_start_to_ready_ms | 602.1 |
| arm_to_holding_ms | 89.0 |
| start_to_active_ms | 20.0 |
| freeze_last_fresh_post_to_stopping_ms | 266.0 |
| stopping_to_axes_holding_ms | 0.0 |
| axes_holding_to_braked_ms | 1056.3 |
| stopping_to_braked_ms | 1056.3 |
| api_stop_to_braked_ms | 1087.5 |
| center_ms | 1223.1 |
| fault_to_all_axes_fault_ms | 22.7 |
| browser→bridge POST 往復 p50 / p95 / max（670 件） | 1.06 / 1.77 / 4.88 |

`freeze_last_fresh_post_to_stopping_ms` は最後に frame_seq が進んだ POST の送信開始から、status で STOPPING を初めて見るまで（poll 周期 10 ms 分の誤差を含む）。

## サーバー telemetry（`GET /v1/status` の `telemetry`）

p50 / p95 / p99 / max は直近 1024 サンプルの窓、`max_all` は起動以来、`count` は起動以来の件数。

### after_tracking

| 指標 | count | p50 | p95 | p99 | max | max_all |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| pose_rx_to_tick_ms | 275 | 5.184 | 9.720 | 10.384 | 10.602 | 10.602 |
| tick_dt_ms | 929 | 9.976 | 10.944 | 11.139 | 11.226 | 11.226 |
| status_age_ms | 928 | 9.969 | 19.691 | 20.072 | 20.244 | 20.244 |
| frame_skew | 714 | 0.000 | 1.000 | 1.000 | 1.000 | 1.000 |

tick_count 930、tick_overruns 0、log_dropped 0、recorder_dropped 0

### final

| 指標 | count | p50 | p95 | p99 | max | max_all |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| pose_rx_to_tick_ms | 626 | 5.293 | 9.888 | 10.647 | 10.948 | 10.948 |
| tick_dt_ms | 2239 | 10.028 | 10.816 | 11.142 | 11.273 | 11.273 |
| status_age_ms | 2238 | 9.917 | 19.560 | 20.007 | 20.220 | 20.244 |
| frame_skew | 1122 | 0.000 | 1.000 | 1.000 | 1.000 | 1.000 |

tick_count 2240、tick_overruns 0、log_dropped 0、recorder_dropped 0

frame_skew の単位はフレーム（ACTIVE 中だけ記録）。pose_rx_to_tick は WS 受信から制御 tick がその Pose を取り込むまで（新しい seq ごとに 1 件）。

## 姿勢ごとの追従（target は `target_counts`、actual は模擬軸の `actual_count`）

| 姿勢 | gain → effective | 期待 count | target_counts | actual | 反映 ms | 収束 ms | limits |
| --- | --- | --- | --- | --- | ---: | ---: | --- |
| pitch+0.6 | 0.3 → 0.3 | [360, -72, 0] | [360, -72, 0] | [360, -72, 0] | 29.7 | 509.3 | - |
| pitch-0.6 | 0.3 → 0.3 | [-359, 72, 0] | [-359, 72, 0] | [-359, 72, 0] | 20.2 | 757.9 | - |
| roll+0.6 | 0.3 → 0.3 | [360, 72, 0] | [360, 72, 0] | [360, 72, 0] | 26.2 | 761.1 | - |
| roll-0.6 | 0.3 → 0.3 | [-359, -72, 0] | [-359, -72, 0] | [-359, -72, 0] | 39.9 | 752.1 | - |
| yaw+0.6 | 0.3 → 0.3 | [0, 0, 72] | [0, 0, 72] | [0, 0, 72] | 24.8 | 522.1 | - |
| yaw-0.6 | 0.3 → 0.3 | [0, 0, -72] | [0, 0, -72] | [0, 0, -72] | 27.4 | 484.1 | - |
| combo(0.3,-0.2,0.4) | 0.3 → 0.3 | [60, -60, 48] | [60, -60, 48] | [60, -60, 48] | 49.3 | 454.5 | - |
| combo(-0.4,0.5,-0.5) | 0.3 → 0.3 | [60, 108, -60] | [60, 108, -60] | [60, 108, -60] | 25.3 | 522.0 | - |
| neutral | 0.3 → 0.3 | [0, 0, 0] | [0, 0, 0] | [0, 0, 0] | 38.8 | 443.2 | - |
| full(1,1,0) gain 0.35 | 0.35 → 0.35 | [1400, 0, 0] | [1400, 0, 0] | [1400, 0, 0] | 27.0 | 1120.4 | - |
| gain 0.6 capped | 0.6 → 0.35 | [350, -70, 70] | [350, -70, 70] | [350, -70, 70] | 41.9 | 941.5 | gain_limited |
| pre-freeze pose | 0.3 → 0.3 | [480, -24, 48] | [480, -24, 48] | [480, -24, 48] | 33.1 | 338.1 | - |
| after re-ARM | 0.3 → 0.3 | [60, 84, 24] | [60, 84, 24] | [60, 84, 24] | 51.8 | 599.6 | - |
| new session tracks | 0.3 → 0.3 | [0, -24, 0] | [0, -24, 0] | [0, -24, 0] | 0.0 | 382.7 | - |
| pre-stop pose | 0.3 → 0.3 | [-419, 36, 60] | [-419, 36, 60] | [-419, 36, 60] | 49.6 | 586.6 | - |
| pre-fault pose | 0.3 → 0.3 | [360, 24, -48] | [360, 24, -48] | [360, 24, -48] | 38.8 | 604.9 | - |

期待 count = 校正（3 点区分線形）∘ 飽和（0..1023）∘ 混合式（h3_direct.py と同じ）を Rust の round（0.5 は 0 から遠い側）で丸めた値。許容差 target ±1、actual ±3。

## チェック一覧

| フェーズ | チェック | 結果 |
| --- | --- | --- |
| connect | bridge backend=server, not armed | PASS |
| connect | connect: all axes DISARMED, rig READY_TO_ARM | PASS |
| connect | freshness_mode frame_seq (require_frame_seq default) | PASS |
| connect | connect + fresh poses: no auto arm/center for 1 s | PASS |
| connect | START before ARM rejected (409) | PASS |
| connect | ARM without token rejected (401) | PASS |
| arm_start | ARM accepted | PASS |
| arm_start | HOLDING holds current position (no move to pose/neutral) | PASS |
| arm_start | HOLDING 0.5 s with fresh poses: still not moving | PASS |
| arm_start | START accepted | PASS |
| arm_start | active_source game | PASS |
| poses | pitch+0.6: still ACTIVE(game) | PASS |
| poses | pitch+0.6: effective_gain | PASS |
| poses | pitch+0.6: virtual_targets == mix(pose) | PASS |
| poses | pitch+0.6: target_counts == calibration(mix(pose)) | PASS |
| poses | pitch-0.6: still ACTIVE(game) | PASS |
| poses | pitch-0.6: effective_gain | PASS |
| poses | pitch-0.6: virtual_targets == mix(pose) | PASS |
| poses | pitch-0.6: target_counts == calibration(mix(pose)) | PASS |
| poses | roll+0.6: still ACTIVE(game) | PASS |
| poses | roll+0.6: effective_gain | PASS |
| poses | roll+0.6: virtual_targets == mix(pose) | PASS |
| poses | roll+0.6: target_counts == calibration(mix(pose)) | PASS |
| poses | roll-0.6: still ACTIVE(game) | PASS |
| poses | roll-0.6: effective_gain | PASS |
| poses | roll-0.6: virtual_targets == mix(pose) | PASS |
| poses | roll-0.6: target_counts == calibration(mix(pose)) | PASS |
| poses | yaw+0.6: still ACTIVE(game) | PASS |
| poses | yaw+0.6: effective_gain | PASS |
| poses | yaw+0.6: virtual_targets == mix(pose) | PASS |
| poses | yaw+0.6: target_counts == calibration(mix(pose)) | PASS |
| poses | yaw-0.6: still ACTIVE(game) | PASS |
| poses | yaw-0.6: effective_gain | PASS |
| poses | yaw-0.6: virtual_targets == mix(pose) | PASS |
| poses | yaw-0.6: target_counts == calibration(mix(pose)) | PASS |
| poses | combo(0.3,-0.2,0.4): still ACTIVE(game) | PASS |
| poses | combo(0.3,-0.2,0.4): effective_gain | PASS |
| poses | combo(0.3,-0.2,0.4): virtual_targets == mix(pose) | PASS |
| poses | combo(0.3,-0.2,0.4): target_counts == calibration(mix(pose)) | PASS |
| poses | combo(-0.4,0.5,-0.5): still ACTIVE(game) | PASS |
| poses | combo(-0.4,0.5,-0.5): effective_gain | PASS |
| poses | combo(-0.4,0.5,-0.5): virtual_targets == mix(pose) | PASS |
| poses | combo(-0.4,0.5,-0.5): target_counts == calibration(mix(pose)) | PASS |
| poses | neutral: still ACTIVE(game) | PASS |
| poses | neutral: effective_gain | PASS |
| poses | neutral: virtual_targets == mix(pose) | PASS |
| poses | neutral: target_counts == calibration(mix(pose)) | PASS |
| poses | full(1,1,0) gain 0.35: still ACTIVE(game) | PASS |
| poses | full(1,1,0) gain 0.35: effective_gain | PASS |
| poses | full(1,1,0) gain 0.35: virtual_targets == mix(pose) | PASS |
| poses | full(1,1,0) gain 0.35: target_counts == calibration(mix(pose)) | PASS |
| poses | max_gain 0.35: full pose does not saturate 0..1023 | PASS |
| gain_cap | gain 0.6 capped: still ACTIVE(game) | PASS |
| gain_cap | gain 0.6 capped: effective_gain | PASS |
| gain_cap | gain 0.6 capped: virtual_targets == mix(pose) | PASS |
| gain_cap | gain 0.6 capped: limits has gain_limited | PASS |
| gain_cap | gain 0.6 capped: target_counts == calibration(mix(pose)) | PASS |
| gain_cap | bridge /state effective_gain == max_gain | PASS |
| freeze | pre-freeze pose: still ACTIVE(game) | PASS |
| freeze | pre-freeze pose: effective_gain | PASS |
| freeze | pre-freeze pose: virtual_targets == mix(pose) | PASS |
| freeze | pre-freeze pose: target_counts == calibration(mix(pose)) | PASS |
| freeze | freeze -> STOPPING (not FAULT) | PASS |
| freeze | STOPPING within source_freshness_ms (+ tick/POST slack) | PASS |
| freeze | HOLD phase observed (rig STOPPING, all axes HOLDING) | PASS |
| freeze | then BRAKED_DISARMED | PASS |
| freeze | no centering: braked where it was | PASS |
| freeze | bridge kept POSTing during freeze | PASS |
| freeze | bridge /state: repeats counted, session not fresh | PASS |
| no_auto_rearm | game resumed: stays BRAKED_DISARMED 1.5 s | PASS |
| no_auto_rearm | bridge /reconnect ok | PASS |
| no_auto_rearm | after reconnect: stays BRAKED_DISARMED 1 s | PASS |
| no_auto_rearm | position unchanged across resume/reconnect | PASS |
| no_auto_rearm | START without re-ARM rejected (409) | PASS |
| no_auto_rearm | explicit re-ARM accepted | PASS |
| no_auto_rearm | re-ARM holds braked position (not neutral) | PASS |
| no_auto_rearm | START accepted after re-ARM | PASS |
| no_auto_rearm | after re-ARM: still ACTIVE(game) | PASS |
| no_auto_rearm | after re-ARM: effective_gain | PASS |
| no_auto_rearm | after re-ARM: virtual_targets == mix(pose) | PASS |
| no_auto_rearm | after re-ARM: target_counts == calibration(mix(pose)) | PASS |
| reload_mid_ride | session change mid-ride -> BRAKED_DISARMED | PASS |
| reload_mid_ride | stopped where it was (no centering) | PASS |
| reload_mid_ride | new session prefix differs | PASS |
| reload_mid_ride | new session: no automatic re-arm | PASS |
| reload_mid_ride | /v1/arm accepted (new session) | PASS |
| reload_mid_ride | /v1/start accepted (new session) | PASS |
| reload_mid_ride | new session tracks: still ACTIVE(game) | PASS |
| reload_mid_ride | new session tracks: effective_gain | PASS |
| reload_mid_ride | new session tracks: virtual_targets == mix(pose) | PASS |
| reload_mid_ride | new session tracks: target_counts == calibration(mix(pose)) | PASS |
| api_stop | pre-stop pose: still ACTIVE(game) | PASS |
| api_stop | pre-stop pose: effective_gain | PASS |
| api_stop | pre-stop pose: virtual_targets == mix(pose) | PASS |
| api_stop | pre-stop pose: target_counts == calibration(mix(pose)) | PASS |
| api_stop | stop without token accepted | PASS |
| api_stop | fresh poses kept arriving | PASS |
| api_stop | late/fresh poses do not restart after STOP | PASS |
| api_stop | stopped where it was | PASS |
| center | ARM accepted | PASS |
| center | CENTER accepted | PASS |
| center | active_source center | PASS |
| center | center ends HOLDING at neutral counts | PASS |
| center | DISARM accepted | PASS |
| operator_fault | /v1/arm accepted | PASS |
| operator_fault | /v1/start accepted | PASS |
| operator_fault | pre-fault pose: still ACTIVE(game) | PASS |
| operator_fault | pre-fault pose: effective_gain | PASS |
| operator_fault | pre-fault pose: virtual_targets == mix(pose) | PASS |
| operator_fault | pre-fault pose: target_counts == calibration(mix(pose)) | PASS |
| operator_fault | operator fault accepted | PASS |
| operator_fault | fault kind operator + note | PASS |
| operator_fault | all axes HOST_FAULT, brake applied | PASS |
| operator_fault | ARM while latched rejected (409 fault_latched) | PASS |
| operator_fault | fault reset accepted | PASS |
| operator_fault | after reset: not armed, axes DISARMED/NONE | PASS |
| operator_fault | after reset: no auto re-arm 1 s | PASS |
| operator_fault | fault_history has operator entry | PASS |
| operator_fault | held through fault (no centering) | PASS |
| shutdown | server exits cleanly on SIGTERM | PASS |

`timeline.jsonl`: フェーズ・チェック・管理 API 呼び出し・status の要約（約 10 Hz と状態変化時）。
