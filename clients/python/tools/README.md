# 実機の調整・計測スクリプト（Issue #25 / #34 / #35）

2026-11-02 の位置ループの調整で使った計測スクリプト。どれも **動いている h3-control-server に HTTP / WS でつなぐだけ**で、シリアルポートは開かない。実機を動かすので、[docs/control-server/hardware-bringup.md](../../../docs/control-server/hardware-bringup.md) §7 の条件（人が見ている、低い制限値）で使う。サーバーの URL は環境変数 `H3CS_URL`（既定 `http://127.0.0.1:8892`）。

`websockets` が入った Python で、標準モジュールを同じ階層のファイルで隠されないよう `-P` を付けて実行する:

```sh
clients/python/.venv/bin/python -P clients/python/tools/hw_ab3.py R3-a1 /tmp/hw.log 20 120
```

| スクリプト | 何をするか |
| --- | --- |
| `hw_ab3.py <label> <logfile> [hold_s] [sine_s]` | READY_TO_ARM から ARM → CENTER → 保持 → 正弦波（pitch ±0.4 @0.07 Hz、roll ±0.4 @0.10 Hz、gain 0.3）→ STOP。#1 / #2 の追従誤差・Duty の跳び（ΔPWM p95）・反転などを出し、`samples_<label>.csv`（10 Hz）をログと同じ場所に書く。FAULT・追従誤差 > 250・1 s 無パルスで中止し、直前 40 サンプルをログに出す。実機は #1 / #2 を想定（#2 を模擬軸にしても動く） |
| `analyze_samples.py <csv>...` | 正弦波区間の #1 について、軌道にない往復の回数・張りつき率・ΔPWM p95 など |
| `segment_jerk.py <csv>...` | #1 の正弦波区間を向き（上り / 下り）と軌道速度の帯で分け、ファームの 100 ms 窓の速度と軌道速度のずれ・速度変化 p90・平均 Duty を出す |
| `center_repro.py <label> <logfile> <pitch>...` | #1 を pose でずらして STOP → ARM → CENTER を 20 Hz で記録（CENTER 開始時の ENCODER_FAULT の再現用） |
| `ff_char_a1.py <logfile>` | #1 だけを一定速度で上下させ、速度・向きごとの平均 Duty を測る（FF の校正用） |

## 注意（計測の限界）

- 状態は `GET /v1/status` を 10 Hz で取っている。**クライアント側の時刻で割った速度（目標・実測の差分 ÷ 取得間隔）は、取得時刻のぶれで大きく跳ねるので使わない**。速度はファームの `velocity_count_s`（100 ms 窓）を使う（`segment_jerk.py` はこちら）。`analyze_samples.py` の `jerk_p95` と `stick` はクライアント時刻に依存するので参考値
- 10 Hz では数 Hz 以上の振動を取りこぼす。細かい振動の評価には 50 Hz のファーム側の計測が要る（Issue #35）
- 比較は同じ軸・同じ条件で。2026-11-02 の基準（#1、r3、kp 6）: 追従誤差 max 14 / rms 3.0、ΔPWM p95 9.0 %、軌道にない往復 10（`docs/control-server/runs/hw_20261102.md`）
