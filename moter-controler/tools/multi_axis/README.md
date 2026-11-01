# multi_axis — PC から複数台の CoreS3 を動かすスクリプト

**実機を動かす。** 主電源リレー・ブレーキ・モーターを操作するので、現場で機構と人の位置を確認してから使う。
pyserial が要るので PlatformIO の Python（`~/.platformio/penv/bin/python`）で実行する。ポート名は差し込み口で変わるので、先に `pio device list` の `SER=` で個体を確認する。

| スクリプト | 用途 |
| --- | --- |
| `query_port.py <port> <cmd>...` | 単発でコマンドを送って応答を表示 |
| `serial_poller.py <logfile> [port]` | `?` を 0.5 s ごとに送って全応答を記録 |
| `drive_axes.py <logfile> seq\|par <label=port>...` | 校正済みの軸に HOME → NEUTRAL → UPPER → NEUTRAL → HOME → NEUTRAL を 1 軸ずつ（seq）または同時（par） |
| `wiggle.py <logfile> <label=port>...` | `WIGGLE_MODE=full\|manual\|neutral`。manual は UPPER → 手動速度で往復（`WIGGLE_MANUAL_S` 秒、端の 600 手前で折り返し）→ HOME |
| `shutdown_axes.py <label=port>...` | 全台に `B 0` → `R 0` を送って状態を読む |

`drive_axes.py` と `wiggle.py` は終了後もポートを開いたまま保持する（`drive_cmd.txt` / `wiggle_cmd.txt` に `quit` を書くと閉じる）。2026-10-31 の記録は `../../docs/multi_axis_20261031.md`。
