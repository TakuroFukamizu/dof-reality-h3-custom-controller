# 実施記録

`tools/run_once.py` または `tools/session.py` が 1 回終わるたびに、ここに 1 ファイルを足す。生の log と csv は `logs/` にあり、gitignore されている。Markdown 側に、要約、カウントが変わった行、EVT、CSV 全文を写す。

分解能の列は、外部の角度か回転数を別に入れるまで UNKNOWN のままにする。2026-10-25 の結果のまとめは [../results_20261025.md](../results_20261025.md)。

| ファイル | 方向 | RUN | 最終状態 | 分解能 |
| --- | --- | --- | --- | --- |
| [run_once_positive_20261022_230347.md](run_once_positive_20261022_230347.md) | positive | 受理 OK state=RUNNING | DISARMED | UNKNOWN |
| [session_dry_20261025_172745.md](session_dry_20261025_172745.md) | - | 未送信（dry、GND 未接続） | DISARMED | UNKNOWN |
| [session_idle_20261025_172835.md](session_idle_20261025_172835.md) | idle | 未送信（GND 未接続、無効） | DISARMED | UNKNOWN |
| [session_idle_20261025_172919.md](session_idle_20261025_172919.md) | idle | 未送信（GND 未接続、無効） | DISARMED | UNKNOWN |
| [session_run_20261025_173016.md](session_run_20261025_173016.md) | 正逆 | 10/15 % ×4、全て encoder_stall（GND 未接続） | DISARMED | UNKNOWN |
| [session_run_20261025_173416.md](session_run_20261025_173416.md) | 正逆 | 10/15 % ×4、計数成功 | DISARMED | UNKNOWN |
| [session_run_20261025_173840.md](session_run_20261025_173840.md) | 正逆 | 15 % 往復 3 組 | DISARMED | UNKNOWN |
| [session_step_20261025_175356.md](session_step_20261025_175356.md) | negative 主 | 1 回転測定 22 回 | DISARMED | **モーター軸 400 count/rev（4 逓倍）** |
| [session_idle_20261025_185246.md](session_idle_20261025_185246.md) | idle | 未送信（GND 接続後） | DISARMED | UNKNOWN |
| [session_idle_20261025_185326.md](session_idle_20261025_185326.md) | idle | 未送信（GND 接続後） | DISARMED | UNKNOWN |
| [session_step_20261025_191952.md](session_step_20261025_191952.md) | 正逆 | 可動範囲測定 37 回 | DISARMED | 全ストローク 5540 count（上端 +4207、下端 −1333） |
| [session_step_20261025_195644.md](session_step_20261025_195644.md) | 正逆 | 高さ 4 点・ブレーキ保持・20/25 % 往復 39 回 | DISARMED | 0.021〜0.030 mm/count（高さ） |
