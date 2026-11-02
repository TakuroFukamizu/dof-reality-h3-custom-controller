# ブレーキ・沈みの計測のまとめ方（analyze.py）

計測の手順・安全は [brake_sag_measurement.md](brake_sag_measurement.md)。ここは、計測の後に `diag_brake.py` の出力
（`moter-controler/docs/data/brake_sag/<日時>_ax<n>_<mode>_<label>.{csv,json,log}`）を条件ごとにまとめ、
r10 の仮値の置き換え候補を出す道具の説明。**実機・シリアルは使わない**（標準ライブラリだけ、numpy 不要）。

```sh
PY=~/.platformio/penv/bin/python
$PY moter-controler/tools/diag_brake/analyze.py moter-controler/docs/data/brake_sag \
    --md /tmp/brake_sag_summary.md --json /tmp/brake_sag_summary.json
# 個別の回だけ: .json / .csv / .log のどれを渡してもよい（同じ名前の組を読む）
$PY moter-controler/tools/diag_brake/analyze.py moter-controler/docs/data/brake_sag/20261103-101500_ax1_sag_N0_empty_S1.json
# テスト（合成の記録。道具の書き方と同じ CSV / JSON を作る）
$PY -m unittest discover -s moter-controler/tools/diag_brake -p 'test_*.py'
```

`--md` が無ければ Markdown を標準出力に出す。出力先を入力のディレクトリに置くと、次の解析で JSON を「summary ではない」として
除外の一覧に出す（害はないが、別の場所に置く方がよい）。

## 1. 条件（まとめる単位）

条件 = **軸 × 位置 × 荷重 × モード × 段 × 向き（ENGAGE のみ）**。軸・モード・ラベルは JSON の `axis` / `mode` / `label` を使い、
ファイル名は解釈しない。向きは `params.dir_`（`firmware_result.drive_pwm_permille` の符号と食い違えば注記）。

ラベルは手順の約束どおり `位置_荷重_段`（例 `N0_empty_S1`、`P15_ballast20kg_E2`）。末尾に回数（`_r2` / `_2`）が付いてもよい。
位置 = `N0` / `P<数>` / `M<数>`、段 = `S<数>` / `E<数>`、間は全部荷重（`_` を含んでもよい）。荷重が `empty`（`noload` / `none` / `0kg`）
なら無負荷、それ以外はバラストとして扱う。約束に合わないラベルはラベルそのものを条件にし、注記を付ける（落とさない）。

## 2. 除外と注記（黙って捨てない）

除外した回は、Markdown の「除外した回（理由）」と JSON の `runs[].excluded` に理由を全部出す。

| 除外 | 理由 |
| --- | --- |
| `result` ≠ OK（FAULT / CANCELLED） | 計測が途中で止まった（画面の非常停止・HOST_FAULT・中止など） |
| `flags & TRUNCATED` | 記録が 1024 点を超えた（後半が無い） |
| `samples_read` < `sample_count`、CSV の行数 ≠ `samples_read`、CSV が無い・読めない、`t_us` が単調でない | 記録の欠け |
| ENGAGE の止まり方が `NO_PULSE` / `REVERSE` / `SAG_GUARD` / `OVERSPEED` / `CANCEL` / `FAULT` | 狙った速さで作動していない（作動の解析に使えない） |
| `t_engage_us` = NONE | 作動指令が無い |
| CONTROL_OVERRUN（`fault_code` か `fault_events`） | 1 ms 記録の時刻が信用できない |
| 記録の間隔の最大 > `--gap-exclude-us`（既定 20000） | 20 ms 窓の値が出せない |
| JSON が無い（`.log` だけ） | 道具が拒否・通信の失敗で書かなかった（「JSON の無い回」として一覧に出す） |

注記（解析には使う）: 記録の間隔 > 2 周期（2000 us）の回数、`NOT_SETTLED`（作動時間・滑りは下限）、SAG の `D_MAX` / `END_MARGIN`
（沈みの途中で作動。その後の時刻の沈みは分からない）、FAULT_EVENT の件数、最初の点が解除指令でない、ラベル・段とモードの食い違い。

## 3. 値の定義

記録の t = 0 はブレーキ解除指令（最初の点）。作動指令の点の count は指令の瞬間の測定値なので、解除中の値に含める。

| 値 | 定義 |
| --- | --- |
| 沈み @50/100/150/300 ms | 解除が T まで続いた回（`t_engage_us` ≥ T）だけ、T 以前の最後の点の変位。続かなかった回は **分からない**（0 でも作動後の値でもない）。表の括弧は届いた回数 |
| 最悪 100 ms 窓（SAG） | `diag_brake.host_metrics` の `released_max_rate_100ms_count_s` と同じ（解除中の点だけ。解除が 100 ms より短ければその長さの窓） |
| 沈みの向き | 作動時の変位 `disp_at_engage` の符号。\|変位\| < 2 は「0（動かず）」 |
| 加速 | 解除の後半の沈みが前半の 1.5 倍を超え、かつ 3 count 以上（手順 §5 の「長いほど速くなるか」。あれば速さでなく時刻ごとの沈みで見る） |
| 解除→動き出し | 解除指令から \|変位\| ≥ 2（`--motion-count`）まで。作動指令までに動かなければ「動かず」（離れたか分からない） |
| ENGAGE の動き出し | 走り始め（`t_drive_us`）の後、走り始めの位置から \|変位\| ≥ 2 になった時刻（解除指令から）。ブレーキが離れた時刻の上側（`--delay` と走り出しを含む） |
| 作動→静止 | 作動指令の点から、その点から 20 ms の間 ±band に収まり始めた最初の点まで。band = max(1, floor(`--still-thr` × 窓))。既定 50 count/s × 20 ms = 1 count（= ファームの `t_settle` と同じ定義）。ファームの `t_settle_us` との大きい方を使う。記録の終わりまで静止しなければ、作動後の記録の長さを **下限** として境界に入れ、「≥」を付ける |
| 作動時の速さ | `firmware_result.v_at_engage_count_s`（ファームの 20 ms 窓）。参考に CSV からの 20 ms 窓も出す（`v_at_engage_host_count_s`） |
| 作動後の滑り | 作動指令の点から記録の終わりまでの \|変位\| の最大（`host_metrics.max_abs_disp_after_engage` と同じ） |
| 静止後の最大 \|v\| | 静止の窓の後の 20 ms 窓の最大 \|v\|（ブレーキが掴んだ後に荷重で滑るか）。1 count / 20 ms = 50 count/s が分解能 |
| VIOUT | ADC 生値（換算なし）。区間: release = ブレーキ解除・PWM 0、drive = PWM ≠ 0、brake = 作動指令から静止まで、rest = 静止の後。記録は解除指令から始まるので、解除前の「休み」は無い（`adc_first` = 解除指令の点） |

## 4. 境界（最悪側）

各値について `n`・最大・最小・平均（参考）・最大 + 分解能・（n ≥ 5 なら）平均 + 3σ（標本標準偏差）を出し、
**保守側 = 最大 + 分解能 と 平均 + 3σ の大きい方**。分解能は時間 1 ms、100 ms 窓の速さ 10 count/s（短い窓なら 1 count / 窓）、
静止後の \|v\| 50 count/s。手順 §5 のとおり余裕の掛け方は r10 の側で決める。`--margin-pct` を付けたときだけ「+x %」の値を併記する
（既定では掛けない）。

作動中の滑りの包絡線 d ≤ a + b·\|v\|（ENGAGE、作動後の滑りと作動時の \|v\|）は、全点を上から押さえる直線（a ≥ 0, b ≥ 0）のうち
[0, v_max] の下の面積が最小のもの。速さが 2 種類未満なら作らない。**v_max より速い速度へは外挿しない**。

## 5. 置き換え候補（r10 の定数）

仮値は `origin/hw-r10-integrated-test:moter-controler/include/config.h`（2026-11-02 時点）。どれも「測った値から出したもの」で、
採用は r10 の側で決める。対象の回が無ければ値を作らず **`insufficient data`** と出す。

| 定数（仮値） | 使う回・値 | 注 |
| --- | --- | --- |
| `V1_SUP_BACKDRIVE_COUNT_S`（200 count/s） | SAG で **カウント減の向き** に沈んだ回の最悪 100 ms 窓（全軸・全位置・全荷重） | r10 はこの値を下向き（カウント減）にだけ足す。上向きに沈んだ回は別の所見として回数と最大を出す。動かなかった回は n に入れない |
| `V1_BRAKE_ENGAGE_MS`（150 ms） | SAG・ENGAGE の全回の「作動→静止」（ファームの `t_settle` との大きい方） | 静止しなかった回は下限で入る（最大に「≥」）。作動時の \|v\| の範囲を併記（それより速い作動は測っていない） |
| `BRAKE_RELEASE_WAIT_MS`（300 ms） | SAG の解除→動き出しと、ENGAGE の動き出し（解除指令から） | SAG で動かなかった回は n に入れない（荷重が軽いと離れても動かない）。ENGAGE の値は `--delay` を含む上側 |
| `V1_SUP_HOLD_ENV_COUNT_S`（150 count/s） | **バラスト** 条件の SAG 最悪 100 ms 窓と、静止後の最大 \|v\| | 無負荷の回は使わない（バラストが無ければ `insufficient data`） |

実測の最大が仮値を超えたら「**仮値を超えた**」と出す（手順 §5: 1 回でも仮値より悪ければ、その値が新しい境界の候補）。

## 6. 未確認・限界

- 実機の記録ではまだ一度も回していない（2026-11-02 時点）。テストは道具の書き方（`diag_brake.run()` の CSV / JSON）を写した合成の記録と、
  `diag_brake.host_metrics` との一致で確かめただけ。道具の出力の形が変わったら（列の追加は可、必須の列 `t_us` / `count` / `pwm_permille` /
  `brake_release` / `adc_raw` が変わったら不可）`test_analyze.py` の `write_run` も合わせる。
- 静止の判定は 1 count の分解能に縛られる（±1 count / 20 ms ≈ 50 count/s より遅い沈みは「静止」と出うる）。SAG では静止後の最大 \|v\| と
  作動後の滑りを併せて見る。
- 「保持の包絡線」は手順 §5 の対応（バラストの沈み・作動後の速さ）に従ったもの。r10 の `V1_SUP_HOLD_ENV_COUNT_S` は「荷重が無くなったときに
  保持出力で動きうる速さ」の見積もりで、ブレーキの計測だけでは直接は測れない。採用の前に意味を突き合わせる。
- 加速の目安（後半 > 前半 × 1.5）は粗い。疑わしければ CSV を直接見る。
