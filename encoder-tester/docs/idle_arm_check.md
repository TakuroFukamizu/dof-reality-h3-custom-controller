# PWM なしの ARM 切り分け

まだ実施していない。軸が支持され、運転の許可が出てから、ファームウェアを書いて ARM を 1 回送る。

この試験は Duty を上げない。RUN は送らない。PWM 指令は全期間 0 のまま、ARM から READY の時間切れ、通常停止、DISARMED の 2 秒後までを記録する。自動でもう一度 ARM しない。

ブレーキ解除指令は READY のあいだ HIGH になる。PWM が 0 でも、支持が無い軸は動ける。電流監視は接続していない。画面と USB の停止は物理の非常停止ではない。

## 前回 23:03 のログから分かったこと

受信ファイルは `logs/run_once_positive_20261022_230347.csv` と、同じ名前の `.log`。

カウントの更新は制御周期の先頭で行う。`encoder_get` は `pcnt_unit_get_count` の値で、PCNT の取得値である。`count_total` は、その int32 を取りこぼさない差分で int64 に伸ばし、`ENCODER_SIGN` を掛けたもの。SIGN は +1 だった。DISARMED に戻っても `count_total` は消えない。

`trial` と `coast` は、RUN が RUNNING に入った試験だけに付く。

- `trial` の開始カウントは、RUNNING に入った周期の、GPIO を書く前の `count_total`。
- `pwm_off` のカウントは、RUNNING を出た周期の、同じく GPIO 前の値。
- `coast` は、BRAKE_APPLY に入った周期の GPIO 前の値から、`pwm_off` の値を引いたもの。この時点で確定し、その後のカウントでは更新しない。
- CSV の `count_trial_x4` は RUNNING、STOPPING、BRAKE_APPLY、POWER_OFF のあいだだけ `count_total - 開始カウント`。DISARMED では 0 になる。`count_total` はそのまま残る。

主電源指令とブレーキ解除指令は、状態機械のあとに GPIO へ書く。EVT の時刻は、その周期の先頭で読んだ `esp_timer_get_time` である。CSV は 20 ms ごとで、その周期の GPIO 前カウントを出す。

23:03 の変化は 2 箇所。

- seq 3（t_us 165423319、DISARMED、count 0、pwm_raw 0）の次の seq 4（165448320、POWER_WAIT、pwm_raw 0、power_gpio 1）で count が 2。電源指令の EVT は 165438320。変化は電源 GPIO を書いた周期のサンプルより後で、次の CSV までの 10 ms 以内。
- seq 77（167069319、STOPPING、count 2、pwm_raw 0、ブレーキ解除指令 1）の次の seq 78（167089319、BRAKE_APPLY、pwm_raw 0、ブレーキ解除指令 0、brake_gpio 0）で count が -258。`EVT trial_done` と `brake_release_cmd,0` は 167079319 で、差分は 0,0,0。この EVT は BRAKE_APPLY へ入った周期の GPIO 前カウントであり、その値はまだ 2。-260 はその次のサンプルに出ている。

`coast` が 0 で最終 `count_total` だけ -258 なのは、coast を確定したあとにカウントが進んだからである。CSV の速度 -2476 は、その段差を 100 ms 窓が後から割った値で、別の移動ではない。RUNNING 中の count は 2 のまま、pwm_raw は 51 だった。

A/B の列がその行で 0 なのは、その瞬間のレベルが 0 だったというだけで、間のパルスは見えていない。

## 次の記録

制御タスクは 5 ms ごとに、メモリ上の配列へ 1 行足す。シリアルへはその場で出さない。DISARMED または FAULT から 2 秒後に封をし、loop が少しずつ `TRACE` 行を出す。起動後 1 回で終わり、次の ARM は記録しない。

リレー指令が変わった周期は 2 行ある。`tick` は GPIO を書く前の PCNT とレベル。`post` は書いた直後のもう 1 回の PCNT とレベル。どちらも高速波形ではない。5 ms のあいだに入ったエッジは、カウントの差分に積算されるだけで、形は残らない。

列は、MCU 時刻、状態、主電源指令、ブレーキ解除指令、両方の GPIO レベル、PWM 設定値、PCNT 取得値、count_total、直前との差分、G17/G18 のその瞬間のレベル。

合意済みの 16 個、`ENCODER_SIGN=1`、`GLITCH_NS=0` は変えない。値が違っていたら SET も ARM もしない。未設定なら、同じ 16 個を入れて読み返す。

ホストは `tools/run_once.py --idle-arm`。RUN は送らない。この文書を書いた時点では、書き込みも ARM もしていない。
