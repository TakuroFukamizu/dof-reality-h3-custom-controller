#!/usr/bin/env python3
"""ブレーキ・沈みの計測（計測専用ファーム env:m5stack-cores3-v1-diag）。サーバーを止めた状態で MCU のポートを直接開く。

手順・安全の注意・計測の計画: moter-controler/docs/brake_sag_measurement.md

    PY=~/.platformio/penv/bin/python   # pyserial と tomllib（3.11）が入っている
    $PY moter-controler/tools/diag_brake/diag_brake.py --port /dev/cu.usbmodem12501 --mac 1C:DB:D4:BA:8D:58 \\
        --config control-server/config/h3-bringup-01-live-20261102.toml --axis 1 --label n0_empty \\
        sag --t-max 150 --d-max 60

やること:
  1. ポートを開く前に、そのポートの USB Serial Number（= eFuse MAC）が --mac と一致することを確かめる（違えば開かない）
  2. HELLO で device_id（5131 + MAC）・選択中の系統（--axis）・計測専用ファーム（capabilities の DIAG_BRAKE）を確かめる
  3. STATUS で DISARMED・POSITION_REFERENCED・異常なし・ソフトリミットまで --min-room 以上を確かめ、設定ファイルの値で CONFIG を送る
  4. 電源担当でない軸なら、電源担当軸（--power-port / --power-mac）に POWER_HOLD を送り、その区間が SETTLED（主電源 ON 指令から
     ≥ 5 s）を名乗るまで待ってから、計測する軸に DIAG_START を送る（計測する軸の区間もさらに 5 s 待ってから解除する）
  5. 同じ区間（主電源を入れたまま）で --repeat 回: DIAG_START → DIAG_RESULT → DIAG_READ で記録を全部読む → CSV / JSON・1 行の要約。
     不成功（OK 以外）・区間が閉じた・余裕が --min-room を割ったらそこで止める
  6. 終わり・Ctrl-C・例外のどれでも DIAG_START(CANCEL) を送って区間を閉じる（主電源 OFF。記録中なら作動後の記録を終えてから）。
     道具が落ちて何も送れなくても、ファームは待機・電源待ちの間の受信途絶 2 s で区間を閉じる（記録中の 1 s 弱は自分の時計で終える）
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import random
import sys
import time
import tomllib

import v1mini as v1

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # テスト（host_metrics）だけなら不要
    serial = None


class ToolError(Exception):
    pass


# ---------------------------------------------------------------------------
# 記録の解析（ホスト側。ファームの要約と同じものに加え、保守側の値を出す）
# ---------------------------------------------------------------------------
def host_metrics(samples: list[dict], res: dict) -> dict:
    out: dict = {}
    if not samples:
        return out
    c0 = res["start_count"]
    released = [s for s in samples if s["flags"] & v1.SAMPLE_FLAG_BRAKE_RELEASE]
    # 解除指令（t = 0）から |変位| ≥ 2 count になるまで（ブレーキが離れて動き始めた時刻の目安。±1 のちらつきを除く）
    out["release_to_motion_ms"] = None
    for s in samples:
        if abs(s["count"] - c0) >= 2:
            out["release_to_motion_ms"] = s["t_us"] / 1000.0
            break
    # 解除中の最悪の 100 ms 窓の速さ（解除が 100 ms より短ければその長さの窓）[count/s]
    out["released_max_rate_100ms_count_s"] = None
    if len(released) >= 2:
        span = released[-1]["t_us"] - released[0]["t_us"]
        w = min(100_000, span)
        best = 0.0
        j = 0
        for i, si in enumerate(released):
            j = max(j, i)
            while j + 1 < len(released) and released[j + 1]["t_us"] - si["t_us"] <= w:
                j += 1
            dtw = released[j]["t_us"] - si["t_us"]
            if dtw >= w - 1000 and dtw > 0:
                best = max(best, abs(released[j]["count"] - si["count"]) * 1e6 / dtw)
        out["released_max_rate_100ms_count_s"] = round(best, 1)
        out["released_window_ms"] = w / 1000.0
    te = res.get("t_engage_us", v1.TIME_NONE)
    if te == v1.TIME_NONE:
        return out
    ie = next((i for i, s in enumerate(samples) if s["t_us"] >= te), None)
    if ie is None:
        return out
    out["stop_disp_after_engage"] = res["final_disp"] - res["disp_at_engage"]
    out["max_abs_disp_after_engage"] = max(abs(s["count"] - samples[ie]["count"]) for s in samples[ie:])
    # 作動指令から静止（20 ms の間 ±1 count）まで（ファームの t_settle と同じ定義）
    out["engage_to_still_ms"] = None
    for i in range(ie, len(samples)):
        if samples[-1]["t_us"] - samples[i]["t_us"] < 20_000:
            break
        ok = all(abs(samples[j]["count"] - samples[i]["count"]) <= 1
                 for j in range(i + 1, len(samples)) if samples[j]["t_us"] - samples[i]["t_us"] <= 20_000)
        if ok:
            out["engage_to_still_ms"] = (samples[i]["t_us"] - te) / 1000.0
            break
    # 作動指令から、10 ms 窓の速さが作動時の半分を下回るまで（ブレーキが荷重を受け始めた時刻の目安）
    out["engage_to_half_speed_ms"] = None
    v_e = abs(res.get("v_at_engage_count_s", 0))
    if v_e >= 100:
        for i in range(ie, len(samples)):
            k = i
            while k > 0 and samples[i]["t_us"] - samples[k]["t_us"] < 10_000:
                k -= 1
            dtw = samples[i]["t_us"] - samples[k]["t_us"]
            if dtw >= 10_000 and abs(samples[i]["count"] - samples[k]["count"]) * 1e6 / dtw < v_e / 2:
                out["engage_to_half_speed_ms"] = (samples[i]["t_us"] - te) / 1000.0
                break
    return out


# ---------------------------------------------------------------------------
# リンク
# ---------------------------------------------------------------------------
def check_port_identity(port: str, mac: str) -> None:
    """開く前に USB Serial Number を見る（開くとつながっている装置がリセットされることがある）"""
    want = v1.normalize_mac(mac)
    for p in serial.tools.list_ports.comports():
        if p.device == port:
            got = v1.normalize_mac(p.serial_number or "")
            if got != want:
                raise ToolError(f"{port}: USB Serial Number {p.serial_number!r} は --mac {mac} と違う。開かない")
            return
    raise ToolError(f"{port} が見つからない（ポートを列挙し直す: ioreg -p IOUSB -l -w0 | grep 'USB Serial Number'）")


class Link:
    def __init__(self, name: str, port: str, mac: str, log):
        self.name = name
        self.port = port
        self.mac = mac
        self.log = log
        self.split = v1.Splitter()
        self.rid = random.randint(1000, 1 << 30)
        check_port_identity(port, mac)
        s = serial.Serial()
        s.port = port
        s.baudrate = 115200
        s.timeout = 0
        # tools/multi_axis と同じ開き方（DTR を立てないと MCU の送信が届かない。macOS の open は元から DTR / RTS を立てる）
        s.dtr = True
        s.rts = True
        s.open()
        self.s = s
        self.status: dict | None = None
        self.hello: dict | None = None
        self.inbox: list[v1.Msg] = []
        self.fault_events: list[dict] = []

    def next_rid(self) -> int:
        self.rid += 1
        return self.rid

    def send(self, data: bytes) -> None:
        self.s.write(data)

    def pump(self) -> None:
        chunk = self.s.read(4096)
        if not chunk:
            return
        for enc in self.split.push(chunk):
            m = v1.decode(enc)
            if m is None:
                continue
            if m.type == v1.STATUS:
                self.status = m.fields
            elif m.type == v1.FAULT_EVENT:
                self.fault_events.append(m.fields)
                self.log(f"[{self.name}] FAULT_EVENT {v1.FAULT_NAMES.get(m.fields['fault_code'], m.fields['fault_code'])}"
                         f" actual={m.fields['actual_count']}")
            else:
                self.inbox.append(m)

    def wait(self, pred, timeout: float, others: list["Link"] = ()):
        """pred(msg) に合う受信を待つ（他のリンクも回す）"""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.pump()
            self.keepalive()  # 区間の電源待ち（5 s）・待機の間も受信途絶（2 s）で閉じられないように
            for o in others:
                o.pump()
                o.keepalive()
            for i, m in enumerate(self.inbox):
                if pred(m):
                    return self.inbox.pop(i)
            time.sleep(0.002)
        return None

    def request(self, data_fn, pred, timeout: float, tries: int, others=()):
        for _ in range(tries):
            self.send(data_fn())
            m = self.wait(pred, timeout, others)
            if m is not None:
                return m
        return None

    def do_hello(self, expect_axis: int) -> dict:
        rid = self.next_rid()
        m = self.request(lambda: v1.hello_req(rid), lambda x: x.type == v1.HELLO and x.fields["request_id"] == rid,
                         0.2, 10)
        if m is None:
            raise ToolError(f"[{self.name}] HELLO に答えない（Issue #11 の無応答なら USB を差し直す）")
        h = m.fields
        want = v1.device_id_from_mac(self.mac)
        if h["device_id"] != want:
            raise ToolError(f"[{self.name}] device_id {h['device_id']:016x} は期待 {want:016x} と違う")
        if h["protocol_version"] != 1:
            raise ToolError(f"[{self.name}] protocol_version {h['protocol_version']}")
        if not h["capabilities"] & v1.CAP_DIAG_BRAKE:
            raise ToolError(f"[{self.name}] 計測専用ファーム（env:m5stack-cores3-v1-diag）ではない（capabilities "
                            f"0x{h['capabilities']:x}）")
        if h["selected_axis"] != expect_axis:
            raise ToolError(f"[{self.name}] 選択中の系統 #{h['selected_axis']} は --axis {expect_axis} と違う")
        self.hello = h
        self.last_keepalive = time.monotonic()
        return h

    def wait_status(self, timeout: float = 1.0) -> dict:
        self.status = None
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end and self.status is None:
            self.pump()
            time.sleep(0.002)
        if self.status is None:
            raise ToolError(f"[{self.name}] STATUS が来ない")
        return self.status

    def command(self, make, msg_type: int, others=()) -> dict:
        rid = self.next_rid()
        data = make(rid)
        m = self.request(lambda: data,
                         lambda x: x.type == v1.ACK and x.fields["request_id"] == rid and x.fields["acked_type"] == msg_type,
                         0.3, 3, others)
        if m is None:
            raise ToolError(f"[{self.name}] 0x{msg_type:02x} の ACK が来ない")
        a = m.fields
        if a["result"] == 1:
            raise ToolError(f"[{self.name}] 0x{msg_type:02x} 拒否: {v1.ACK_REASON_NAMES.get(a['reason'], a['reason'])}")
        return a

    def session_status(self, others=()) -> dict:
        """DIAG_READ（範囲外の offset）で区間の状態だけ読む: session_flags（ACTIVE / POWER_CMD / SETTLED / TEST_BUSY）と経過 ms"""
        rid = self.next_rid()
        m = self.request(lambda: v1.diag_read(rid, v1.DIAG_READ_STATUS_ONLY),
                         lambda x: x.type == v1.DIAG_DATA and x.fields["request_id"] == rid, 0.3, 5, others)
        if m is None:
            raise ToolError(f"[{self.name}] DIAG_DATA（区間の状態）が来ない")
        return dict(session_flags=m.fields["session_flags"], session_ms=m.fields["session_ms"])

    def read_samples(self, total: int, others=()) -> list[dict]:
        samples: list[dict] = []
        while len(samples) < total:
            off = len(samples)
            rid = self.next_rid()
            m = self.request(lambda: v1.diag_read(rid, off),
                             lambda x: x.type == v1.DIAG_DATA and x.fields["request_id"] == rid, 0.3, 5, others)
            if m is None:
                raise ToolError(f"[{self.name}] DIAG_DATA が来ない（offset {off}）")
            if m.fields["n"] == 0:
                break
            samples.extend(m.fields["samples"])
        return samples

    def wait_state(self, state: int, timeout: float) -> dict | None:
        """STATUS の mcu_state が state になるまで待つ（区間を閉じた後の DISARMED など）"""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            self.pump()
            if self.status is not None and self.status["mcu_state"] == state:
                return self.status
            time.sleep(0.01)
        return self.status

    def keepalive(self) -> None:
        if self.hello is not None and time.monotonic() - self.last_keepalive >= 0.5:
            self.last_keepalive = time.monotonic()
            self.send(v1.hello_req(self.next_rid()))

    def cancel(self) -> None:
        try:
            self.send(v1.diag_start(self.next_rid(), v1.MODE_CANCEL))
        except Exception:
            pass

    def close(self) -> None:
        try:
            self.s.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
def load_axis_config(path: str, axis: int) -> tuple[dict, dict]:
    with open(path, "rb") as f:
        cfg = tomllib.load(f)
    ax = next((a for a in cfg.get("axis", []) if a.get("axis_id") == axis), None)
    if ax is None:
        raise ToolError(f"{path} に axis_id = {axis} が無い")
    return cfg, ax


def send_config(link: Link, cfg: dict, ax: dict, power: bool, others=()) -> dict:
    lim = cfg["limits"]
    tim = cfg["timing"]
    epoch = random.randint(1, 1 << 31)
    fields = dict(soft_min=int(ax["soft_min_count"]), soft_max=int(ax["soft_max_count"]),
                  max_vel=int(lim["max_vel_count_s"]), max_acc=int(lim["max_acc_count_s2"]),
                  fe_limit=int(lim["following_error_limit_count"]), overspeed=int(lim["overspeed_count_s"]),
                  command_timeout_ms=int(tim["mcu_command_timeout_ms"]), stop_timeout_ms=int(tim["mcu_stop_timeout_ms"]),
                  role_flags=v1.ROLE_POWER_CONTROLLER if power else 0)
    link.command(lambda rid: v1.config(rid, epoch, 0, **fields), v1.CONFIG, others)
    return dict(session_epoch=epoch, **fields)


def require_ready(link: Link, need_reference: bool) -> dict:
    st = link.wait_status()
    if st["mcu_state"] != 1:
        raise ToolError(f"[{link.name}] {v1.STATE_NAMES.get(st['mcu_state'], st['mcu_state'])}（DISARMED でない。"
                        f"fault={v1.FAULT_NAMES.get(st['fault_code'], st['fault_code'])}）")
    if need_reference and not st["flags"] & v1.FLAG_POSITION_REFERENCED:
        raise ToolError(f"[{link.name}] POSITION_REFERENCED でない（サーバーから校正してからサーバーを止める）")
    return st


def soft_room(ax: dict, actual: int) -> int:
    """いまの位置からソフトリミットまでの近い方の距離 [count]"""
    return min(actual - int(ax["soft_min_count"]), int(ax["soft_max_count"]) - actual)


def check_min_room(link: "Link", ax: dict, actual: int, min_room: int) -> None:
    room = soft_room(ax, actual)
    if room < min_room:
        raise ToolError(f"[{link.name}] actual={actual} からソフトリミットまで {room} count（--min-room {min_room} 未満）。"
                        "最初は中立から測る（手順書 §2。止まるまでの距離を測ってから --min-room を下げる）")


def wait_power_settled(power: "Link", others, timeout: float, log) -> dict:
    """電源担当軸の区間が SETTLED（主電源 ON 指令から ≥ 5 s）を名乗るまで待つ。他の軸の計測はこの後に始める"""
    want = v1.SESSION_ACTIVE | v1.SESSION_POWER_CMD | v1.SESSION_SETTLED
    t_end = time.monotonic() + timeout
    last = None
    while time.monotonic() < t_end:
        st = power.session_status(others)
        last = st
        if not st["session_flags"] & v1.SESSION_ACTIVE:
            raise ToolError(f"[{power.name}] POWER_HOLD の区間が閉じている（flags=0x{st['session_flags']:x}）")
        if st["session_flags"] & want == want:
            log(f"[{power.name}] 主電源 SETTLED（区間 {st['session_ms']} ms）")
            return st
        power.wait(lambda m: False, 0.2, others)  # 受信を回しながら待つ（keepalive も送る）
    raise ToolError(f"[{power.name}] 主電源が SETTLED にならない（最後の状態 {last}）")


def run_session(target: "Link", power: "Link | None", ax: dict, mode: int, kw: dict, args, log, on_result) -> int:
    """同じ区間（主電源を入れたまま）で計測を args.repeat 回。電源担当でない軸なら、電源担当軸の SETTLED を確かめてから始める。
    不成功（OK 以外）・待機に残らない（SESSION_KEPT なし）・余裕が --min-room を割ったらそこで止める。
    on_result(i, res, samples) が 1 回ごとの保存・表示をする。戻り値: 0 = 全部 OK、2 = 途中で止めた"""
    others = [power] if power is not None else []
    if power is not None:
        power.command(lambda rid: v1.diag_start(rid, v1.MODE_POWER_HOLD, hold_ms=args.power_hold_ms),
                      v1.DIAG_START, [target])
        log(f"[{power.name}] POWER_HOLD {args.power_hold_ms} ms")
        wait_power_settled(power, [target], args.settle_timeout, log)
    actual = target.status["actual_count"] if target.status else None
    for i in range(args.repeat):
        if actual is not None:
            check_min_room(target, ax, actual, args.min_room)
        if power is not None:
            # 期限を延ばし、まだ SETTLED のままかを見る（#1 の USB が途切れて 24 V が落ちていたら始めない）
            power.command(lambda rid: v1.diag_start(rid, v1.MODE_POWER_HOLD, hold_ms=args.power_hold_ms),
                          v1.DIAG_START, [target])
            st = power.session_status([target])
            need = v1.SESSION_ACTIVE | v1.SESSION_POWER_CMD | v1.SESSION_SETTLED
            if st["session_flags"] & need != need:
                raise ToolError(f"[{power.name}] 主電源の区間が SETTLED でない（flags=0x{st['session_flags']:x}）")
        ack = target.command(lambda rid: v1.diag_start(rid, mode, **kw), v1.DIAG_START, others)
        test_rid = ack["request_id"]  # DIAG_RESULT の test_id
        log(f"[{target.name}] DIAG_START #{i + 1} {kw} ack={ack}")
        # 最初の 1 回は区間の電源待ち（≥ 5 s）を含む
        timeout = args.result_timeout + (v1.POWER_SETTLE_MS / 1000.0 if i == 0 else 0.0)
        res_msg = target.wait(lambda x: x.type == v1.DIAG_RESULT and x.fields["test_id"] == test_rid, timeout, others)
        if res_msg is None:
            raise ToolError(f"[{target.name}] DIAG_RESULT が来ない")
        res = res_msg.fields
        log(f"[{target.name}] DIAG_RESULT {res}")
        samples = target.read_samples(res["sample_count"], others)
        on_result(i, res, samples)
        if res["result"] != v1.RESULT_OK:
            log(f"[{target.name}] 結果 {v1.RESULT_NAMES.get(res['result'])}: ここで止める")
            return 2
        if not res["flags"] & v1.RESULT_FLAG_SESSION_KEPT:
            log(f"[{target.name}] 区間が閉じた: ここで止める")
            return 2
        actual = res["start_count"] + res["final_disp"]
    return 0


def run(args) -> int:
    if serial is None:
        raise ToolError("pyserial が無い（~/.platformio/penv/bin/python で動かす）")
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    os.makedirs(args.out_dir, exist_ok=True)
    base0 = os.path.join(args.out_dir, f"{stamp}_ax{args.axis}_{args.mode}_{args.label}")
    logf = open(base0 + ".log", "w", buffering=1)

    def log(msg: str) -> None:
        line = f"{dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]} {msg}"
        logf.write(line + "\n")
        if args.verbose:
            print(line, file=sys.stderr)

    cfg, ax = load_axis_config(args.config, args.axis)
    pc_axis = int(cfg.get("power_controller_axis", 0))
    is_power = pc_axis == args.axis
    expect_dev = ax.get("device_id")
    if expect_dev and expect_dev != "0" and int(expect_dev, 16) != v1.device_id_from_mac(args.mac):
        raise ToolError(f"--mac {args.mac} は設定の axis {args.axis} device_id {expect_dev} と違う")
    if not is_power and not (args.power_port and args.power_mac):
        raise ToolError(f"axis {args.axis} は電源担当（axis {pc_axis}）ではない: --power-port / --power-mac で電源担当軸を指定する"
                        "（24 V が入らないとブレーキも外れず、ENGAGE は無パルスで ENCODER_FAULT になる）")

    if args.mode == "sag":
        kw = dict(t_max_ms=args.t_max, d_max_count=args.d_max)
        mode = v1.MODE_SAG
    else:
        kw = dict(dir_=args.dir, duty_permille=args.duty, d_max_count=args.d_run, v_target_count_s=args.v_target,
                  delay_ms=args.delay)
        mode = v1.MODE_ENGAGE

    target = None
    power = None
    try:
        target = Link(f"ax{args.axis}", args.port, args.mac, log)
        h = target.do_hello(args.axis)
        log(f"[{target.name}] HELLO dev={h['device_id']:016x} boot={h['boot_id']} state={v1.STATE_NAMES.get(h['mcu_state'])}")
        st0 = require_ready(target, need_reference=True)
        log(f"[{target.name}] STATUS actual={st0['actual_count']} flags=0x{st0['flags']:x}")
        check_min_room(target, ax, st0["actual_count"], args.min_room)
        cfg_sent = send_config(target, cfg, ax, is_power)
        if not is_power:
            _, pax = load_axis_config(args.config, pc_axis)
            power = Link(f"ax{pc_axis}", args.power_port, args.power_mac, log)
            ph = power.do_hello(pc_axis)
            log(f"[{power.name}] HELLO dev={ph['device_id']:016x}")
            require_ready(power, need_reference=False)
            send_config(power, cfg, pax, True, [target])

        def on_result(i: int, res: dict, samples: list[dict]) -> None:
            base = base0 if args.repeat == 1 else f"{base0}_r{i + 1}"
            write_run(base, args, h, kw, cfg_sent, st0, res, samples, target)

        rc = run_session(target, power, ax, mode, kw, args, log, on_result)
        close_session(target, power, log)
        return rc
    finally:
        # 区間を閉じる（記録中ならファームは作動後の記録を終えてから閉じる）。ここに来るのが例外・Ctrl-C でも主電源を残さない
        if target is not None:
            target.cancel()
        if power is not None:
            power.cancel()
        if target is not None:
            target.close()
        if power is not None:
            power.close()
        logf.close()


def close_session(target: "Link", power: "Link | None", log) -> None:
    """DIAG_START(CANCEL) で区間を閉じ、DISARMED（主電源 OFF）を確かめる"""
    others = [power] if power is not None else []
    target.command(lambda rid: v1.diag_start(rid, v1.MODE_CANCEL), v1.DIAG_START, others)
    st = target.wait_state(1, 2.0)
    log(f"[{target.name}] 区間を閉じた: {v1.STATE_NAMES.get(st['mcu_state']) if st else '?'}")
    if power is not None:
        power.command(lambda rid: v1.diag_start(rid, v1.MODE_CANCEL), v1.DIAG_START, [target])
        pst = power.wait_state(1, 2.0)
        log(f"[{power.name}] POWER_HOLD を閉じた: {v1.STATE_NAMES.get(pst['mcu_state']) if pst else '?'}")


def write_run(base: str, args, h: dict, kw: dict, cfg_sent: dict, st0: dict, res: dict, samples: list[dict],
              target: "Link") -> None:
    with open(base + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_us", "t_ms", "count", "disp", "pwm_permille", "brake_release", "power_on", "adc_raw"])
        for s in samples:
            w.writerow([s["t_us"], f"{s['t_us'] / 1000:.3f}", s["count"], s["count"] - res["start_count"],
                        s["pwm_permille"], int(bool(s["flags"] & v1.SAMPLE_FLAG_BRAKE_RELEASE)),
                        int(bool(s["flags"] & v1.SAMPLE_FLAG_POWER_ON)), s["adc_raw"]])
    metrics = host_metrics(samples, res)
    named = dict(res)
    named["mode_name"] = v1.MODE_NAMES.get(res["mode"])
    named["result_name"] = v1.RESULT_NAMES.get(res["result"])
    named["stop_reason_name"] = v1.STOP_NAMES.get(res["stop_reason"])
    named["fault_name"] = v1.FAULT_NAMES.get(res["fault_code"])
    warnings = run_warnings(res)
    summary = dict(
        when=os.path.basename(base), label=args.label, axis=args.axis, port=args.port, mac=args.mac,
        device_id=f"{h['device_id']:016x}", boot_id=h["boot_id"], mode=args.mode, params=kw, config=cfg_sent,
        config_file=args.config, status_before=st0, status_now=target.status, firmware_result=named,
        host_metrics=metrics, samples_read=len(samples), fault_events=target.fault_events, warnings=warnings,
    )
    with open(base + ".json", "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(summary_line(args.axis, args.mode, args.label, res, metrics, len(samples)) + f" -> {base}.csv")
    for wmsg in warnings:
        print(f"  WARN {wmsg}")


def run_warnings(res: dict) -> list[str]:
    """最初の実機の回で必ず見る項目（手順書 §2・§6）: 記録の間隔・逸脱・作動後の端・満杯"""
    out = []
    if res["max_sample_gap_us"] > 2000:
        out.append(f"max_sample_gap_us={res['max_sample_gap_us']}（1 ms 記録が崩れている。制御タスクの負荷を疑う。大きい段へ進まない）")
    if res["fault_code"] == 6:
        out.append("CONTROL_OVERRUN（制御周期の逸脱）。大きい段へ進まない")
    if res["flags"] & v1.RESULT_FLAG_POST_MARGIN:
        out.append("作動後に端の余裕を越えた（止まるまでの見積もりより滑った）。区間は閉じた。余裕を見直すまで進まない")
    if res["flags"] & v1.RESULT_FLAG_TRUNCATED:
        out.append("記録が満杯（BUFFER_FULL）")
    if res["stop_reason"] == 13:
        out.append("解除の直前に余裕が足りず解除しなかった（ROOM_LOST）")
    if res["t_engage_us"] != v1.TIME_NONE and res["session_ms_at_release"] < v1.POWER_SETTLE_MS:
        out.append(f"解除時の区間の経過 {res['session_ms_at_release']} ms < {v1.POWER_SETTLE_MS}（ファームの電源待ちが効いていない）")
    return out


def summary_line(axis: int, mode: str, label: str, res: dict, metrics: dict, n: int) -> str:
    te = res["t_engage_us"]
    ts = res["t_settle_us"]
    slide = res["final_disp"] - res["disp_at_engage"] if te != v1.TIME_NONE else None
    return (f"ax{axis} {mode} {label}: {v1.RESULT_NAMES.get(res['result'])} stop={v1.STOP_NAMES.get(res['stop_reason'])} "
            f"start={res['start_count']} engage@{'-' if te == v1.TIME_NONE else f'{te / 1000:.1f}ms'} "
            f"disp@engage={res['disp_at_engage']} final={res['final_disp']} slide={slide} "
            f"v@engage={res['v_at_engage_count_s']} settle={'-' if ts == v1.TIME_NONE else f'{ts / 1000:.1f}ms'} "
            f"rel_rate100ms={metrics.get('released_max_rate_100ms_count_s')} n={n} gap_max={res['max_sample_gap_us']}us "
            f"session@release={res['session_ms_at_release']}ms ref_after={res['referenced_after']} "
            f"kept={int(bool(res['flags'] & v1.RESULT_FLAG_SESSION_KEPT))}")


def info(args) -> int:
    """動かさずに、個体・状態・位置だけを見る"""
    if serial is None:
        raise ToolError("pyserial が無い（~/.platformio/penv/bin/python で動かす）")
    link = Link(f"ax{args.axis}", args.port, args.mac, lambda m: None)
    try:
        h = link.do_hello(args.axis)
        st = link.wait_status()
        ss = link.session_status()
        print(f"ax{args.axis} dev={h['device_id']:016x} boot={h['boot_id']} state={v1.STATE_NAMES.get(st['mcu_state'])} "
              f"fault={v1.FAULT_NAMES.get(st['fault_code'])} actual={st['actual_count']} flags=0x{st['flags']:x} "
              f"(R={int(bool(st['flags'] & v1.FLAG_POSITION_REFERENCED))} C={int(bool(st['flags'] & v1.FLAG_CONFIGURED))}) "
              f"session=0x{ss['session_flags']:x}/{ss['session_ms']}ms")
        return 0
    finally:
        link.close()


def fault_reset(args) -> int:
    """FAULT を解除する（サーバーを HOLDING のまま止めて COMMAND_TIMEOUT になったときなど）。画面の非常停止がラッチ中ならファームが拒否する。
    ENCODER_FAULT の解除は基準を消す（既存の意味）ので、その後はサーバーから校正し直す。"""
    if serial is None:
        raise ToolError("pyserial が無い（~/.platformio/penv/bin/python で動かす）")
    link = Link(f"ax{args.axis}", args.port, args.mac, lambda m: None)
    try:
        link.do_hello(args.axis)
        st = link.wait_status()
        if st["mcu_state"] != 7:
            print(f"ax{args.axis} {v1.STATE_NAMES.get(st['mcu_state'])}（FAULT ではない。何もしない）")
            return 0
        before = v1.FAULT_NAMES.get(st["fault_code"], st["fault_code"])
        link.command(lambda rid: v1.fault_reset(rid), v1.FAULT_RESET)
        st = link.wait_status()
        print(f"ax{args.axis} FAULT({before}) → {v1.STATE_NAMES.get(st['mcu_state'])} actual={st['actual_count']} "
              f"R={int(bool(st['flags'] & v1.FLAG_POSITION_REFERENCED))}")
        return 0
    finally:
        link.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="計測する軸の /dev/cu.usbmodemXXXX")
    ap.add_argument("--mac", required=True, help="そのポートの USB Serial Number（eFuse MAC、例 1C:DB:D4:BA:8D:58）")
    ap.add_argument("--axis", type=int, required=True, help="その個体が選んでいる系統（1〜3）")
    ap.add_argument("--config", default="control-server/config/h3-bringup-01-live-20261102.toml",
                    help="ソフトリミット・保護値・電源担当軸を読む設定（サーバーと同じもの）")
    ap.add_argument("--power-port", help="電源担当軸のポート（計測する軸が電源担当でないとき）")
    ap.add_argument("--power-mac", help="電源担当軸の USB Serial Number")
    ap.add_argument("--power-hold-ms", type=int, default=120000,
                    help="POWER_HOLD の期限（計測ごとに延ばす。終われば中止を送る。≤ 120000）")
    ap.add_argument("--repeat", type=int, default=1, help="同じ区間（主電源を入れたまま）で続けて測る回数。不成功ならそこで止める")
    ap.add_argument("--min-room", type=int, default=2000,
                    help="計測の前に要る、ソフトリミットまでの距離 [count]（最初は中立から。止まるまでの距離を測ってから下げる）")
    ap.add_argument("--settle-timeout", type=float, default=12.0, help="電源担当軸の SETTLED を待つ最長 [s]")
    ap.add_argument("--result-timeout", type=float, default=4.0, help="DIAG_RESULT を待つ最長 [s]（最初の回は +5 s）")
    ap.add_argument("--label", default="run", help="ファイル名に付ける印（位置・荷重など。例 n0_empty）")
    ap.add_argument("--out-dir", default="moter-controler/docs/data/brake_sag")
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="mode", required=True)
    sub.add_parser("info", help="動かさずに個体・状態・位置を表示")
    sub.add_parser("fault-reset", help="FAULT を解除する（動かさない。ENCODER_FAULT の解除は基準を消す）")
    sp = sub.add_parser("sag", help="PWM 0 のままブレーキ解除 → 沈みを記録 → 作動")
    sp.add_argument("--t-max", type=int, required=True, help="解除の最長 [ms]（≤ 500）")
    sp.add_argument("--d-max", type=int, required=True, help="|変位| の上限 [count]（≤ 200）")
    ep = sub.add_parser("engage", help="一定 Duty で走らせて、PWM 0 と同時にブレーキ作動")
    ep.add_argument("--dir", type=int, choices=(-1, 1), required=True, help="+1 = カウント増の向き")
    ep.add_argument("--duty", type=int, required=True, help="|Duty| [permille]（100〜250）")
    ep.add_argument("--d-run", type=int, required=True, help="走行距離の上限 [count]（≤ 600）")
    ep.add_argument("--v-target", type=int, required=True, help="この速さで止める [count/s]（≤ 3000）")
    ep.add_argument("--delay", type=int, default=50, help="解除指令から走り始めるまで [ms]（≤ 200）")
    args = ap.parse_args(argv)
    try:
        if args.mode == "info":
            return info(args)
        if args.mode == "fault-reset":
            return fault_reset(args)
        return run(args)
    except ToolError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("中断（DIAG_START(CANCEL) を送った）", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
