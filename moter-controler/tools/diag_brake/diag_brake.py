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
  3. STATUS で DISARMED・POSITION_REFERENCED・異常なしを確かめ、設定ファイルの値で CONFIG を送る
  4. 電源担当でない軸なら、電源担当軸（--power-port / --power-mac）に POWER_HOLD を送り、計測の間 HELLO_REQ で生かしておく
  5. DIAG_START → DIAG_RESULT を待つ → DIAG_READ で記録を全部読む → CSV と JSON を書き、1 行の要約を出す
  6. 途中で Ctrl-C / 例外なら DIAG_START(CANCEL) を送ってから閉じる（ファームは中止でブレーキ作動・PWM 0）
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


def run(args) -> int:
    if serial is None:
        raise ToolError("pyserial が無い（~/.platformio/penv/bin/python で動かす）")
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    os.makedirs(args.out_dir, exist_ok=True)
    base = os.path.join(args.out_dir, f"{stamp}_ax{args.axis}_{args.mode}_{args.label}")
    logf = open(base + ".log", "w", buffering=1)

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

    target = None
    power = None
    started = False
    power_started = False
    try:
        target = Link(f"ax{args.axis}", args.port, args.mac, log)
        h = target.do_hello(args.axis)
        log(f"[{target.name}] HELLO dev={h['device_id']:016x} boot={h['boot_id']} state={v1.STATE_NAMES.get(h['mcu_state'])}")
        st0 = require_ready(target, need_reference=True)
        log(f"[{target.name}] STATUS actual={st0['actual_count']} flags=0x{st0['flags']:x}")
        cfg_sent = send_config(target, cfg, ax, is_power)
        others: list[Link] = []
        if not is_power:
            _, pax = load_axis_config(args.config, pc_axis)
            power = Link(f"ax{pc_axis}", args.power_port, args.power_mac, log)
            ph = power.do_hello(pc_axis)
            log(f"[{power.name}] HELLO dev={ph['device_id']:016x}")
            require_ready(power, need_reference=False)
            send_config(power, cfg, pax, True, [target])
            power.command(lambda rid: v1.diag_start(rid, v1.MODE_POWER_HOLD, hold_ms=args.power_hold_ms),
                          v1.DIAG_START, [target])
            power_started = True
            log(f"[{power.name}] POWER_HOLD {args.power_hold_ms} ms")
            others = [power]

        if args.mode == "sag":
            kw = dict(t_max_ms=args.t_max, d_max_count=args.d_max)
            mode = v1.MODE_SAG
        else:
            kw = dict(dir_=args.dir, duty_permille=args.duty, d_max_count=args.d_run, v_target_count_s=args.v_target,
                      delay_ms=args.delay)
            mode = v1.MODE_ENGAGE
        ack = target.command(lambda rid: v1.diag_start(rid, mode, **kw), v1.DIAG_START, others)
        test_rid = ack["request_id"]  # DIAG_RESULT の test_id
        started = True
        log(f"[{target.name}] DIAG_START {args.mode} {kw} ack={ack}")
        res_msg = target.wait(lambda x: x.type == v1.DIAG_RESULT and x.fields["test_id"] == test_rid, 8.0, others)
        if res_msg is None:
            raise ToolError(f"[{target.name}] DIAG_RESULT が来ない")
        started = False
        res = res_msg.fields
        log(f"[{target.name}] DIAG_RESULT {res}")
        if power_started:
            power.command(lambda rid: v1.diag_start(rid, v1.MODE_CANCEL), v1.DIAG_START, [target])
            power_started = False

        samples: list[dict] = []
        while len(samples) < res["sample_count"]:
            off = len(samples)
            rid = target.next_rid()
            m = target.request(lambda: v1.diag_read(rid, off),
                               lambda x: x.type == v1.DIAG_DATA and x.fields["request_id"] == rid, 0.3, 5)
            if m is None:
                raise ToolError(f"[{target.name}] DIAG_DATA が来ない（offset {off}）")
            if m.fields["n"] == 0:
                break
            samples.extend(m.fields["samples"])
        st1 = target.wait_status()

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
        summary = dict(
            when=stamp, label=args.label, axis=args.axis, port=args.port, mac=args.mac, device_id=f"{h['device_id']:016x}",
            boot_id=h["boot_id"], mode=args.mode, params=kw, config=cfg_sent, config_file=args.config,
            status_before=st0, status_after=st1, firmware_result=named, host_metrics=metrics,
            samples_read=len(samples), fault_events=target.fault_events,
        )
        with open(base + ".json", "w") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        te = res["t_engage_us"]
        ts = res["t_settle_us"]
        print(f"ax{args.axis} {args.mode} {args.label}: {named['result_name']} stop={named['stop_reason_name']} "
              f"start={res['start_count']} engage@{'-' if te == v1.TIME_NONE else f'{te / 1000:.1f}ms'} "
              f"disp@engage={res['disp_at_engage']} final={res['final_disp']} v@engage={res['v_at_engage_count_s']} "
              f"settle={'-' if ts == v1.TIME_NONE else f'{ts / 1000:.1f}ms'} "
              f"rel_rate100ms={metrics.get('released_max_rate_100ms_count_s')} n={len(samples)} gap_max={res['max_sample_gap_us']}us "
              f"ref_after={res['referenced_after']} state_after={v1.STATE_NAMES.get(st1['mcu_state'])} -> {base}.csv")
        return 0 if res["result"] == 0 else 2
    finally:
        if target is not None and started:
            target.cancel()
            log(f"[{target.name}] DIAG_START(CANCEL) を送った（中断）")
        if power is not None and power_started:
            power.cancel()
            log(f"[{power.name}] POWER_HOLD を中止した")
        if target is not None:
            target.close()
        if power is not None:
            power.close()
        logf.close()


def info(args) -> int:
    """動かさずに、個体・状態・位置だけを見る"""
    if serial is None:
        raise ToolError("pyserial が無い（~/.platformio/penv/bin/python で動かす）")
    link = Link(f"ax{args.axis}", args.port, args.mac, lambda m: None)
    try:
        h = link.do_hello(args.axis)
        st = link.wait_status()
        print(f"ax{args.axis} dev={h['device_id']:016x} boot={h['boot_id']} state={v1.STATE_NAMES.get(st['mcu_state'])} "
              f"fault={v1.FAULT_NAMES.get(st['fault_code'])} actual={st['actual_count']} flags=0x{st['flags']:x} "
              f"(R={int(bool(st['flags'] & v1.FLAG_POSITION_REFERENCED))} C={int(bool(st['flags'] & v1.FLAG_CONFIGURED))})")
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
    ap.add_argument("--power-hold-ms", type=int, default=30000, help="POWER_HOLD の最長（終われば中止を送る）")
    ap.add_argument("--label", default="run", help="ファイル名に付ける印（位置・荷重など。例 n0_empty）")
    ap.add_argument("--out-dir", default="moter-controler/docs/data/brake_sag")
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="mode", required=True)
    sub.add_parser("info", help="動かさずに個体・状態・位置を表示")
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
        return run(args)
    except ToolError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("中断（DIAG_START(CANCEL) を送った）", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
