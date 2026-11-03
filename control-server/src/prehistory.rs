//! FAULT 直前の経過（Issue #34）: 軸ごとの STATUS の有界リングと、ラッチ時に
//! 切り出す間引き済みの記録。
//!
//! - [`StatusRing`] は固定容量（[`RING_CAPACITY`]）のリング。生成時に 1 回だけ
//!   確保し、`push` は既存の枠へ上書きするだけ（STATUS ごとの確保なし）。
//! - [`StatusRing::prehistory`] はラッチ時刻 `now` から [`WINDOW`]（2 s）さかのぼり、
//!   [`INTERVAL`]（50 ms = 20 Hz）の時間枠ごとに最も新しい 1 件を残して古い順に
//!   並べる。1 軸あたり最大 `WINDOW / INTERVAL + 1` 件。
//! - 出力は列指向（[`AxisPrehistory`]: 項目ごとの配列、同じ添字が同じ STATUS）。
//!   行オブジェクトにすると項目名の繰り返しで 4 倍ほどに膨らむため
//!   （`GET /v1/status` の `fault_history[]` とログに 1 回載る）。

use std::time::Duration;

use serde::Serialize;
use tokio::time::Instant;

use crate::protocol::Status;
use crate::types::{AxisId, Counts, FaultCode, McuState, Pose, RigState};

/// ラッチ時刻からさかのぼる長さ。
pub const WINDOW: Duration = Duration::from_millis(2000);
/// 間引きの時間枠（20 Hz）。
pub const INTERVAL: Duration = Duration::from_millis(50);
/// 軸ごとのリングの容量: STATUS 50 Hz（20 ms）＋状態変化時の即時送信でも 2 s 分
/// （128 Hz まで）を保つ。
pub const RING_CAPACITY: usize = 256;

/// リングの 1 件（STATUS 1 通＋その時点のサーバー側の指令）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceSample {
    /// PC が STATUS を復号した時刻（`LinkEvent::Status::received_at`）。
    pub at: Instant,
    pub mcu_state: u8,
    pub fault_code: FaultCode,
    pub flags: u16,
    pub actual_count: Counts,
    /// MCU のセットポイント（STATUS.target_count）。
    pub target_count: Counts,
    pub following_error_count: i32,
    pub velocity_count_s: i32,
    pub pwm_permille: i16,
    /// 直近の SET_POSITION の target_count（planner の出力をソフトリミットで切った値）。
    /// armed 中だけ。
    pub server_target_count: Option<Counts>,
    /// 直近の姿勢から求めた目標（planner に入れる前、ACTIVE（game）中だけ）。
    pub requested_count: Option<Counts>,
}

impl TraceSample {
    pub fn new(
        at: Instant,
        s: &Status,
        server_target_count: Option<Counts>,
        requested_count: Option<Counts>,
    ) -> Self {
        Self {
            at,
            mcu_state: s.mcu_state,
            fault_code: s.fault_code,
            flags: s.flags,
            actual_count: s.actual_count,
            target_count: s.target_count,
            following_error_count: s.following_error_count,
            velocity_count_s: s.velocity_count_s,
            pwm_permille: s.pwm_permille,
            server_target_count,
            requested_count,
        }
    }
}

/// 固定容量のリング（古いものから上書き）。
#[derive(Debug, Clone)]
pub struct StatusRing {
    buf: Vec<TraceSample>,
    cap: usize,
    /// 次に書く位置（満杯のときは最古の位置でもある）。
    next: usize,
}

impl Default for StatusRing {
    fn default() -> Self {
        Self::new(RING_CAPACITY)
    }
}

impl StatusRing {
    /// `capacity` 0 は 1 として扱う。確保はここで 1 回だけ。
    pub fn new(capacity: usize) -> Self {
        let cap = capacity.max(1);
        Self {
            buf: Vec::with_capacity(cap),
            cap,
            next: 0,
        }
    }

    pub fn capacity(&self) -> usize {
        self.cap
    }

    pub fn len(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    pub fn push(&mut self, s: TraceSample) {
        if self.buf.len() < self.cap {
            self.buf.push(s);
        } else {
            self.buf[self.next] = s;
        }
        self.next = (self.next + 1) % self.cap;
    }

    /// 新しい順。
    pub fn iter_newest_first(&self) -> impl Iterator<Item = &TraceSample> {
        let n = self.buf.len();
        let newest = (self.next + self.cap - 1) % self.cap;
        // 満杯でないときは next == n なので同じ式で 0..n に収まる。
        (0..n).map(move |k| &self.buf[(newest + self.cap - k) % self.cap])
    }

    /// `now` から [`WINDOW`] さかのぼった範囲を [`INTERVAL`] ごとに間引いた記録
    /// （古い順）。`now` より後の STATUS は含めない。
    pub fn prehistory(&self, axis: AxisId, now: Instant) -> AxisPrehistory {
        let mut out = AxisPrehistory::empty(axis);
        let mut picked: Vec<(i64, &TraceSample)> = Vec::new();
        let mut last_bucket: Option<u128> = None;
        for s in self.iter_newest_first() {
            let Some(age) = now.checked_duration_since(s.at) else {
                continue;
            };
            if age > WINDOW {
                break;
            }
            let bucket = age.as_millis() / INTERVAL.as_millis();
            if last_bucket == Some(bucket) {
                continue;
            }
            last_bucket = Some(bucket);
            picked.push((-(age.as_millis() as i64), s));
        }
        for (t_ms, s) in picked.into_iter().rev() {
            out.t_ms.push(t_ms);
            out.mcu_state.push(McuState::from_u8(s.mcu_state));
            out.fault_code.push(s.fault_code);
            out.flags.push(s.flags);
            out.actual_count.push(s.actual_count);
            out.target_count.push(s.target_count);
            out.server_target_count.push(s.server_target_count);
            out.requested_count.push(s.requested_count);
            out.following_error_count.push(s.following_error_count);
            out.velocity_count_s.push(s.velocity_count_s);
            out.pwm_permille.push(s.pwm_permille);
        }
        out
    }
}

/// 1 軸ぶんの直前の経過（列指向。各配列は同じ長さで、同じ添字が同じ STATUS）。
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AxisPrehistory {
    pub axis: AxisId,
    /// ラッチ時刻からの ms（負、古い順）。PC の受信時刻。
    pub t_ms: Vec<i64>,
    /// 未知の値は null。
    pub mcu_state: Vec<Option<McuState>>,
    pub fault_code: Vec<FaultCode>,
    /// STATUS.flags の生の値（bit0 POWER_ON_CMD、bit1 BRAKE_RELEASE_CMD、
    /// bit2 POSITION_REFERENCED、bit3 CONFIGURED、bit4 POWER_CONTROLLER、
    /// bit5 CURRENT_VALID、bit6 CLOCK_SYNCED）。
    pub flags: Vec<u16>,
    pub actual_count: Vec<Counts>,
    /// MCU のセットポイント（STATUS.target_count）。
    pub target_count: Vec<Counts>,
    /// サーバーが送った SET_POSITION の target_count（armed 中以外は null）。
    pub server_target_count: Vec<Option<Counts>>,
    /// 姿勢から求めた planner 前の目標（ACTIVE（game）中以外は null）。
    pub requested_count: Vec<Option<Counts>>,
    pub following_error_count: Vec<i32>,
    pub velocity_count_s: Vec<i32>,
    pub pwm_permille: Vec<i16>,
}

impl AxisPrehistory {
    fn empty(axis: AxisId) -> Self {
        Self {
            axis,
            t_ms: Vec::new(),
            mcu_state: Vec::new(),
            fault_code: Vec::new(),
            flags: Vec::new(),
            actual_count: Vec::new(),
            target_count: Vec::new(),
            server_target_count: Vec::new(),
            requested_count: Vec::new(),
            following_error_count: Vec::new(),
            velocity_count_s: Vec::new(),
            pwm_permille: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.t_ms.len()
    }

    pub fn is_empty(&self) -> bool {
        self.t_ms.is_empty()
    }
}

/// FAULT をラッチした時点の全軸の直前の経過（`FaultRecord::prehistory`）。
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FaultPrehistory {
    pub window_ms: u64,
    pub interval_ms: u64,
    /// ラッチ直前のリグ状態。
    pub rig_state_before: RigState,
    /// ラッチ時点でゲームが指令していた姿勢と gain（無ければ null）。
    pub requested_pose: Option<Pose>,
    pub effective_gain: Option<f64>,
    /// 軸 1..3 の順。STATUS が 1 件も無い軸は配列が空。
    pub axes: Vec<AxisPrehistory>,
}

/// FAULT_EVENT の中身（`FaultRecord::fault_event`）。
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct FaultEventRecord {
    pub axis: AxisId,
    pub boot_id: u32,
    pub mcu_time_us: u64,
    pub fault_code: FaultCode,
    /// 遷移直前の MCU 状態（未知の値は null、生の値は `mcu_state_before_raw`）。
    pub mcu_state_before: Option<McuState>,
    pub mcu_state_before_raw: u8,
    pub actual_count: Counts,
}
