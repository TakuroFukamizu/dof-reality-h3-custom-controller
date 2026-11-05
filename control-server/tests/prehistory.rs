//! prehistory: 軸ごとの STATUS リング（固定容量・上書き）と、ラッチ時の
//! 2 s / 20 Hz の切り出し（Issue #34）。

use std::time::Duration;

use h3cs::prehistory::{StatusRing, TraceSample, INTERVAL, RING_CAPACITY, WINDOW};
use h3cs::protocol::Status;
use h3cs::types::{AxisId, FaultCode, McuState};
use tokio::time::Instant;

fn status(actual: i32, fe: i32) -> Status {
    Status {
        boot_id: 1,
        mcu_time_us: 0,
        mcu_state: McuState::Active as u8,
        selected_axis: 1,
        fault_code: FaultCode::None,
        flags: 0b11,
        pwm_permille: 120,
        actual_count: actual,
        target_count: actual - fe,
        velocity_count_s: 0,
        following_error_count: fe,
        last_applied_frame: 0,
        last_applied_seq: 0,
        last_rx_seq: 0,
        current_ma: 0,
        rx_error_count: 0,
        stale_drop_count: 0,
    }
}

fn ax1() -> AxisId {
    AxisId::new(1).unwrap()
}

/// `n` 件を `period` ごとに積む（k 件目の actual_count = k）。最後の時刻を返す。
fn fill(ring: &mut StatusRing, t0: Instant, n: usize, period: Duration) -> Instant {
    let mut t = t0;
    for k in 0..n {
        t = t0 + period * k as u32;
        ring.push(TraceSample::new(
            t,
            &status(k as i32, 0),
            Some(k as i32 + 5),
            None,
        ));
    }
    t
}

#[test]
fn ring_is_fixed_capacity_and_overwrites_oldest() {
    let mut ring = StatusRing::new(4);
    assert!(ring.is_empty());
    assert_eq!(ring.capacity(), 4);
    let t0 = Instant::now();
    fill(&mut ring, t0, 3, Duration::from_millis(20));
    let got: Vec<i32> = ring.iter_newest_first().map(|s| s.actual_count).collect();
    assert_eq!(got, vec![2, 1, 0], "not yet full");
    fill(&mut ring, t0, 10, Duration::from_millis(20));
    assert_eq!(ring.len(), 4);
    let got: Vec<i32> = ring.iter_newest_first().map(|s| s.actual_count).collect();
    assert_eq!(
        got,
        vec![9, 8, 7, 6],
        "wrapped: newest first, oldest overwritten"
    );
}

#[test]
fn capacity_zero_is_one_and_default_is_ring_capacity() {
    let mut ring = StatusRing::new(0);
    assert_eq!(ring.capacity(), 1);
    let t0 = Instant::now();
    fill(&mut ring, t0, 5, Duration::from_millis(20));
    let got: Vec<i32> = ring.iter_newest_first().map(|s| s.actual_count).collect();
    assert_eq!(got, vec![4]);
    assert_eq!(StatusRing::default().capacity(), RING_CAPACITY);
}

#[test]
fn empty_ring_gives_empty_prehistory() {
    let ring = StatusRing::default();
    let p = ring.prehistory(ax1(), Instant::now());
    assert_eq!(p.axis, ax1());
    assert!(p.is_empty());
    let v = serde_json::to_value(&p).unwrap();
    assert_eq!(v["t_ms"], serde_json::json!([]));
    assert_eq!(v["actual_count"], serde_json::json!([]));
}

/// 50 Hz で 5 s 積んで、ラッチ時刻から 2 s を 50 ms 枠ごとに 1 件（各枠の最新）。
#[test]
fn prehistory_keeps_last_window_downsampled_oldest_first() {
    let mut ring = StatusRing::default();
    let t0 = Instant::now();
    let last = fill(&mut ring, t0, 250, Duration::from_millis(20));
    let p = ring.prehistory(ax1(), last);
    let max = (WINDOW.as_millis() / INTERVAL.as_millis()) as usize + 1;
    assert!(p.len() <= max, "{} > {max}", p.len());
    assert!(p.len() >= max - 2, "{} samples", p.len());
    // 列はすべて同じ長さ。
    for n in [
        p.mcu_state.len(),
        p.fault_code.len(),
        p.flags.len(),
        p.actual_count.len(),
        p.target_count.len(),
        p.server_target_count.len(),
        p.requested_count.len(),
        p.following_error_count.len(),
        p.velocity_count_s.len(),
        p.pwm_permille.len(),
    ] {
        assert_eq!(n, p.len());
    }
    // 古い順、窓の中、最新はラッチ時刻の STATUS そのもの。
    assert!(p.t_ms.windows(2).all(|w| w[0] < w[1]), "{:?}", p.t_ms);
    assert!(p.t_ms[0] >= -(WINDOW.as_millis() as i64));
    assert_eq!(*p.t_ms.last().unwrap(), 0);
    assert_eq!(*p.actual_count.last().unwrap(), 249);
    assert_eq!(*p.server_target_count.last().unwrap(), Some(254));
    assert_eq!(*p.requested_count.last().unwrap(), None);
    assert_eq!(*p.mcu_state.last().unwrap(), Some(McuState::Active));
    // 間隔は 50 ms 枠（20 ms 周期なので 40 か 60 ms）。
    for w in p.t_ms.windows(2) {
        let d = w[1] - w[0];
        assert!((40..=60).contains(&d), "spacing {d} in {:?}", p.t_ms);
    }
}

/// ラッチ時刻より後の STATUS（同じ tick で後から来たもの）は入れない。
#[test]
fn prehistory_ignores_samples_after_latch() {
    let mut ring = StatusRing::default();
    let t0 = Instant::now();
    fill(&mut ring, t0, 100, Duration::from_millis(20));
    let latch = t0 + Duration::from_millis(20 * 50);
    let p = ring.prehistory(ax1(), latch);
    assert_eq!(*p.actual_count.last().unwrap(), 50);
    assert_eq!(*p.t_ms.last().unwrap(), 0);
    // 1 s しか前が無い軸は 1 s 分だけ。
    assert!(p.t_ms[0] >= -1000);
}

/// 短い窓（接続直後の FAULT など）でも落ちず、ある分だけ返す。
#[test]
fn prehistory_with_few_samples() {
    let mut ring = StatusRing::default();
    let t0 = Instant::now();
    let last = fill(&mut ring, t0, 3, Duration::from_millis(100));
    let p = ring.prehistory(ax1(), last + Duration::from_millis(5));
    assert_eq!(p.t_ms, vec![-205, -105, -5]);
    assert_eq!(p.actual_count, vec![0, 1, 2]);
}
