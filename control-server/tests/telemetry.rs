//! telemetry: percentiles, ring overwrite, drop counting, JSONL recorder.

use std::time::Duration;

use h3cs::telemetry::{EventLog, LatencyStats, Recorder, Telemetry};
use serde_json::json;
use tokio::time::Instant;

#[test]
fn percentiles_use_nearest_rank() {
    let mut s = LatencyStats::new(1024);
    // Insert out of order.
    for v in (1..=100).rev() {
        s.record(v as f64);
    }
    assert_eq!(s.percentile(50.0), Some(50.0));
    assert_eq!(s.percentile(95.0), Some(95.0));
    assert_eq!(s.percentile(99.0), Some(99.0));
    assert_eq!(s.percentile(100.0), Some(100.0));
    assert_eq!(s.percentile(0.0), Some(1.0));
    assert_eq!(s.percentile(f64::NAN), None);
    let sum = s.summary();
    assert_eq!(
        (sum.count, sum.p50, sum.p95, sum.p99, sum.max),
        (100, Some(50.0), Some(95.0), Some(99.0), Some(100.0))
    );
}

#[test]
fn single_sample_and_empty() {
    let mut s = LatencyStats::default();
    assert_eq!(s.capacity(), LatencyStats::DEFAULT_CAPACITY);
    assert!(s.is_empty());
    let sum = s.summary();
    assert_eq!(
        (sum.count, sum.p50, sum.max, sum.max_all),
        (0, None, None, None)
    );
    assert_eq!(
        serde_json::to_value(sum).unwrap(),
        json!({"count":0,"p50":null,"p95":null,"p99":null,"max":null,"max_all":null})
    );
    s.record_duration(Duration::from_micros(10_500));
    let sum = s.summary();
    assert_eq!(
        (sum.p50, sum.p99, sum.max),
        (Some(10.5), Some(10.5), Some(10.5))
    );
}

#[test]
fn ring_overwrites_oldest_but_keeps_all_time_max_and_count() {
    let mut s = LatencyStats::new(4);
    for v in [100.0, 1.0, 2.0, 3.0, 4.0, 5.0] {
        s.record(v);
    }
    assert_eq!(s.len(), 4);
    assert_eq!(s.count(), 6);
    let sum = s.summary();
    // Window is [2, 3, 4, 5].
    assert_eq!(sum.p50, Some(3.0));
    assert_eq!(sum.max, Some(5.0));
    assert_eq!(sum.max_all, Some(100.0));
}

#[test]
fn non_finite_samples_are_ignored_and_zero_capacity_is_one() {
    let mut s = LatencyStats::new(0);
    assert_eq!(s.capacity(), 1);
    s.record(f64::NAN);
    s.record(f64::INFINITY);
    assert!(s.is_empty());
    s.record(1.0);
    s.record(2.0);
    assert_eq!(s.len(), 1);
    assert_eq!(s.percentile(50.0), Some(2.0));
}

#[tokio::test(start_paused = true)]
async fn event_log_is_bounded_and_counts_drops() {
    let start = Instant::now();
    let mut log = EventLog::new(start, 3);
    for i in 0..5u64 {
        tokio::time::advance(Duration::from_millis(10)).await;
        let id = log.push(Instant::now(), "rig_state", json!({"i": i}));
        assert_eq!(id, i + 1);
    }
    assert_eq!(log.len(), 3);
    assert_eq!(log.dropped(), 2);
    let ids: Vec<u64> = log.iter().map(|e| e.id).collect();
    assert_eq!(ids, vec![3, 4, 5]);
    let recent = log.recent(2);
    assert_eq!(recent.iter().map(|e| e.id).collect::<Vec<_>>(), vec![4, 5]);
    assert_eq!(recent[1].t_ms, 50.0);
    assert_eq!(log.recent(10).len(), 3);
    assert_eq!(
        serde_json::to_value(&recent[1]).unwrap(),
        json!({"id":5,"t_ms":50.0,"kind":"rig_state","detail":{"i":4}})
    );
    // An instant before start does not panic: clamps to 0.
    let mut early = EventLog::new(Instant::now() + Duration::from_secs(1), 0);
    early.push(Instant::now(), "x", json!(null));
    assert_eq!(early.iter().next().unwrap().t_ms, 0.0);
}

#[tokio::test(start_paused = true)]
async fn telemetry_summary_shape() {
    let mut t = Telemetry::new(Instant::now());
    t.tick_count = 3;
    t.tick_dt_ms.record(10.0);
    let v = serde_json::to_value(t.summary(None)).unwrap();
    for key in [
        "tick_count",
        "tick_overruns",
        "tick_dt_ms",
        "pose_rx_to_tick_ms",
        "status_age_ms",
        "frame_skew",
        "log_dropped",
        "recorder_dropped",
    ] {
        assert!(v.get(key).is_some(), "missing {key}");
    }
    assert_eq!(v["tick_count"], 3);
    assert_eq!(v["tick_dt_ms"]["p50"], 10.0);
}

fn temp_path(tag: &str) -> std::path::PathBuf {
    use std::sync::atomic::{AtomicU32, Ordering};
    static N: AtomicU32 = AtomicU32::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "h3cs-telemetry-{tag}-{}-{n}-{}.jsonl",
        std::process::id(),
        rand::random::<u64>()
    ))
}

#[tokio::test]
async fn recorder_writes_jsonl_to_a_file() {
    let path = temp_path("write");
    let rec = Recorder::open(&path, 64).await.unwrap();
    assert!(rec.is_enabled());
    for i in 0..10 {
        assert!(rec.record(&json!({"tick": i, "target": [1, 2, 3]})));
    }
    assert_eq!(rec.dropped(), 0);
    rec.close().await.unwrap();
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len(), 10);
    for (i, l) in lines.iter().enumerate() {
        let v: serde_json::Value = serde_json::from_str(l).unwrap();
        assert_eq!(v["tick"], i);
    }
    assert!(text.ends_with('\n'));
}

#[tokio::test]
async fn recorder_appends_to_existing_file() {
    let path = temp_path("append");
    std::fs::write(&path, "{\"old\":true}\n").unwrap();
    let rec = Recorder::open(&path, 4).await.unwrap();
    assert!(rec.record(&json!({"new": true})));
    rec.close().await.unwrap();
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    assert_eq!(text, "{\"old\":true}\n{\"new\":true}\n");
}

#[tokio::test]
async fn recorder_drops_when_queue_is_full_without_blocking() {
    // The far end is never read, so the writer stalls once the 1-byte pipe fills.
    let (w, _r) = tokio::io::duplex(1);
    let rec = Recorder::spawn(w, 4);
    // No await between sends: the writer task cannot run (current-thread runtime).
    let ok = (0..100).filter(|i| rec.record(&json!({"i": i}))).count();
    assert_eq!(ok, 4);
    assert_eq!(rec.dropped(), 96);
}

#[tokio::test]
async fn recorder_counts_drops_after_writer_fails() {
    let (w, r) = tokio::io::duplex(64);
    drop(r);
    let rec = Recorder::spawn(w, 8);
    assert!(rec.record(&json!({"first": 1})));
    // Let the writer hit the broken pipe and exit.
    for _ in 0..100 {
        tokio::task::yield_now().await;
        if !rec.record(&json!({"probe": 1})) {
            break;
        }
    }
    assert!(rec.dropped() >= 1);
    assert!(!rec.record(&json!({"after": 1})));
    assert!(rec.close().await.is_err());
}

#[test]
fn disabled_recorder_records_nothing() {
    let rec = Recorder::disabled();
    assert!(!rec.is_enabled());
    assert!(!rec.record(&json!({"x": 1})));
    assert_eq!(rec.dropped(), 0);
}

#[test]
fn event_log_recent_after_and_upstream_drops() {
    let t0 = Instant::now();
    let mut log = EventLog::new(t0, 4);
    for k in 0..6 {
        log.push(t0, "e", serde_json::json!(k));
    }
    // Ring of 4 holds ids 3..=6.
    let ids = |v: Vec<h3cs::telemetry::LogEvent>| v.into_iter().map(|e| e.id).collect::<Vec<_>>();
    assert_eq!(ids(log.recent_after(0, 10)), [3, 4, 5, 6]);
    assert_eq!(ids(log.recent_after(4, 10)), [5, 6]);
    assert_eq!(ids(log.recent_after(2, 1)), [6]);
    assert!(log.recent_after(6, 10).is_empty());
    assert_eq!(log.next_id(), 7);
    assert_eq!(log.dropped(), 2);
    log.add_dropped(3);
    assert_eq!(log.dropped(), 5);
}
