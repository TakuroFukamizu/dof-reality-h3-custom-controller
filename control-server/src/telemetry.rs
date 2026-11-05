//! Telemetry (DESIGN.md §1, §7.2): latency statistics over a fixed window,
//! a bounded event log, and an optional non-blocking JSONL recorder.
//! Nothing here may block or slow the control loop: rings overwrite, the
//! recorder drops (and counts) when its queue is full.
//!
//! The event log is shared with the API (`GET /v1/events`) behind a mutex.
//! Both sides hold it only to push one event or copy at most
//! [`EventLog::DEFAULT_CAPACITY`] events, so the control loop never waits long.

use std::collections::VecDeque;
use std::io;
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use serde::Serialize;
use tokio::io::{AsyncWrite, AsyncWriteExt, BufWriter};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::Instant;

// ------------------------------------------------------------------ latency

/// Recent samples in a fixed-capacity ring; percentiles over that window.
/// Unit is the caller's (ms for latencies, frames for skew).
#[derive(Debug, Clone)]
pub struct LatencyStats {
    buf: Vec<f64>,
    cap: usize,
    next: usize,
    count: u64,
    max_all: Option<f64>,
}

/// Serializable summary; `None` (null) when there are no samples.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct LatencySummary {
    /// Samples recorded since start (not just the window).
    pub count: u64,
    pub p50: Option<f64>,
    pub p95: Option<f64>,
    pub p99: Option<f64>,
    /// Max over the window.
    pub max: Option<f64>,
    /// Max since start.
    pub max_all: Option<f64>,
}

impl Default for LatencyStats {
    fn default() -> Self {
        Self::new(Self::DEFAULT_CAPACITY)
    }
}

impl LatencyStats {
    pub const DEFAULT_CAPACITY: usize = 1024;

    /// `capacity` 0 is treated as 1.
    pub fn new(capacity: usize) -> Self {
        let cap = capacity.max(1);
        Self {
            buf: Vec::with_capacity(cap),
            cap,
            next: 0,
            count: 0,
            max_all: None,
        }
    }

    /// Record one sample; non-finite values are ignored.
    pub fn record(&mut self, v: f64) {
        if !v.is_finite() {
            return;
        }
        if self.buf.len() < self.cap {
            self.buf.push(v);
        } else if let Some(slot) = self.buf.get_mut(self.next) {
            *slot = v;
        }
        self.next = (self.next + 1) % self.cap;
        self.count = self.count.saturating_add(1);
        self.max_all = Some(self.max_all.map_or(v, |m| m.max(v)));
    }

    /// Record a duration in milliseconds.
    pub fn record_duration(&mut self, d: Duration) {
        self.record(d.as_secs_f64() * 1000.0);
    }

    /// Samples currently in the window.
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    pub fn capacity(&self) -> usize {
        self.cap
    }

    /// Nearest-rank percentile over the window, `p` in 0..=100.
    pub fn percentile(&self, p: f64) -> Option<f64> {
        let sorted = self.sorted();
        nearest_rank(&sorted, p)
    }

    pub fn summary(&self) -> LatencySummary {
        let sorted = self.sorted();
        LatencySummary {
            count: self.count,
            p50: nearest_rank(&sorted, 50.0),
            p95: nearest_rank(&sorted, 95.0),
            p99: nearest_rank(&sorted, 99.0),
            max: sorted.last().copied(),
            max_all: self.max_all,
        }
    }

    fn sorted(&self) -> Vec<f64> {
        let mut v = self.buf.clone();
        v.sort_by(f64::total_cmp);
        v
    }
}

fn nearest_rank(sorted: &[f64], p: f64) -> Option<f64> {
    if sorted.is_empty() || !p.is_finite() {
        return None;
    }
    let n = sorted.len();
    let rank = ((p.clamp(0.0, 100.0) / 100.0) * n as f64).ceil() as usize;
    sorted.get(rank.clamp(1, n) - 1).copied()
}

// ------------------------------------------------------------------ events

/// One logged event. `t_ms` is relative to the log's start (server start).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct LogEvent {
    /// Monotonic per log, from 1; gaps in a dump mean overwritten events.
    pub id: u64,
    pub t_ms: f64,
    pub kind: String,
    pub detail: serde_json::Value,
}

/// Bounded ring of recent events; overwriting the oldest counts as a drop.
#[derive(Debug, Clone)]
pub struct EventLog {
    start: Instant,
    cap: usize,
    events: VecDeque<LogEvent>,
    next_id: u64,
    dropped: u64,
}

impl EventLog {
    pub const DEFAULT_CAPACITY: usize = 512;

    /// `capacity` 0 is treated as 1.
    pub fn new(start: Instant, capacity: usize) -> Self {
        let cap = capacity.max(1);
        Self {
            start,
            cap,
            events: VecDeque::with_capacity(cap),
            next_id: 1,
            dropped: 0,
        }
    }

    /// Append an event; returns its id.
    pub fn push(
        &mut self,
        now: Instant,
        kind: impl Into<String>,
        detail: serde_json::Value,
    ) -> u64 {
        if self.events.len() >= self.cap {
            self.events.pop_front();
            self.dropped = self.dropped.saturating_add(1);
        }
        let id = self.next_id;
        self.events.push_back(LogEvent {
            id,
            t_ms: ms_since(self.start, now),
            kind: kind.into(),
            detail,
        });
        self.next_id = self.next_id.saturating_add(1);
        id
    }

    /// Oldest first.
    pub fn iter(&self) -> impl Iterator<Item = &LogEvent> {
        self.events.iter()
    }

    /// The newest `n` events, oldest first.
    pub fn recent(&self, n: usize) -> Vec<LogEvent> {
        let skip = self.events.len().saturating_sub(n);
        self.events.iter().skip(skip).cloned().collect()
    }

    /// The newest `n` events with `id > after`, oldest first (polling: pass
    /// the last id seen).
    pub fn recent_after(&self, after: u64, n: usize) -> Vec<LogEvent> {
        let newer = self.events.iter().filter(|e| e.id > after).count();
        let skip = self.events.len() - newer + newer.saturating_sub(n);
        self.events.iter().skip(skip).cloned().collect()
    }

    /// Id the next event will get.
    pub fn next_id(&self) -> u64 {
        self.next_id
    }

    pub fn capacity(&self) -> usize {
        self.cap
    }

    pub fn len(&self) -> usize {
        self.events.len()
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    /// Events lost since start: overwritten here, or dropped before they got
    /// here ([`Self::add_dropped`]).
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Count events that were lost upstream (e.g. the supervisor's queue).
    pub fn add_dropped(&mut self, n: u64) {
        self.dropped = self.dropped.saturating_add(n);
    }

    pub fn start(&self) -> Instant {
        self.start
    }
}

/// The event log shared by the control loop (writer) and the API (reader).
pub type SharedEventLog = Arc<Mutex<EventLog>>;

pub fn shared_event_log(start: Instant) -> SharedEventLog {
    Arc::new(Mutex::new(EventLog::new(start, EventLog::DEFAULT_CAPACITY)))
}

/// Lock the shared log; a poisoned lock still yields the log (it is only data).
pub fn lock_events(log: &SharedEventLog) -> MutexGuard<'_, EventLog> {
    log.lock().unwrap_or_else(|e| e.into_inner())
}

/// Milliseconds from `start` to `now` (0 if `now` is earlier).
pub fn ms_since(start: Instant, now: Instant) -> f64 {
    now.saturating_duration_since(start).as_secs_f64() * 1000.0
}

// ------------------------------------------------------------------ aggregate

/// Control-loop counters and statistics shown in status (§7.2 `telemetry`).
#[derive(Debug, Clone)]
pub struct Telemetry {
    pub tick_count: u64,
    pub tick_overruns: u64,
    pub tick_dt_ms: LatencyStats,
    pub pose_rx_to_tick_ms: LatencyStats,
    pub status_age_ms: LatencyStats,
    pub frame_skew: LatencyStats,
    /// Shared with the API (`GET /v1/events`).
    pub events: SharedEventLog,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TelemetrySummary {
    pub tick_count: u64,
    pub tick_overruns: u64,
    pub tick_dt_ms: LatencySummary,
    pub pose_rx_to_tick_ms: LatencySummary,
    pub status_age_ms: LatencySummary,
    pub frame_skew: LatencySummary,
    /// Event-log entries overwritten.
    pub log_dropped: u64,
    /// JSONL records dropped by the recorder (0 when disabled).
    pub recorder_dropped: u64,
}

impl Telemetry {
    pub fn new(start: Instant) -> Self {
        Self {
            tick_count: 0,
            tick_overruns: 0,
            tick_dt_ms: LatencyStats::default(),
            pose_rx_to_tick_ms: LatencyStats::default(),
            status_age_ms: LatencyStats::default(),
            frame_skew: LatencyStats::default(),
            events: shared_event_log(start),
        }
    }

    /// Same, appending events to an existing shared log.
    pub fn with_event_log(events: SharedEventLog) -> Self {
        let start = lock_events(&events).start();
        Self {
            events,
            ..Self::new(start)
        }
    }

    pub fn summary(&self, recorder: Option<&Recorder>) -> TelemetrySummary {
        TelemetrySummary {
            tick_count: self.tick_count,
            tick_overruns: self.tick_overruns,
            tick_dt_ms: self.tick_dt_ms.summary(),
            pose_rx_to_tick_ms: self.pose_rx_to_tick_ms.summary(),
            status_age_ms: self.status_age_ms.summary(),
            frame_skew: self.frame_skew.summary(),
            log_dropped: lock_events(&self.events).dropped(),
            recorder_dropped: recorder.map_or(0, Recorder::dropped),
        }
    }
}

// ------------------------------------------------------------------ recorder

/// Optional JSONL recorder. `record` serializes on the caller and `try_send`s
/// the line to a writer task; it never blocks. Full queue, closed writer or
/// serialization failure → dropped (counted).
#[derive(Debug)]
pub struct Recorder {
    tx: Option<mpsc::Sender<String>>,
    dropped: Arc<AtomicU64>,
    written: Arc<AtomicU64>,
    task: Option<JoinHandle<io::Result<()>>>,
}

impl Recorder {
    pub const DEFAULT_QUEUE: usize = 4096;

    /// A recorder that records nothing (and counts nothing as dropped).
    pub fn disabled() -> Self {
        Self {
            tx: None,
            dropped: Arc::default(),
            written: Arc::default(),
            task: None,
        }
    }

    /// Append to `path` (created if missing). Must be called inside a runtime.
    pub async fn open(path: &Path, queue: usize) -> io::Result<Self> {
        let file = tokio::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .await?;
        Ok(Self::spawn(file, queue))
    }

    /// Start a writer task on any async writer. `queue` 0 is treated as 1.
    pub fn spawn<W>(writer: W, queue: usize) -> Self
    where
        W: AsyncWrite + Unpin + Send + 'static,
    {
        let (tx, rx) = mpsc::channel(queue.max(1));
        let written = Arc::new(AtomicU64::new(0));
        let task = tokio::spawn(write_loop(writer, rx, written.clone()));
        Self {
            tx: Some(tx),
            dropped: Arc::default(),
            written,
            task: Some(task),
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.tx.is_some()
    }

    /// Queue one record as a JSON line. Returns false if it was dropped
    /// (always false, uncounted, when disabled).
    pub fn record<T: Serialize + ?Sized>(&self, rec: &T) -> bool {
        let Some(tx) = &self.tx else { return false };
        let ok = serde_json::to_string(rec)
            .ok()
            .is_some_and(|line| tx.try_send(line).is_ok());
        if !ok {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
        ok
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Lines written by the task so far.
    pub fn written(&self) -> u64 {
        self.written.load(Ordering::Relaxed)
    }

    /// Stop accepting records, let the task drain and flush, and return its
    /// result (write errors surface here).
    pub async fn close(mut self) -> io::Result<()> {
        self.tx = None;
        match self.task.take() {
            Some(task) => task.await.map_err(io::Error::other)?,
            None => Ok(()),
        }
    }
}

async fn write_loop<W>(
    writer: W,
    mut rx: mpsc::Receiver<String>,
    written: Arc<AtomicU64>,
) -> io::Result<()>
where
    W: AsyncWrite + Unpin,
{
    let mut w = BufWriter::new(writer);
    let result = async {
        while let Some(line) = rx.recv().await {
            w.write_all(line.as_bytes()).await?;
            w.write_all(b"\n").await?;
            written.fetch_add(1, Ordering::Relaxed);
            // Flush when caught up so the file is current without a timer.
            if rx.is_empty() {
                w.flush().await?;
            }
        }
        w.flush().await
    }
    .await;
    if let Err(e) = &result {
        tracing::warn!("telemetry recorder stopped: {e}");
    }
    // Dropping `rx` makes further `record` calls fail and count as dropped.
    result
}
