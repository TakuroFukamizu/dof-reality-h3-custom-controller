//! One USB port = one async task (DESIGN.md §1, §2; protocol.md §1).
//!
//! # Lifecycle of a link
//!
//! 1. **Handshake.** Only HELLO_REQ is written (every `timing.hello_retry_ms`)
//!    until a HELLO arrives or `timing.hello_timeout_ms` passes. STATUS and
//!    other frames seen before HELLO are ignored. The HELLO must carry
//!    `protocol_version == 1`, the POSITION_MODE capability and a `device_id`
//!    that exactly matches a non-placeholder `[[axis]]`. Otherwise the link
//!    emits [`LinkEvent::UnknownDevice`], closes the port and stops: nothing
//!    but HELLO_REQ is ever written to it. A handshake that ends without any
//!    HELLO (timeout, EOF, write failure) emits [`LinkEvent::PortClosed`].
//! 2. **Identified.** [`LinkEvent::Identified`] is emitted, then a reader and
//!    a writer run concurrently. The reader forwards STATUS / ACK / FAULT_EVENT;
//!    a HELLO with the same device_id is forwarded as `Identified` again (the
//!    supervisor detects a reboot from boot_id); a HELLO with another
//!    device_id ends the link. CLOCK_SYNC_REPLY is dropped (v1 does not use it).
//!    The writer sends admin messages first (bounded queue, [`ADMIN_QUEUE_LEN`])
//!    and then the latest SET_POSITION (watch, never queued). Every write is
//!    bounded by [`WRITE_TIMEOUT`].
//! 3. **End.** Exactly one [`LinkEvent::Disconnected`] per identified link:
//!    EOF, read/write error, write timeout, STATUS silent for
//!    `timing.axis_status_timeout_ms`, device change, or the handle closing it.
//!    The port is closed when the task ends.
//!
//! # Decisions not spelled out in DESIGN.md
//!
//! - The link never emits `PortClosed` after `UnknownDevice`: the supervisor
//!   clears the unknown-device ARM blocker on `PortClosed`, so the server should
//!   send it only once the port has physically gone (e.g. it disappeared from
//!   [`list_serial_ports`]).
//! - The link cannot tell that it is a *duplicate* of an already-connected
//!   axis; events are tagged with the port so the server can drop events of a
//!   port other than `Supervisor::axis_port(axis)`.
//! - Decode errors (COBS / CRC / version / length / unknown type, oversize
//!   frames) are counted, never fatal, and reported as `DecodeErrors` deltas at
//!   most every [`DECODE_REPORT_PERIOD`] and once more when the link ends.
//! - Dropping the [`LinkHandle`] closes the link like [`LinkHandle::shutdown`]
//!   (reason [`reason::CLOSED_BY_HOST`]) without waiting for the task.

use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::time::Duration;

use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::sync::{mpsc, oneshot, watch};
use tokio::task::JoinHandle;
use tokio::time::Instant;

use crate::config::{bounded_ms, Config};
use crate::protocol::cobs::FrameAccumulator;
use crate::protocol::{
    capabilities, decode_mcu_frame, Hello, HostMsg, McuMsg, SetPosition, MAX_ENCODED_LEN,
    PROTOCOL_VERSION,
};
use crate::supervisor::LinkEvent;
use crate::types::{AxisId, DeviceId};

/// Capacity of the per-link admin queue. A full queue is a send error
/// (the caller treats it as a fault, DESIGN.md §2).
pub const ADMIN_QUEUE_LEN: usize = 32;
/// Upper bound for one frame write (including flush).
pub const WRITE_TIMEOUT: Duration = Duration::from_millis(50);
/// Minimum interval between `DecodeErrors` reports of one link.
pub const DECODE_REPORT_PERIOD: Duration = Duration::from_millis(1000);

const READ_BUF: usize = 512;

/// `Disconnected.reason` values. Reasons with details are `"<prefix>: <detail>"`.
pub mod reason {
    pub const EOF: &str = "eof";
    pub const STATUS_TIMEOUT: &str = "status_timeout";
    pub const WRITE_TIMEOUT: &str = "write_timeout";
    pub const CLOSED_BY_HOST: &str = "closed_by_host";
    pub const EVENTS_CLOSED: &str = "events_closed";
    /// Prefix; detail is the new device_id.
    pub const DEVICE_CHANGED: &str = "device_changed";
    /// Prefix; detail says what was wrong with a mid-session HELLO.
    pub const BAD_HELLO: &str = "bad_hello";
    /// Prefix; detail is the I/O error.
    pub const READ_ERROR: &str = "read_error";
    /// Prefix; detail is the I/O error.
    pub const WRITE_ERROR: &str = "write_error";
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum LinkSendError {
    /// Handshake not finished: only HELLO_REQ may reach this port.
    #[error("link not identified")]
    NotIdentified,
    /// Admin queue full (the link is not keeping up).
    #[error("admin queue full")]
    Full,
    /// Link ended (unknown device, disconnected, shut down).
    #[error("link closed")]
    Closed,
    /// SET_POSITION goes through [`LinkHandle::set_position`], never the queue.
    #[error("SET_POSITION must use set_position()")]
    SetPositionOnAdminQueue,
}

// Link phase shared with the handle: 0 = handshake, 1..=3 = identified axis,
// CLOSED = ended.
const PHASE_HANDSHAKE: u8 = 0;
const PHASE_CLOSED: u8 = 0xFF;

#[derive(Debug)]
struct Phase(AtomicU8);

impl Phase {
    fn get(&self) -> Result<AxisId, LinkSendError> {
        match self.0.load(Ordering::SeqCst) {
            PHASE_HANDSHAKE => Err(LinkSendError::NotIdentified),
            v => AxisId::new(v).ok_or(LinkSendError::Closed),
        }
    }
    fn set_axis(&self, axis: AxisId) {
        self.0.store(axis.get(), Ordering::SeqCst);
    }
    fn close(&self) {
        self.0.store(PHASE_CLOSED, Ordering::SeqCst);
    }
}

/// Handle of one running axis link. See the module docs.
#[derive(Debug)]
pub struct LinkHandle {
    pub port: String,
    admin_tx: mpsc::Sender<HostMsg>,
    setpos_tx: watch::Sender<Option<SetPosition>>,
    phase: Arc<Phase>,
    shutdown: Option<oneshot::Sender<()>>,
    task: Option<JoinHandle<()>>,
}

impl LinkHandle {
    /// Queue an admin message (never waits). `Full` means the link is not
    /// keeping up and the caller should fault.
    pub fn send_admin(&self, msg: HostMsg) -> Result<(), LinkSendError> {
        if matches!(msg, HostMsg::SetPosition(_)) {
            return Err(LinkSendError::SetPositionOnAdminQueue);
        }
        self.phase.get()?;
        self.admin_tx.try_send(msg).map_err(|e| match e {
            mpsc::error::TrySendError::Full(_) => LinkSendError::Full,
            mpsc::error::TrySendError::Closed(_) => LinkSendError::Closed,
        })
    }

    /// Replace the pending SET_POSITION. Only the newest one is ever written;
    /// one not yet written is overwritten, not queued.
    pub fn set_position(&self, sp: SetPosition) -> Result<(), LinkSendError> {
        self.phase.get()?;
        self.setpos_tx
            .send(Some(sp))
            .map_err(|_| LinkSendError::Closed)
    }

    /// The axis this port identified as (None during handshake / after the end).
    pub fn axis(&self) -> Option<AxisId> {
        self.phase.get().ok()
    }

    /// True once the link task has ended (port closed).
    pub fn is_finished(&self) -> bool {
        self.task.as_ref().is_none_or(|t| t.is_finished())
    }

    /// Close the link and wait for the task. An identified link reports
    /// `Disconnected { reason: "closed_by_host" }` if the event queue has room.
    pub async fn shutdown(mut self) {
        if let Some(tx) = self.shutdown.take() {
            let _ = tx.send(());
        }
        if let Some(task) = self.task.take() {
            let _ = task.await;
        }
    }
}

impl Drop for LinkHandle {
    fn drop(&mut self) {
        // Dropping the oneshot sender wakes the task, which closes the port.
        self.shutdown.take();
    }
}

/// Start the link task for one opened port (`io` is the serial stream or a
/// simulated axis' duplex end). Events are tagged with `port`.
pub fn spawn_link<T>(
    io: T,
    port: String,
    cfg: Arc<Config>,
    events: mpsc::Sender<(String, LinkEvent)>,
) -> LinkHandle
where
    T: AsyncRead + AsyncWrite + Unpin + Send + 'static,
{
    let (admin_tx, admin_rx) = mpsc::channel(ADMIN_QUEUE_LEN);
    let (setpos_tx, setpos_rx) = watch::channel(None);
    let (shutdown_tx, shutdown_rx) = oneshot::channel();
    let phase = Arc::new(Phase(AtomicU8::new(PHASE_HANDSHAKE)));
    let link = Link {
        port: port.clone(),
        cfg,
        events,
        phase: phase.clone(),
        acc: FrameAccumulator::new(MAX_ENCODED_LEN),
        decode: DecodeStats::default(),
    };
    let task = tokio::spawn(link.run(io, admin_rx, setpos_rx, shutdown_rx));
    LinkHandle {
        port,
        admin_tx,
        setpos_tx,
        phase,
        shutdown: Some(shutdown_tx),
        task: Some(task),
    }
}

/// Open a USB CDC serial port (8N1, no flow control).
///
/// DTR/RTS are left as the library opens them (`dtr_on_open` unset = not
/// touched; Linux still asserts DTR on open). The ESP32-S3 USB-Serial/JTAG
/// and some CDC stacks reset or enter the bootloader on certain DTR/RTS
/// transitions: this must be verified on the real CoreS3 before relying on it.
pub async fn open_serial(
    port: &str,
    baud: u32,
) -> Result<tokio_serial::SerialStream, tokio_serial::Error> {
    // Exclusive access is the library default on unix (TIOCEXCL).
    tokio_serial::SerialStream::open(&tokio_serial::new(port, baud))
}

/// Serial port names currently present (enumeration only; nothing is opened).
/// On macOS prefer `/dev/cu.*` over `/dev/tty.*` (the latter waits for DCD).
pub fn list_serial_ports() -> Vec<String> {
    let mut v: Vec<String> = tokio_serial::available_ports()
        .map(|ports| ports.into_iter().map(|p| p.port_name).collect())
        .unwrap_or_default();
    v.sort();
    v.dedup();
    v
}

// ----------------------------------------------------------------------
// Task internals
// ----------------------------------------------------------------------

#[derive(Debug, Default)]
struct DecodeStats {
    total: u64,
    reported: u64,
    /// `FrameAccumulator::dropped()` already counted in `total`.
    oversize_seen: u64,
}

/// How a handshake ended without identification.
enum HandshakeEnd {
    /// Unknown device / bad version / missing capability.
    Rejected(DeviceId, String),
    /// No HELLO (timeout, EOF, I/O error).
    NoHello(String),
    Shutdown,
}

struct Link {
    port: String,
    cfg: Arc<Config>,
    events: mpsc::Sender<(String, LinkEvent)>,
    phase: Arc<Phase>,
    acc: FrameAccumulator,
    decode: DecodeStats,
}

impl Link {
    async fn run<T>(
        mut self,
        io: T,
        admin_rx: mpsc::Receiver<HostMsg>,
        setpos_rx: watch::Receiver<Option<SetPosition>>,
        mut shutdown_rx: oneshot::Receiver<()>,
    ) where
        T: AsyncRead + AsyncWrite + Unpin + Send + 'static,
    {
        let (mut rd, mut wr) = tokio::io::split(io);
        let (axis, hello, pending) = match self.handshake(&mut rd, &mut wr, &mut shutdown_rx).await
        {
            Ok(v) => v,
            Err(end) => {
                // Refuse further sends before anyone learns the outcome.
                self.phase.close();
                drop(admin_rx);
                drop(setpos_rx);
                drop(rd);
                drop(wr);
                self.report_decode_errors(None).await;
                match end {
                    HandshakeEnd::Rejected(device_id, why) => {
                        tracing::warn!(port = %self.port, %device_id, "{why}; port closed");
                        self.emit(LinkEvent::UnknownDevice {
                            port: self.port.clone(),
                            device_id,
                        })
                        .await;
                    }
                    HandshakeEnd::NoHello(why) => {
                        tracing::info!(port = %self.port, "no HELLO ({why}); port closed");
                        self.emit(LinkEvent::PortClosed {
                            port: self.port.clone(),
                        })
                        .await;
                    }
                    HandshakeEnd::Shutdown => {
                        let _ = self.events.try_send((
                            self.port.clone(),
                            LinkEvent::PortClosed {
                                port: self.port.clone(),
                            },
                        ));
                    }
                }
                return;
            }
        };

        self.phase.set_axis(axis);
        tracing::info!(port = %self.port, %axis, device_id = %hello.device_id, boot_id = hello.boot_id, "axis identified");
        self.report_decode_errors(None).await;
        self.emit(LinkEvent::Identified {
            axis,
            hello,
            port: self.port.clone(),
        })
        .await;

        let mut by_host = false;
        let why = {
            let writer = write_loop(&mut wr, admin_rx, setpos_rx);
            let reader = self.read_loop(axis, hello.device_id, &mut rd, pending);
            tokio::select! {
                r = reader => r,
                w = writer => w,
                _ = &mut shutdown_rx => {
                    by_host = true;
                    reason::CLOSED_BY_HOST.to_string()
                }
            }
        };
        self.phase.close();
        drop(rd);
        drop(wr);
        tracing::info!(port = %self.port, %axis, reason = %why, "axis link closed");
        let ev = LinkEvent::Disconnected { axis, reason: why };
        if by_host {
            // The server may be waiting on shutdown() and not draining events.
            // Disconnected first: it matters more than the decode count.
            let _ = self.events.try_send((self.port.clone(), ev));
            self.flush_decode_try(Some(axis));
        } else {
            self.report_decode_errors(Some(axis)).await;
            self.emit(ev).await;
        }
    }

    /// Returns the axis, its HELLO and the frames received after the HELLO in
    /// the same read.
    async fn handshake<R, W>(
        &mut self,
        rd: &mut R,
        wr: &mut W,
        shutdown_rx: &mut oneshot::Receiver<()>,
    ) -> Result<(AxisId, Hello, Vec<Vec<u8>>), HandshakeEnd>
    where
        R: AsyncRead + Unpin,
        W: AsyncWrite + Unpin,
    {
        let t = &self.cfg.timing;
        // Clamped: the config may not have been validated (no Instant overflow).
        let retry = bounded_ms(t.hello_retry_ms.max(1));
        let deadline = Instant::now() + bounded_ms(t.hello_timeout_ms);
        let mut request_id = nonzero_random();
        let mut next_req = Instant::now();
        let mut buf = [0u8; READ_BUF];
        loop {
            let now = Instant::now();
            if now >= deadline {
                return Err(HandshakeEnd::NoHello("hello_timeout".into()));
            }
            if now >= next_req {
                let wire = HostMsg::HelloReq { request_id }.to_wire();
                write_frame(wr, &wire)
                    .await
                    .map_err(HandshakeEnd::NoHello)?;
                request_id = request_id.wrapping_add(1).max(1);
                next_req = Instant::now() + retry;
            }
            let wake = next_req.min(deadline);
            let n = tokio::select! {
                _ = &mut *shutdown_rx => return Err(HandshakeEnd::Shutdown),
                r = rd.read(&mut buf) => match r {
                    Ok(0) => return Err(HandshakeEnd::NoHello(reason::EOF.into())),
                    Ok(n) => n,
                    Err(e) => return Err(HandshakeEnd::NoHello(format!("{}: {e}", reason::READ_ERROR))),
                },
                _ = tokio::time::sleep_until(wake) => continue,
            };
            let frames = self.push_bytes(buf.get(..n).unwrap_or_default());
            let mut it = frames.into_iter();
            while let Some(f) = it.next() {
                match decode_mcu_frame(&f) {
                    Ok(McuMsg::Hello(h)) => {
                        return match self.match_hello(&h) {
                            Ok(axis) => Ok((axis, h, it.collect())),
                            Err(why) => Err(HandshakeEnd::Rejected(h.device_id, why)),
                        };
                    }
                    // STATUS etc. before HELLO: nobody to route them to.
                    Ok(_) => {}
                    Err(_) => self.decode.total += 1,
                }
            }
        }
    }

    /// Map a HELLO to a configured axis, or say why it cannot be used.
    fn match_hello(&self, h: &Hello) -> Result<AxisId, String> {
        if h.protocol_version != PROTOCOL_VERSION {
            return Err(format!(
                "unsupported protocol_version {}",
                h.protocol_version
            ));
        }
        if h.capabilities & capabilities::POSITION_MODE == 0 {
            return Err("POSITION_MODE capability missing".into());
        }
        self.cfg
            .axes
            .iter()
            .find(|a| !a.device_id.is_unassigned() && a.device_id == h.device_id)
            .map(|a| a.axis_id)
            .ok_or_else(|| format!("device_id {} is not in [[axis]]", h.device_id))
    }

    /// Reader + STATUS watchdog of an identified link. Returns the disconnect reason.
    async fn read_loop<R>(
        &mut self,
        axis: AxisId,
        device_id: DeviceId,
        rd: &mut R,
        pending: Vec<Vec<u8>>,
    ) -> String
    where
        R: AsyncRead + Unpin,
    {
        let status_timeout = bounded_ms(self.cfg.timing.axis_status_timeout_ms);
        let watchdog = tokio::time::sleep(status_timeout);
        tokio::pin!(watchdog);
        let mut report =
            tokio::time::interval_at(Instant::now() + DECODE_REPORT_PERIOD, DECODE_REPORT_PERIOD);
        report.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        let mut buf = [0u8; READ_BUF];
        let mut frames = pending;
        loop {
            for f in std::mem::take(&mut frames) {
                if let Err(why) = self
                    .on_frame(axis, device_id, &f, watchdog.as_mut(), status_timeout)
                    .await
                {
                    return why;
                }
            }
            tokio::select! {
                r = rd.read(&mut buf) => match r {
                    Ok(0) => return reason::EOF.into(),
                    Ok(n) => frames = self.push_bytes(buf.get(..n).unwrap_or_default()),
                    Err(e) => return format!("{}: {e}", reason::READ_ERROR),
                },
                _ = &mut watchdog => return reason::STATUS_TIMEOUT.into(),
                _ = report.tick() => {
                    if !self.report_decode_errors(Some(axis)).await {
                        return reason::EVENTS_CLOSED.into();
                    }
                }
            }
        }
    }

    async fn on_frame(
        &mut self,
        axis: AxisId,
        device_id: DeviceId,
        f: &[u8],
        watchdog: std::pin::Pin<&mut tokio::time::Sleep>,
        status_timeout: Duration,
    ) -> Result<(), String> {
        let ev = match decode_mcu_frame(f) {
            Ok(McuMsg::Status(status)) => {
                let received_at = Instant::now();
                watchdog.reset(received_at + status_timeout);
                LinkEvent::Status {
                    axis,
                    status,
                    received_at,
                }
            }
            Ok(McuMsg::Ack(ack)) => LinkEvent::Ack { axis, ack },
            Ok(McuMsg::FaultEvent(ev)) => LinkEvent::FaultEvent { axis, ev },
            Ok(McuMsg::Hello(h)) => {
                if h.device_id != device_id {
                    return Err(format!("{}: {}", reason::DEVICE_CHANGED, h.device_id));
                }
                if let Err(why) = self.match_hello(&h) {
                    return Err(format!("{}: {why}", reason::BAD_HELLO));
                }
                LinkEvent::Identified {
                    axis,
                    hello: h,
                    port: self.port.clone(),
                }
            }
            Ok(McuMsg::ClockSyncReply(_)) => return Ok(()),
            Err(_) => {
                self.decode.total += 1;
                return Ok(());
            }
        };
        if self.emit(ev).await {
            Ok(())
        } else {
            Err(reason::EVENTS_CLOSED.into())
        }
    }

    fn push_bytes(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        let frames = self.acc.push(bytes);
        let dropped = self.acc.dropped();
        self.decode.total += dropped.saturating_sub(self.decode.oversize_seen);
        self.decode.oversize_seen = dropped;
        frames
    }

    fn take_decode_delta(&mut self) -> u64 {
        let d = self.decode.total.saturating_sub(self.decode.reported);
        self.decode.reported = self.decode.total;
        d
    }

    /// Emit the decode-error delta, if any. False if the event queue is closed.
    async fn report_decode_errors(&mut self, axis: Option<AxisId>) -> bool {
        let count = self.take_decode_delta();
        if count == 0 {
            return true;
        }
        let port = self.port.clone();
        self.emit(LinkEvent::DecodeErrors { axis, port, count })
            .await
    }

    fn flush_decode_try(&mut self, axis: Option<AxisId>) {
        let count = self.take_decode_delta();
        if count > 0 {
            let port = self.port.clone();
            let _ = self.events.try_send((
                self.port.clone(),
                LinkEvent::DecodeErrors { axis, port, count },
            ));
        }
    }

    /// Send an event (waits for room). False if the receiver is gone.
    async fn emit(&self, ev: LinkEvent) -> bool {
        self.events.send((self.port.clone(), ev)).await.is_ok()
    }
}

/// Writer of an identified link: admin queue first, then the latest
/// SET_POSITION. Returns the disconnect reason.
async fn write_loop<W>(
    wr: &mut W,
    mut admin_rx: mpsc::Receiver<HostMsg>,
    mut setpos_rx: watch::Receiver<Option<SetPosition>>,
) -> String
where
    W: AsyncWrite + Unpin,
{
    // Nothing set before identification may be sent.
    let _ = setpos_rx.borrow_and_update();
    loop {
        let msg = tokio::select! {
            biased;
            m = admin_rx.recv() => match m {
                Some(m) => m,
                None => return reason::CLOSED_BY_HOST.into(),
            },
            r = setpos_rx.changed() => match r {
                Ok(()) => match *setpos_rx.borrow_and_update() {
                    Some(sp) => HostMsg::SetPosition(sp),
                    None => continue,
                },
                Err(_) => return reason::CLOSED_BY_HOST.into(),
            },
        };
        if let Err(why) = write_frame(wr, &msg.to_wire()).await {
            return why;
        }
    }
}

/// Write one wire frame within [`WRITE_TIMEOUT`].
async fn write_frame<W: AsyncWrite + Unpin>(wr: &mut W, wire: &[u8]) -> Result<(), String> {
    let io = async {
        wr.write_all(wire).await?;
        wr.flush().await
    };
    match tokio::time::timeout(WRITE_TIMEOUT, io).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(e)) => Err(format!("{}: {e}", reason::WRITE_ERROR)),
        Err(_) => Err(reason::WRITE_TIMEOUT.into()),
    }
}

fn nonzero_random() -> u32 {
    rand::random::<u32>().max(1)
}
