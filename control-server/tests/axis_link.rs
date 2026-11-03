//! axis_link: handshake gating, latest-value SET_POSITION, admin priority,
//! STATUS watchdog, write timeout isolation and decode-error counting
//! (DESIGN.md §1, §2; protocol.md §1). All on a paused tokio clock.

use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use h3cs::axis_link::{reason, spawn_link, LinkHandle, LinkSendError, WRITE_TIMEOUT};
use h3cs::config::Config;
use h3cs::protocol::cobs::{self, FrameAccumulator};
use h3cs::protocol::{
    capabilities, decode_host_frame, frame, msg_type, Config as CfgMsg, Hello, HostMsg, McuMsg,
    SetPosition, Status, MAX_ENCODED_LEN,
};
use h3cs::sim_axis::{sim_pair, SimParams};
use h3cs::supervisor::LinkEvent;
use h3cs::types::{AxisId, DeviceId, FaultCode, McuState};
use tokio::io::{AsyncReadExt, AsyncWriteExt, DuplexStream};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio::time::{sleep, timeout, Instant};

const SIM: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/config/sim.toml");
const UNKNOWN: DeviceId = DeviceId(0x0BAD_0000_0000_0042);

type Events = mpsc::Receiver<(String, LinkEvent)>;

fn cfg() -> Arc<Config> {
    Arc::new(Config::load(Path::new(SIM)).unwrap())
}

fn ax(n: u8) -> AxisId {
    AxisId::new(n).unwrap()
}

fn dev(axis: u8) -> DeviceId {
    let c = cfg();
    c.axis(ax(axis)).unwrap().device_id
}

fn events() -> (mpsc::Sender<(String, LinkEvent)>, Events) {
    mpsc::channel(1024)
}

fn sim_params(device_id: DeviceId) -> SimParams {
    SimParams {
        device_id,
        boot_id: 77,
        ..SimParams::default()
    }
}

fn hello(device_id: DeviceId, boot_id: u32) -> Hello {
    Hello {
        request_id: 1,
        protocol_version: 1,
        selected_axis: 0,
        mcu_state: 1,
        device_id,
        boot_id,
        fw_version: [0, 1, 0],
        capabilities: capabilities::POSITION_MODE,
        calibration_hash: 0,
    }
}

fn status(boot_id: u32) -> Status {
    Status {
        boot_id,
        mcu_time_us: 0,
        mcu_state: McuState::Disarmed as u8,
        selected_axis: 0,
        fault_code: FaultCode::None,
        flags: 0,
        pwm_permille: 0,
        actual_count: 0,
        target_count: 0,
        velocity_count_s: 0,
        following_error_count: 0,
        last_applied_frame: 0,
        last_applied_seq: 0,
        last_rx_seq: 0,
        current_ma: 0,
        rx_error_count: 0,
        stale_drop_count: 0,
    }
}

fn config_msg(c: &Config, axis: AxisId, request_id: u32, epoch: u32) -> HostMsg {
    let a = c.axis(axis).unwrap();
    HostMsg::Config(CfgMsg {
        request_id,
        session_epoch: epoch,
        calibration_hash: 0x1234,
        soft_min_count: a.soft_min_count,
        soft_max_count: a.soft_max_count,
        max_vel_count_s: c.limits.max_vel_count_s,
        max_acc_count_s2: c.limits.max_acc_count_s2,
        following_error_limit_count: c.limits.following_error_limit_count,
        overspeed_count_s: c.limits.overspeed_count_s,
        command_timeout_ms: 100,
        stop_timeout_ms: 1500,
        role_flags: 0,
    })
}

fn setpos(epoch: u32, seq: u32) -> SetPosition {
    SetPosition {
        session_epoch: epoch,
        seq,
        group_frame_id: seq,
        target_count: 0,
        max_vel_count_s: 1000,
        max_acc_count_s2: 10000,
        valid_for_ms: 50,
        apply_at_us: 0,
    }
}

/// Next event within `within` (fails the test instead of hanging).
async fn next(rx: &mut Events, within: Duration) -> (String, LinkEvent) {
    timeout(within, rx.recv())
        .await
        .expect("timed out waiting for a link event")
        .expect("event channel closed")
}

/// Skip events until `pred` matches; returns the match.
async fn wait_for(
    rx: &mut Events,
    within: Duration,
    pred: impl Fn(&str, &LinkEvent) -> bool,
) -> (String, LinkEvent) {
    let deadline = Instant::now() + within;
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let (p, ev) = next(rx, left).await;
        if pred(&p, &ev) {
            return (p, ev);
        }
    }
}

async fn wait_identified(rx: &mut Events, port: &str) -> (AxisId, Hello) {
    let (_, ev) = wait_for(rx, Duration::from_secs(3), |p, e| {
        p == port && matches!(e, LinkEvent::Identified { .. })
    })
    .await;
    match ev {
        LinkEvent::Identified { axis, hello, .. } => (axis, hello),
        _ => unreachable!(),
    }
}

async fn wait_disconnected(rx: &mut Events, port: &str, within: Duration) -> String {
    let (_, ev) = wait_for(rx, within, |p, e| {
        p == port && matches!(e, LinkEvent::Disconnected { .. })
    })
    .await;
    match ev {
        LinkEvent::Disconnected { reason, .. } => reason,
        _ => unreachable!(),
    }
}

/// Wait for the Ack of `request_id` on `port`.
async fn wait_ack(rx: &mut Events, port: &str, request_id: u32) {
    wait_for(rx, Duration::from_secs(1), |p, e| {
        p == port && matches!(e, LinkEvent::Ack { ack, .. } if ack.request_id == request_id)
    })
    .await;
}

// ------------------------------------------------------------- fake MCU peer

#[derive(Clone)]
struct PeerOpts {
    hello: Option<Hello>,
    /// HELLO_REQs to ignore before answering.
    ignore_hello_reqs: usize,
    /// Periodic STATUS (starts after the first HELLO is sent).
    status_every: Option<Duration>,
    /// Stop reading once HELLO has been sent (the read half stays open).
    stop_reading_after_hello: bool,
}

impl Default for PeerOpts {
    fn default() -> Self {
        Self {
            hello: None,
            ignore_hello_reqs: 0,
            status_every: Some(Duration::from_millis(20)),
            stop_reading_after_hello: false,
        }
    }
}

/// Scriptable MCU on the far end of a duplex: logs every decoded host frame.
struct Peer {
    log: Arc<Mutex<Vec<HostMsg>>>,
    eof: Arc<Mutex<bool>>,
    raw: mpsc::Sender<Vec<u8>>,
    tasks: Vec<JoinHandle<()>>,
}

impl Peer {
    fn spawn(io: DuplexStream, opts: PeerOpts) -> Self {
        let log = Arc::new(Mutex::new(Vec::new()));
        let eof = Arc::new(Mutex::new(false));
        let (raw_tx, mut raw_rx) = mpsc::channel::<Vec<u8>>(64);
        let (hello_sent_tx, hello_sent_rx) = tokio::sync::watch::channel(false);
        let (mut rd, mut wr) = tokio::io::split(io);

        let reader = {
            let log = log.clone();
            let eof = eof.clone();
            let raw = raw_tx.clone();
            let opts = opts.clone();
            tokio::spawn(async move {
                let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
                let mut buf = [0u8; 512];
                let mut ignored = 0;
                loop {
                    let n = match rd.read(&mut buf).await {
                        Ok(0) | Err(_) => {
                            *eof.lock().unwrap() = true;
                            return;
                        }
                        Ok(n) => n,
                    };
                    for f in acc.push(&buf[..n]) {
                        let Ok(m) = decode_host_frame(&f) else {
                            continue;
                        };
                        log.lock().unwrap().push(m);
                        if let (HostMsg::HelloReq { request_id }, Some(h)) = (m, opts.hello) {
                            if ignored < opts.ignore_hello_reqs {
                                ignored += 1;
                                continue;
                            }
                            let h = Hello { request_id, ..h };
                            let _ = raw.send(McuMsg::Hello(h).to_wire()).await;
                            let _ = hello_sent_tx.send(true);
                            if opts.stop_reading_after_hello {
                                // Keep the read half open but never read again.
                                std::future::pending::<()>().await;
                            }
                        }
                    }
                }
            })
        };

        let writer = {
            let boot = opts.hello.map_or(0, |h| h.boot_id);
            let every = opts.status_every;
            let mut hello_sent = hello_sent_rx;
            tokio::spawn(async move {
                let mut tick = every.map(tokio::time::interval);
                let mut watching = true;
                loop {
                    let streaming = *hello_sent.borrow();
                    tokio::select! {
                        b = raw_rx.recv() => match b {
                            Some(b) => if wr.write_all(&b).await.is_err() { return },
                            None => return,
                        },
                        _ = async { tick.as_mut().unwrap().tick().await }, if tick.is_some() && streaming => {
                            if wr.write_all(&McuMsg::Status(status(boot)).to_wire()).await.is_err() {
                                return;
                            }
                        }
                        r = hello_sent.changed(), if watching && !streaming => if r.is_err() {
                            // Reader gone; keep serving raw writes only.
                            watching = false;
                        },
                    }
                }
            })
        };

        Self {
            log,
            eof,
            raw: raw_tx,
            tasks: vec![reader, writer],
        }
    }

    fn log(&self) -> Vec<HostMsg> {
        self.log.lock().unwrap().clone()
    }

    fn saw_eof(&self) -> bool {
        *self.eof.lock().unwrap()
    }

    async fn send_raw(&self, bytes: Vec<u8>) {
        self.raw.send(bytes).await.unwrap();
    }
}

impl Drop for Peer {
    fn drop(&mut self) {
        for t in &self.tasks {
            t.abort();
        }
    }
}

fn peer_link(
    opts: PeerOpts,
    port: &str,
    ev_tx: mpsc::Sender<(String, LinkEvent)>,
) -> (Peer, LinkHandle) {
    let (host, mcu) = tokio::io::duplex(64 * 1024);
    let peer = Peer::spawn(mcu, opts);
    let link = spawn_link(host, port.to_string(), cfg(), ev_tx);
    (peer, link)
}

fn only_hello_reqs(log: &[HostMsg]) -> bool {
    !log.is_empty() && log.iter().all(|m| matches!(m, HostMsg::HelloReq { .. }))
}

// --------------------------------------------------------------------- tests

#[tokio::test(start_paused = true)]
async fn handshake_maps_device_id_to_axis_regardless_of_port() {
    let (tx, mut rx) = events();
    // Port order deliberately differs from axis order.
    let order = [3u8, 1, 2];
    let mut sims = Vec::new();
    let mut links = Vec::new();
    for (i, a) in order.iter().enumerate() {
        let (io, sim) = sim_pair(sim_params(dev(*a)));
        sims.push(sim);
        links.push(spawn_link(
            io,
            format!("/dev/cu.port{i}"),
            cfg(),
            tx.clone(),
        ));
    }
    let mut seen = std::collections::BTreeMap::new();
    while seen.len() < 3 {
        let (port, ev) = next(&mut rx, Duration::from_secs(1)).await;
        match ev {
            LinkEvent::Identified {
                axis,
                hello,
                port: p,
            } => {
                assert_eq!(p, port);
                assert_eq!(hello.device_id, dev(axis.get()));
                seen.insert(port, axis);
            }
            // STATUS sent before HELLO must not be forwarded.
            other => assert!(
                seen.contains_key(&port),
                "{port}: {other:?} before Identified"
            ),
        }
    }
    for (i, a) in order.iter().enumerate() {
        assert_eq!(seen[&format!("/dev/cu.port{i}")], ax(*a));
        assert_eq!(links[i].axis(), Some(ax(*a)));
    }
    // Identified links forward STATUS.
    wait_for(&mut rx, Duration::from_millis(100), |_, e| {
        matches!(e, LinkEvent::Status { .. })
    })
    .await;
}

#[tokio::test(start_paused = true)]
async fn unknown_device_gets_only_hello_req_and_port_is_closed() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(UNKNOWN, 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    assert_eq!(
        link.send_admin(HostMsg::Disarm { request_id: 9 }),
        Err(LinkSendError::NotIdentified)
    );
    let (port, ev) = next(&mut rx, Duration::from_secs(1)).await;
    assert_eq!(port, "p");
    assert!(
        matches!(ev, LinkEvent::UnknownDevice { ref port, device_id } if port == "p" && device_id == UNKNOWN),
        "{ev:?}"
    );
    // The handle refuses everything from now on.
    let c = cfg();
    assert_eq!(
        link.send_admin(config_msg(&c, ax(1), 1, 1)),
        Err(LinkSendError::Closed)
    );
    assert_eq!(link.set_position(setpos(1, 1)), Err(LinkSendError::Closed));
    assert_eq!(link.axis(), None);

    sleep(Duration::from_millis(500)).await;
    assert!(only_hello_reqs(&peer.log()), "{:?}", peer.log());
    assert!(peer.saw_eof(), "port must be closed");
    assert!(link.is_finished());
    // No PortClosed: the unknown-device blocker must persist (see module docs).
    assert!(rx.try_recv().is_err());
}

#[tokio::test(start_paused = true)]
async fn unknown_device_sim_is_never_configured() {
    let (tx, mut rx) = events();
    let (io, sim) = sim_pair(sim_params(UNKNOWN));
    let link = spawn_link(io, "sim:x".into(), cfg(), tx);
    let c = cfg();
    assert_eq!(
        link.send_admin(config_msg(&c, ax(1), 1, 1)),
        Err(LinkSendError::NotIdentified)
    );
    assert_eq!(
        link.set_position(setpos(1, 1)),
        Err(LinkSendError::NotIdentified)
    );
    let (_, ev) = next(&mut rx, Duration::from_secs(1)).await;
    assert!(matches!(ev, LinkEvent::UnknownDevice { .. }), "{ev:?}");
    assert_eq!(
        link.send_admin(config_msg(&c, ax(1), 2, 1)),
        Err(LinkSendError::Closed)
    );
    sleep(Duration::from_millis(300)).await;
    let s = sim.snapshot();
    assert!(!s.configured);
    assert_eq!(s.state, McuState::Disarmed);
    assert_eq!(s.stale_drop_count, 0, "no SET_POSITION reached the device");
    assert_eq!(s.rx_error_count, 0);
}

#[tokio::test(start_paused = true)]
async fn bad_protocol_version_or_missing_capability_is_unknown_device() {
    for h in [
        Hello {
            protocol_version: 2,
            ..hello(dev(1), 5)
        },
        Hello {
            capabilities: 0,
            ..hello(dev(1), 5)
        },
    ] {
        let (tx, mut rx) = events();
        let (peer, link) = peer_link(
            PeerOpts {
                hello: Some(h),
                ..PeerOpts::default()
            },
            "p",
            tx,
        );
        let (_, ev) = next(&mut rx, Duration::from_secs(1)).await;
        assert!(
            matches!(ev, LinkEvent::UnknownDevice { device_id, .. } if device_id == dev(1)),
            "{ev:?}"
        );
        sleep(Duration::from_millis(300)).await;
        assert!(only_hello_reqs(&peer.log()));
        assert!(link.is_finished());
    }
}

#[tokio::test(start_paused = true)]
async fn hello_req_is_retried_until_answered() {
    let (tx, mut rx) = events();
    let c = cfg();
    let t0 = Instant::now();
    let (peer, _link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(2), 5)),
            ignore_hello_reqs: 2,
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    let (axis, _) = wait_identified(&mut rx, "p").await;
    assert_eq!(axis, ax(2));
    let retry = Duration::from_millis(c.timing.hello_retry_ms);
    let el = t0.elapsed();
    assert!(
        el >= retry * 2 && el < retry * 2 + Duration::from_millis(20),
        "{el:?}"
    );
    let log = peer.log();
    assert_eq!(log.len(), 3);
    let ids: Vec<u32> = log
        .iter()
        .map(|m| match m {
            HostMsg::HelloReq { request_id } => *request_id,
            other => panic!("{other:?}"),
        })
        .collect();
    assert!(ids.iter().all(|&i| i != 0));
    assert!(ids[0] != ids[1] && ids[1] != ids[2]);
}

#[tokio::test(start_paused = true)]
async fn request_ids_start_random() {
    let mut first = Vec::new();
    for _ in 0..4 {
        let (tx, mut rx) = events();
        let (peer, _link) = peer_link(
            PeerOpts {
                hello: Some(hello(dev(1), 5)),
                ..PeerOpts::default()
            },
            "p",
            tx,
        );
        wait_identified(&mut rx, "p").await;
        if let Some(HostMsg::HelloReq { request_id }) = peer.log().first() {
            first.push(*request_id);
        }
    }
    first.dedup();
    assert!(
        first.len() > 1,
        "HELLO_REQ request_id must not start at a fixed value: {first:?}"
    );
}

#[tokio::test(start_paused = true)]
async fn hello_timeout_closes_port_with_port_closed() {
    let (tx, mut rx) = events();
    let c = cfg();
    let t0 = Instant::now();
    let (peer, link) = peer_link(PeerOpts::default(), "p", tx);
    assert_eq!(
        link.set_position(setpos(1, 1)),
        Err(LinkSendError::NotIdentified)
    );
    let (_, ev) = next(&mut rx, Duration::from_secs(5)).await;
    assert!(
        matches!(ev, LinkEvent::PortClosed { ref port } if port == "p"),
        "{ev:?}"
    );
    let el = t0.elapsed();
    let limit = Duration::from_millis(c.timing.hello_timeout_ms);
    assert!(
        el >= limit && el < limit + Duration::from_millis(20),
        "{el:?}"
    );
    let log = peer.log();
    assert!(only_hello_reqs(&log));
    let expected = (c.timing.hello_timeout_ms / c.timing.hello_retry_ms) as usize;
    assert!(
        (expected - 1..=expected + 1).contains(&log.len()),
        "{} HELLO_REQs",
        log.len()
    );
    sleep(Duration::from_millis(10)).await;
    assert!(peer.saw_eof());
    assert_eq!(
        link.send_admin(HostMsg::Disarm { request_id: 3 }),
        Err(LinkSendError::Closed)
    );
}

#[tokio::test(start_paused = true)]
async fn set_position_is_latest_value_not_queued() {
    // Sim, no CONFIG: every SET_POSITION that reaches it is a stale drop.
    let (tx, mut rx) = events();
    let (io, sim) = sim_pair(sim_params(dev(1)));
    let link = spawn_link(io, "sim:1".into(), cfg(), tx);
    wait_identified(&mut rx, "sim:1").await;
    sleep(Duration::from_millis(30)).await;
    for seq in 1..=10 {
        link.set_position(setpos(1, seq)).unwrap();
    }
    sleep(Duration::from_millis(50)).await;
    assert_eq!(sim.snapshot().stale_drop_count, 1);

    // Fake peer: exactly one SET_POSITION, the newest.
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    wait_identified(&mut rx, "p").await;
    for seq in 1..=10 {
        link.set_position(setpos(1, seq)).unwrap();
    }
    sleep(Duration::from_millis(30)).await;
    let sps: Vec<u32> = peer
        .log()
        .iter()
        .filter_map(|m| match m {
            HostMsg::SetPosition(sp) => Some(sp.seq),
            _ => None,
        })
        .collect();
    assert_eq!(sps, vec![10]);
}

#[tokio::test(start_paused = true)]
async fn admin_messages_are_written_before_set_position() {
    // Sim: SET_POSITION published first, CONFIG queued after, no yield between.
    // If CONFIG goes out first the frame is accepted (no stale drop).
    let (tx, mut rx) = events();
    let c = cfg();
    let (io, sim) = sim_pair(sim_params(dev(1)));
    let link = spawn_link(io, "sim:1".into(), cfg(), tx);
    wait_identified(&mut rx, "sim:1").await;
    sleep(Duration::from_millis(30)).await;
    link.set_position(setpos(42, 1)).unwrap();
    link.send_admin(config_msg(&c, ax(1), 1000, 42)).unwrap();
    wait_ack(&mut rx, "sim:1", 1000).await;
    sleep(Duration::from_millis(20)).await;
    let s = sim.snapshot();
    assert!(s.configured);
    assert_eq!(s.stale_drop_count, 0);
    assert_eq!(s.last_rx_seq, 1);

    // Fake peer: several admin messages all precede the pending SET_POSITION.
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    wait_identified(&mut rx, "p").await;
    link.set_position(setpos(1, 1)).unwrap();
    for rid in 1..=3 {
        link.send_admin(HostMsg::Disarm { request_id: rid })
            .unwrap();
    }
    sleep(Duration::from_millis(30)).await;
    let types: Vec<u8> = peer.log().iter().skip(1).map(|m| m.msg_type()).collect();
    assert_eq!(
        types,
        vec![
            msg_type::DISARM,
            msg_type::DISARM,
            msg_type::DISARM,
            msg_type::SET_POSITION
        ]
    );
}

/// CONTROLLED_STOP is an ordinary admin frame. It is written even when no
/// SET_POSITION has been published, and a pending SET_POSITION does not go
/// out ahead of it.
#[tokio::test(start_paused = true)]
async fn controlled_stop_is_written_without_waiting_for_set_position() {
    let stop = HostMsg::ControlledStop {
        request_id: 9,
        session_epoch: 7,
        reason: 1,
    };

    // No SET_POSITION at all: the stop is still written.
    {
        let (tx, mut rx) = events();
        let (peer, link) = peer_link(
            PeerOpts {
                hello: Some(hello(dev(1), 5)),
                ..PeerOpts::default()
            },
            "p",
            tx,
        );
        wait_identified(&mut rx, "p").await;
        link.send_admin(stop).unwrap();
        sleep(Duration::from_millis(30)).await;
        let msgs: Vec<HostMsg> = peer
            .log()
            .into_iter()
            .filter(|m| !matches!(m, HostMsg::HelloReq { .. }))
            .collect();
        assert!(
            matches!(
                msgs.as_slice(),
                [HostMsg::ControlledStop { request_id: 9, .. }]
            ),
            "{msgs:?}"
        );
    }

    // Both pending, no yield between them. Admin wins, so the stop is first.
    {
        let (tx, mut rx) = events();
        let (peer, link) = peer_link(
            PeerOpts {
                hello: Some(hello(dev(1), 5)),
                ..PeerOpts::default()
            },
            "p",
            tx,
        );
        wait_identified(&mut rx, "p").await;
        let mut governed = setpos(7, 1);
        governed.max_vel_count_s = 300;
        governed.max_acc_count_s2 = 1250;
        governed.target_count = 10;
        link.set_position(governed).unwrap();
        link.send_admin(stop).unwrap();
        sleep(Duration::from_millis(30)).await;
        let msgs: Vec<HostMsg> = peer
            .log()
            .into_iter()
            .filter(|m| !matches!(m, HostMsg::HelloReq { .. }))
            .collect();
        assert!(
            matches!(
                msgs.first(),
                Some(HostMsg::ControlledStop { request_id: 9, .. })
            ),
            "stop waited for SET_POSITION: {msgs:?}"
        );
        match msgs.get(1) {
            Some(HostMsg::SetPosition(sp)) => {
                assert_eq!(sp.max_acc_count_s2, 1250);
                assert_eq!(sp.max_vel_count_s, 300);
            }
            other => panic!("frame after stop: {other:?}"),
        }
    }
}

#[tokio::test(start_paused = true)]
async fn set_position_on_admin_queue_is_refused() {
    let (tx, mut rx) = events();
    let (_peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    wait_identified(&mut rx, "p").await;
    assert_eq!(
        link.send_admin(HostMsg::SetPosition(setpos(1, 1))),
        Err(LinkSendError::SetPositionOnAdminQueue)
    );
}

#[tokio::test(start_paused = true)]
async fn full_admin_queue_is_an_error() {
    // A peer that stops reading: the writer blocks, the queue fills.
    let (tx, mut rx) = events();
    let (host, mcu) = tokio::io::duplex(32);
    let _peer = Peer::spawn(
        mcu,
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            stop_reading_after_hello: true,
            ..PeerOpts::default()
        },
    );
    let link = spawn_link(host, "p".into(), cfg(), tx);
    wait_identified(&mut rx, "p").await;
    let mut full = false;
    for rid in 1..=200 {
        match link.send_admin(HostMsg::Disarm { request_id: rid }) {
            Ok(()) => {}
            Err(LinkSendError::Full) => {
                full = true;
                break;
            }
            Err(e) => panic!("{e:?}"),
        }
    }
    assert!(full);
}

#[tokio::test(start_paused = true)]
async fn frozen_status_disconnects_within_status_timeout() {
    let (tx, mut rx) = events();
    let c = cfg();
    let (io, sim) = sim_pair(sim_params(dev(3)));
    let link = spawn_link(io, "sim:3".into(), cfg(), tx);
    let (axis, _) = wait_identified(&mut rx, "sim:3").await;
    assert_eq!(axis, ax(3));
    // Healthy for a while first.
    sleep(Duration::from_millis(500)).await;
    wait_for(&mut rx, Duration::from_millis(50), |_, e| {
        matches!(e, LinkEvent::Status { .. })
    })
    .await;
    sim.set_freeze_status(true);
    let t0 = Instant::now();
    let limit = Duration::from_millis(c.timing.axis_status_timeout_ms);
    let r = wait_disconnected(&mut rx, "sim:3", limit + Duration::from_millis(100)).await;
    let el = t0.elapsed();
    assert_eq!(r, reason::STATUS_TIMEOUT);
    let period = Duration::from_millis(SimParams::default().status_period_ms);
    assert!(
        el + period >= limit && el <= limit + Duration::from_millis(5),
        "{el:?}"
    );
    sleep(Duration::from_millis(5)).await;
    assert!(link.is_finished());
    assert_eq!(
        link.send_admin(HostMsg::Disarm { request_id: 1 }),
        Err(LinkSendError::Closed)
    );
}

#[tokio::test(start_paused = true)]
async fn mid_session_hello_from_other_device_disconnects() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    wait_identified(&mut rx, "p").await;
    sleep(Duration::from_millis(100)).await;

    // Same device, new boot_id: forwarded as Identified again, link stays up.
    peer.send_raw(McuMsg::Hello(hello(dev(1), 6)).to_wire())
        .await;
    let (axis, h) = wait_identified(&mut rx, "p").await;
    assert_eq!((axis, h.boot_id), (ax(1), 6));
    assert!(!link.is_finished());

    peer.send_raw(McuMsg::Hello(hello(dev(2), 7)).to_wire())
        .await;
    let r = wait_disconnected(&mut rx, "p", Duration::from_millis(100)).await;
    assert!(r.starts_with(reason::DEVICE_CHANGED), "{r}");
    sleep(Duration::from_millis(5)).await;
    assert!(peer.saw_eof());
    assert!(link.is_finished());
}

#[tokio::test(start_paused = true)]
async fn blocked_peer_times_out_without_delaying_other_link() {
    let (tx, mut rx) = events();
    let c = cfg();
    // Link A: duplex buffer fits one HELLO_REQ (12 B) but not a CONFIG (52 B);
    // the peer answers HELLO, keeps sending STATUS and never reads again.
    let (host_a, mcu_a) = tokio::io::duplex(32);
    let _peer_a = Peer::spawn(
        mcu_a,
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            stop_reading_after_hello: true,
            ..PeerOpts::default()
        },
    );
    let link_a = spawn_link(host_a, "a".into(), cfg(), tx.clone());
    // Link B: a healthy simulated axis 2.
    let (io_b, sim_b) = sim_pair(sim_params(dev(2)));
    let link_b = spawn_link(io_b, "b".into(), cfg(), tx);
    // Both Identified, in either order.
    let mut ids = std::collections::BTreeSet::new();
    while ids.len() < 2 {
        let (p, _) = wait_for(&mut rx, Duration::from_secs(1), |_, e| {
            matches!(e, LinkEvent::Identified { .. })
        })
        .await;
        ids.insert(p);
    }
    link_b.send_admin(config_msg(&c, ax(2), 500, 9)).unwrap();
    wait_ack(&mut rx, "b", 500).await;

    let t0 = Instant::now();
    link_a.send_admin(config_msg(&c, ax(1), 1, 9)).unwrap();
    let mut seq = 0;
    let mut a_reason = None;
    let mut b_progress_while_blocked = false;
    while t0.elapsed() < Duration::from_millis(300) {
        seq += 1;
        link_b.set_position(setpos(9, seq)).unwrap();
        sleep(Duration::from_millis(10)).await;
        if a_reason.is_none() && sim_b.snapshot().last_rx_seq == seq {
            b_progress_while_blocked = true;
        }
        while let Ok((p, ev)) = rx.try_recv() {
            match ev {
                LinkEvent::Disconnected { reason, .. } if p == "a" => {
                    assert!(a_reason.is_none(), "one Disconnected per link");
                    let el = t0.elapsed();
                    assert!(
                        el >= WRITE_TIMEOUT && el <= WRITE_TIMEOUT + Duration::from_millis(15),
                        "{el:?}"
                    );
                    a_reason = Some(reason);
                }
                LinkEvent::Disconnected { .. } => panic!("link b disconnected"),
                _ => {}
            }
        }
    }
    assert_eq!(a_reason.as_deref(), Some(reason::WRITE_TIMEOUT));
    assert!(b_progress_while_blocked);
    let s = sim_b.snapshot();
    assert!(
        s.last_rx_seq >= seq - 1,
        "b fell behind: {} of {seq}",
        s.last_rx_seq
    );
    assert_eq!(s.stale_drop_count, 0);
    assert!(link_a.is_finished());
    assert!(!link_b.is_finished());
}

#[tokio::test(start_paused = true)]
async fn garbage_on_the_wire_is_counted_not_fatal() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(2), 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    wait_identified(&mut rx, "p").await;

    // 1) noise, 2) CRC-corrupted STATUS, 3) oversize run, 4) unknown type.
    let mut junk = vec![0x13, 0x37, 0xFF, 0x42, 0x00];
    let mut f = frame::build(
        msg_type::STATUS,
        &McuMsg::Status(status(5)).encode_payload(),
    );
    if let Some(b) = f.last_mut() {
        *b ^= 0x5A;
    }
    junk.extend(cobs::encode(&f));
    junk.push(0);
    junk.extend(std::iter::repeat_n(0x11, 400));
    junk.push(0);
    junk.extend(frame::to_wire(0x99, &[1, 2, 3]));
    peer.send_raw(junk).await;

    let (_, ev) = wait_for(&mut rx, Duration::from_secs(2), |p, e| {
        p == "p"
            && matches!(
                e,
                LinkEvent::DecodeErrors { .. } | LinkEvent::Disconnected { .. }
            )
    })
    .await;
    match ev {
        LinkEvent::DecodeErrors { axis, port, count } => {
            assert_eq!((axis, port.as_str(), count), (Some(ax(2)), "p", 4));
        }
        other => panic!("{other:?}"),
    }
    // Still alive and forwarding.
    wait_for(&mut rx, Duration::from_millis(100), |p, e| {
        p == "p" && matches!(e, LinkEvent::Status { .. })
    })
    .await;
    assert!(!link.is_finished());

    // Deltas: a second burst reports only the new error.
    peer.send_raw(vec![0x02, 0x55, 0x00]).await;
    let (_, ev) = wait_for(&mut rx, Duration::from_secs(2), |_, e| {
        matches!(e, LinkEvent::DecodeErrors { .. })
    })
    .await;
    assert!(
        matches!(ev, LinkEvent::DecodeErrors { count: 1, .. }),
        "{ev:?}"
    );
}

#[tokio::test(start_paused = true)]
async fn garbage_before_hello_is_reported_without_axis() {
    let (tx, mut rx) = events();
    let (host, mcu) = tokio::io::duplex(4096);
    let peer = Peer::spawn(
        mcu,
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            ignore_hello_reqs: 1,
            ..PeerOpts::default()
        },
    );
    peer.send_raw(vec![0x05, 0x01, 0x02, 0x03, 0x04, 0x00])
        .await;
    let _link = spawn_link(host, "p".into(), cfg(), tx);
    let (_, ev) = next(&mut rx, Duration::from_secs(1)).await;
    assert!(
        matches!(
            ev,
            LinkEvent::DecodeErrors {
                axis: None,
                count: 1,
                ..
            }
        ),
        "{ev:?}"
    );
    let (_, ev) = next(&mut rx, Duration::from_secs(1)).await;
    assert!(matches!(ev, LinkEvent::Identified { .. }), "{ev:?}");
}

#[tokio::test(start_paused = true)]
async fn sim_corrupt_frame_is_counted() {
    let (tx, mut rx) = events();
    let (io, sim) = sim_pair(sim_params(dev(1)));
    let link = spawn_link(io, "sim:1".into(), cfg(), tx);
    wait_identified(&mut rx, "sim:1").await;
    sim.corrupt_next_tx();
    let (_, ev) = wait_for(&mut rx, Duration::from_secs(2), |_, e| {
        matches!(
            e,
            LinkEvent::DecodeErrors { .. } | LinkEvent::Disconnected { .. }
        )
    })
    .await;
    assert!(
        matches!(ev, LinkEvent::DecodeErrors { count: 1, .. }),
        "{ev:?}"
    );
    assert!(!link.is_finished());
}

#[tokio::test(start_paused = true)]
async fn eof_disconnects() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    wait_identified(&mut rx, "p").await;
    drop(peer);
    sleep(Duration::from_millis(1)).await;
    let r = wait_disconnected(&mut rx, "p", Duration::from_millis(50)).await;
    assert!(
        r == reason::EOF || r.starts_with(reason::READ_ERROR) || r.starts_with(reason::WRITE_ERROR),
        "{r}"
    );
    sleep(Duration::from_millis(1)).await;
    assert!(link.is_finished());
}

#[tokio::test(start_paused = true)]
async fn shutdown_reports_closed_by_host_once() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    wait_identified(&mut rx, "p").await;
    link.shutdown().await;
    let r = wait_disconnected(&mut rx, "p", Duration::from_millis(10)).await;
    assert_eq!(r, reason::CLOSED_BY_HOST);
    sleep(Duration::from_millis(5)).await;
    assert!(peer.saw_eof());
    while let Ok((_, ev)) = rx.try_recv() {
        assert!(
            !matches!(ev, LinkEvent::Disconnected { .. }),
            "second Disconnected"
        );
    }
}

#[tokio::test(start_paused = true)]
async fn dropping_the_handle_closes_the_port() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(hello(dev(1), 5)),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    wait_identified(&mut rx, "p").await;
    drop(link);
    let r = wait_disconnected(&mut rx, "p", Duration::from_millis(10)).await;
    assert_eq!(r, reason::CLOSED_BY_HOST);
    sleep(Duration::from_millis(5)).await;
    assert!(peer.saw_eof());
}

#[tokio::test(start_paused = true)]
async fn identified_link_forwards_ack_and_fault_event() {
    let (tx, mut rx) = events();
    let c = cfg();
    let (io, sim) = sim_pair(sim_params(dev(1)));
    let link = spawn_link(io, "sim:1".into(), cfg(), tx);
    wait_identified(&mut rx, "sim:1").await;
    link.send_admin(config_msg(&c, ax(1), 77, 3)).unwrap();
    wait_ack(&mut rx, "sim:1", 77).await;
    link.send_admin(HostMsg::HostFault {
        request_id: 78,
        fault_code: FaultCode::HostFault,
    })
    .unwrap();
    wait_for(&mut rx, Duration::from_millis(100), |_, e| {
        matches!(e, LinkEvent::FaultEvent { axis, ev } if *axis == ax(1) && ev.fault_code == FaultCode::HostFault)
    })
    .await;
    assert_eq!(sim.state(), McuState::Fault);
}

#[test]
fn list_serial_ports_does_not_panic() {
    let ports = h3cs::axis_link::list_serial_ports();
    let mut sorted = ports.clone();
    sorted.sort();
    sorted.dedup();
    assert_eq!(ports, sorted);
}

#[tokio::test(start_paused = true)]
async fn huge_unvalidated_timings_do_not_panic_the_link() {
    // Not validated: these would overflow `Instant + Duration` unclamped.
    let mut c = (*cfg()).clone();
    c.timing.hello_timeout_ms = u64::MAX;
    c.timing.hello_retry_ms = u64::MAX;
    c.timing.axis_status_timeout_ms = u64::MAX;
    let (tx, mut rx) = events();
    let (io, _sim) = sim_pair(sim_params(dev(1)));
    let link = spawn_link(io, "p".into(), Arc::new(c), tx);
    wait_identified(&mut rx, "p").await;
    // STATUS resets the (clamped) watchdog without panicking.
    for _ in 0..3 {
        wait_for(&mut rx, Duration::from_millis(100), |p, e| {
            p == "p" && matches!(e, LinkEvent::Status { .. })
        })
        .await;
    }
    assert_eq!(link.axis(), Some(ax(1)));
}

// ------------------------------------------------- axis selection (DESIGN §10.1)

fn axis_select(boot_id: u32, selected_axis: u8, prev: u8) -> Vec<u8> {
    McuMsg::AxisSelect(h3cs::protocol::AxisSelect {
        boot_id,
        selected_axis,
        prev_selected_axis: prev,
        mcu_state: McuState::Disarmed as u8,
    })
    .to_wire()
}

/// Collect events of `port` (STATUS dropped) until an `Identified` or
/// `UnknownDevice` arrives.
async fn collect_until_reidentified(rx: &mut Events, port: &str) -> Vec<LinkEvent> {
    let deadline = Instant::now() + Duration::from_secs(1);
    let mut out = Vec::new();
    loop {
        let left = deadline.saturating_duration_since(Instant::now());
        let (p, ev) = next(rx, left).await;
        if p != port
            || matches!(
                ev,
                LinkEvent::Status { .. } | LinkEvent::DecodeErrors { .. }
            )
        {
            continue;
        }
        let done = matches!(
            ev,
            LinkEvent::Identified { .. } | LinkEvent::UnknownDevice { .. }
        );
        out.push(ev);
        if done {
            return out;
        }
    }
}

#[tokio::test(start_paused = true)]
async fn handshake_assigns_from_selected_axis_not_from_device_id() {
    // Axis 1's configured device selecting axis 2 → axis 2; an unknown device
    // selecting axis 3 → axis 3. device_id is a cross-check for the supervisor.
    for (device, selected, want) in [(dev(1), 2u8, 2u8), (UNKNOWN, 3, 3), (dev(3), 0, 3)] {
        let (tx, mut rx) = events();
        let (peer, link) = peer_link(
            PeerOpts {
                hello: Some(Hello {
                    selected_axis: selected,
                    ..hello(device, 5)
                }),
                ..PeerOpts::default()
            },
            "p",
            tx,
        );
        let (axis, h) = wait_identified(&mut rx, "p").await;
        assert_eq!(axis, ax(want), "device {device} selected {selected}");
        assert_eq!((h.device_id, h.selected_axis), (device, selected));
        assert_eq!(link.axis(), Some(ax(want)));
        assert!(!peer.saw_eof());
    }
}

#[tokio::test(start_paused = true)]
async fn selected_axis_out_of_range_is_unknown_device() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(Hello {
                selected_axis: 4,
                ..hello(dev(1), 5)
            }),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    let (_, ev) = next(&mut rx, Duration::from_secs(1)).await;
    assert!(
        matches!(ev, LinkEvent::UnknownDevice { device_id, .. } if device_id == dev(1)),
        "{ev:?}"
    );
    sleep(Duration::from_millis(300)).await;
    assert!(only_hello_reqs(&peer.log()));
    assert!(link.is_finished());
}

#[tokio::test(start_paused = true)]
async fn axis_select_reidentifies_on_the_same_port() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(Hello {
                selected_axis: 1,
                ..hello(dev(1), 5)
            }),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    assert_eq!(wait_identified(&mut rx, "p").await.0, ax(1));
    sleep(Duration::from_millis(50)).await;

    peer.send_raw(axis_select(5, 2, 1)).await;
    let evs = collect_until_reidentified(&mut rx, "p").await;
    assert_eq!(evs.len(), 2, "{evs:?}");
    assert_eq!(
        evs[0],
        LinkEvent::Disconnected {
            axis: ax(1),
            reason: "axis_changed: 1->2".into()
        }
    );
    match &evs[1] {
        LinkEvent::Identified {
            axis,
            hello: h,
            port,
        } => {
            assert_eq!((*axis, port.as_str()), (ax(2), "p"));
            assert_eq!((h.device_id, h.boot_id, h.selected_axis), (dev(1), 5, 2));
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(link.axis(), Some(ax(2)));
    assert!(!link.is_finished());
    assert!(!peer.saw_eof());
    // Later frames are tagged with the new axis; the link still writes.
    let (_, ev) = wait_for(&mut rx, Duration::from_millis(100), |p, e| {
        p == "p" && matches!(e, LinkEvent::Status { .. })
    })
    .await;
    assert!(matches!(ev, LinkEvent::Status { axis, .. } if axis == ax(2)));
    link.send_admin(HostMsg::Disarm { request_id: 9 }).unwrap();
    sleep(Duration::from_millis(20)).await;
    assert!(peer
        .log()
        .iter()
        .any(|m| matches!(m, HostMsg::Disarm { request_id: 9 })));

    // The same value again: ignored (no Disconnected / Identified).
    peer.send_raw(axis_select(5, 2, 2)).await;
    sleep(Duration::from_millis(100)).await;
    while let Ok((_, ev)) = rx.try_recv() {
        assert!(matches!(ev, LinkEvent::Status { .. }), "{ev:?}");
    }
    assert_eq!(link.axis(), Some(ax(2)));
}

#[tokio::test(start_paused = true)]
async fn status_or_hello_with_another_selected_axis_switches_too() {
    // (c) STATUS: AXIS_SELECT lost → the next STATUS carries the new value.
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(Hello {
                selected_axis: 1,
                ..hello(dev(1), 5)
            }),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    assert_eq!(wait_identified(&mut rx, "p").await.0, ax(1));
    sleep(Duration::from_millis(50)).await;
    // The peer's periodic STATUS says 0 (old firmware): never a switch.
    while let Ok((_, ev)) = rx.try_recv() {
        assert!(
            matches!(ev, LinkEvent::Status { axis, .. } if axis == ax(1)),
            "{ev:?}"
        );
    }
    peer.send_raw(
        McuMsg::Status(Status {
            selected_axis: 3,
            ..status(5)
        })
        .to_wire(),
    )
    .await;
    let evs = collect_until_reidentified(&mut rx, "p").await;
    assert_eq!(evs.len(), 2, "{evs:?}");
    assert!(
        matches!(&evs[0], LinkEvent::Disconnected { axis, reason } if *axis == ax(1) && reason == "axis_changed: 1->3")
    );
    assert!(
        matches!(&evs[1], LinkEvent::Identified { axis, hello: h, .. } if *axis == ax(3) && h.selected_axis == 3)
    );
    // That STATUS itself is forwarded for the new axis.
    let (_, ev) = next(&mut rx, Duration::from_millis(50)).await;
    assert!(
        matches!(ev, LinkEvent::Status { axis, status, .. } if axis == ax(3) && status.selected_axis == 3),
        "{ev:?}"
    );
    assert_eq!(link.axis(), Some(ax(3)));

    // (b) HELLO naming another axis.
    peer.send_raw(
        McuMsg::Hello(Hello {
            selected_axis: 2,
            ..hello(dev(1), 5)
        })
        .to_wire(),
    )
    .await;
    let evs = collect_until_reidentified(&mut rx, "p").await;
    assert_eq!(evs.len(), 2, "{evs:?}");
    assert!(
        matches!(&evs[0], LinkEvent::Disconnected { axis, reason } if *axis == ax(3) && reason == "axis_changed: 3->2")
    );
    assert!(
        matches!(&evs[1], LinkEvent::Identified { axis, hello: h, .. } if *axis == ax(2) && h.selected_axis == 2 && h.boot_id == 5)
    );
    assert_eq!(link.axis(), Some(ax(2)));
    assert!(!link.is_finished());
}

#[tokio::test(start_paused = true)]
async fn axis_select_to_zero_without_config_fallback_ends_as_unknown_device() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(Hello {
                selected_axis: 2,
                ..hello(UNKNOWN, 5)
            }),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    assert_eq!(wait_identified(&mut rx, "p").await.0, ax(2));
    sleep(Duration::from_millis(50)).await;
    peer.send_raw(axis_select(5, 0, 2)).await;
    let evs = collect_until_reidentified(&mut rx, "p").await;
    assert_eq!(evs.len(), 2, "{evs:?}");
    assert_eq!(
        evs[0],
        LinkEvent::Disconnected {
            axis: ax(2),
            reason: "axis_changed: 2->0".into()
        }
    );
    assert_eq!(
        evs[1],
        LinkEvent::UnknownDevice {
            port: "p".into(),
            device_id: UNKNOWN
        }
    );
    sleep(Duration::from_millis(50)).await;
    assert!(link.is_finished());
    assert!(peer.saw_eof(), "port closed");
    assert_eq!(link.axis(), None);
    assert_eq!(
        link.send_admin(HostMsg::Disarm { request_id: 1 }),
        Err(LinkSendError::Closed)
    );
    // Exactly one Disconnected, no PortClosed, nothing after UnknownDevice.
    assert!(rx.try_recv().is_err());
}

#[tokio::test(start_paused = true)]
async fn axis_select_to_zero_falls_back_to_the_configured_device_id() {
    let (tx, mut rx) = events();
    let (peer, link) = peer_link(
        PeerOpts {
            hello: Some(Hello {
                selected_axis: 1,
                ..hello(dev(3), 5)
            }),
            ..PeerOpts::default()
        },
        "p",
        tx,
    );
    assert_eq!(wait_identified(&mut rx, "p").await.0, ax(1));
    sleep(Duration::from_millis(50)).await;
    peer.send_raw(axis_select(5, 0, 1)).await;
    let evs = collect_until_reidentified(&mut rx, "p").await;
    assert_eq!(evs.len(), 2, "{evs:?}");
    assert!(
        matches!(&evs[0], LinkEvent::Disconnected { axis, reason } if *axis == ax(1) && reason == "axis_changed: 1->3")
    );
    assert!(
        matches!(&evs[1], LinkEvent::Identified { axis, hello: h, .. } if *axis == ax(3) && h.selected_axis == 0)
    );
    assert_eq!(link.axis(), Some(ax(3)));
    assert!(!peer.saw_eof());
}

#[tokio::test(start_paused = true)]
async fn sim_select_axis_switches_the_link() {
    let (tx, mut rx) = events();
    let (io, sim) = sim_pair(SimParams {
        selected_axis: 1,
        ..sim_params(dev(1))
    });
    let link = spawn_link(io, "sim:p".into(), cfg(), tx);
    assert_eq!(wait_identified(&mut rx, "sim:p").await.0, ax(1));
    sleep(Duration::from_millis(30)).await;
    sim.select_axis(3);
    let evs = collect_until_reidentified(&mut rx, "sim:p").await;
    assert!(
        matches!(&evs[0], LinkEvent::Disconnected { axis, reason } if *axis == ax(1) && reason == "axis_changed: 1->3"),
        "{evs:?}"
    );
    assert!(
        matches!(&evs[1], LinkEvent::Identified { axis, hello: h, .. } if *axis == ax(3) && h.selected_axis == 3),
        "{evs:?}"
    );
    assert_eq!(link.axis(), Some(ax(3)));
    assert_eq!(sim.snapshot().selected_axis, 3);
}

// ------------------------------------------------------------------ serial open (DTR)

/// Records the modem-line writes of `assert_cdc_dtr`. Nothing is opened: the
/// real CDC port cannot be exercised in tests (never open a port here).
#[derive(Default)]
struct ModemLog {
    dtr: Vec<bool>,
    rts: Vec<bool>,
    fail_dtr: bool,
}

impl std::io::Read for ModemLog {
    fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
        Ok(0)
    }
}
impl std::io::Write for ModemLog {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

fn unsupported<T>() -> tokio_serial::Result<T> {
    Err(tokio_serial::Error::new(
        tokio_serial::ErrorKind::Unknown,
        "not a port",
    ))
}

impl tokio_serial::SerialPort for ModemLog {
    fn name(&self) -> Option<String> {
        None
    }
    fn baud_rate(&self) -> tokio_serial::Result<u32> {
        unsupported()
    }
    fn data_bits(&self) -> tokio_serial::Result<tokio_serial::DataBits> {
        unsupported()
    }
    fn flow_control(&self) -> tokio_serial::Result<tokio_serial::FlowControl> {
        unsupported()
    }
    fn parity(&self) -> tokio_serial::Result<tokio_serial::Parity> {
        unsupported()
    }
    fn stop_bits(&self) -> tokio_serial::Result<tokio_serial::StopBits> {
        unsupported()
    }
    fn timeout(&self) -> Duration {
        Duration::ZERO
    }
    fn set_baud_rate(&mut self, _: u32) -> tokio_serial::Result<()> {
        unsupported()
    }
    fn set_data_bits(&mut self, _: tokio_serial::DataBits) -> tokio_serial::Result<()> {
        unsupported()
    }
    fn set_flow_control(&mut self, _: tokio_serial::FlowControl) -> tokio_serial::Result<()> {
        unsupported()
    }
    fn set_parity(&mut self, _: tokio_serial::Parity) -> tokio_serial::Result<()> {
        unsupported()
    }
    fn set_stop_bits(&mut self, _: tokio_serial::StopBits) -> tokio_serial::Result<()> {
        unsupported()
    }
    fn set_timeout(&mut self, _: Duration) -> tokio_serial::Result<()> {
        unsupported()
    }
    fn write_request_to_send(&mut self, level: bool) -> tokio_serial::Result<()> {
        self.rts.push(level);
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, level: bool) -> tokio_serial::Result<()> {
        if self.fail_dtr {
            return Err(tokio_serial::Error::new(
                tokio_serial::ErrorKind::Io(std::io::ErrorKind::Unsupported),
                "ENOTTY",
            ));
        }
        self.dtr.push(level);
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> tokio_serial::Result<bool> {
        unsupported()
    }
    fn read_data_set_ready(&mut self) -> tokio_serial::Result<bool> {
        unsupported()
    }
    fn read_ring_indicator(&mut self) -> tokio_serial::Result<bool> {
        unsupported()
    }
    fn read_carrier_detect(&mut self) -> tokio_serial::Result<bool> {
        unsupported()
    }
    fn bytes_to_read(&self) -> tokio_serial::Result<u32> {
        Ok(0)
    }
    fn bytes_to_write(&self) -> tokio_serial::Result<u32> {
        Ok(0)
    }
    fn clear(&self, _: tokio_serial::ClearBuffer) -> tokio_serial::Result<()> {
        Ok(())
    }
    fn try_clone(&self) -> tokio_serial::Result<Box<dyn tokio_serial::SerialPort>> {
        unsupported()
    }
    fn set_break(&self) -> tokio_serial::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> tokio_serial::Result<()> {
        Ok(())
    }
}

#[test]
fn open_asserts_dtr_once_and_never_writes_rts() {
    // ESP32-S3 HWCDC transmits only while DTR is asserted; RTS must stay
    // untouched (an RTS+DTR sequence is the download-mode / reset trigger).
    let mut log = ModemLog::default();
    h3cs::axis_link::assert_cdc_dtr(&mut log).expect("DTR asserted");
    assert_eq!(log.dtr, vec![true]);
    assert!(log.rts.is_empty(), "RTS was written: {:?}", log.rts);
}

#[test]
fn dtr_failure_is_reported_not_panicked() {
    let mut log = ModemLog {
        fail_dtr: true,
        ..Default::default()
    };
    let err = h3cs::axis_link::assert_cdc_dtr(&mut log).expect_err("driver without the ioctl");
    assert!(err.to_string().contains("ENOTTY"), "{err}");
    assert!(log.rts.is_empty());
}
