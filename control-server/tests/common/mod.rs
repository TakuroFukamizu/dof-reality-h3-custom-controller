//! Shared helpers for the API / server tests: a tiny HTTP/1.1 client over a
//! raw TCP stream, WebSocket helpers and a sim server starter.
#![allow(dead_code)]

use std::net::SocketAddr;
use std::path::Path;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use h3cs::config::Config;
use h3cs::server::{self, RunningServer, ServerOptions};
use h3cs::supervisor::StatusSnapshot;
use h3cs::types::RigState;
use serde_json::{json, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::{MaybeTlsStream, WebSocketStream};

pub const SIM: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/config/sim.toml");
pub const TOKEN: &str = "test-admin-token-0123456789abcdef";

pub type Ws = WebSocketStream<MaybeTlsStream<TcpStream>>;

pub fn sim_cfg() -> Config {
    Config::load(Path::new(SIM)).expect("config/sim.toml")
}

/// No config path: `POST /v1/calibration/export` answers 500 `no_config_path`
/// (a test must never write into the repository's `config/`).
pub async fn start_sim(cfg: Config) -> RunningServer {
    start_sim_with_path(cfg, None).await
}

/// Like [`start_sim`] with an explicit config path (where
/// `POST /v1/calibration/export` writes; `None` → 500 `no_config_path`).
pub async fn start_sim_with_path(cfg: Config, config_path: Option<std::path::PathBuf>) -> RunningServer {
    server::start(
        cfg,
        ServerOptions {
            admin_token: Some(TOKEN.into()),
            bind_override: Some("127.0.0.1:0".parse().unwrap()),
            config_path,
        },
    )
    .await
    .expect("server start")
}

#[derive(Debug)]
pub struct Resp {
    pub status: u16,
    pub body: String,
}

impl Resp {
    pub fn json(&self) -> Value {
        serde_json::from_str(&self.body).unwrap_or(Value::Null)
    }
}

/// One HTTP/1.1 request with `Connection: close`.
pub async fn http(addr: SocketAddr, method: &str, path: &str, headers: &[(&str, &str)], body: Option<&str>) -> Resp {
    let mut s = TcpStream::connect(addr).await.expect("connect");
    let mut req = format!("{method} {path} HTTP/1.1\r\nHost: {addr}\r\nConnection: close\r\n");
    for (k, v) in headers {
        req.push_str(&format!("{k}: {v}\r\n"));
    }
    let body = body.unwrap_or("");
    if !body.is_empty() || method == "POST" {
        req.push_str(&format!("Content-Type: application/json\r\nContent-Length: {}\r\n", body.len()));
    }
    req.push_str("\r\n");
    req.push_str(body);
    s.write_all(req.as_bytes()).await.expect("write");
    let mut raw = Vec::new();
    let _ = tokio::time::timeout(Duration::from_secs(5), s.read_to_end(&mut raw)).await;
    let text = String::from_utf8_lossy(&raw).to_string();
    let (head, rest) = text.split_once("\r\n\r\n").unwrap_or((&text, ""));
    let status = head.split_whitespace().nth(1).and_then(|c| c.parse().ok()).unwrap_or(0);
    let chunked = head.to_ascii_lowercase().contains("transfer-encoding: chunked");
    let body = if chunked { dechunk(rest) } else { rest.to_string() };
    Resp { status, body }
}

fn dechunk(mut s: &str) -> String {
    let mut out = String::new();
    while let Some((len, rest)) = s.split_once("\r\n") {
        let Ok(n) = usize::from_str_radix(len.trim(), 16) else { break };
        if n == 0 || rest.len() < n {
            break;
        }
        out.push_str(&rest[..n]);
        s = rest[n..].trim_start_matches("\r\n");
    }
    out
}

pub async fn admin(addr: SocketAddr, path: &str) -> Resp {
    let auth = format!("Bearer {TOKEN}");
    http(addr, "POST", path, &[("Authorization", &auth)], None).await
}

pub async fn new_session(addr: SocketAddr) -> String {
    let r = http(addr, "POST", "/v1/session", &[], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    r.json()["source_session"].as_str().expect("source_session").to_string()
}

pub async fn ws_connect(addr: SocketAddr, session: &str) -> Ws {
    let (ws, _) = tokio_tungstenite::connect_async(format!("ws://{addr}/v1/motion?session={session}"))
        .await
        .expect("ws connect");
    ws
}

pub fn pose_json(session: &str, seq: u64, frame_seq: u64, pitch: f64, roll: f64, yaw: f64, gain: f64) -> String {
    json!({"type": "pose", "protocol": 1, "source_session": session, "seq": seq, "frame_seq": frame_seq,
           "source_age_ms": 1.0, "pitch": pitch, "roll": roll, "yaw": yaw, "gain": gain})
    .to_string()
}

pub async fn ws_send(ws: &mut Ws, text: String) {
    ws.send(Message::Text(text.into())).await.expect("ws send");
}

/// Next ack (status messages are skipped).
pub async fn next_ack(ws: &mut Ws) -> Value {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    loop {
        let m = tokio::time::timeout_at(deadline, ws.next()).await.expect("ack in time");
        let Some(Ok(Message::Text(t))) = m else { panic!("ws closed or non-text: {m:?}") };
        let v: Value = serde_json::from_str(t.as_str()).expect("json");
        if v["type"] == "ack" {
            return v;
        }
    }
}

/// Next status message (acks are skipped).
pub async fn next_status(ws: &mut Ws) -> Value {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    loop {
        let m = tokio::time::timeout_at(deadline, ws.next()).await.expect("status in time");
        let Some(Ok(Message::Text(t))) = m else { panic!("ws closed or non-text: {m:?}") };
        let v: Value = serde_json::from_str(t.as_str()).expect("json");
        if v["type"] == "status" {
            return v;
        }
    }
}

/// Poll the server's status until `pred` holds (panics after `within`).
pub async fn wait_status(
    srv: &RunningServer,
    within: Duration,
    what: &str,
    pred: impl Fn(&StatusSnapshot) -> bool,
) -> std::sync::Arc<StatusSnapshot> {
    let deadline = tokio::time::Instant::now() + within;
    loop {
        let s = srv.status();
        if pred(&s) {
            return s;
        }
        if tokio::time::Instant::now() >= deadline {
            panic!("timed out waiting for {what}; rig_state {:?} blockers {:?} fault {:?}", s.rig_state, s.blockers, s.fault);
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

pub async fn wait_state(srv: &RunningServer, state: RigState, within: Duration) -> std::sync::Arc<StatusSnapshot> {
    wait_status(srv, within, &format!("{state:?}"), |s| s.rig_state == state).await
}
