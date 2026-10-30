//! Game input session and pose freshness (DESIGN.md §3, wire format §7.1).
//!
//! Single owner: issuing a session invalidates the previous one. A session is
//! not drive authority (ARM / START are admin-only). Freshness advances only
//! when `frame_seq` advances (or, in timer-only mode, per new `seq`); resends,
//! gain-only updates and anything else never extend it.

use std::time::Duration;

use serde::{Deserialize, Serialize};
use tokio::time::Instant;

use crate::compat::{self, CheckedPose, PoseError};
use crate::config::GameConfig;
use crate::types::{LimitReason, Pose};

/// Game API protocol version.
pub const PROTOCOL_VERSION: u32 = 1;
/// Hex digits of the session id published in status.
pub const SESSION_PREFIX_LEN: usize = 8;

// ------------------------------------------------------------------ wire

/// Client → server message on WS /v1/motion.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClientMsg {
    Pose(PoseMsg),
    Gain(GainMsg),
}

impl ClientMsg {
    /// Parse one JSON text message; any parse/shape error → `BadMessage`.
    pub fn parse(text: &str) -> Result<Self, RejectReason> {
        serde_json::from_str(text).map_err(|_| RejectReason::BadMessage)
    }

    pub fn seq(&self) -> u64 {
        match self {
            ClientMsg::Pose(p) => p.seq,
            ClientMsg::Gain(g) => g.seq,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct PoseMsg {
    pub protocol: u32,
    pub source_session: String,
    pub seq: u64,
    /// Game update counter; `None` only accepted when `allow_missing_frame_seq`.
    #[serde(default)]
    pub frame_seq: Option<u64>,
    /// Age of the pose at send time (ms since the game produced it).
    #[serde(default)]
    pub source_age_ms: f64,
    pub pitch: f64,
    pub roll: f64,
    pub yaw: f64,
    pub gain: f64,
}

impl PoseMsg {
    pub fn pose(&self) -> Pose {
        Pose { pitch: self.pitch, roll: self.roll, yaw: self.yaw }
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct GainMsg {
    pub protocol: u32,
    pub source_session: String,
    pub seq: u64,
    pub gain: f64,
}

/// Why a client message was not accepted (ack `reason`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, thiserror::Error)]
#[serde(rename_all = "snake_case")]
pub enum RejectReason {
    #[error("source_session is not the current session")]
    StaleSession,
    #[error("seq is not greater than the last accepted seq")]
    OldSeq,
    #[error("frame_seq is older than the last accepted frame_seq")]
    OldFrame,
    #[error("non-finite value")]
    NonFinite,
    #[error("pose component outside -1..=1")]
    OutOfRange,
    #[error("source_age_ms must be >= 0")]
    InvalidAge,
    #[error("frame_seq is required")]
    MissingFrameSeq,
    #[error("unsupported protocol version")]
    BadProtocol,
    #[error("malformed message")]
    BadMessage,
}

/// Server → client ack (§7.1). Sent for accepted and rejected messages.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AckMsg {
    #[serde(rename = "type")]
    pub kind: &'static str,
    /// `None` when the message could not be parsed.
    pub seq: Option<u64>,
    pub accepted: bool,
    pub reason: Option<RejectReason>,
    pub effective_gain: Option<f64>,
    pub limits: Vec<LimitReason>,
    /// Whether the session's latest pose is fresh after handling this message.
    pub fresh: bool,
}

impl AckMsg {
    pub fn accepted(seq: u64, effective_gain: f64, limits: Vec<LimitReason>, fresh: bool) -> Self {
        Self { kind: "ack", seq: Some(seq), accepted: true, reason: None, effective_gain: Some(effective_gain), limits, fresh }
    }
    pub fn rejected(seq: Option<u64>, reason: RejectReason, fresh: bool) -> Self {
        Self { kind: "ack", seq, accepted: false, reason: Some(reason), effective_gain: None, limits: Vec::new(), fresh }
    }
}

// ------------------------------------------------------------------ intake

/// How freshness of the latest pose is established.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FreshnessMode {
    /// Extended only when `frame_seq` advances.
    FrameSeq,
    /// No `frame_seq`: extended per new `seq` (cannot detect a frozen game loop).
    TimerOnly,
}

/// A pose the session accepted; the supervisor reads the latest one.
#[derive(Debug, Clone, PartialEq)]
pub struct AcceptedPose {
    /// As requested by the game (before clamping).
    pub pose: Pose,
    pub requested_gain: f64,
    /// Clamped pose, effective gain and limit reasons.
    pub checked: CheckedPose,
    /// Pose is usable for driving while `now < fresh_until`.
    pub fresh_until: Instant,
    pub frame_seq: Option<u64>,
    pub seq: u64,
    pub received_at: Instant,
    /// `received_at − source_age_ms`.
    pub produced_at: Instant,
    pub freshness_mode: FreshnessMode,
    /// Generation of the session that sent it ([`PoseIntake::session_gen`]).
    /// The supervisor binds a game drive to the generation current at START.
    pub session_gen: u64,
}

impl AcceptedPose {
    pub fn is_fresh(&self, now: Instant) -> bool {
        now < self.fresh_until
    }
}

/// Result of an accepted gain-only message.
#[derive(Debug, Clone, PartialEq)]
pub struct GainUpdate {
    pub requested_gain: f64,
    pub effective_gain: f64,
    pub limits: Vec<LimitReason>,
    /// The latest pose with the new gain applied (freshness unchanged), if any.
    pub pose: Option<AcceptedPose>,
}

/// `session` object of the status snapshot (§7.2).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SessionSnapshot {
    /// First [`SESSION_PREFIX_LEN`] hex digits of the current session. The
    /// full id is returned only by POST /v1/session (it gates pose input).
    pub source_session_prefix: Option<String>,
    pub fresh: bool,
    pub freshness_mode: FreshnessMode,
    /// Current age of the latest pose (`now − produced_at`), ms.
    pub source_age_ms: Option<f64>,
    /// Last accepted seq; 0 before any.
    pub last_seq: u64,
    pub last_frame_seq: Option<u64>,
}

/// Owns the current game session and decides which messages are accepted.
#[derive(Debug, Clone, Default)]
pub struct PoseIntake {
    session: Option<String>,
    /// +1 per issued session; 0 = none issued yet.
    session_gen: u64,
    last_seq: Option<u64>,
    last_frame_seq: Option<u64>,
    latest: Option<AcceptedPose>,
}

/// DESIGN.md §3 name for the same object.
pub type SessionManager = PoseIntake;

impl PoseIntake {
    pub fn new() -> Self {
        Self::default()
    }

    /// Issue a new session (128-bit random hex). The previous one becomes
    /// stale; seq/frame tracking and the latest pose are dropped.
    pub fn new_session(&mut self) -> String {
        let id = format!("{:032x}", rand::random::<u128>());
        let session_gen = self.session_gen.wrapping_add(1);
        *self = Self { session: Some(id.clone()), session_gen, ..Self::default() };
        id
    }

    pub fn current(&self) -> Option<&str> {
        self.session.as_deref()
    }

    /// Generation of the current session (+1 per [`Self::new_session`]).
    pub fn session_gen(&self) -> u64 {
        self.session_gen
    }

    pub fn latest(&self) -> Option<&AcceptedPose> {
        self.latest.as_ref()
    }

    pub fn is_fresh(&self, now: Instant) -> bool {
        self.latest.as_ref().is_some_and(|p| p.is_fresh(now))
    }

    /// Dispatch a parsed client message. Gain-only messages return the latest
    /// pose with the new gain, or `None` if there is no pose yet.
    pub fn accept(&mut self, msg: &ClientMsg, recv_at: Instant, cfg: &GameConfig, source_freshness_ms: u64)
        -> Result<Option<AcceptedPose>, RejectReason> {
        match msg {
            ClientMsg::Pose(p) => self.accept_pose(p, recv_at, cfg, source_freshness_ms).map(Some),
            ClientMsg::Gain(g) => self.accept_gain(g, cfg).map(|u| u.pose),
        }
    }

    /// Apply the §3 pose rules in order. State changes only on acceptance.
    pub fn accept_pose(&mut self, msg: &PoseMsg, recv_at: Instant, cfg: &GameConfig, source_freshness_ms: u64)
        -> Result<AcceptedPose, RejectReason> {
        self.check_header(msg.protocol, &msg.source_session, msg.seq)?;
        if !msg.source_age_ms.is_finite() {
            return Err(RejectReason::NonFinite);
        }
        let checked = compat::check_pose(msg.pose(), msg.gain, cfg.max_gain, cfg.clamp_out_of_range)
            .map_err(map_pose_error)?;
        if msg.source_age_ms < 0.0 {
            return Err(RejectReason::InvalidAge);
        }
        let age = Duration::try_from_secs_f64(msg.source_age_ms / 1000.0).map_err(|_| RejectReason::InvalidAge)?;
        let produced_at = recv_at.checked_sub(age).ok_or(RejectReason::InvalidAge)?;
        let window = Duration::from_millis(source_freshness_ms);
        let refreshed = produced_at.checked_add(window).unwrap_or(recv_at);

        let (fresh_until, mode) = match msg.frame_seq {
            Some(f) => {
                let fresh_until = match self.last_frame_seq {
                    Some(last) if f < last => return Err(RejectReason::OldFrame),
                    // Same frame resent: take the values, keep the old deadline.
                    Some(last) if f == last => match &self.latest {
                        Some(prev) => prev.fresh_until,
                        None => refreshed,
                    },
                    _ => refreshed,
                };
                (fresh_until, FreshnessMode::FrameSeq)
            }
            None if cfg.allow_missing_frame_seq => (refreshed, FreshnessMode::TimerOnly),
            None => return Err(RejectReason::MissingFrameSeq),
        };

        let accepted = AcceptedPose {
            pose: msg.pose(),
            requested_gain: msg.gain,
            checked,
            fresh_until,
            frame_seq: msg.frame_seq,
            seq: msg.seq,
            received_at: recv_at,
            produced_at,
            freshness_mode: mode,
            session_gen: self.session_gen,
        };
        self.last_seq = Some(msg.seq);
        if msg.frame_seq.is_some() {
            self.last_frame_seq = msg.frame_seq;
        }
        self.latest = Some(accepted.clone());
        Ok(accepted)
    }

    /// Gain-only update: re-checks the latest pose with the new gain and
    /// never touches its freshness.
    pub fn accept_gain(&mut self, msg: &GainMsg, cfg: &GameConfig) -> Result<GainUpdate, RejectReason> {
        self.check_header(msg.protocol, &msg.source_session, msg.seq)?;
        let gain_only = compat::check_pose(Pose::ZERO, msg.gain, cfg.max_gain, cfg.clamp_out_of_range)
            .map_err(map_pose_error)?;
        let pose = match &self.latest {
            Some(prev) => {
                let checked = compat::check_pose(prev.pose, msg.gain, cfg.max_gain, cfg.clamp_out_of_range)
                    .map_err(map_pose_error)?;
                Some(AcceptedPose { requested_gain: msg.gain, checked, ..prev.clone() })
            }
            None => None,
        };
        self.last_seq = Some(msg.seq);
        if let Some(p) = &pose {
            self.latest = Some(p.clone());
        }
        let (effective_gain, limits) = match &pose {
            Some(p) => (p.checked.effective_gain, p.checked.limits.clone()),
            None => (gain_only.effective_gain, gain_only.limits),
        };
        Ok(GainUpdate { requested_gain: msg.gain, effective_gain, limits, pose })
    }

    pub fn snapshot(&self, now: Instant) -> SessionSnapshot {
        SessionSnapshot {
            source_session_prefix: self
                .session
                .as_ref()
                .map(|s| s.chars().take(SESSION_PREFIX_LEN).collect()),
            fresh: self.is_fresh(now),
            freshness_mode: self.latest.as_ref().map_or(FreshnessMode::FrameSeq, |p| p.freshness_mode),
            source_age_ms: self
                .latest
                .as_ref()
                .map(|p| now.saturating_duration_since(p.produced_at).as_secs_f64() * 1000.0),
            last_seq: self.last_seq.unwrap_or(0),
            last_frame_seq: self.last_frame_seq,
        }
    }

    /// Rules 1–2: protocol, session, seq.
    fn check_header(&self, protocol: u32, session: &str, seq: u64) -> Result<(), RejectReason> {
        if protocol != PROTOCOL_VERSION {
            return Err(RejectReason::BadProtocol);
        }
        if self.session.as_deref() != Some(session) {
            return Err(RejectReason::StaleSession);
        }
        if self.last_seq.is_some_and(|last| seq <= last) {
            return Err(RejectReason::OldSeq);
        }
        Ok(())
    }
}

fn map_pose_error(e: PoseError) -> RejectReason {
    match e {
        PoseError::NonFinite(_) => RejectReason::NonFinite,
        PoseError::OutOfRange(_) => RejectReason::OutOfRange,
    }
}
