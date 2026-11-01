//! session: single-owner sessions, pose acceptance and freshness rules.

use std::time::Duration;

use h3cs::config::GameConfig;
use h3cs::limits::{LimitsPatch, MotionLimits};
use h3cs::session::{AckMsg, ClientMsg, FreshnessMode, GainMsg, LimitsMsg, PoseIntake, PoseMsg, RejectReason};
use h3cs::types::{LimitReason, Pose};
use tokio::time::Instant;

const FRESH_MS: u64 = 250;

fn cfg() -> GameConfig {
    GameConfig::default()
}

/// Gain ceiling handed to the intake (the shared `[limits].max_gain`, which
/// used to be `game.max_gain = 0.35`).
const MAX_GAIN: f64 = 0.35;

fn pose_msg(session: &str, seq: u64, frame_seq: Option<u64>) -> PoseMsg {
    PoseMsg {
        protocol: 1,
        source_session: session.to_string(),
        seq,
        frame_seq,
        source_age_ms: 0.0,
        pitch: 0.3,
        roll: -0.2,
        yaw: 0.0,
        gain: 0.3,
    }
}

fn gain_msg(session: &str, seq: u64, gain: f64) -> GainMsg {
    GainMsg { protocol: 1, source_session: session.to_string(), seq, gain }
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

fn intake() -> (PoseIntake, String) {
    let mut s = PoseIntake::new();
    let id = s.new_session();
    (s, id)
}

#[test]
fn new_session_is_128_bit_hex_and_replaces_the_previous() {
    let mut s = PoseIntake::new();
    assert_eq!(s.current(), None);
    let a = s.new_session();
    assert_eq!(a.len(), 32);
    assert!(a.chars().all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase()));
    let b = s.new_session();
    assert_ne!(a, b);
    assert_eq!(s.current(), Some(b.as_str()));
    let now = Instant::now();
    assert_eq!(s.accept_pose(&pose_msg(&a, 1, Some(1)), now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::StaleSession));
    assert!(s.accept_pose(&pose_msg(&b, 1, Some(1)), now, &cfg(), MAX_GAIN, FRESH_MS).is_ok());
}

#[test]
fn no_session_rejects_everything() {
    let mut s = PoseIntake::new();
    let r = s.accept_pose(&pose_msg("", 1, Some(1)), Instant::now(), &cfg(), MAX_GAIN, FRESH_MS);
    assert_eq!(r, Err(RejectReason::StaleSession));
}

#[tokio::test(start_paused = true)]
async fn accepted_pose_is_fresh_for_the_window() {
    let (mut s, id) = intake();
    let t0 = Instant::now();
    let p = s.accept_pose(&pose_msg(&id, 1, Some(1)), t0, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(p.fresh_until, t0 + ms(FRESH_MS));
    assert_eq!(p.produced_at, t0);
    assert_eq!(p.freshness_mode, FreshnessMode::FrameSeq);
    assert_eq!(p.pose, Pose { pitch: 0.3, roll: -0.2, yaw: 0.0 });
    assert_eq!(p.checked.effective_gain, 0.3);
    assert!(s.is_fresh(t0));
    tokio::time::advance(ms(249)).await;
    assert!(s.is_fresh(Instant::now()));
    tokio::time::advance(ms(1)).await;
    assert!(!s.is_fresh(Instant::now()));
}

#[tokio::test(start_paused = true)]
async fn resend_of_same_frame_seq_does_not_extend_freshness() {
    let (mut s, id) = intake();
    let t0 = Instant::now();
    s.accept_pose(&pose_msg(&id, 1, Some(10)), t0, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    tokio::time::advance(ms(200)).await;
    let mut m = pose_msg(&id, 2, Some(10));
    m.pitch = 0.9;
    let p = s.accept_pose(&m, Instant::now(), &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    // Values taken, deadline unchanged.
    assert_eq!(p.pose.pitch, 0.9);
    assert_eq!(s.latest().unwrap().pose.pitch, 0.9);
    assert_eq!(p.fresh_until, t0 + ms(FRESH_MS));
    tokio::time::advance(ms(60)).await;
    assert!(!s.is_fresh(Instant::now()));
    // A new frame extends again.
    let now = Instant::now();
    let p = s.accept_pose(&pose_msg(&id, 3, Some(11)), now, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(p.fresh_until, now + ms(FRESH_MS));
    assert!(s.is_fresh(now));
}

#[tokio::test(start_paused = true)]
async fn gain_only_update_does_not_extend_freshness() {
    let (mut s, id) = intake();
    let t0 = Instant::now();
    s.accept_pose(&pose_msg(&id, 1, Some(1)), t0, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    tokio::time::advance(ms(200)).await;
    let u = s.accept_gain(&gain_msg(&id, 2, 0.1), &cfg(), MAX_GAIN).unwrap();
    assert_eq!(u.effective_gain, 0.1);
    let p = u.pose.unwrap();
    assert_eq!(p.fresh_until, t0 + ms(FRESH_MS));
    assert_eq!(p.requested_gain, 0.1);
    assert_eq!(p.checked.effective_gain, 0.1);
    assert_eq!(p.seq, 1, "pose seq is the pose message's");
    assert_eq!(s.latest().unwrap().checked.effective_gain, 0.1);
    tokio::time::advance(ms(60)).await;
    assert!(!s.is_fresh(Instant::now()));
    // The gain message consumed seq 2.
    assert_eq!(s.accept_pose(&pose_msg(&id, 2, Some(2)), Instant::now(), &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::OldSeq));
    assert_eq!(s.snapshot(Instant::now()).last_seq, 2);
}

#[test]
fn gain_without_a_pose_reports_effective_gain() {
    let (mut s, id) = intake();
    let u = s.accept_gain(&gain_msg(&id, 1, 0.9), &cfg(), MAX_GAIN).unwrap();
    assert!(u.pose.is_none());
    assert_eq!(u.effective_gain, 0.35);
    assert_eq!(u.limits, vec![LimitReason::GainLimited]);
    assert!(s.latest().is_none());
}

#[test]
fn gain_message_rules() {
    let (mut s, id) = intake();
    let now = Instant::now();
    s.accept_pose(&pose_msg(&id, 5, Some(1)), now, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(s.accept_gain(&gain_msg(&id, 5, 0.1), &cfg(), MAX_GAIN), Err(RejectReason::OldSeq));
    assert_eq!(s.accept_gain(&gain_msg("other", 6, 0.1), &cfg(), MAX_GAIN), Err(RejectReason::StaleSession));
    assert_eq!(s.accept_gain(&gain_msg(&id, 6, f64::NAN), &cfg(), MAX_GAIN), Err(RejectReason::NonFinite));
    let mut g = gain_msg(&id, 6, 0.1);
    g.protocol = 2;
    assert_eq!(s.accept_gain(&g, &cfg(), MAX_GAIN), Err(RejectReason::BadProtocol));
    let u = s.accept_gain(&gain_msg(&id, 6, -0.1), &cfg(), MAX_GAIN).unwrap();
    assert_eq!(u.effective_gain, 0.0);
    assert_eq!(u.limits, vec![LimitReason::GainNegative]);
}

#[test]
fn old_seq_is_rejected_and_state_unchanged() {
    let (mut s, id) = intake();
    let now = Instant::now();
    s.accept_pose(&pose_msg(&id, 10, Some(10)), now, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(s.accept_pose(&pose_msg(&id, 10, Some(11)), now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::OldSeq));
    assert_eq!(s.accept_pose(&pose_msg(&id, 9, Some(12)), now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::OldSeq));
    let snap = s.snapshot(now);
    assert_eq!(snap.last_seq, 10);
    assert_eq!(snap.last_frame_seq, Some(10));
}

#[test]
fn first_seq_may_be_zero() {
    let (mut s, id) = intake();
    assert!(s.accept_pose(&pose_msg(&id, 0, Some(0)), Instant::now(), &cfg(), MAX_GAIN, FRESH_MS).is_ok());
}

#[test]
fn old_frame_is_rejected() {
    let (mut s, id) = intake();
    let now = Instant::now();
    s.accept_pose(&pose_msg(&id, 1, Some(10)), now, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(s.accept_pose(&pose_msg(&id, 2, Some(9)), now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::OldFrame));
    // Rejection does not consume the seq.
    assert!(s.accept_pose(&pose_msg(&id, 2, Some(11)), now, &cfg(), MAX_GAIN, FRESH_MS).is_ok());
}

#[test]
fn stale_session_and_bad_protocol() {
    let (mut s, id) = intake();
    let now = Instant::now();
    assert_eq!(s.accept_pose(&pose_msg("deadbeef", 1, Some(1)), now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::StaleSession));
    let mut m = pose_msg(&id, 1, Some(1));
    m.protocol = 0;
    assert_eq!(s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::BadProtocol));
}

#[tokio::test(start_paused = true)]
async fn new_session_resets_tracking_and_drops_latest() {
    let (mut s, id) = intake();
    let now = Instant::now();
    s.accept_pose(&pose_msg(&id, 100, Some(100)), now, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert!(s.is_fresh(now));
    let id2 = s.new_session();
    assert!(s.latest().is_none());
    assert!(!s.is_fresh(now));
    let snap = s.snapshot(now);
    assert_eq!(snap.last_seq, 0);
    assert_eq!(snap.last_frame_seq, None);
    assert!(s.accept_pose(&pose_msg(&id2, 1, Some(1)), now, &cfg(), MAX_GAIN, FRESH_MS).is_ok());
}

#[tokio::test(start_paused = true)]
async fn source_age_shortens_freshness() {
    let (mut s, id) = intake();
    let now = Instant::now();
    let mut m = pose_msg(&id, 1, Some(1));
    m.source_age_ms = 100.0;
    let p = s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(p.produced_at, now - ms(100));
    assert_eq!(p.fresh_until, now + ms(150));
    assert_eq!(p.received_at, now);
    let snap = s.snapshot(now);
    assert_eq!(snap.source_age_ms, Some(100.0));
    tokio::time::advance(ms(150)).await;
    assert!(!s.is_fresh(Instant::now()));

    // Older than the window: accepted, but never fresh.
    let mut m = pose_msg(&id, 2, Some(2));
    m.source_age_ms = 300.0;
    let now = Instant::now();
    let p = s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert!(!p.is_fresh(now));
    assert!(!s.is_fresh(now));
}

#[tokio::test(start_paused = true)]
async fn invalid_age_is_rejected() {
    let (mut s, id) = intake();
    // Move the clock forward so subtracting a small age cannot underflow.
    tokio::time::advance(ms(1000)).await;
    let now = Instant::now();
    let mut m = pose_msg(&id, 1, Some(1));
    m.source_age_ms = -1.0;
    assert_eq!(s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::InvalidAge));
    m.source_age_ms = f64::NAN;
    assert_eq!(s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::NonFinite));
    m.source_age_ms = f64::INFINITY;
    assert_eq!(s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::NonFinite));
    // Finite but absurd (beyond Duration / before the clock origin): no panic.
    m.source_age_ms = 1e300;
    assert_eq!(s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::InvalidAge));
    m.source_age_ms = 1e15;
    assert!(matches!(s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::InvalidAge) | Ok(_)));
}

#[tokio::test(start_paused = true)]
async fn missing_frame_seq_is_rejected_by_default() {
    let (mut s, id) = intake();
    let r = s.accept_pose(&pose_msg(&id, 1, None), Instant::now(), &cfg(), MAX_GAIN, FRESH_MS);
    assert_eq!(r, Err(RejectReason::MissingFrameSeq));
}

#[tokio::test(start_paused = true)]
async fn missing_frame_seq_allowed_is_timer_only_and_refreshes_per_seq() {
    let (mut s, id) = intake();
    let c = GameConfig { allow_missing_frame_seq: true, ..GameConfig::default() };
    let t0 = Instant::now();
    let p = s.accept_pose(&pose_msg(&id, 1, None), t0, &c, MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(p.freshness_mode, FreshnessMode::TimerOnly);
    assert_eq!(s.snapshot(t0).freshness_mode, FreshnessMode::TimerOnly);
    tokio::time::advance(ms(200)).await;
    let t1 = Instant::now();
    let p = s.accept_pose(&pose_msg(&id, 2, None), t1, &c, MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(p.fresh_until, t1 + ms(FRESH_MS));
    // A repeated seq still does not extend.
    assert_eq!(s.accept_pose(&pose_msg(&id, 2, None), t1, &c, MAX_GAIN, FRESH_MS), Err(RejectReason::OldSeq));
}

#[test]
fn non_finite_pose_is_rejected() {
    let (mut s, id) = intake();
    let now = Instant::now();
    for f in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for field in 0..4 {
            let mut m = pose_msg(&id, 1, Some(1));
            match field {
                0 => m.pitch = f,
                1 => m.roll = f,
                2 => m.yaw = f,
                _ => m.gain = f,
            }
            assert_eq!(s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS), Err(RejectReason::NonFinite));
        }
    }
    assert!(s.latest().is_none());
    assert_eq!(s.snapshot(now).last_seq, 0);
}

#[test]
fn out_of_range_is_clamped_or_rejected_per_config() {
    let (mut s, id) = intake();
    let now = Instant::now();
    let mut m = pose_msg(&id, 1, Some(1));
    m.roll = 1.5;
    let p = s.accept_pose(&m, now, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(p.pose.roll, 1.5, "requested pose kept as sent");
    assert_eq!(p.checked.pose.roll, 1.0);
    assert!(p.checked.limits.contains(&LimitReason::PoseClamped));

    let strict = GameConfig { clamp_out_of_range: false, ..GameConfig::default() };
    let mut m = pose_msg(&id, 2, Some(2));
    m.yaw = -1.01;
    assert_eq!(s.accept_pose(&m, now, &strict, MAX_GAIN, FRESH_MS), Err(RejectReason::OutOfRange));
}

#[test]
fn gain_over_max_is_limited_with_effective_gain() {
    let (mut s, id) = intake();
    let mut m = pose_msg(&id, 1, Some(1));
    m.gain = 0.6;
    let p = s.accept_pose(&m, Instant::now(), &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    assert_eq!(p.requested_gain, 0.6);
    assert_eq!(p.checked.effective_gain, 0.35);
    assert_eq!(p.checked.limits, vec![LimitReason::GainLimited]);
    let u = s.accept_gain(&gain_msg(&id, 2, 0.5), &cfg(), MAX_GAIN).unwrap();
    assert_eq!(u.effective_gain, 0.35);
    assert_eq!(u.limits, vec![LimitReason::GainLimited]);
    let u = s.accept_gain(&gain_msg(&id, 3, 0.2), &cfg(), MAX_GAIN).unwrap();
    assert_eq!(u.effective_gain, 0.2);
    assert!(u.limits.is_empty());
}

#[tokio::test(start_paused = true)]
async fn accept_dispatches_by_type() {
    let (mut s, id) = intake();
    let now = Instant::now();
    let m = ClientMsg::Pose(pose_msg(&id, 1, Some(1)));
    assert_eq!(m.seq(), 1);
    assert!(s.accept(&m, now, &cfg(), MAX_GAIN, FRESH_MS).unwrap().is_some());
    let g = ClientMsg::Gain(gain_msg(&id, 2, 0.2));
    let p = s.accept(&g, now, &cfg(), MAX_GAIN, FRESH_MS).unwrap().unwrap();
    assert_eq!(p.checked.effective_gain, 0.2);
}

#[test]
fn snapshot_shape_matches_status_contract() {
    let (s, id) = intake();
    let v = serde_json::to_value(s.snapshot(Instant::now())).unwrap();
    assert_eq!(
        v,
        serde_json::json!({"source_session_prefix": &id[..8], "fresh": false, "freshness_mode": "frame_seq",
                           "source_age_ms": null, "last_seq": 0, "last_frame_seq": null})
    );
}

// ------------------------------------------------------------------ wire

#[test]
fn parses_pose_and_gain_messages() {
    let m = ClientMsg::parse(
        r#"{"type":"pose","protocol":1,"source_session":"ab","seq":1250,"frame_seq":2408,"source_age_ms":3.0,
            "pitch":0.3,"roll":-0.2,"yaw":0.0,"gain":0.35}"#,
    )
    .unwrap();
    let ClientMsg::Pose(p) = m else { panic!("not a pose") };
    assert_eq!((p.seq, p.frame_seq, p.source_age_ms, p.gain), (1250, Some(2408), 3.0, 0.35));

    // frame_seq and source_age_ms are optional; unknown keys are tolerated.
    let m = ClientMsg::parse(
        r#"{"type":"pose","protocol":1,"source_session":"ab","seq":1,"pitch":0,"roll":0,"yaw":0,"gain":0.3,"extra":1}"#,
    )
    .unwrap();
    let ClientMsg::Pose(p) = m else { panic!("not a pose") };
    assert_eq!((p.frame_seq, p.source_age_ms), (None, 0.0));

    let m = ClientMsg::parse(r#"{"type":"gain","protocol":1,"source_session":"ab","seq":1251,"gain":0.3}"#).unwrap();
    assert_eq!(m, ClientMsg::Gain(GainMsg { protocol: 1, source_session: "ab".into(), seq: 1251, gain: 0.3 }));
}

#[test]
fn malformed_messages_are_bad_message() {
    for text in [
        "",
        "not json",
        "{}",
        r#"{"type":"teleport"}"#,
        r#"{"type":"pose","protocol":1,"source_session":"ab","seq":-1,"pitch":0,"roll":0,"yaw":0,"gain":0}"#,
        r#"{"type":"pose","protocol":1,"source_session":"ab","seq":1,"pitch":"0","roll":0,"yaw":0,"gain":0}"#,
        r#"{"type":"gain","protocol":1,"source_session":"ab","seq":1}"#,
        r#"[1,2,3]"#,
    ] {
        assert_eq!(ClientMsg::parse(text), Err(RejectReason::BadMessage), "{text}");
    }
}

#[test]
fn reject_reasons_serialize_snake_case() {
    let all = [
        (RejectReason::StaleSession, "stale_session"),
        (RejectReason::OldSeq, "old_seq"),
        (RejectReason::OldFrame, "old_frame"),
        (RejectReason::NonFinite, "non_finite"),
        (RejectReason::OutOfRange, "out_of_range"),
        (RejectReason::InvalidAge, "invalid_age"),
        (RejectReason::MissingFrameSeq, "missing_frame_seq"),
        (RejectReason::BadProtocol, "bad_protocol"),
        (RejectReason::BadMessage, "bad_message"),
    ];
    for (r, s) in all {
        assert_eq!(serde_json::to_value(r).unwrap(), serde_json::json!(s));
    }
}

#[test]
fn ack_serializes_like_the_contract() {
    let a = AckMsg::accepted(1250, 0.35, vec![LimitReason::PoseClamped], true);
    assert_eq!(
        serde_json::to_value(&a).unwrap(),
        serde_json::json!({"type":"ack","seq":1250,"accepted":true,"reason":null,"effective_gain":0.35,
                           "limits":["pose_clamped"],"fresh":true})
    );
    let r = AckMsg::rejected(None, RejectReason::BadMessage, false);
    assert_eq!(
        serde_json::to_value(&r).unwrap(),
        serde_json::json!({"type":"ack","seq":null,"accepted":false,"reason":"bad_message","effective_gain":null,
                           "limits":[],"fresh":false})
    );
}

// ------------------------------------------------------------------ limits (design 2026-11-01 §5.3)

fn limits_msg(session: &str, seq: u64, patch: LimitsPatch) -> LimitsMsg {
    LimitsMsg { protocol: 1, source_session: session.to_string(), seq, patch }
}

#[test]
fn limits_message_consumes_seq_only_on_success_and_never_touches_freshness() {
    let (mut s, id) = intake();
    let t0 = Instant::now();
    s.accept_pose(&pose_msg(&id, 1, Some(1)), t0, &cfg(), MAX_GAIN, FRESH_MS).unwrap();
    let before = s.latest().unwrap().clone();
    let m = limits_msg(&id, 2, LimitsPatch { max_gain: Some(0.9), ..Default::default() });
    // The caller's patch application fails: seq not consumed.
    assert_eq!(s.accept_limits(&m, || Err::<(), _>(RejectReason::OutOfRange)), Err(RejectReason::OutOfRange));
    assert_eq!(s.snapshot(t0).last_seq, 1);
    // Succeeds: seq consumed, latest pose and freshness unchanged.
    assert_eq!(s.accept_limits(&m, || Ok(7)), Ok(7));
    assert_eq!(s.snapshot(t0).last_seq, 2);
    assert_eq!(s.latest().unwrap(), &before);
    // Header rules as for pose / gain.
    assert_eq!(s.accept_limits(&m, || Ok(())), Err(RejectReason::OldSeq));
    let stale = limits_msg("other", 3, LimitsPatch::default());
    assert_eq!(s.accept_limits(&stale, || Ok(())), Err(RejectReason::StaleSession));
    let mut bad = limits_msg(&id, 3, LimitsPatch::default());
    bad.protocol = 2;
    assert_eq!(s.accept_limits(&bad, || Ok(())), Err(RejectReason::BadProtocol));
    // Through `accept`: a limits message yields no pose.
    let m = ClientMsg::Limits(limits_msg(&id, 3, LimitsPatch::default()));
    assert_eq!(s.accept(&m, t0, &cfg(), MAX_GAIN, FRESH_MS), Ok(None));
    assert_eq!(s.snapshot(t0).last_seq, 3);
}

#[test]
fn limits_message_parses_present_fields_only() {
    let m = ClientMsg::parse(r#"{"type":"limits","protocol":1,"source_session":"abc","seq":5,"max_vel_count_s":1200,"duty_max_percent":35}"#)
        .unwrap();
    let ClientMsg::Limits(l) = m else { panic!("{m:?}") };
    assert_eq!(l.seq, 5);
    assert_eq!(l.source_session, "abc");
    assert_eq!(
        l.patch,
        LimitsPatch { max_vel_count_s: Some(1200.0), duty_max_percent: Some(35.0), ..Default::default() }
    );
    let none = ClientMsg::parse(r#"{"type":"limits","protocol":1,"source_session":"abc","seq":6}"#).unwrap();
    assert!(matches!(none, ClientMsg::Limits(l) if l.patch.is_empty()));
    assert_eq!(
        ClientMsg::parse(r#"{"type":"limits","protocol":1,"source_session":"abc","seq":"x"}"#),
        Err(RejectReason::BadMessage)
    );
}

#[test]
fn limits_ack_carries_the_effective_set() {
    let e = MotionLimits { max_gain: 0.8, max_vel_count_s: 6000, max_acc_count_s2: 15000, duty_max_percent: 40 };
    let a = AckMsg::accepted_limits(9, e, vec![LimitReason::CeilingClamped], false);
    let v: serde_json::Value = serde_json::from_str(&serde_json::to_string(&a).unwrap()).unwrap();
    assert_eq!(v["accepted"], true);
    assert_eq!(v["effective_gain"], 0.8);
    assert_eq!(v["effective_limits"]["max_vel_count_s"], 6000);
    assert_eq!(v["limits"][0], "ceiling_clamped");
    // Pose acks do not carry the field at all.
    let p = AckMsg::accepted(1, 0.3, vec![], true);
    let v: serde_json::Value = serde_json::from_str(&serde_json::to_string(&p).unwrap()).unwrap();
    assert!(v.get("effective_limits").is_none());
}
