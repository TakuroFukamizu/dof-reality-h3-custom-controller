//! h3-control-server CLI (DESIGN.md §1).
//!
//! - `h3-control-server --config <path> [--bind <addr>]` runs the server.
//! - `h3-control-server check-config <path>` validates a config and prints
//!   calibration hashes and the ARM blockers knowable without hardware.
//! - `h3-control-server list-ports` lists serial ports (opens nothing).

use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand};
use h3cs::calibration::{calibration_hash, AxisCalibration};
use h3cs::config::{BackendConfig, Config};
use h3cs::server::{self, ServerOptions};

#[derive(Parser)]
#[command(name = "h3-control-server", version, about = "DOF Reality H3 control server")]
#[command(args_conflicts_with_subcommands = true)]
struct Cli {
    #[command(subcommand)]
    cmd: Option<Cmd>,
    /// Configuration file (TOML).
    #[arg(long)]
    config: Option<PathBuf>,
    /// Override `server.bind` (must be loopback unless allowed in the config).
    #[arg(long)]
    bind: Option<SocketAddr>,
}

#[derive(Subcommand)]
enum Cmd {
    /// Validate a configuration and print per-axis calibration hashes and
    /// static ARM blockers.
    CheckConfig { path: PathBuf },
    /// List serial ports (enumeration only; nothing is opened).
    ListPorts,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match cli.cmd {
        Some(Cmd::CheckConfig { path }) => check_config(&path),
        Some(Cmd::ListPorts) => list_ports(),
        None => match cli.config {
            Some(path) => run(&path, cli.bind),
            None => {
                eprintln!("error: --config <path> is required (see --help)");
                ExitCode::from(2)
            }
        },
    }
}

fn check_config(path: &Path) -> ExitCode {
    let cfg = match Config::load(path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{}: {e}", path.display());
            return ExitCode::FAILURE;
        }
    };
    let backend = match &cfg.backend {
        BackendConfig::Sim(_) => "sim".to_string(),
        BackendConfig::Serial(s) => format!("serial {:?}", s.ports),
    };
    println!("{}: OK (rig_id {}, bind {}, backend {backend})", path.display(), cfg.rig_id, cfg.server.bind);
    let mut blockers = Vec::new();
    let mut axes: Vec<_> = cfg.axes.iter().collect();
    axes.sort_by_key(|a| a.axis_id);
    for a in axes {
        let cal = AxisCalibration::from_config(a);
        println!(
            "  axis {} {:?}: device_id {}{} calibration_hash 0x{:08x} calibration_valid {}",
            a.axis_id.get(),
            a.name,
            a.device_id,
            if a.device_id.is_unassigned() { " (no cross-check; assigned by the MCU's selected_axis)" } else { "" },
            calibration_hash(&cfg.rig_id, &cal),
            a.calibration_valid
        );
        if !a.calibration_valid {
            blockers.push(format!("calibration_invalid: axis{} calibration_valid = false", a.axis_id.get()));
        }
        if let Err(e) = cal.validate() {
            blockers.push(format!("calibration_invalid: axis{} {e}", a.axis_id.get()));
        }
    }
    if blockers.is_empty() {
        println!("static ARM blockers: none (runtime checks still apply: connection, reference, CONFIG, position)");
    } else {
        println!("static ARM blockers:");
        for b in blockers {
            println!("  - {b}");
        }
    }
    ExitCode::SUCCESS
}

fn list_ports() -> ExitCode {
    let ports = h3cs::axis_link::list_serial_ports();
    if ports.is_empty() {
        println!("(no serial ports found)");
    }
    for p in ports {
        println!("{p}");
    }
    ExitCode::SUCCESS
}

fn run(path: &Path, bind: Option<SocketAddr>) -> ExitCode {
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    tracing_subscriber::fmt().with_env_filter(filter).with_writer(std::io::stderr).init();

    let cfg = match Config::load(path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{}: {e}", path.display());
            return ExitCode::FAILURE;
        }
    };
    let rt = match tokio::runtime::Runtime::new() {
        Ok(rt) => rt,
        Err(e) => {
            eprintln!("tokio runtime: {e}");
            return ExitCode::FAILURE;
        }
    };
    rt.block_on(async move {
        let srv = match server::start(cfg, ServerOptions { admin_token: None, bind_override: bind }).await {
            Ok(s) => s,
            Err(e) => {
                eprintln!("start failed: {e}");
                return ExitCode::FAILURE;
            }
        };
        tracing::info!(addr = %srv.local_addr, "h3-control-server running; Ctrl-C to stop");
        let ended_itself = tokio::select! {
            _ = shutdown_signal() => false,
            _ = srv.closed() => true,
        };
        tracing::info!("shutting down (DISARM, no center)");
        srv.shutdown().await;
        if ended_itself {
            ExitCode::FAILURE
        } else {
            ExitCode::SUCCESS
        }
    })
}

async fn shutdown_signal() {
    #[cfg(unix)]
    {
        use tokio::signal::unix::{signal, SignalKind};
        match signal(SignalKind::terminate()) {
            Ok(mut term) => {
                tokio::select! {
                    _ = tokio::signal::ctrl_c() => {}
                    _ = term.recv() => {}
                }
            }
            Err(_) => {
                let _ = tokio::signal::ctrl_c().await;
            }
        }
    }
    #[cfg(not(unix))]
    {
        let _ = tokio::signal::ctrl_c().await;
    }
}
