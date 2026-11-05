//! Hot apply (operator console, design 2026-11-05 §6): rewrite the running
//! config file in place for one axis, safely.
//!
//! Order: read the file → replace the axis's `key = value` lines (comments
//! kept, [`crate::calibration_export::rewrite_axis_values`]) → the new text
//! must load exactly like a start would (`Config::from_toml_str`: parse,
//! scaled governor defaults, `validate`) and its `[[axis]]` must equal the
//! in-memory axis the caller is about to swap in (so file and memory never
//! disagree; a file edited by hand since the start is refused) → copy the
//! original next to it (`<name>.bak-<UTC timestamp>[-k]`, never overwritten)
//! → write `<name>.tmp-<pid>`, fsync, rename over the original. Nothing is
//! changed in memory here; the caller swaps only after `Ok`.

use std::io::Write as _;
use std::path::{Path, PathBuf};

use crate::calibration_export::{rewrite_axis_values, utc_timestamp};
use crate::config::{AxisConfig, Config};
use crate::types::AxisId;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EditError {
    /// Error code for the API (`config_read_failed`, `rewrite_failed`,
    /// `invalid_result`, `config_file_changed`, `write_failed`).
    pub code: &'static str,
    pub detail: String,
}

fn err(code: &'static str, detail: impl Into<String>) -> EditError {
    EditError { code, detail: detail.into() }
}

/// Write `values` into `axis`'s block of the file at `path`. `expect` is the
/// axis configuration that must result. Returns the backup path.
pub fn persist_axis(
    path: &Path,
    axis: AxisId,
    values: &[(&'static str, String)],
    expect: &AxisConfig,
) -> Result<PathBuf, EditError> {
    let text = std::fs::read_to_string(path).map_err(|e| err("config_read_failed", format!("{}: {e}", path.display())))?;
    let rw = rewrite_axis_values(&text, &[(axis, values.to_vec())]).map_err(|e| err("rewrite_failed", e))?;
    let parsed = Config::from_toml_str(&rw.text).map_err(|e| err("invalid_result", e.to_string()))?;
    match parsed.axis(axis) {
        Some(a) if a == expect => {}
        Some(a) => {
            return Err(err(
                "config_file_changed",
                format!("{}: [[axis]] {} in the file does not match the running configuration: {a:?}", path.display(), axis.get()),
            ))
        }
        None => return Err(err("invalid_result", format!("axis {} missing", axis.get()))),
    }
    let backup = backup_copy(path, &text)?;
    replace_file(path, &rw.text)?;
    Ok(backup)
}

/// Put `backup` back as `path` (best effort undo of [`persist_axis`]).
pub fn restore(path: &Path, backup: &Path) -> Result<(), EditError> {
    let text = std::fs::read_to_string(backup).map_err(|e| err("config_read_failed", format!("{}: {e}", backup.display())))?;
    replace_file(path, &text)
}

fn backup_copy(path: &Path, text: &str) -> Result<PathBuf, EditError> {
    let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| "config.toml".into());
    let ts = utc_timestamp(std::time::SystemTime::now());
    for k in 0..1000 {
        let file = if k == 0 { format!("{name}.bak-{ts}") } else { format!("{name}.bak-{ts}-{k}") };
        let out = path.with_file_name(file);
        match std::fs::OpenOptions::new().write(true).create_new(true).open(&out) {
            Ok(mut f) => {
                f.write_all(text.as_bytes())
                    .and_then(|()| f.sync_all())
                    .map_err(|e| err("write_failed", format!("{}: {e}", out.display())))?;
                return Ok(out);
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(e) => return Err(err("write_failed", format!("{}: {e}", out.display()))),
        }
    }
    Err(err("write_failed", "no free backup name"))
}

fn replace_file(path: &Path, text: &str) -> Result<(), EditError> {
    let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| "config.toml".into());
    let tmp = path.with_file_name(format!("{name}.tmp-{}", std::process::id()));
    let write = || -> std::io::Result<()> {
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(text.as_bytes())?;
        f.sync_all()?;
        drop(f);
        std::fs::rename(&tmp, path)
    };
    write().map_err(|e| {
        let _ = std::fs::remove_file(&tmp);
        err("write_failed", format!("{}: {e}", path.display()))
    })
}
