//! Path helpers and the native file chooser used by the graphical workspace.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

/// Kind of file dialog the GUI can open.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DialogKind {
    /// Open a job, part, IR JSON, or G-code file.
    Open,
    /// Import a DXF, SVG, STEP or STL part.
    Import,
    /// Save a `campanion-job/1` file.
    SaveJob,
    /// Export posted G-code.
    ExportGcode,
    /// Choose a sample `.pm7m` for print settings (`--params-from`).
    ParamsFrom,
    /// Write a sliced `.pm7m`.
    ExportPm7m,
}

/// Outcome of a native file chooser request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DialogResult {
    /// The user picked a path.
    Picked(PathBuf),
    /// The user dismissed the chooser.
    Cancelled,
    /// No system chooser was available.
    Unavailable,
}

/// File-name stem used for G-code export.
///
/// Compound suffixes such as `.job.json` and `.ir.json` are stripped so
/// `laser_rect.job.json` becomes `laser_rect`, not `laser_rect.job`.
#[must_use]
pub fn export_stem(path: &Path) -> Option<String> {
    let name = path.file_name()?.to_str()?;
    let lower = name.to_ascii_lowercase();
    for suffix in [".job.json", ".ir.json"] {
        if lower.ends_with(suffix) && name.len() > suffix.len() {
            return Some(name[..name.len() - suffix.len()].to_string());
        }
    }
    path.file_stem().and_then(|stem| stem.to_str()).map(ToString::to_string)
}

/// Expand `~` and join a relative path to `cwd`.
pub fn resolve_user_path(input: &str, cwd: &Path, home: Option<&Path>) -> Result<PathBuf, String> {
    let trimmed = input.trim();
    if trimmed.is_empty() {
        return Err("enter a file path".to_string());
    }
    let expanded = if trimmed == "~" {
        home.map(Path::to_path_buf).ok_or_else(|| "home directory is not set".to_string())?
    } else if let Some(rest) = trimmed.strip_prefix("~/") {
        let home = home.ok_or_else(|| "home directory is not set".to_string())?;
        if rest.is_empty() { home.to_path_buf() } else { home.join(rest) }
    } else {
        PathBuf::from(trimmed)
    };
    if expanded.is_absolute() { Ok(expanded) } else { Ok(cwd.join(expanded)) }
}

/// Display form of the current directory for the typed-path fallback.
#[must_use]
pub fn current_dir_display(cwd: &Path) -> String {
    cwd.display().to_string()
}

/// Open a native file chooser. Falls back to [`DialogResult::Unavailable`] when the
/// xdg-desktop-portal (or zenity) backend cannot be used.
pub fn pick_path(kind: DialogKind, start_dir: Option<&Path>, suggested_name: Option<&str>) -> DialogResult {
    if std::env::var_os("CAMPANION_GUI_NO_NATIVE_DIALOG").is_some() || !native_chooser_available() {
        return DialogResult::Unavailable;
    }
    let start = Instant::now();
    let picked = match kind {
        DialogKind::Open | DialogKind::Import | DialogKind::ParamsFrom => build_dialog(kind, start_dir, suggested_name).pick_file(),
        DialogKind::SaveJob | DialogKind::ExportGcode | DialogKind::ExportPm7m => build_dialog(kind, start_dir, suggested_name).save_file(),
    };
    match picked {
        Some(path) => DialogResult::Picked(path),
        None if start.elapsed() < Duration::from_millis(300) && !native_chooser_confirmed() => DialogResult::Unavailable,
        None => DialogResult::Cancelled,
    }
}

fn build_dialog(kind: DialogKind, start_dir: Option<&Path>, suggested_name: Option<&str>) -> rfd::FileDialog {
    let mut dialog = rfd::FileDialog::new();
    if let Some(dir) = start_dir {
        dialog = dialog.set_directory(dir);
    }
    if let Some(name) = suggested_name
        && matches!(kind, DialogKind::SaveJob | DialogKind::ExportGcode | DialogKind::ExportPm7m)
    {
        dialog = dialog.set_file_name(name);
    }
    match kind {
        DialogKind::Open => dialog
            .add_filter("Jobs and IR", &["json"])
            .add_filter("Parts", &["dxf", "svg", "step", "stp", "stl"])
            .add_filter("G-code", &["ngc", "gcode", "nc"])
            .add_filter("All files", &["*"])
            .set_title("Open"),
        DialogKind::Import => {
            dialog.add_filter("Parts", &["dxf", "svg", "step", "stp", "stl"]).add_filter("All files", &["*"]).set_title("Import part")
        }
        DialogKind::SaveJob => dialog.add_filter("Job JSON", &["json"]).add_filter("All files", &["*"]).set_title("Save job"),
        DialogKind::ExportGcode => dialog.add_filter("G-code", &["ngc", "gcode", "nc"]).add_filter("All files", &["*"]).set_title("Export G-code"),
        DialogKind::ParamsFrom => {
            dialog.add_filter("Photon Mono M7 Max", &["pm7m"]).add_filter("All files", &["*"]).set_title("Print settings from sample .pm7m")
        }
        DialogKind::ExportPm7m => dialog.add_filter("Photon Mono M7 Max", &["pm7m"]).add_filter("All files", &["*"]).set_title("Export .pm7m"),
    }
}

/// True when a portal or zenity file chooser looks runnable.
#[must_use]
pub fn native_chooser_available() -> bool {
    if std::env::var_os("CAMPANION_GUI_NO_NATIVE_DIALOG").is_some() {
        return false;
    }
    command_exists("zenity") || command_exists("qarma") || portal_name_owned()
}

fn native_chooser_confirmed() -> bool {
    command_exists("zenity") || command_exists("qarma") || portal_name_owned()
}

fn portal_name_owned() -> bool {
    if std::env::var_os("DBUS_SESSION_BUS_ADDRESS").is_none() {
        return false;
    }
    if run_silent("busctl", &["--user", "status", "org.freedesktop.portal.Desktop"]) {
        return true;
    }
    run_silent(
        "gdbus",
        &[
            "call",
            "--session",
            "--dest",
            "org.freedesktop.portal.Desktop",
            "--object-path",
            "/org/freedesktop/portal/desktop",
            "--method",
            "org.freedesktop.DBus.Peer.Ping",
        ],
    )
}

fn run_silent(program: &str, args: &[&str]) -> bool {
    Command::new(program)
        .args(args)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|status| status.success())
        .unwrap_or(false)
}

fn command_exists(name: &str) -> bool {
    let Some(paths) = std::env::var_os("PATH") else {
        return false;
    };
    std::env::split_paths(&paths).any(|dir| {
        let candidate = dir.join(name);
        candidate.is_file()
    })
}

#[cfg(test)]
mod tests {
    use super::{export_stem, resolve_user_path};
    use std::path::{Path, PathBuf};

    #[test]
    fn export_stem_strips_job_and_ir_json() {
        assert_eq!(export_stem(Path::new("laser_rect.job.json")).as_deref(), Some("laser_rect"));
        assert_eq!(export_stem(Path::new("/tmp/part.ir.json")).as_deref(), Some("part"));
        assert_eq!(export_stem(Path::new("plate.svg")).as_deref(), Some("plate"));
        assert_eq!(export_stem(Path::new("cut.ngc")).as_deref(), Some("cut"));
    }

    #[test]
    fn resolve_user_path_expands_tilde_and_joins_relative() {
        let home = Path::new("/home/cam");
        let cwd = Path::new("/work");
        assert_eq!(resolve_user_path("~/parts/a.svg", cwd, Some(home)).expect("tilde"), PathBuf::from("/home/cam/parts/a.svg"));
        assert_eq!(resolve_user_path("rel/part.svg", cwd, Some(home)).expect("rel"), PathBuf::from("/work/rel/part.svg"));
        assert_eq!(resolve_user_path("/abs/part.svg", cwd, Some(home)).expect("abs"), PathBuf::from("/abs/part.svg"));
        assert!(resolve_user_path("   ", cwd, Some(home)).is_err());
        assert!(resolve_user_path("~/a.svg", cwd, None).is_err());
    }
}
