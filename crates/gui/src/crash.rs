//! Panic hook that writes a crash log the user can send or inspect.
//!
//! The log is written under `$CAMPANION_CACHE_DIR` when that is set, otherwise
//! `$XDG_CACHE_HOME/campanion`, otherwise `~/.cache/campanion`.

use std::fs;
use std::io::Write;
use std::panic::PanicHookInfo;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

/// Directory that will hold `crash-*.log` files.
pub fn crash_log_dir() -> PathBuf {
    if let Some(dir) = std::env::var_os("CAMPANION_CACHE_DIR") {
        return PathBuf::from(dir);
    }
    if let Some(xdg) = std::env::var_os("XDG_CACHE_HOME") {
        return PathBuf::from(xdg).join("campanion");
    }
    let home = std::env::var_os("HOME").map(PathBuf::from).unwrap_or_else(|| PathBuf::from("."));
    home.join(".cache").join("campanion")
}

/// Write `message` to a new `crash-*.log` under `dir` and return its path.
pub fn write_crash_log(dir: &Path, message: &str) -> Result<PathBuf, String> {
    fs::create_dir_all(dir).map_err(|err| format!("{}: {err}", dir.display()))?;
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis()).unwrap_or(0);
    let mut path = dir.join(format!("crash-{stamp}.log"));
    let mut suffix = 0u32;
    while path.exists() {
        suffix = suffix.saturating_add(1);
        path = dir.join(format!("crash-{stamp}-{suffix}.log"));
    }
    let mut file = fs::File::create(&path).map_err(|err| format!("{}: {err}", path.display()))?;
    file.write_all(message.as_bytes()).map_err(|err| format!("{}: {err}", path.display()))?;
    if !message.ends_with('\n') {
        file.write_all(b"\n").map_err(|err| format!("{}: {err}", path.display()))?;
    }
    Ok(path)
}

/// Human-readable message pointing at a crash log path.
#[must_use]
pub fn crash_user_message(path: &Path) -> String {
    format!("The program hit an error and wrote a crash log to {}.", path.display())
}

/// Format a panic payload and location into a log body.
pub fn format_panic_log(info: &PanicHookInfo<'_>) -> String {
    let payload = panic_payload(info);
    let location =
        info.location().map(|loc| format!("{}:{}:{}", loc.file(), loc.line(), loc.column())).unwrap_or_else(|| "unknown location".to_string());
    format!("CAMpanion crash\nlocation: {location}\n{payload}\n")
}

/// Install a hook that writes a crash log and prints its path to stderr.
pub fn install_panic_hook() {
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let body = format_panic_log(info);
        match write_crash_log(&crash_log_dir(), &body) {
            Ok(path) => eprintln!("campanion-gui: {}", crash_user_message(&path)),
            Err(err) => eprintln!("campanion-gui: could not write a crash log ({err}). Panic: {body}"),
        }
        previous(info);
    }));
}

/// Write a crash log for a panic caught with `catch_unwind`.
pub fn log_caught_panic(payload: &dyn std::any::Any) -> (String, Option<PathBuf>) {
    let message = format!("CAMpanion crash\nlocation: worker thread\n{payload}\n", payload = panic_any(payload));
    match write_crash_log(&crash_log_dir(), &message) {
        Ok(path) => (crash_user_message(&path), Some(path)),
        Err(err) => (format!("The program hit an error and could not write a crash log ({err})."), None),
    }
}

fn panic_payload(info: &PanicHookInfo<'_>) -> String {
    panic_any(info.payload())
}

fn panic_any(payload: &dyn std::any::Any) -> String {
    if let Some(text) = payload.downcast_ref::<&str>() {
        (*text).to_string()
    } else if let Some(text) = payload.downcast_ref::<String>() {
        text.clone()
    } else {
        "box of unknown panic payload".to_string()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    static COUNTER: AtomicU64 = AtomicU64::new(0);

    fn unique_dir() -> PathBuf {
        let n = COUNTER.fetch_add(1, Ordering::SeqCst);
        let dir = std::env::temp_dir().join(format!("campanion-crash-test-{}-{n}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        dir
    }

    #[test]
    fn write_crash_log_creates_a_file_and_names_its_path() {
        let dir = unique_dir();
        let path = write_crash_log(&dir, "test panic: index out of range").expect("write");
        let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
        assert!(name.starts_with("crash-"), "{name}");
        assert!(name.ends_with(".log"), "{name}");
        let text = fs::read_to_string(&path).expect("read");
        assert!(text.contains("test panic: index out of range"), "{text}");
        let message = crash_user_message(&path);
        assert!(message.contains(&path.display().to_string()), "{message}");
        let _ = fs::remove_dir_all(&dir);
    }
}
