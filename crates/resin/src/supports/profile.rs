//! Optional stderr timings when `CAMPANION_RESIN_PROFILE` is set. Not a public API.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

pub(crate) fn enabled() -> bool {
    static FLAG: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *FLAG.get_or_init(|| std::env::var_os("CAMPANION_RESIN_PROFILE").is_some())
}

pub(crate) fn log(label: &str, start: Instant) {
    if enabled() {
        eprintln!("profile: {label}: {:.3} ms", start.elapsed().as_secs_f64() * 1000.0);
    }
}

pub(crate) fn log_count(label: &str, count: u64) {
    if enabled() {
        eprintln!("profile: {label}: {count}");
    }
}

static HIT_CALLS: AtomicU64 = AtomicU64::new(0);
static HIT_TRIS: AtomicU64 = AtomicU64::new(0);

pub(crate) fn reset_hits() {
    HIT_CALLS.store(0, Ordering::Relaxed);
    HIT_TRIS.store(0, Ordering::Relaxed);
}

pub(crate) fn record_hit(triangles_tested: u64) {
    if !enabled() {
        return;
    }
    HIT_CALLS.fetch_add(1, Ordering::Relaxed);
    HIT_TRIS.fetch_add(triangles_tested, Ordering::Relaxed);
}

pub(crate) fn log_hits() {
    if enabled() {
        eprintln!("profile: first_hit calls: {}, triangles tested: {}", HIT_CALLS.load(Ordering::Relaxed), HIT_TRIS.load(Ordering::Relaxed));
    }
}
