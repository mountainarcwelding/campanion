//! Cooperative cancel and progress for long support-generation work.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};

use crate::error::{ResinError, Result};

/// A flag a worker checks between layers or stages so the UI can stop the work.
#[derive(Debug, Default)]
pub struct CancelFlag {
    cancelled: std::sync::atomic::AtomicBool,
}

impl CancelFlag {
    /// A flag that has not been cancelled.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Ask the worker to stop at the next check.
    pub fn cancel(&self) {
        self.cancelled.store(true, Ordering::SeqCst);
    }

    /// Whether [`Self::cancel`] has been called.
    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::SeqCst)
    }

    /// Return [`ResinError::Cancelled`] when the flag is set.
    pub fn check(&self) -> Result<()> {
        if self.is_cancelled() { Err(ResinError::cancelled()) } else { Ok(()) }
    }
}

/// Live progress a GUI thread can read while generation runs.
#[derive(Debug)]
pub struct GenerateProgress {
    layer: AtomicU32,
    layers: AtomicU32,
    stage: Mutex<String>,
}

impl Default for GenerateProgress {
    fn default() -> Self {
        Self::new()
    }
}

impl GenerateProgress {
    /// Zero layers, empty stage.
    #[must_use]
    pub fn new() -> Self {
        Self { layer: AtomicU32::new(0), layers: AtomicU32::new(0), stage: Mutex::new(String::new()) }
    }

    #[must_use]
    pub fn layer(&self) -> u32 {
        self.layer.load(Ordering::SeqCst)
    }

    #[must_use]
    pub fn layers(&self) -> u32 {
        self.layers.load(Ordering::SeqCst)
    }

    /// A short stage label such as "Detecting islands".
    #[must_use]
    pub fn stage(&self) -> String {
        self.stage.lock().map_or_else(|err| err.into_inner().clone(), |g| g.clone())
    }

    pub(crate) fn set_stage(&self, stage: &str) {
        if let Ok(mut slot) = self.stage.lock() {
            slot.clear();
            slot.push_str(stage);
        }
    }

    pub(crate) fn set_layers(&self, layers: u32) {
        self.layers.store(layers, Ordering::SeqCst);
    }

    pub(crate) fn set_layer(&self, layer: u32) {
        self.layer.store(layer, Ordering::SeqCst);
    }
}

/// Check an optional cancel flag.
pub(crate) fn check_cancel(cancel: Option<&CancelFlag>) -> Result<()> {
    match cancel {
        Some(flag) => flag.check(),
        None => Ok(()),
    }
}

/// Optional pause for the GUI screenshot harness. Cap is 5 s; ignored when the env is unset.
pub(crate) fn screenshot_hold() {
    let Ok(raw) = std::env::var("CAMPANION_GUI_SCREENSHOT_HOLD_MS") else {
        return;
    };
    let Ok(ms) = raw.parse::<u64>() else {
        return;
    };
    if ms == 0 || ms > 5_000 {
        return;
    }
    std::thread::sleep(std::time::Duration::from_millis(ms));
}
