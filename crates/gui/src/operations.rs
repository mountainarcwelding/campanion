//! G2 Operations helpers: head-limited types, cut-order, and auto-regeneration.

use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use campanion_cli::PlanCancel;

use crate::SubProcess;
use crate::session::{JobSession, OpDraft, PlannedSession, generate_toolpaths_cancellable};

/// Wait after the last edit before starting a background regenerate.
pub const REGEN_DEBOUNCE: Duration = Duration::from_millis(300);

/// Operation types the Add Operation menu may offer for a process. No 5-axis types.
#[must_use]
pub fn kinds_for_process(process: Option<SubProcess>) -> &'static [(&'static str, &'static str)] {
    match process {
        Some(SubProcess::Laser) => &[("profile", "Cut (profile)"), ("engrave", "Score (engrave)"), ("laser_raster", "Fill (raster)")],
        Some(SubProcess::Plasma | SubProcess::Waterjet | SubProcess::OxyFuel) => &[("profile", "Profile")],
        Some(SubProcess::Milling | SubProcess::Routing) => &[("face", "Face"), ("profile", "Profile"), ("pocket", "Pocket"), ("drill", "Drill")],
        Some(SubProcess::WireEdm) => &[("profile", "Profile")],
        Some(SubProcess::Turning) => &[("profile", "Profile")],
        _ => &[("profile", "Profile")],
    }
}

/// Whether `kind` is allowed for the current process head.
#[must_use]
pub fn kind_allowed(process: Option<SubProcess>, kind: &str) -> bool {
    kinds_for_process(process).iter().any(|(id, _)| *id == kind)
}

/// A new operation draft of `kind` with empty numeric fields.
#[must_use]
pub fn new_operation(name: impl Into<String>, kind: &str, tool_id: u32) -> OpDraft {
    let mut op = OpDraft::new_profile(name, tool_id);
    op.kind = kind.to_string();
    if kind != "profile" {
        op.side.clear();
    }
    op
}

/// A new operation assigned to every part in `session` (no layer filter).
#[must_use]
pub fn new_operation_on_session(session: &JobSession, name: impl Into<String>, kind: &str, tool_id: u32) -> OpDraft {
    let mut op = new_operation(name, kind, tool_id);
    crate::session::assign_to_all_parts(&mut op, session);
    op
}

/// In-flight auto-regenerate state.
#[derive(Default)]
pub struct RegenState {
    last_edit: Option<Instant>,
    generation: u64,
    cancel: Option<Arc<PlanCancel>>,
    receiver: Option<mpsc::Receiver<RegenOutcome>>,
    /// True while the visible toolpaths are older than the current drafts.
    pub stale: bool,
}

struct RegenOutcome {
    generation: u64,
    result: Result<PlannedSession, String>,
}

impl RegenState {
    /// Remember an edit and cancel any in-flight plan.
    pub fn note_edit(&mut self, had_toolpaths: bool) {
        self.generation = self.generation.saturating_add(1);
        self.last_edit = Some(Instant::now());
        if let Some(cancel) = &self.cancel {
            cancel.cancel();
        }
        self.stale = had_toolpaths;
    }

    /// Start a cancelled-able plan when the debounce has elapsed. Call once per frame.
    pub fn tick(&mut self, session: &JobSession, store: &campanion_cli::ProfileStore) -> Option<(Result<PlannedSession, String>, bool)> {
        if let Some(receiver) = &self.receiver
            && let Ok(outcome) = receiver.try_recv()
        {
            self.receiver = None;
            self.cancel = None;
            if outcome.generation == self.generation {
                if outcome.result.is_ok() {
                    self.stale = false;
                    self.last_edit = None;
                }
                return Some((outcome.result, true));
            }
        }
        let last_edit = self.last_edit?;
        if last_edit.elapsed() < REGEN_DEBOUNCE {
            return None;
        }
        if self.receiver.is_some() {
            return None;
        }
        if session.operations.is_empty() {
            self.last_edit = None;
            return None;
        }
        let generation = self.generation;
        let cancel = Arc::new(PlanCancel::new());
        let (sender, receiver) = mpsc::channel();
        self.cancel = Some(Arc::clone(&cancel));
        self.receiver = Some(receiver);
        let session = session.clone();
        let store = store.clone();
        thread::spawn(move || {
            let result = campanion_cli::with_store(&store, || generate_toolpaths_cancellable(&session, Some(cancel.as_ref())));
            let _ = sender.send(RegenOutcome { generation, result });
        });
        None
    }

    /// Request a repaint while a debounce or an in-flight plan is outstanding.
    #[must_use]
    pub fn needs_repaint(&self) -> bool {
        self.last_edit.is_some() || self.receiver.is_some()
    }
}

/// Swap two operations and mark the list as an explicit cut order.
pub fn move_operation(session: &mut JobSession, from: usize, to: usize) {
    let len = session.operations.len();
    if from == to || from >= len || to >= len {
        return;
    }
    session.operations.swap(from, to);
    session.cut_order_explicit = true;
    session.dirty = true;
}

/// Reorder so `from` moves to `to`, shifting neighbours.
pub fn reorder_operation(session: &mut JobSession, from: usize, to: usize) {
    let len = session.operations.len();
    if from == to || from >= len || to >= len {
        return;
    }
    let op = session.operations.remove(from);
    session.operations.insert(to, op);
    session.cut_order_explicit = true;
    session.dirty = true;
}

/// Drop an explicit cut order so automatic `order_rules` apply again.
pub fn reset_cut_order(session: &mut JobSession) {
    session.cut_order_explicit = false;
    session.dirty = true;
}

/// Record that the user overrode `field` after a material was applied.
pub fn mark_override(op: &mut OpDraft, field: &str) {
    if op.material_name.is_empty() {
        return;
    }
    if !op.override_fields.iter().any(|name| name == field) {
        op.override_fields.push(field.to_string());
    }
}

/// Clear per-contour / per-field overrides and remember that a reset was requested.
pub fn reset_overrides(op: &mut OpDraft) {
    op.override_fields.clear();
}
