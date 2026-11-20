//! In-process Grbl-like serial simulator for tests and screenshots.
//!
//! Behaviour is taken from the public Grbl v1.1 interface and commands documents:
//! real-time characters are intercepted and never occupy the RX buffer; response
//! messages are `ok` / `error:X`; a soft-reset while in motion raises `ALARM:3`.

use crate::docs::{REALTIME_HOLD, REALTIME_RESET, REALTIME_RESUME, REALTIME_STATUS};
use crate::protocol::encode_line;

/// A scripted fault the simulator can raise on the next program line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MockFault {
    Error(u32),
    Alarm(u32),
    Disconnect,
}

/// Simulated Grbl controller with a finite RX character buffer.
pub struct MockGrbl {
    rx_buffer: usize,
    rx: Vec<u8>,
    tx: Vec<u8>,
    state: &'static str,
    hold_complete: bool,
    in_motion: bool,
    machine_pos: [f64; 3],
    work_offset: [f64; 3],
    spindle_on: bool,
    next_fault: Option<MockFault>,
    disconnected: bool,
    overflowed: bool,
    planner: usize,
    defer_ok: bool,
    deferred: usize,
}

impl MockGrbl {
    /// A simulator whose RX buffer is `rx_buffer` characters (must be user-chosen in production;
    /// tests pass an explicit size).
    pub fn new(rx_buffer: usize) -> Self {
        Self {
            rx_buffer: rx_buffer.max(1),
            rx: Vec::new(),
            tx: Vec::new(),
            state: "Idle",
            hold_complete: false,
            in_motion: false,
            machine_pos: [0.0, 0.0, 0.0],
            work_offset: [0.0, 0.0, 0.0],
            spindle_on: false,
            next_fault: None,
            disconnected: false,
            overflowed: false,
            planner: 0,
            defer_ok: false,
            deferred: 0,
        }
    }

    pub fn disconnected(&self) -> bool {
        self.disconnected
    }

    pub fn set_in_motion(&mut self, value: bool) {
        self.in_motion = value;
    }

    /// Parse incoming lines but hold `ok`/`error` until [`Self::release_ok`].
    pub fn defer_ok(&mut self, defer: bool) {
        self.defer_ok = defer;
    }

    /// Release up to `count` deferred response messages.
    pub fn release_ok(&mut self, count: usize) {
        let n = count.min(self.deferred);
        for _ in 0..n {
            self.deferred -= 1;
            // Push immediately; `respond_ok` would re-queue while defer is still on.
            self.push_line("ok");
        }
    }

    pub fn overflowed(&self) -> bool {
        self.overflowed
    }

    pub fn deferred_count(&self) -> usize {
        self.deferred
    }

    pub fn state(&self) -> &'static str {
        self.state
    }

    pub fn buffered_chars(&self) -> usize {
        self.rx.len()
    }

    /// Queue a one-shot fault for the next parsed line (or disconnect on write).
    pub fn fault(&mut self, fault: MockFault) {
        self.next_fault = Some(fault);
    }

    /// Force a position for DRO screenshots or tests.
    pub fn set_position(&mut self, machine: [f64; 3], work_offset: [f64; 3]) {
        self.machine_pos = machine;
        self.work_offset = work_offset;
    }

    /// Write the welcome message as if the port had just opened or a reset completed.
    pub fn emit_welcome(&mut self) {
        self.push_line("Grbl 1.1h ['$' for help]");
    }

    /// Bytes waiting to be read by the host.
    pub fn take_tx(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.tx)
    }

    /// Accept host bytes. Real-time characters are handled immediately and do not
    /// occupy the RX buffer. Returns whether the write was accepted.
    pub fn write(&mut self, bytes: &[u8]) -> Result<(), MockError> {
        if self.disconnected {
            return Err(MockError::Disconnected);
        }
        if self.next_fault == Some(MockFault::Disconnect) {
            self.disconnected = true;
            self.next_fault = None;
            return Err(MockError::Disconnected);
        }
        for &byte in bytes {
            match byte {
                REALTIME_STATUS => self.emit_status(),
                REALTIME_HOLD => self.feed_hold(),
                REALTIME_RESUME => self.resume(),
                REALTIME_RESET => self.soft_reset(),
                _ => {
                    if self.rx.len() >= self.rx_buffer {
                        self.overflowed = true;
                        continue;
                    }
                    self.rx.push(byte);
                }
            }
        }
        self.drain_rx();
        Ok(())
    }

    fn drain_rx(&mut self) {
        while let Some(end) = self.rx.iter().position(|b| *b == b'\n' || *b == b'\r') {
            let raw: Vec<u8> = self.rx.drain(..=end).collect();
            let line = String::from_utf8_lossy(&raw).trim_end_matches(['\r', '\n']).to_string();
            self.handle_line(&line);
        }
    }

    fn handle_line(&mut self, line: &str) {
        if let Some(MockFault::Error(code)) = self.next_fault {
            self.next_fault = None;
            self.push_line(&format!("error:{code}"));
            return;
        }
        if let Some(MockFault::Alarm(code)) = self.next_fault {
            self.next_fault = None;
            self.state = "Alarm";
            self.in_motion = false;
            self.planner = 0;
            self.push_line(&format!("ALARM:{code}"));
            return;
        }
        if self.state == "Alarm" && line != "$X" {
            if line.starts_with('$') && line != "$X" {
                self.push_line("error:8");
                return;
            }
            if !line.is_empty() && line != "$X" {
                self.push_line("error:9");
                return;
            }
        }
        if line == "$X" {
            if self.state == "Alarm" {
                self.state = "Idle";
                self.respond_ok();
            } else {
                self.push_line("error:8");
            }
            return;
        }
        if let Some(rest) = line.strip_prefix("$J=") {
            if self.state != "Idle" && self.state != "Jog" {
                self.push_line("error:9");
                return;
            }
            if rest.trim().is_empty() {
                self.push_line("error:16");
                return;
            }
            self.apply_jog(rest);
            self.state = "Jog";
            self.in_motion = true;
            self.respond_ok();
            // A single queued jog completes immediately for tests; status may still show Jog
            // until the host queries after we drop back to Idle on the next drain? Keep Jog
            // until a status query after ok, then Idle — simpler: return to Idle now so
            // can_jog is true after ack, which matches a finished short jog.
            self.state = "Idle";
            self.in_motion = false;
            return;
        }
        if line.eq_ignore_ascii_case("M3") || line.eq_ignore_ascii_case("M4") {
            self.spindle_on = true;
            self.respond_ok();
            return;
        }
        if line.eq_ignore_ascii_case("M5") {
            self.spindle_on = false;
            self.respond_ok();
            return;
        }
        if line.starts_with('S') {
            if line.trim() == "S0" {
                self.spindle_on = false;
            }
            self.respond_ok();
            return;
        }
        if looks_like_motion(line) {
            if self.state == "Hold" {
                self.planner += 1;
                self.respond_ok();
                return;
            }
            self.in_motion = true;
            self.state = "Run";
            self.planner += 1;
            self.nudge_from_line(line);
            self.respond_ok();
            return;
        }
        self.respond_ok();
    }

    fn apply_jog(&mut self, body: &str) {
        for token in body.split_whitespace() {
            let Some(axis) = token.chars().next() else {
                continue;
            };
            let Ok(value) = token[1..].parse::<f64>() else {
                continue;
            };
            let incremental = body.contains("G91");
            match axis {
                'X' | 'x' => self.machine_pos[0] = if incremental { self.machine_pos[0] + value } else { value },
                'Y' | 'y' => self.machine_pos[1] = if incremental { self.machine_pos[1] + value } else { value },
                'Z' | 'z' => self.machine_pos[2] = if incremental { self.machine_pos[2] + value } else { value },
                _ => {}
            }
        }
    }

    fn nudge_from_line(&mut self, line: &str) {
        for token in line.split_whitespace() {
            let Some(axis) = token.chars().next() else {
                continue;
            };
            let rest = &token[1..];
            let Ok(value) = rest.parse::<f64>() else {
                continue;
            };
            match axis {
                'X' | 'x' => self.machine_pos[0] = value,
                'Y' | 'y' => self.machine_pos[1] = value,
                'Z' | 'z' => self.machine_pos[2] = value,
                _ => {}
            }
        }
    }

    fn feed_hold(&mut self) {
        if self.state == "Run" || self.state == "Jog" || self.state == "Idle" {
            self.state = "Hold";
            self.hold_complete = true;
            self.in_motion = false;
        }
    }

    fn resume(&mut self) {
        if self.state == "Hold" {
            self.state = if self.planner > 0 { "Run" } else { "Idle" };
            self.hold_complete = false;
        }
    }

    fn soft_reset(&mut self) {
        let mid_motion = self.in_motion || self.state == "Run" || (self.state == "Hold" && !self.hold_complete);
        self.rx.clear();
        self.planner = 0;
        self.spindle_on = false;
        self.in_motion = false;
        self.hold_complete = false;
        if mid_motion {
            self.state = "Alarm";
            self.push_line("ALARM:3");
        } else {
            self.state = "Idle";
        }
        self.emit_welcome();
    }

    fn emit_status(&mut self) {
        if self.disconnected {
            return;
        }
        let state = if self.state == "Hold" { if self.hold_complete { "Hold:0" } else { "Hold:1" } } else { self.state };
        let [x, y, z] = self.machine_pos;
        let [ox, oy, oz] = self.work_offset;
        let wpos = [x - ox, y - oy, z - oz];
        let mut body = format!("{state}|MPos:{x:.3},{y:.3},{z:.3}|WPos:{:.3},{:.3},{:.3}|WCO:{ox:.3},{oy:.3},{oz:.3}", wpos[0], wpos[1], wpos[2]);
        if self.spindle_on {
            body.push_str("|A:S");
        }
        self.push_line(&format!("<{body}>"));
    }

    fn push_line(&mut self, line: &str) {
        self.tx.extend_from_slice(&encode_line(line));
    }
}

impl MockGrbl {
    fn respond_ok(&mut self) {
        if self.defer_ok {
            self.deferred += 1;
        } else {
            self.push_line("ok");
        }
    }
}

fn looks_like_motion(line: &str) -> bool {
    let upper = line.to_ascii_uppercase();
    upper.contains("G0") || upper.contains("G1") || upper.contains("G2") || upper.contains("G3")
}

/// Write failed because the mock dropped the link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MockError {
    Disconnected,
}

impl std::fmt::Display for MockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Disconnected => f.write_str("mock serial port disconnected"),
        }
    }
}

impl std::error::Error for MockError {}

/// In-memory host↔device cable used by tests and by `CAMPANION_STREAM_MOCK`.
pub struct MockLink {
    pub device: MockGrbl,
}

impl MockLink {
    pub fn new(rx_buffer: usize) -> Self {
        let mut device = MockGrbl::new(rx_buffer);
        device.emit_welcome();
        Self { device }
    }

    /// Write host bytes to the device and return the device's reply.
    pub fn exchange(&mut self, bytes: &[u8]) -> Result<Vec<u8>, MockError> {
        self.device.write(bytes)?;
        Ok(self.device.take_tx())
    }
}

/// Environment variable that enables the in-process mock port in the GUI and CLI.
pub const MOCK_ENV: &str = "CAMPANION_STREAM_MOCK";

/// Port name advertised when [`MOCK_ENV`] is set (or when screenshots run).
pub const MOCK_PORT_NAME: &str = "mock-grbl";

pub fn mock_env_enabled() -> bool {
    matches!(std::env::var(MOCK_ENV), Ok(value) if !value.is_empty() && value != "0")
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::assertions_on_constants, clippy::unwrap_used, clippy::manual_unwrap_or_default)]
mod tests {
    use super::*;
    use crate::streamer::{JobKind, StreamEvent, StreamMode, Streamer};

    fn pump(streamer: &mut Streamer, link: &mut MockLink, outgoing: &[u8]) -> Vec<StreamEvent> {
        let mut events = Vec::new();
        let mut pending = outgoing.to_vec();
        let mut steps = 0;
        while !pending.is_empty() && steps < 64 {
            steps += 1;
            let reply = match link.exchange(&pending) {
                Ok(reply) => reply,
                Err(_) => {
                    events.push(streamer.on_disconnect());
                    return events;
                }
            };
            pending.clear();
            let (more, tx) = streamer.on_rx(&reply);
            events.extend(more);
            pending.extend(tx.bytes);
        }
        events
    }

    #[test]
    fn mock_counts_rx_characters_and_does_not_buffer_realtime() {
        let mut mock = MockGrbl::new(10);
        // 10-byte line fills the buffer: "G1 X12.3\n" is 9? "G1 X12.30\n" = 10
        let ok = mock.write(b"G1 X12.30\n");
        assert!(ok.is_ok());
        assert_eq!(mock.buffered_chars(), 0); // drained after a complete line
        // Real-time must still be accepted when the buffer is conceptually full.
        let mut mock = MockGrbl::new(8);
        let _ = mock.write(b"G1 X1\n"); // 6 bytes, processed
        let _ = mock.write(&[REALTIME_STATUS]);
        let bytes = mock.take_tx();
        let tx = String::from_utf8_lossy(&bytes);
        assert!(tx.contains('<'));
    }

    #[test]
    fn hold_resume_reset_mid_job_and_disconnect() {
        let mut streamer = match StreamMode::character_counting(128) {
            Ok(mode) => Streamer::new(mode),
            Err(_) => return assert!(false, "mode"),
        };
        let mut link = MockLink::new(128);
        link.device.defer_ok(true);
        let open = streamer.on_port_opened();
        // Welcome is a push, not deferred; status '?' is realtime.
        let events = pump(&mut streamer, &mut link, &open.bytes);
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Welcome(_))));

        let program = vec!["G1 X1.0 F300".into(), "G1 X2.0".into(), "G1 X3.0".into(), "G1 X4.0".into()];
        let start = match streamer.start(program, JobKind::Program) {
            Ok(tx) => tx,
            Err(err) => return assert!(false, "{err}"),
        };
        let _ = pump(&mut streamer, &mut link, &start.bytes);
        link.device.release_ok(1);
        let reply = link.device.take_tx();
        let (events, tx) = streamer.on_rx(&reply);
        if !tx.bytes.is_empty() {
            let _ = pump(&mut streamer, &mut link, &tx.bytes);
        }
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Ack { index: 0, .. })), "{events:?}");
        assert_eq!(streamer.last_acked_program_line(), Some(0));
        let pause = match streamer.pause() {
            Ok(tx) => tx,
            Err(err) => return assert!(false, "{err}"),
        };
        let _ = pump(&mut streamer, &mut link, &pause.bytes);
        let (_, status) = streamer.on_rx(&{
            match link.exchange(&[REALTIME_STATUS]) {
                Ok(bytes) => bytes,
                Err(_) => Vec::new(),
            }
        });
        let _ = status;
        assert_eq!(link.device.state(), "Hold");
        assert!(!streamer.can_jog());

        let resume = match streamer.resume() {
            Ok(tx) => tx,
            Err(err) => return assert!(false, "{err}"),
        };
        let _ = pump(&mut streamer, &mut link, &resume.bytes);

        // Reset mid-job: mock raises ALARM:3 (reset while in motion) then welcome.
        link.device.set_in_motion(true);
        let stop = match streamer.stop() {
            Ok(tx) => tx,
            Err(err) => return assert!(false, "{err}"),
        };
        let events = pump(&mut streamer, &mut link, &stop.bytes);
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Alarm { code: 3, .. })));
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Welcome(_))));
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Stopped { .. })));

        // Disconnect mid-job on a fresh run.
        let mut streamer = match StreamMode::character_counting(128) {
            Ok(mode) => Streamer::new(mode),
            Err(_) => return assert!(false, "mode"),
        };
        let mut link = MockLink::new(128);
        let open = streamer.on_port_opened();
        let _ = pump(&mut streamer, &mut link, &open.bytes);
        let start = match streamer.start(vec!["G1 X1".into(), "G1 X2".into()], JobKind::Program) {
            Ok(tx) => tx,
            Err(err) => return assert!(false, "{err}"),
        };
        link.device.fault(MockFault::Disconnect);
        let events = pump(&mut streamer, &mut link, &start.bytes);
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Message(text) if text.contains("not complete"))));
    }

    #[test]
    fn mock_reports_error_codes() {
        let mut streamer = match StreamMode::character_counting(128) {
            Ok(mode) => Streamer::new(mode),
            Err(_) => return assert!(false, "mode"),
        };
        let mut link = MockLink::new(128);
        let open = streamer.on_port_opened();
        let _ = pump(&mut streamer, &mut link, &open.bytes);
        link.device.fault(MockFault::Error(20));
        let start = match streamer.start(vec!["G99".into()], JobKind::Program) {
            Ok(tx) => tx,
            Err(err) => return assert!(false, "{err}"),
        };
        let events = pump(&mut streamer, &mut link, &start.bytes);
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Error { code: 20, .. })));
    }
}
