//! Host-side streaming state machine: send-response and character-counting.
//!
//! Follows the public Grbl v1.1 interface and commands documents. The RX buffer
//! size is never assumed; character-counting refuses to start unless the caller
//! supplies a user-entered size.

use crate::docs::{REALTIME_HOLD, REALTIME_RESET, REALTIME_RESUME, REALTIME_STATUS};
use crate::protocol::{MachineState, RxMessage, StatusReport, alarm_description, encode_line, error_description, parse_line, take_lines};

/// How program lines are paced toward Grbl.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamMode {
    /// Send one line and wait for `ok`/`error:` before the next. Needs no RX size.
    SendResponse,
    /// Keep the RX buffer full of complete lines, counting every sent byte including LF.
    CharacterCounting { rx_buffer: usize },
}

impl StreamMode {
    /// Character-counting from a user-entered buffer size. Refuses zero.
    pub fn character_counting(rx_buffer: usize) -> Result<Self, StreamError> {
        if rx_buffer == 0 {
            return Err(StreamError::Message("serial RX buffer must be a positive number of characters".to_string()));
        }
        Ok(Self::CharacterCounting { rx_buffer })
    }

    fn room(&self, used: usize) -> Option<usize> {
        match self {
            Self::SendResponse => None,
            Self::CharacterCounting { rx_buffer } => Some(rx_buffer.saturating_sub(used)),
        }
    }
}

/// Why a command was refused, or a protocol failure the caller must surface.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StreamError {
    Message(String),
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Message(text) => f.write_str(text),
        }
    }
}

impl std::error::Error for StreamError {}

/// Kind of program currently being streamed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobKind {
    Program,
    Frame,
}

/// What the host is doing, distinct from Grbl's reported state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostPhase {
    Disconnected,
    Connecting,
    Idle,
    Streaming,
    Holding,
    Stopping,
    Alarm,
    Jogging,
}

/// Events the GUI or CLI should show.
#[derive(Debug, Clone, PartialEq)]
pub enum StreamEvent {
    Welcome(String),
    Status(StatusReport),
    /// A streamed line was acknowledged with `ok`. `index` is 0-based in the current job.
    Ack {
        index: usize,
        kind: JobKind,
    },
    Error {
        code: u32,
        text: String,
        index: Option<usize>,
    },
    Alarm {
        code: u32,
        text: String,
    },
    /// Soft-reset completed; `m5_verified` is true when `M5` was accepted or status shows the spindle off.
    Stopped {
        m5_verified: bool,
        last_line: Option<usize>,
    },
    JobFinished {
        kind: JobKind,
    },
    Push(String),
    Message(String),
}

/// Bytes the caller must write to the serial port, in order.
#[derive(Debug, Clone, Default)]
pub struct Tx {
    pub bytes: Vec<u8>,
}

impl Tx {
    fn push(&mut self, bytes: &[u8]) {
        self.bytes.extend_from_slice(bytes);
    }

    fn realtime(&mut self, byte: u8) {
        self.bytes.push(byte);
    }
}

struct Inflight {
    index: usize,
    bytes: usize,
    kind: InflightKind,
}

#[derive(Clone, Copy)]
enum InflightKind {
    Job { kind: JobKind },
    StopM5,
    Jog,
    Other,
}

/// Character-counting / send-response host.
pub struct Streamer {
    mode: StreamMode,
    phase: HostPhase,
    pending_rx: Vec<u8>,
    program: Vec<String>,
    job_kind: JobKind,
    next_send: usize,
    inflight: Vec<Inflight>,
    /// Last acked job-program line (not frame). Kept across Stop.
    last_acked_program: Option<usize>,
    /// Last acked line of the current job (program or frame).
    last_acked: Option<usize>,
    status: Option<StatusReport>,
    last_error: Option<(u32, String)>,
    last_alarm: Option<(u32, String)>,
    halt_send: bool,
    stop_m5_sent: bool,
    stop_m5_ok: bool,
    welcome_seen: bool,
}

impl Streamer {
    /// A disconnected host. `mode` is used once a job starts.
    pub fn new(mode: StreamMode) -> Self {
        Self {
            mode,
            phase: HostPhase::Disconnected,
            pending_rx: Vec::new(),
            program: Vec::new(),
            job_kind: JobKind::Program,
            next_send: 0,
            inflight: Vec::new(),
            last_acked_program: None,
            last_acked: None,
            status: None,
            last_error: None,
            last_alarm: None,
            halt_send: false,
            stop_m5_sent: false,
            stop_m5_ok: false,
            welcome_seen: false,
        }
    }

    /// Switch pacing mode when idle. Character-counting still requires a user-entered size.
    pub fn set_mode(&mut self, mode: StreamMode) -> Result<(), StreamError> {
        if !matches!(self.phase, HostPhase::Disconnected | HostPhase::Idle | HostPhase::Alarm) {
            return Err(StreamError::Message("cannot change streaming mode while a job is active".into()));
        }
        self.mode = mode;
        Ok(())
    }

    pub fn mode(&self) -> StreamMode {
        self.mode
    }

    pub fn phase(&self) -> HostPhase {
        self.phase
    }

    pub fn connected(&self) -> bool {
        !matches!(self.phase, HostPhase::Disconnected)
    }

    pub fn last_status(&self) -> Option<&StatusReport> {
        self.status.as_ref()
    }

    pub fn last_error(&self) -> Option<(u32, &str)> {
        self.last_error.as_ref().map(|(code, text)| (*code, text.as_str()))
    }

    pub fn last_alarm(&self) -> Option<(u32, &str)> {
        self.last_alarm.as_ref().map(|(code, text)| (*code, text.as_str()))
    }

    /// Last acknowledged program line, kept when Stop is used.
    pub fn last_acked_program_line(&self) -> Option<usize> {
        self.last_acked_program
    }

    pub fn last_acked_line(&self) -> Option<usize> {
        self.last_acked
    }

    pub fn job_line_count(&self) -> usize {
        self.program.len()
    }

    pub fn job_kind(&self) -> JobKind {
        self.job_kind
    }

    pub fn machine_state(&self) -> Option<&MachineState> {
        self.status.as_ref().map(|s| &s.state)
    }

    /// The port was opened. Wait for a welcome or first status report.
    pub fn on_port_opened(&mut self) -> Tx {
        self.phase = HostPhase::Connecting;
        self.welcome_seen = false;
        self.last_error = None;
        self.last_alarm = None;
        let mut tx = Tx::default();
        tx.realtime(REALTIME_STATUS);
        tx
    }

    /// The port is gone. Mid-job disconnects are not a successful finish.
    pub fn on_disconnect(&mut self) -> StreamEvent {
        let during_job = matches!(self.phase, HostPhase::Streaming | HostPhase::Holding | HostPhase::Stopping | HostPhase::Jogging);
        self.phase = HostPhase::Disconnected;
        self.inflight.clear();
        self.program.clear();
        self.next_send = 0;
        self.halt_send = false;
        self.stop_m5_sent = false;
        if during_job {
            StreamEvent::Message("disconnected while a job was active; the run is not complete".into())
        } else {
            StreamEvent::Message("disconnected".into())
        }
    }

    pub fn can_stop(&self) -> bool {
        self.connected()
    }

    pub fn can_pause(&self) -> bool {
        self.phase == HostPhase::Streaming
    }

    pub fn can_resume(&self) -> bool {
        self.phase == HostPhase::Holding
    }

    pub fn can_start(&self) -> bool {
        self.ready_for_job()
    }

    pub fn can_frame(&self) -> bool {
        self.ready_for_job()
    }

    pub fn can_jog(&self) -> bool {
        self.ready_for_job() && self.machine_allows_jog()
    }

    pub fn can_unlock(&self) -> bool {
        self.connected() && (self.phase == HostPhase::Alarm || matches!(self.machine_state(), Some(MachineState::Alarm)))
    }

    fn ready_for_job(&self) -> bool {
        matches!(self.phase, HostPhase::Idle | HostPhase::Connecting)
            && !matches!(self.machine_state(), Some(MachineState::Hold { .. } | MachineState::Alarm | MachineState::Run | MachineState::Jog))
    }

    fn machine_allows_jog(&self) -> bool {
        match self.machine_state() {
            Some(state) => state.can_jog(),
            // No status yet: allow only once Idle has been seen later; refuse Hold/Run blindly.
            None => self.phase == HostPhase::Idle,
        }
    }

    /// Start streaming `lines` (already posted G-code, or a frame program).
    pub fn start(&mut self, lines: Vec<String>, kind: JobKind) -> Result<Tx, StreamError> {
        if !self.can_start() {
            return Err(StreamError::Message(self.not_ready_reason("Start")));
        }
        self.check_mode_for_lines(&lines)?;
        self.program = lines;
        self.job_kind = kind;
        self.next_send = 0;
        self.inflight.clear();
        self.last_acked = None;
        if kind == JobKind::Program {
            self.last_acked_program = None;
        }
        self.halt_send = false;
        self.phase = HostPhase::Streaming;
        let mut tx = Tx::default();
        self.fill(&mut tx);
        Ok(tx)
    }

    fn check_mode_for_lines(&self, lines: &[String]) -> Result<(), StreamError> {
        match self.mode {
            StreamMode::SendResponse => Ok(()),
            StreamMode::CharacterCounting { rx_buffer } => {
                if rx_buffer == 0 {
                    return Err(StreamError::Message(Self::missing_buffer_message()));
                }
                if let Some(line) = lines.iter().map(|l| encode_line(l)).find(|b| b.len() > rx_buffer) {
                    return Err(StreamError::Message(format!("line of {} characters exceeds the entered RX buffer of {rx_buffer}", line.len())));
                }
                Ok(())
            }
        }
    }

    /// Message used when character-counting is requested without a user-entered buffer.
    pub fn missing_buffer_message() -> String {
        "character-counting streaming needs a serial RX buffer size entered on the machine profile; use send-response safe mode if you have not entered one".to_string()
    }

    fn not_ready_reason(&self, action: &str) -> String {
        if !self.connected() {
            return format!("{action} needs a connected controller");
        }
        if self.phase == HostPhase::Alarm || matches!(self.machine_state(), Some(MachineState::Alarm)) {
            return format!("{action} is locked while Grbl is in alarm; unlock only with confirm");
        }
        if matches!(self.machine_state(), Some(MachineState::Hold { .. })) || self.phase == HostPhase::Holding {
            return format!("{action} is locked while in hold; resume or STOP first");
        }
        if matches!(self.phase, HostPhase::Streaming | HostPhase::Stopping | HostPhase::Jogging)
            || matches!(self.machine_state(), Some(MachineState::Run | MachineState::Jog))
        {
            return format!("{action} is locked while a job is running; only Pause and STOP stay available");
        }
        format!("{action} is not available in the current state")
    }

    /// Feed hold (`!`). Does not turn the spindle off; that is documented Grbl behaviour.
    pub fn pause(&mut self) -> Result<Tx, StreamError> {
        if !self.can_pause() {
            return Err(StreamError::Message("Pause is only available while a job is running".into()));
        }
        self.phase = HostPhase::Holding;
        let mut tx = Tx::default();
        tx.realtime(REALTIME_HOLD);
        Ok(tx)
    }

    /// Cycle start / resume (`~`).
    pub fn resume(&mut self) -> Result<Tx, StreamError> {
        if !self.can_resume() {
            return Err(StreamError::Message("Resume is only available while in hold".into()));
        }
        self.phase = HostPhase::Streaming;
        let mut tx = Tx::default();
        tx.realtime(REALTIME_RESUME);
        self.fill(&mut tx);
        Ok(tx)
    }

    /// Soft-reset (`0x18`). After the welcome message, `M5` is sent to verify the beam is off.
    pub fn stop(&mut self) -> Result<Tx, StreamError> {
        if !self.can_stop() {
            return Err(StreamError::Message("STOP is live once a controller is connected".into()));
        }
        self.phase = HostPhase::Stopping;
        self.halt_send = true;
        self.inflight.clear();
        self.next_send = self.program.len();
        self.stop_m5_sent = false;
        self.stop_m5_ok = false;
        self.welcome_seen = false;
        let mut tx = Tx::default();
        tx.realtime(REALTIME_RESET);
        Ok(tx)
    }

    /// `$X` kill-alarm-lock. The caller must have confirmed with the user first.
    pub fn unlock_after_confirm(&mut self) -> Result<Tx, StreamError> {
        if !self.can_unlock() {
            return Err(StreamError::Message("$X is only available in alarm, after you confirm".into()));
        }
        let mut tx = Tx::default();
        self.send_system(&mut tx, "$X", InflightKind::Other);
        Ok(tx)
    }

    /// Incremental jog: `$J=G91 G21 {axis}{delta} F{feed}` as required by the jogging document.
    pub fn jog(&mut self, axis: usize, delta_mm: f64, feed_mm_min: f64) -> Result<Tx, StreamError> {
        if !self.can_jog() {
            return Err(StreamError::Message(self.not_ready_reason("Jog")));
        }
        if axis > 2 || !delta_mm.is_finite() || delta_mm == 0.0 {
            return Err(StreamError::Message("jog needs a finite, non-zero increment".into()));
        }
        if !feed_mm_min.is_finite() || feed_mm_min <= 0.0 {
            return Err(StreamError::Message("jog needs a feed you entered, in millimetres per minute".into()));
        }
        let word = match axis {
            0 => "X",
            1 => "Y",
            _ => "Z",
        };
        let line = format!("$J=G91 G21 {word}{delta_mm} F{feed_mm_min}");
        self.phase = HostPhase::Jogging;
        let mut tx = Tx::default();
        self.send_system(&mut tx, &line, InflightKind::Jog);
        Ok(tx)
    }

    /// Real-time status query. May be sent at any time while connected (commands document).
    pub fn query_status(&self) -> Option<u8> {
        if self.connected() { Some(REALTIME_STATUS) } else { None }
    }

    /// Handle bytes read from the controller. Returns events and any immediate reply bytes
    /// (for example `M5` after a soft-reset welcome).
    pub fn on_rx(&mut self, bytes: &[u8]) -> (Vec<StreamEvent>, Tx) {
        let mut events = Vec::new();
        let mut tx = Tx::default();
        let lines = take_lines(&mut self.pending_rx, bytes);
        for line in lines {
            if let Some(message) = parse_line(&line) {
                self.handle_message(message, &mut events, &mut tx);
            }
        }
        if self.phase == HostPhase::Streaming && !self.halt_send {
            self.fill(&mut tx);
        }
        (events, tx)
    }

    fn handle_message(&mut self, message: RxMessage, events: &mut Vec<StreamEvent>, tx: &mut Tx) {
        match message {
            RxMessage::Ok => self.on_ack(true, events),
            RxMessage::Error { code, raw } => {
                let text = format!("{raw} — {}", error_description(code));
                self.last_error = Some((code, text.clone()));
                let index = self.inflight.first().map(|item| item.index);
                events.push(StreamEvent::Error { code, text, index });
                self.on_ack(false, events);
                self.halt_send = true;
                if self.phase == HostPhase::Streaming {
                    self.phase = HostPhase::Idle;
                }
            }
            RxMessage::Alarm { code, raw } => {
                let text = format!("{raw} — {}", alarm_description(code));
                self.last_alarm = Some((code, text.clone()));
                self.halt_send = true;
                // A soft-reset mid-motion raises ALARM:3. Stay in Stopping so M5 still follows
                // the welcome message; finish_stop then lands in Alarm and keeps the last line.
                if self.phase != HostPhase::Stopping {
                    self.phase = HostPhase::Alarm;
                }
                events.push(StreamEvent::Alarm { code, text });
            }
            RxMessage::Welcome(text) => {
                self.welcome_seen = true;
                events.push(StreamEvent::Welcome(text));
                if self.phase == HostPhase::Stopping {
                    self.send_stop_m5(tx);
                } else if self.phase == HostPhase::Connecting {
                    self.phase = HostPhase::Idle;
                }
            }
            RxMessage::Status(status) => {
                if self.phase == HostPhase::Connecting {
                    self.phase = if matches!(status.state, MachineState::Alarm) { HostPhase::Alarm } else { HostPhase::Idle };
                }
                if self.phase == HostPhase::Stopping && self.stop_m5_sent {
                    let verified = self.stop_m5_ok || !status.spindle_on();
                    if verified {
                        self.finish_stop(events);
                    }
                }
                if self.phase == HostPhase::Jogging && status.state.is_idle() && self.inflight.is_empty() {
                    self.phase = HostPhase::Idle;
                }
                if matches!(status.state, MachineState::Alarm) && self.phase != HostPhase::Stopping {
                    self.phase = HostPhase::Alarm;
                }
                self.status = Some(status.clone());
                events.push(StreamEvent::Status(status));
            }
            RxMessage::Push(text) => events.push(StreamEvent::Push(text)),
        }
    }

    fn on_ack(&mut self, ok: bool, events: &mut Vec<StreamEvent>) {
        let Some(item) = self.inflight.first() else {
            return;
        };
        let index = item.index;
        let kind = item.kind;
        self.inflight.remove(0);
        match kind {
            InflightKind::Job { kind } => {
                if ok {
                    self.last_acked = Some(index);
                    if kind == JobKind::Program {
                        self.last_acked_program = Some(index);
                    }
                    events.push(StreamEvent::Ack { index, kind });
                }
                if self.phase == HostPhase::Streaming && self.next_send >= self.program.len() && self.inflight.is_empty() {
                    self.phase = HostPhase::Idle;
                    events.push(StreamEvent::JobFinished { kind });
                }
            }
            InflightKind::StopM5 => {
                if ok {
                    self.stop_m5_ok = true;
                }
                let spindle_off = self.status.as_ref().is_none_or(|s| !s.spindle_on());
                if self.stop_m5_ok || spindle_off {
                    self.finish_stop(events);
                }
            }
            InflightKind::Jog => {
                if self.phase == HostPhase::Jogging {
                    self.phase = HostPhase::Idle;
                }
            }
            InflightKind::Other => {
                if self.phase == HostPhase::Alarm && ok {
                    // $X accepted.
                    self.phase = HostPhase::Idle;
                    self.last_alarm = None;
                    events.push(StreamEvent::Message("alarm lock cleared with $X".into()));
                }
            }
        }
    }

    fn send_stop_m5(&mut self, tx: &mut Tx) {
        if self.stop_m5_sent {
            return;
        }
        // In alarm after reset-while-in-motion, G-code is locked (error:9). We still try M5
        // as required, and also rely on status accessory bits / reset having idled the spindle.
        self.send_system(tx, "M5", InflightKind::StopM5);
        self.stop_m5_sent = true;
        if let Some(query) = self.query_status() {
            tx.realtime(query);
        }
    }

    fn finish_stop(&mut self, events: &mut Vec<StreamEvent>) {
        if self.phase != HostPhase::Stopping {
            return;
        }
        let m5_verified = self.stop_m5_ok || self.status.as_ref().is_none_or(|s| !s.spindle_on());
        let last_line = self.last_acked_program;
        // Alarm 3 (reset while in motion) may already have been recorded; stay in Alarm if so.
        self.phase = if self.last_alarm.is_some() { HostPhase::Alarm } else { HostPhase::Idle };
        events.push(StreamEvent::Stopped { m5_verified, last_line });
    }

    fn send_system(&mut self, tx: &mut Tx, line: &str, kind: InflightKind) {
        let bytes = encode_line(line);
        let len = bytes.len();
        tx.push(&bytes);
        self.inflight.push(Inflight { index: 0, bytes: len, kind });
    }

    fn fill(&mut self, tx: &mut Tx) {
        if self.halt_send || self.phase != HostPhase::Streaming {
            return;
        }
        while self.next_send < self.program.len() {
            let Some(line) = self.program.get(self.next_send) else {
                break;
            };
            let bytes = encode_line(line);
            let used: usize = self.inflight.iter().map(|item| item.bytes).sum();
            match self.mode {
                StreamMode::SendResponse => {
                    if !self.inflight.is_empty() {
                        break;
                    }
                }
                StreamMode::CharacterCounting { rx_buffer } => {
                    if used + bytes.len() > rx_buffer {
                        break;
                    }
                }
            }
            tx.push(&bytes);
            self.inflight.push(Inflight { index: self.next_send, bytes: bytes.len(), kind: InflightKind::Job { kind: self.job_kind } });
            self.next_send += 1;
        }
        let _ = self.mode.room(self.inflight.iter().map(|item| item.bytes).sum());
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::assertions_on_constants, clippy::unwrap_used, clippy::manual_unwrap_or_default)]
mod tests {
    use super::*;

    fn counting(n: usize) -> Streamer {
        match StreamMode::character_counting(n) {
            Ok(mode) => Streamer::new(mode),
            Err(_) => Streamer::new(StreamMode::SendResponse),
        }
    }

    fn connect(streamer: &mut Streamer) {
        let _ = streamer.on_port_opened();
        let _ = streamer.on_rx(b"Grbl 1.1h ['$' for help]\n<Idle|MPos:0.000,0.000,0.000>\n");
    }

    #[test]
    fn character_counting_fills_without_exceeding_the_entered_buffer() {
        let mut streamer = counting(20);
        connect(&mut streamer);
        // "G1 X1\n" is 6 bytes; 20/6 = 3 lines fit, fourth waits.
        let lines: Vec<String> = (0..5).map(|i| format!("G1 X{i}")).collect();
        let tx = streamer.start(lines, JobKind::Program).unwrap_or_else(|_| Tx::default());
        let sent = tx.bytes.iter().filter(|b| **b == b'\n').count();
        assert_eq!(sent, 3);
        let (events, tx2) = streamer.on_rx(b"ok\n");
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Ack { index: 0, .. })));
        let added = tx2.bytes.iter().filter(|b| **b == b'\n').count();
        assert_eq!(added, 1);
        assert_eq!(streamer.last_acked_program_line(), Some(0));
    }

    #[test]
    fn send_response_waits_for_ok() {
        let mut streamer = Streamer::new(StreamMode::SendResponse);
        connect(&mut streamer);
        let tx = streamer.start(vec!["G1 X0".into(), "G1 X1".into()], JobKind::Program).unwrap_or_else(|_| Tx::default());
        assert_eq!(tx.bytes.iter().filter(|b| **b == b'\n').count(), 1);
        let (_, tx2) = streamer.on_rx(b"ok\n");
        assert_eq!(tx2.bytes.iter().filter(|b| **b == b'\n').count(), 1);
    }

    #[test]
    fn refuses_character_counting_without_a_buffer() {
        let err = StreamMode::character_counting(0);
        assert!(err.is_err());
    }

    #[test]
    fn pause_resume_and_lockouts() {
        let mut streamer = counting(128);
        connect(&mut streamer);
        assert!(streamer.can_start());
        assert!(streamer.can_jog());
        let _ = streamer.start(vec!["G1 X0".into(), "G1 X1".into(), "G1 X2".into()], JobKind::Program);
        assert!(streamer.can_pause());
        assert!(streamer.can_stop());
        assert!(!streamer.can_jog());
        assert!(!streamer.can_start());
        let tx = streamer.pause().unwrap_or_else(|_| Tx::default());
        assert_eq!(tx.bytes, vec![REALTIME_HOLD]);
        assert!(streamer.can_resume());
        assert!(!streamer.can_jog());
        assert!(streamer.can_stop());
        let _ = streamer.resume();
        assert!(streamer.can_pause());
    }

    #[test]
    fn stop_keeps_the_last_acked_line() {
        let mut streamer = counting(128);
        connect(&mut streamer);
        let _ = streamer.start(vec!["G1 X0".into(), "G1 X1".into()], JobKind::Program);
        let _ = streamer.on_rx(b"ok\n");
        assert_eq!(streamer.last_acked_program_line(), Some(0));
        let tx = streamer.stop().unwrap_or_else(|_| Tx::default());
        assert_eq!(tx.bytes, vec![REALTIME_RESET]);
        let (events, tx2) = streamer.on_rx(b"Grbl 1.1h ['$' for help]\n");
        assert!(tx2.bytes.windows(3).any(|w| w == b"M5\n"));
        let _ = events;
        let (events, _) = streamer.on_rx(b"ok\n<Idle|MPos:0.000,0.000,0.000>\n");
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Stopped { m5_verified: true, last_line: Some(0) })));
        assert_eq!(streamer.last_acked_program_line(), Some(0));
    }

    #[test]
    fn error_and_alarm_include_the_code() {
        let mut streamer = counting(128);
        connect(&mut streamer);
        let _ = streamer.start(vec!["G999".into()], JobKind::Program);
        let (events, _) = streamer.on_rx(b"error:20\n");
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Error { code: 20, .. })));
        let (events, _) = streamer.on_rx(b"ALARM:3\n");
        assert!(events.iter().any(|e| matches!(e, StreamEvent::Alarm { code: 3, .. })));
        assert!(streamer.can_unlock());
        assert!(!streamer.can_jog());
        assert!(!streamer.can_start());
    }

    #[test]
    fn disconnect_mid_job_does_not_claim_success() {
        let mut streamer = counting(128);
        connect(&mut streamer);
        let _ = streamer.start(vec!["G1 X0".into(), "G1 X1".into()], JobKind::Program);
        let event = streamer.on_disconnect();
        match event {
            StreamEvent::Message(text) => assert!(text.contains("not complete")),
            _ => assert!(false, "expected a failure message"),
        }
    }

    #[test]
    fn jog_is_refused_in_hold() {
        let mut streamer = counting(128);
        connect(&mut streamer);
        let _ = streamer.start(vec!["G1 X0".into(), "G1 X1".into()], JobKind::Program);
        let _ = streamer.pause();
        let _ = streamer.on_rx(b"<Hold:0|MPos:0.000,0.000,0.000>\n");
        assert!(streamer.jog(0, 1.0, 300.0).is_err());
    }
}
