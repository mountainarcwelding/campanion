//! Run-tab connection to a Grbl-class controller (or the in-process mock).

use std::time::{Duration, Instant};

use campanion_stream::mock::MOCK_PORT_NAME;
use campanion_stream::session::MockTransport;
use campanion_stream::{
    HostPhase, JobKind, Session, StreamEvent, StreamMode, Transport, list_ports, mode_from_profile, open_named, parse_baud, parse_feed,
};

use crate::session::JobSession;
use crate::{GuiState, RunPhase};

/// Banner on the Run tab and in About, in addition to the CLI-first notice.
pub const STREAM_HARDWARE_NOTICE: &str = "NOT hardware-tested. First run: low power, hand on the power switch.";

const CONSOLE_LIMIT: usize = 200;
const STATUS_PERIOD: Duration = Duration::from_millis(200);

/// A live serial (or mock) session owned by the window.
pub enum LiveSession {
    Mock(Session<MockTransport>),
    Serial(Session<Box<dyn Transport + Send>>),
}

impl LiveSession {
    fn streamer(&self) -> &campanion_stream::Streamer {
        match self {
            Self::Mock(session) => &session.streamer,
            Self::Serial(session) => &session.streamer,
        }
    }

    fn pump(&mut self) -> Result<Vec<StreamEvent>, String> {
        match self {
            Self::Mock(session) => session.pump().map_err(|err| err.to_string()),
            Self::Serial(session) => session.pump().map_err(|err| err.to_string()),
        }
    }

    fn write_and_pump(&mut self, op: SessionOp) -> Result<Vec<StreamEvent>, String> {
        match self {
            Self::Mock(session) => dispatch(session, op),
            Self::Serial(session) => dispatch(session, op),
        }
    }
}

enum SessionOp {
    Open,
    Start(Vec<String>, JobKind),
    Pause,
    Resume,
    Stop,
    Jog { axis: usize, delta: f64, feed: f64 },
    Unlock,
    Poll,
    Close,
}

fn dispatch<T: Transport>(session: &mut Session<T>, op: SessionOp) -> Result<Vec<StreamEvent>, String> {
    match op {
        SessionOp::Open => session.open().map_err(|err| err.to_string()),
        SessionOp::Start(lines, kind) => session.start(lines, kind).map_err(|err| err.to_string()),
        SessionOp::Pause => session.pause().map_err(|err| err.to_string()),
        SessionOp::Resume => session.resume().map_err(|err| err.to_string()),
        SessionOp::Stop => session.stop().map_err(|err| err.to_string()),
        SessionOp::Jog { axis, delta, feed } => session.jog(axis, delta, feed).map_err(|err| err.to_string()),
        SessionOp::Unlock => session.unlock_after_confirm().map_err(|err| err.to_string()),
        SessionOp::Poll => session.poll_status().map_err(|err| err.to_string()),
        SessionOp::Close => session.close().map(|event| vec![event]).map_err(|err| err.to_string()),
    }
}

/// Connection + console owned by the GUI.
pub struct RunConnection {
    live: LiveSession,
    pub console: Vec<String>,
    last_poll: Instant,
    pub confirm_unlock: bool,
}

impl RunConnection {
    pub fn connect(port: &str, baud_text: &str, mode: StreamMode) -> Result<(Self, Vec<StreamEvent>), String> {
        if port.trim().is_empty() {
            return Err("select a serial port".to_string());
        }
        let baud = parse_baud(baud_text).map_err(|err| err.to_string())?;
        let rx_hint = match mode {
            StreamMode::CharacterCounting { rx_buffer } => rx_buffer,
            StreamMode::SendResponse => 4096,
        };
        let live = if port == MOCK_PORT_NAME {
            let transport = MockTransport::new(rx_hint);
            LiveSession::Mock(Session::new(mode, transport))
        } else {
            let transport = open_named(port, baud, rx_hint).map_err(|err| err.to_string())?;
            LiveSession::Serial(Session::new(mode, transport))
        };
        let mut conn = Self { live, console: Vec::new(), last_poll: Instant::now(), confirm_unlock: false };
        let events = conn.live.write_and_pump(SessionOp::Open)?;
        Ok((conn, events))
    }

    pub fn mock_mut(&mut self) -> Option<&mut campanion_stream::mock::MockGrbl> {
        match &mut self.live {
            LiveSession::Mock(session) => Some(session.transport.device_mut()),
            LiveSession::Serial(_) => None,
        }
    }

    pub fn phase(&self) -> HostPhase {
        self.live.streamer().phase()
    }

    pub fn start(&mut self, lines: Vec<String>, kind: JobKind) -> Result<Vec<StreamEvent>, String> {
        self.live.write_and_pump(SessionOp::Start(lines, kind))
    }

    pub fn pause(&mut self) -> Result<Vec<StreamEvent>, String> {
        self.live.write_and_pump(SessionOp::Pause)
    }

    pub fn resume(&mut self) -> Result<Vec<StreamEvent>, String> {
        self.live.write_and_pump(SessionOp::Resume)
    }

    pub fn stop(&mut self) -> Result<Vec<StreamEvent>, String> {
        self.live.write_and_pump(SessionOp::Stop)
    }

    pub fn jog(&mut self, axis: usize, delta: f64, feed: f64) -> Result<Vec<StreamEvent>, String> {
        self.live.write_and_pump(SessionOp::Jog { axis, delta, feed })
    }

    pub fn unlock_after_confirm(&mut self) -> Result<Vec<StreamEvent>, String> {
        self.confirm_unlock = false;
        self.live.write_and_pump(SessionOp::Unlock)
    }

    pub fn tick(&mut self, force_status: bool) -> Result<Vec<StreamEvent>, String> {
        let mut events = self.live.pump()?;
        if force_status || self.last_poll.elapsed() >= STATUS_PERIOD {
            self.last_poll = Instant::now();
            events.extend(self.live.write_and_pump(SessionOp::Poll)?);
        }
        Ok(events)
    }

    pub fn close(&mut self) -> Result<Vec<StreamEvent>, String> {
        self.live.write_and_pump(SessionOp::Close)
    }

    pub fn last_acked_program_line(&self) -> Option<usize> {
        self.live.streamer().last_acked_program_line()
    }

    pub fn last_alarm(&self) -> Option<(u32, String)> {
        self.live.streamer().last_alarm().map(|(code, text)| (code, text.to_string()))
    }

    pub fn last_error(&self) -> Option<(u32, String)> {
        self.live.streamer().last_error().map(|(code, text)| (code, text.to_string()))
    }

    pub fn append(&mut self, line: impl Into<String>) {
        self.console.push(line.into());
        if self.console.len() > CONSOLE_LIMIT {
            let drop = self.console.len() - CONSOLE_LIMIT;
            self.console.drain(..drop);
        }
    }
}

/// Ports for the Run tab picker. Includes the mock during screenshots or when the env var is set.
pub fn available_ports(include_mock: bool) -> Vec<String> {
    list_ports(include_mock)
}

pub fn stream_mode_from_session(session: &JobSession, store: &campanion_cli::ProfileStore, safe_mode: bool) -> Result<StreamMode, String> {
    let rx_buffer = match crate::machines::session_profile(store, session) {
        Ok(Some((profile, _))) => profile.rx_buffer,
        Ok(None) => None,
        Err(err) => return Err(err),
    };
    mode_from_profile(rx_buffer, safe_mode).map_err(|err| err.to_string())
}

pub fn apply_events(state: &mut GuiState, events: &[StreamEvent], console: &mut RunConnection) {
    for event in events {
        match event {
            StreamEvent::Welcome(text) | StreamEvent::Push(text) | StreamEvent::Message(text) => {
                console.append(text.clone());
                state.status = text.clone();
            }
            StreamEvent::Status(status) => {
                if let Some(work) = status.work_or_computed() {
                    state.work_pos = work;
                }
                if let Some(machine) = status.machine_or_computed() {
                    state.machine_pos = machine;
                }
            }
            StreamEvent::Ack { index, kind } => {
                if *kind == JobKind::Program {
                    state.run_line = u32::try_from(*index + 1).unwrap_or(u32::MAX);
                }
                console.append(format!("ok line {}", index + 1));
            }
            StreamEvent::Error { code, text, index } => {
                let where_ = index.map(|i| format!(" on line {}", i + 1)).unwrap_or_default();
                let line = format!("error:{code}{where_}: {text}");
                console.append(line.clone());
                state.status = line;
            }
            StreamEvent::Alarm { code, text } => {
                let line = format!("ALARM:{code}: {text}");
                console.append(line.clone());
                state.status = line;
                state.run = RunPhase::Stopped;
            }
            StreamEvent::Stopped { m5_verified, last_line } => {
                state.run = RunPhase::Stopped;
                if let Some(index) = last_line {
                    state.run_line = u32::try_from(*index + 1).unwrap_or(u32::MAX);
                }
                let line = if *m5_verified {
                    "STOP complete; M5 verified (beam command accepted or spindle reported off)".to_string()
                } else {
                    "STOP complete; M5 could not be verified — check the beam and the alarm".to_string()
                };
                console.append(line.clone());
                state.status = line;
            }
            StreamEvent::JobFinished { kind } => {
                if *kind == JobKind::Program {
                    state.run = RunPhase::Finished;
                    if state.run_lines > 0 {
                        state.run_line = state.run_lines;
                    }
                } else {
                    state.run = RunPhase::Idle;
                }
                console.append("job finished".to_string());
                state.status = "Job finished".to_string();
            }
        }
    }
    sync_phase(state, console.phase());
}

fn sync_phase(state: &mut GuiState, phase: HostPhase) {
    state.connected = !matches!(phase, HostPhase::Disconnected);
    state.run = match phase {
        HostPhase::Disconnected => RunPhase::Idle,
        HostPhase::Connecting | HostPhase::Idle | HostPhase::Jogging => {
            if state.run == RunPhase::Finished || state.run == RunPhase::Stopped {
                state.run
            } else {
                RunPhase::Idle
            }
        }
        HostPhase::Streaming => RunPhase::Running,
        HostPhase::Holding => RunPhase::Paused,
        HostPhase::Stopping | HostPhase::Alarm => RunPhase::Stopped,
    };
    if matches!(phase, HostPhase::Disconnected) {
        state.connected = false;
    }
    state.in_alarm = matches!(phase, HostPhase::Alarm);
}

pub fn parse_jog_or_frame_feed(text: &str) -> Result<f64, String> {
    parse_feed(text)
}
