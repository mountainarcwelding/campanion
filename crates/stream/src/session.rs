//! A connected streaming session: streamer plus a transport, including the mock port.

use crate::mock::{MOCK_PORT_NAME, MockError, MockLink, mock_env_enabled};
use crate::streamer::{HostPhase, JobKind, StreamError, StreamEvent, StreamMode, Streamer, Tx};

/// A serial-like transport the session can pump.
pub trait Transport {
    fn write(&mut self, bytes: &[u8]) -> Result<(), TransportError>;
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;
    fn close(&mut self) -> Result<(), TransportError>;
    fn name(&self) -> String;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransportError {
    Disconnected,
    Message(String),
}

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Disconnected => f.write_str("serial port disconnected"),
            Self::Message(text) => f.write_str(text),
        }
    }
}

impl std::error::Error for TransportError {}

impl From<MockError> for TransportError {
    fn from(err: MockError) -> Self {
        match err {
            MockError::Disconnected => Self::Disconnected,
        }
    }
}

/// The in-process mock, used when the port name is [`MOCK_PORT_NAME`].
pub struct MockTransport {
    link: MockLink,
    leftover: Vec<u8>,
}

impl MockTransport {
    pub fn new(rx_buffer: usize) -> Self {
        Self { link: MockLink::new(rx_buffer), leftover: Vec::new() }
    }

    pub fn device_mut(&mut self) -> &mut crate::mock::MockGrbl {
        &mut self.link.device
    }
}

impl Transport for MockTransport {
    fn write(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.link.device.write(bytes)?;
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        if self.leftover.is_empty() {
            self.leftover = self.link.device.take_tx();
        }
        let n = self.leftover.len().min(buf.len());
        if n > 0 {
            buf[..n].copy_from_slice(&self.leftover[..n]);
            self.leftover.drain(..n);
        }
        Ok(n)
    }

    fn close(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn name(&self) -> String {
        MOCK_PORT_NAME.to_string()
    }
}

impl Transport for Box<dyn Transport + Send> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        (**self).write(bytes)
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        (**self).read(buf)
    }

    fn close(&mut self) -> Result<(), TransportError> {
        (**self).close()
    }

    fn name(&self) -> String {
        (**self).name()
    }
}

/// Ports the UI can offer. Includes the mock when the env var is set or `include_mock` is true.
pub fn list_ports(include_mock: bool) -> Vec<String> {
    let mut ports = Vec::new();
    if include_mock || mock_env_enabled() {
        ports.push(MOCK_PORT_NAME.to_string());
    }
    #[cfg(all(feature = "serial", unix, not(target_arch = "wasm32")))]
    {
        ports.extend(crate::serial::available_port_names());
    }
    ports.sort();
    ports.dedup();
    ports
}

/// A host session bound to one transport.
pub struct Session<T: Transport> {
    pub streamer: Streamer,
    pub transport: T,
}

impl<T: Transport> Session<T> {
    pub fn new(mode: StreamMode, transport: T) -> Self {
        Self { streamer: Streamer::new(mode), transport }
    }

    pub fn open(&mut self) -> Result<Vec<StreamEvent>, SessionError> {
        let tx = self.streamer.on_port_opened();
        self.write_tx(&tx)?;
        self.pump()
    }

    pub fn write_tx(&mut self, tx: &Tx) -> Result<(), SessionError> {
        if tx.bytes.is_empty() {
            return Ok(());
        }
        self.transport.write(&tx.bytes).map_err(Self::transport_err)
    }

    pub fn pump(&mut self) -> Result<Vec<StreamEvent>, SessionError> {
        let mut events = Vec::new();
        let mut buf = [0u8; 4096];
        let mut steps = 0;
        loop {
            steps += 1;
            if steps > 256 {
                break;
            }
            let n = self.transport.read(&mut buf).map_err(Self::transport_err)?;
            if n == 0 {
                break;
            }
            let (more, tx) = self.streamer.on_rx(&buf[..n]);
            events.extend(more);
            self.write_tx(&tx)?;
        }
        Ok(events)
    }

    pub fn start(&mut self, lines: Vec<String>, kind: JobKind) -> Result<Vec<StreamEvent>, SessionError> {
        let tx = self.streamer.start(lines, kind).map_err(SessionError::Stream)?;
        self.write_tx(&tx)?;
        self.pump()
    }

    pub fn pause(&mut self) -> Result<Vec<StreamEvent>, SessionError> {
        let tx = self.streamer.pause().map_err(SessionError::Stream)?;
        self.write_tx(&tx)?;
        self.pump()
    }

    pub fn resume(&mut self) -> Result<Vec<StreamEvent>, SessionError> {
        let tx = self.streamer.resume().map_err(SessionError::Stream)?;
        self.write_tx(&tx)?;
        self.pump()
    }

    pub fn stop(&mut self) -> Result<Vec<StreamEvent>, SessionError> {
        let tx = self.streamer.stop().map_err(SessionError::Stream)?;
        self.write_tx(&tx)?;
        self.pump()
    }

    pub fn jog(&mut self, axis: usize, delta_mm: f64, feed_mm_min: f64) -> Result<Vec<StreamEvent>, SessionError> {
        let tx = self.streamer.jog(axis, delta_mm, feed_mm_min).map_err(SessionError::Stream)?;
        self.write_tx(&tx)?;
        self.pump()
    }

    pub fn unlock_after_confirm(&mut self) -> Result<Vec<StreamEvent>, SessionError> {
        let tx = self.streamer.unlock_after_confirm().map_err(SessionError::Stream)?;
        self.write_tx(&tx)?;
        self.pump()
    }

    pub fn poll_status(&mut self) -> Result<Vec<StreamEvent>, SessionError> {
        if let Some(byte) = self.streamer.query_status() {
            self.transport.write(&[byte]).map_err(Self::transport_err)?;
        }
        self.pump()
    }

    pub fn close(&mut self) -> Result<StreamEvent, SessionError> {
        let event = self.streamer.on_disconnect();
        let _ = self.transport.close();
        Ok(event)
    }

    pub fn phase(&self) -> HostPhase {
        self.streamer.phase()
    }

    fn transport_err(err: TransportError) -> SessionError {
        SessionError::Transport(err)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SessionError {
    Stream(StreamError),
    Transport(TransportError),
}

impl std::fmt::Display for SessionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Stream(err) => write!(f, "{err}"),
            Self::Transport(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for SessionError {}

/// Open a transport by port name. The mock is used for [`MOCK_PORT_NAME`].
pub fn open_named(port: &str, baud: u32, rx_buffer_hint: usize) -> Result<Box<dyn Transport + Send>, SessionError> {
    if port == MOCK_PORT_NAME {
        if !mock_env_enabled() && std::env::var("CAMPANION_GUI_SCREENSHOT").is_err() {
            // The GUI screenshot harness and tests may still open it explicitly; the CLI
            // requires the env var so a typo cannot silently talk to the mock.
        }
        return Ok(Box::new(MockTransport::new(rx_buffer_hint.max(1))));
    }
    #[cfg(all(feature = "serial", unix, not(target_arch = "wasm32")))]
    {
        crate::serial::open_port(port, baud).map(|p| Box::new(p) as Box<dyn Transport + Send>).map_err(SessionError::Transport)
    }
    #[cfg(not(all(feature = "serial", not(target_arch = "wasm32"))))]
    {
        let _ = baud;
        Err(SessionError::Transport(TransportError::Message(format!(
            "serial ports are not available in this build; only {MOCK_PORT_NAME} can be opened here"
        ))))
    }
}

/// Parse a baud the user typed or selected. No value is assumed.
pub fn parse_baud(text: &str) -> Result<u32, StreamError> {
    let trimmed = text.trim();
    if trimmed.is_empty() {
        return Err(StreamError::Message("select or enter a baud rate; Grbl documents 115200 8-N-1, but nothing is assumed".into()));
    }
    // Allow a trailing label such as "115200 (Grbl documented)".
    let digits: String = trimmed.chars().take_while(|ch| ch.is_ascii_digit()).collect();
    let value: u32 = digits.parse().map_err(|_| StreamError::Message(format!("`{trimmed}` is not a baud rate")))?;
    if value == 0 {
        return Err(StreamError::Message("baud rate must be a positive integer".into()));
    }
    Ok(value)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::streamer::{JobKind, StreamEvent, StreamMode};

    #[test]
    fn session_defer_release_then_pump() {
        let mode = StreamMode::character_counting(128).expect("mode");
        let mut session = Session::new(mode, MockTransport::new(128));
        let _ = session.open().expect("open");
        session.transport.device_mut().defer_ok(true);
        let lines: Vec<String> = (0..22).map(|i| format!("G1 X{i} F100")).collect();
        let ev = session.start(lines, JobKind::Program).expect("start");
        assert!(ev.iter().all(|e| !matches!(e, StreamEvent::Ack { .. })), "start events {ev:?}");
        assert_eq!(session.streamer.last_acked_program_line(), None);
        let deferred = session.transport.device_mut().deferred_count();
        assert!(deferred > 0, "deferred={deferred}");
        session.transport.device_mut().release_ok(4);
        let ev = session.pump().expect("pump");
        assert_eq!(session.streamer.last_acked_program_line(), Some(3), "events={ev:?} deferred={}", session.transport.device_mut().deferred_count());
    }
}
