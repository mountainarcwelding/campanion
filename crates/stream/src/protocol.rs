//! Parse Grbl response and push messages per the public interface document.

use crate::docs::WELCOME_PREFIX;

/// A complete line received from Grbl, classified per the interface document.
#[derive(Debug, Clone, PartialEq)]
pub enum RxMessage {
    /// `ok` — the last streamed line was parsed (or queued to be executed).
    Ok,
    /// `error:X` — the last streamed line was purged. `code` is X.
    Error { code: u32, raw: String },
    /// `ALARM:X` — Grbl entered alarm. `code` is X.
    Alarm { code: u32, raw: String },
    /// Welcome / reset complete: `Grbl X.Xx ['$' for help]`.
    Welcome(String),
    /// `<...>` real-time status report.
    Status(StatusReport),
    /// Any other push message (`[MSG:...]`, `$x=val`, and so on).
    Push(String),
}

/// Machine state from the first field of a status report.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MachineState {
    Idle,
    Run,
    Hold {
        complete: bool,
    },
    Jog,
    Alarm,
    Door,
    Check,
    Home,
    Sleep,
    /// A state word this crate does not special-case. The raw token is kept.
    Other(String),
}

impl MachineState {
    /// Parse the first status-report field (`Idle`, `Hold:0`, …).
    pub fn parse(field: &str) -> Self {
        let (name, sub) = match field.split_once(':') {
            Some((name, sub)) => (name, Some(sub)),
            None => (field, None),
        };
        match name {
            "Idle" => Self::Idle,
            "Run" => Self::Run,
            "Hold" => Self::Hold { complete: sub == Some("0") },
            "Jog" => Self::Jog,
            "Alarm" => Self::Alarm,
            "Door" => Self::Door,
            "Check" => Self::Check,
            "Home" => Self::Home,
            "Sleep" => Self::Sleep,
            _ => Self::Other(field.to_string()),
        }
    }

    /// True when Grbl reports Idle.
    pub fn is_idle(&self) -> bool {
        matches!(self, Self::Idle)
    }

    /// True when a `$J=` jog is acceptable (Idle or Jog), per the jogging document.
    pub fn can_jog(&self) -> bool {
        matches!(self, Self::Idle | Self::Jog)
    }
}

/// Parsed `<state|MPos:...|WPos:...|WCO:...|A:...>` report.
#[derive(Debug, Clone, PartialEq)]
pub struct StatusReport {
    pub raw: String,
    pub state: MachineState,
    /// Machine position in the report's units, when `MPos:` is present.
    pub machine_pos: Option<[f64; 3]>,
    /// Work position in the report's units, when `WPos:` is present.
    pub work_pos: Option<[f64; 3]>,
    /// Work coordinate offset, when `WCO:` is present.
    pub wco: Option<[f64; 3]>,
    /// Accessory letters after `A:` (`S`/`C` spindle, `F` flood, `M` mist).
    pub accessories: String,
}

impl StatusReport {
    /// Work position: `WPos` if present, otherwise `MPos - WCO` when both exist.
    pub fn work_or_computed(&self) -> Option<[f64; 3]> {
        if let Some(work) = self.work_pos {
            return Some(work);
        }
        match (self.machine_pos, self.wco) {
            (Some(mpos), Some(wco)) => Some([mpos[0] - wco[0], mpos[1] - wco[1], mpos[2] - wco[2]]),
            _ => None,
        }
    }

    /// Machine position: `MPos` if present, otherwise `WPos + WCO` when both exist.
    pub fn machine_or_computed(&self) -> Option<[f64; 3]> {
        if let Some(machine) = self.machine_pos {
            return Some(machine);
        }
        match (self.work_pos, self.wco) {
            (Some(wpos), Some(wco)) => Some([wpos[0] + wco[0], wpos[1] + wco[1], wpos[2] + wco[2]]),
            _ => None,
        }
    }

    /// True when the accessory field includes spindle CW (`S`) or CCW (`C`).
    pub fn spindle_on(&self) -> bool {
        self.accessories.contains('S') || self.accessories.contains('C')
    }
}

/// Parse one newline-trimmed line from the serial stream.
pub fn parse_line(line: &str) -> Option<RxMessage> {
    let line = line.trim_end_matches(['\r', '\n']).trim();
    if line.is_empty() {
        return None;
    }
    if line == "ok" {
        return Some(RxMessage::Ok);
    }
    if let Some(rest) = line.strip_prefix("error:") {
        return Some(RxMessage::Error { code: parse_code(rest), raw: line.to_string() });
    }
    if let Some(rest) = line.strip_prefix("ALARM:") {
        return Some(RxMessage::Alarm { code: parse_code(rest), raw: line.to_string() });
    }
    if line.starts_with(WELCOME_PREFIX) {
        return Some(RxMessage::Welcome(line.to_string()));
    }
    if line.starts_with('<') && line.ends_with('>') {
        return Some(RxMessage::Status(parse_status(line)));
    }
    Some(RxMessage::Push(line.to_string()))
}

fn parse_code(rest: &str) -> u32 {
    rest.trim().split(|ch: char| !ch.is_ascii_digit()).next().and_then(|n| n.parse().ok()).unwrap_or(0)
}

fn parse_status(raw: &str) -> StatusReport {
    let inner = raw.trim_start_matches('<').trim_end_matches('>');
    let mut fields = inner.split('|');
    let state = MachineState::parse(fields.next().unwrap_or(""));
    let mut machine_pos = None;
    let mut work_pos = None;
    let mut wco = None;
    let mut accessories = String::new();
    for field in fields {
        if let Some(value) = field.strip_prefix("MPos:") {
            machine_pos = parse_xyz(value);
        } else if let Some(value) = field.strip_prefix("WPos:") {
            work_pos = parse_xyz(value);
        } else if let Some(value) = field.strip_prefix("WCO:") {
            wco = parse_xyz(value);
        } else if let Some(value) = field.strip_prefix("A:") {
            accessories = value.to_string();
        }
    }
    StatusReport { raw: raw.to_string(), state, machine_pos, work_pos, wco, accessories }
}

fn parse_xyz(value: &str) -> Option<[f64; 3]> {
    let mut parts = value.split(',');
    let x = parts.next()?.parse().ok()?;
    let y = parts.next()?.parse().ok()?;
    let z = parts.next()?.parse().ok()?;
    Some([x, y, z])
}

/// Official short description for an `error:` code, from the interface document table.
pub fn error_description(code: u32) -> &'static str {
    match code {
        1 => "G-code words consist of a letter and a value. Letter was not found.",
        2 => "Numeric value format is not valid or missing an expected value.",
        3 => "Grbl '$' system command was not recognized or supported.",
        4 => "Negative value received for an expected positive value.",
        8 => "Grbl '$' command cannot be used unless Grbl is IDLE.",
        9 => "G-code locked out during alarm or jog state.",
        11 => "Max characters per line exceeded. Line was not processed.",
        15 => "Jog target exceeds machine travel. Command ignored.",
        16 => "Jog command with no '=' or contains prohibited g-code.",
        20 => "Unsupported or invalid g-code command found in block.",
        22 => "Feed rate has not yet been set or is undefined.",
        _ => "See the Grbl v1.1 interface document error table.",
    }
}

/// Official short description for an `ALARM:` code, from the interface document table.
pub fn alarm_description(code: u32) -> &'static str {
    match code {
        1 => "Hard limit triggered. Machine position is likely lost.",
        2 => "G-code motion target exceeds machine travel. Position retained.",
        3 => "Reset while in motion. Grbl cannot guarantee position.",
        4 => "Probe fail. Probe not in the expected initial state.",
        5 => "Probe fail. Probe did not contact the workpiece.",
        6 => "Homing fail. Reset during active homing cycle.",
        7 => "Homing fail. Safety door was opened during homing.",
        8 => "Homing fail. Cycle failed to clear limit switch when pulling off.",
        9 => "Homing fail. Could not find limit switch within search distance.",
        _ => "See the Grbl v1.1 interface document alarm table.",
    }
}

/// Split incoming bytes into complete lines, leaving a trailing partial line in `pending`.
pub fn take_lines(pending: &mut Vec<u8>, incoming: &[u8]) -> Vec<String> {
    pending.extend_from_slice(incoming);
    let mut lines = Vec::new();
    while let Some(index) = pending.iter().position(|b| *b == b'\n' || *b == b'\r') {
        let raw: Vec<u8> = pending.drain(..=index).collect();
        let text = String::from_utf8_lossy(&raw).to_string();
        if text.trim_end_matches(['\r', '\n']).is_empty() {
            continue;
        }
        lines.push(text);
    }
    lines
}

/// Encode a program line for the serial stream: trailing CR/LF stripped, one LF appended.
/// The returned length is what character-counting must track.
pub fn encode_line(line: &str) -> Vec<u8> {
    let trimmed = line.trim_end_matches(['\r', '\n']);
    let mut bytes = trimmed.as_bytes().to_vec();
    bytes.extend_from_slice(crate::docs::LINE_ENDING);
    bytes
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::assertions_on_constants, clippy::unwrap_used, clippy::manual_unwrap_or_default)]
mod tests {
    use super::*;

    #[test]
    fn parses_ok_error_alarm_welcome_and_status() {
        assert_eq!(parse_line("ok"), Some(RxMessage::Ok));
        match parse_line("error:9") {
            Some(RxMessage::Error { code, .. }) => assert_eq!(code, 9),
            other => panic!("expected error, got {other:?}"),
        }
        match parse_line("ALARM:3") {
            Some(RxMessage::Alarm { code, .. }) => assert_eq!(code, 3),
            other => panic!("expected alarm, got {other:?}"),
        }
        assert!(matches!(parse_line("Grbl 1.1h ['$' for help]"), Some(RxMessage::Welcome(_))));
        let RxMessage::Status(status) = parse_line("<Idle|MPos:1.000,2.000,3.000|WCO:0.500,0.000,0.000|A:S>").expect("status") else {
            panic!("expected status");
        };
        assert!(status.state.is_idle());
        assert_eq!(status.machine_pos, Some([1.0, 2.0, 3.0]));
        assert_eq!(status.work_or_computed(), Some([0.5, 2.0, 3.0]));
        assert!(status.spindle_on());
    }

    #[test]
    fn hold_substate_and_wpos_plus_wco() {
        let RxMessage::Status(status) = parse_line("<Hold:0|WPos:1.0,2.0,3.0|WCO:4.0,5.0,6.0>").expect("status") else {
            panic!("expected status");
        };
        assert_eq!(status.state, MachineState::Hold { complete: true });
        assert_eq!(status.machine_or_computed(), Some([5.0, 7.0, 9.0]));
    }
}
