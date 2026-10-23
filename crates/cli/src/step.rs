//! Turn one planar face of a STEP part into DXF text.
//!
//! SolidCraft stays a separate program. This module runs its documented
//! `solidcraft-cli run` command and does not link or copy SolidCraft.
//! [`UpAxis::Auto`] probes the exporter header. A Y-up file is opened with
//! `doc.open` `"up": "y"` so the face point is in a Z-up frame. Z up keeps
//! `run <script> --in <step> --quiet`.

use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::error::CliError;

/// Env var that names the solidcraft-cli executable.
#[allow(dead_code)]
pub const CLI_ENV: &str = "CAMPANION_SOLIDCRAFT_CLI";

const CLI_NAME: &str = "solidcraft-cli";
const TIMEOUT: Duration = Duration::from_secs(120);
const POLL_EVERY: Duration = Duration::from_millis(50);
/// Same 32 MiB cap the CLI uses for other imported text.
const MAX_DXF_BYTES: usize = 32 * 1024 * 1024;
const MAX_LOG_BYTES: usize = 256 * 1024;
const MAX_LOG_LINES: usize = 20;
const TEMP_ATTEMPTS: usize = 64;
/// Header replies are a few kilobytes. Anything larger is an unusable probe, same as bad JSON.
const MAX_PROBE_BYTES: usize = 1024 * 1024;
/// Exporter names in the stderr note. A longer or multi-line value is not printed.
const NOTE_EXPORTER_MAX: usize = 200;

/// Which STEP axis is up when a face is exported.
///
/// SolidCraft does not turn a file on its own. [`Self::Z`] keeps the file's axes.
/// [`Self::Y`] turns the part +90° about +X (file Y becomes Z, file Z becomes -Y).
/// [`Self::Auto`] uses Y only when the exporter header says `y`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UpAxis {
    /// Keep the file's axes. The `up` parameter is omitted (`"z"` is the default, and older
    /// SolidCraft builds reject the parameter).
    Z,
    /// Turn Y up onto Z up.
    Y,
    /// Header `exporter_up == "y"` selects [`Self::Y`]. Anything else selects [`Self::Z`].
    Auto,
}

impl UpAxis {
    /// `z`, `y` or `auto`, case-sensitive.
    pub fn parse(text: &str) -> Result<Self, CliError> {
        match text {
            "z" => Ok(Self::Z),
            "y" => Ok(Self::Y),
            "auto" => Ok(Self::Auto),
            _ => Err(CliError::usage(format!("--up-axis must be z, y or auto (got {text})"))),
        }
    }
}

/// Finds solidcraft-cli: [`CLI_ENV`] if set and non-empty (must be an existing file), else
/// `solidcraft-cli` searched on `PATH` ([`std::env::split_paths`]). `None` when not found.
#[allow(dead_code)]
pub fn find_cli() -> Option<PathBuf> {
    match std::env::var_os(CLI_ENV) {
        Some(value) if !value.is_empty() => {
            let path = PathBuf::from(value);
            path.is_file().then_some(path)
        }
        _ => search_path(),
    }
}

fn search_path() -> Option<PathBuf> {
    let paths = std::env::var_os("PATH")?;
    for dir in std::env::split_paths(&paths) {
        let candidate = dir.join(CLI_NAME);
        if candidate.is_file() {
            return Some(candidate);
        }
    }
    None
}

/// Exports the planar face of `step` that contains `face` (mm) as DXF text.
///
/// `face` is in the frame after `up` is applied. [`UpAxis::Y`], and [`UpAxis::Auto`] when the
/// header says Y, turn the part inside SolidCraft and then use `face` in that Z-up frame.
/// [`UpAxis::Z`] keeps `solidcraft-cli run <script> --in <step> --quiet`. [`UpAxis::Auto`] probes
/// first. A probe that exits non-zero is an error. A probe that exits zero but does not return a
/// usable reply is treated as Z up, and nothing is printed.
#[allow(dead_code)]
pub fn face_to_dxf(cli: &Path, step: &Path, face: [f64; 3], up: UpAxis) -> Result<String, CliError> {
    let [x, y, z] = face;
    if ![x, y, z].iter().all(|coord| coord.is_finite()) {
        return Err(CliError::invalid("face point must be three finite millimetre coordinates"));
    }

    let temp = TempDir::create()?;
    let turn = match up {
        UpAxis::Z => false,
        UpAxis::Y => true,
        UpAxis::Auto => probe_needs_turn(cli, step, temp.path())?,
    };
    let script = temp.path.join("script.json");
    let dxf = temp.path.join("face.dxf");
    if turn {
        let absolute = absolute_utf8(step)?;
        write_script(&script, &dxf, x, y, z, Some(absolute.as_str()))?;
        run_cli(cli, &script, step, temp.path(), CliMode::Script)?;
    } else {
        write_script(&script, &dxf, x, y, z, None)?;
        run_cli(cli, &script, step, temp.path(), CliMode::Import { step })?;
    }
    read_dxf(&dxf)
}

/// The error used when the CLI is missing.
#[allow(dead_code)]
pub fn missing_cli_error() -> CliError {
    CliError::run(
        "STEP import needs SolidCraft's solidcraft-cli: install SolidCraft or set CAMPANION_SOLIDCRAFT_CLI to the solidcraft-cli executable",
    )
}

/// Private directory removed on every exit, including panic.
struct TempDir {
    path: PathBuf,
}

impl TempDir {
    fn create() -> Result<Self, CliError> {
        let mut seen = None;
        for _ in 0..TEMP_ATTEMPTS {
            let nanos = SystemTime::now().duration_since(UNIX_EPOCH).map(|elapsed| elapsed.as_nanos()).unwrap_or(0);
            if seen == Some(nanos) {
                thread::sleep(Duration::from_millis(1));
                continue;
            }
            seen = Some(nanos);
            let path = std::env::temp_dir().join(format!("campanion-step-{}-{nanos}", std::process::id()));
            match fs::create_dir(&path) {
                Ok(()) => return Ok(Self { path }),
                Err(err) if err.kind() == io::ErrorKind::AlreadyExists => continue,
                Err(err) => return Err(CliError::run(format!("cannot create {}: {err}", path.display()))),
            }
        }
        Err(CliError::run("cannot create a private temp directory for solidcraft-cli"))
    }

    fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

/// How `solidcraft-cli run` is invoked. Z up passes the part with `--in`. A turned part opens
/// itself in the script. The header probe asks for a JSON reply and does not pass `--in`.
enum CliMode<'a> {
    Import { step: &'a Path },
    Script,
    Probe { json_out: &'a Path },
}

struct ProbeReply {
    y_up: bool,
    exporter: Option<String>,
}

impl ProbeReply {
    fn keep() -> Self {
        Self { y_up: false, exporter: None }
    }
}

fn absolute_utf8(path: &Path) -> Result<String, CliError> {
    let absolute = std::path::absolute(path).map_err(|err| CliError::run(format!("cannot resolve {}: {err}", path.display())))?;
    match absolute.to_str() {
        Some(text) => Ok(text.to_string()),
        None => Err(CliError::run(format!("path {} is not utf-8", path.display()))),
    }
}

/// Header probe in `dir`. A non-zero exit is an error. Odd output selects Z and prints nothing.
fn probe_needs_turn(cli: &Path, step: &Path, dir: &Path) -> Result<bool, CliError> {
    let absolute = absolute_utf8(step)?;
    let script = dir.join("probe-script.json");
    let results = dir.join("probe.json");
    write_probe_script(&script, &absolute)?;
    run_cli(cli, &script, step, dir, CliMode::Probe { json_out: &results })?;
    let reply = read_probe_reply(&results);
    if reply.y_up {
        print_y_up_note(reply.exporter.as_deref())?;
    }
    Ok(reply.y_up)
}

fn write_probe_script(script: &Path, step_path: &str) -> Result<(), CliError> {
    // No `up` parameter. `"z"` is the default, and older SolidCraft builds reject the key.
    let value = serde_json::json!([
        { "command": "doc.open", "params": { "path": step_path } }
    ]);
    write_json(script, &value)
}

fn write_script(script: &Path, dxf: &Path, x: f64, y: f64, z: f64, open_step: Option<&str>) -> Result<(), CliError> {
    let Some(dxf_path) = dxf.to_str() else {
        return Err(CliError::run(format!("temp path {} is not utf-8", dxf.display())));
    };
    let mut commands = Vec::new();
    if let Some(step_path) = open_step {
        commands.push(serde_json::json!({
            "command": "doc.open",
            "params": { "path": step_path, "up": "y" }
        }));
    }
    commands.push(serde_json::json!({
        "command": "sketch.create",
        "params": {
            "plane": { "face": [x, y, z] },
            "name": "campanion_face",
            "project_edges": true
        }
    }));
    commands.push(serde_json::json!({
        "command": "sketch.export_dxf",
        "params": {
            "sketch": "campanion_face",
            "path": dxf_path
        }
    }));
    write_json(script, &serde_json::Value::Array(commands))
}

fn write_json(path: &Path, value: &serde_json::Value) -> Result<(), CliError> {
    let text = serde_json::to_string(value).map_err(|err| CliError::run(format!("script json: {err}")))?;
    fs::write(path, text).map_err(|err| CliError::run(format!("{}: {err}", path.display())))
}

/// First reply object only. Unknown fields are ignored. Anything that is not an array whose first
/// element is an object, or that is not JSON, keeps the file's axes.
fn probe_reply(text: &str) -> ProbeReply {
    let Ok(value) = serde_json::from_str::<serde_json::Value>(text) else {
        return ProbeReply::keep();
    };
    let Some(first) = value.as_array().and_then(|items| items.first()) else {
        return ProbeReply::keep();
    };
    let Some(object) = first.as_object() else {
        return ProbeReply::keep();
    };
    let exporter = object.get("exporter").and_then(serde_json::Value::as_str).map(str::to_string);
    let y_up = object.get("exporter_up").and_then(serde_json::Value::as_str) == Some("y");
    ProbeReply { y_up, exporter }
}

fn read_probe_reply(path: &Path) -> ProbeReply {
    let Ok(file) = File::open(path) else {
        return ProbeReply::keep();
    };
    let mut bytes = Vec::new();
    let limit = (MAX_PROBE_BYTES as u64).saturating_add(1);
    if file.take(limit).read_to_end(&mut bytes).is_err() || bytes.len() > MAX_PROBE_BYTES {
        return ProbeReply::keep();
    }
    let Ok(text) = String::from_utf8(bytes) else {
        return ProbeReply::keep();
    };
    probe_reply(&text)
}

fn y_up_note(exporter: Option<&str>) -> String {
    let name = match exporter {
        Some(text) if exporter_name_ok(text) => text,
        _ => "unknown",
    };
    format!("note: STEP exported by {name} (Y up); turned Y up to Z up. Use --up-axis z to keep the file's axes.")
}

fn exporter_name_ok(text: &str) -> bool {
    !text.is_empty() && text.chars().count() <= NOTE_EXPORTER_MAX && !text.chars().any(char::is_control)
}

fn print_y_up_note(exporter: Option<&str>) -> Result<(), CliError> {
    let mut out = io::stderr().lock();
    writeln!(out, "{}", y_up_note(exporter))?;
    out.flush()?;
    Ok(())
}

struct Running {
    child: Child,
    /// Set once `try_wait` has reaped the child, so [`Drop`] does not kill a recycled pid.
    reaped: bool,
}

impl Running {
    fn spawn(cli: &Path, script: &Path, dir: &Path, mode: CliMode<'_>) -> Result<Self, CliError> {
        let stdout_path = dir.join("stdout.txt");
        let stderr_path = dir.join("stderr.txt");
        let stdout = File::create(&stdout_path).map_err(|err| CliError::run(format!("{}: {err}", stdout_path.display())))?;
        let stderr = File::create(&stderr_path).map_err(|err| CliError::run(format!("{}: {err}", stderr_path.display())))?;
        let mut command = Command::new(cli);
        command.arg("run").arg(script);
        match mode {
            CliMode::Import { step } => {
                command.arg("--in").arg(step);
            }
            CliMode::Script => {}
            CliMode::Probe { json_out } => {
                command.arg("--results").arg("--json-out").arg(json_out);
            }
        }
        let child = command
            .arg("--quiet")
            .stdin(Stdio::null())
            .stdout(stdout)
            .stderr(stderr)
            .spawn()
            .map_err(|err| CliError::run(format!("failed to spawn {}: {err}", cli.display())))?;
        Ok(Self { child, reaped: false })
    }

    fn wait_timeout(&mut self) -> Result<ExitStatus, CliError> {
        let started = Instant::now();
        loop {
            match self.child.try_wait() {
                Ok(Some(status)) => {
                    self.reaped = true;
                    return Ok(status);
                }
                Ok(None) if started.elapsed() >= TIMEOUT => {
                    let _ = self.child.kill();
                    let _ = self.child.wait();
                    self.reaped = true;
                    return Err(CliError::run("solidcraft-cli timed out after 120 s"));
                }
                Ok(None) => thread::sleep(POLL_EVERY),
                Err(err) => return Err(CliError::run(format!("solidcraft-cli: {err}"))),
            }
        }
    }
}

impl Drop for Running {
    fn drop(&mut self) {
        if self.reaped {
            return;
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn run_cli(cli: &Path, script: &Path, step: &Path, dir: &Path, mode: CliMode<'_>) -> Result<(), CliError> {
    let mut running = Running::spawn(cli, script, dir, mode)?;
    let status = running.wait_timeout()?;
    if status.success() {
        return Ok(());
    }
    let detail = failure_detail(&dir.join("stdout.txt"), &dir.join("stderr.txt"));
    Err(CliError::run(format!("solidcraft-cli failed on {}: {detail}", step.display())))
}

fn failure_detail(stdout_path: &Path, stderr_path: &Path) -> String {
    let stderr = read_capped(stderr_path);
    let chosen = if stderr.is_empty() { read_capped(stdout_path) } else { stderr };
    first_lines(&chosen, MAX_LOG_LINES)
}

fn read_capped(path: &Path) -> String {
    let Ok(file) = File::open(path) else {
        return String::new();
    };
    let mut bytes = Vec::new();
    if file.take(MAX_LOG_BYTES as u64).read_to_end(&mut bytes).is_err() {
        return String::new();
    }
    String::from_utf8_lossy(&bytes).into_owned()
}

fn first_lines(text: &str, max_lines: usize) -> String {
    text.lines().take(max_lines).collect::<Vec<_>>().join("\n")
}

fn read_dxf(path: &Path) -> Result<String, CliError> {
    let file = match File::open(path) {
        Ok(file) => file,
        Err(err) if err.kind() == io::ErrorKind::NotFound => {
            return Err(CliError::run("solidcraft-cli wrote no DXF: is the point on a planar face of the part?"));
        }
        Err(err) => return Err(CliError::run(format!("{}: {err}", path.display()))),
    };
    if let Ok(meta) = file.metadata()
        && meta.len() > MAX_DXF_BYTES as u64
    {
        return Err(dxf_too_large(path));
    }
    let mut bytes = Vec::new();
    file.take(MAX_DXF_BYTES as u64 + 1).read_to_end(&mut bytes).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    if bytes.len() > MAX_DXF_BYTES {
        return Err(dxf_too_large(path));
    }
    String::from_utf8(bytes).map_err(|_| CliError::run(format!("{} is not valid UTF-8", path.display())))
}

fn dxf_too_large(path: &Path) -> CliError {
    CliError::run(format!("{} is larger than {MAX_DXF_BYTES} bytes", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::ffi::OsStr;
    use std::process::Command;
    use std::sync::Mutex;

    /// Serializes the probe tests. They do not call `set_var` (edition 2024 makes that
    /// `unsafe`, and this crate forbids `unsafe`), but each one re-executes the test
    /// binary, and the lock keeps those children from piling up on one another.
    static ENV_LOCK: Mutex<()> = Mutex::new(());

    const PROBE: &str = "CAMPANION_STEP_FIND_CLI_PROBE";
    const EXPECT: &str = "CAMPANION_STEP_FIND_CLI_EXPECT";
    const DXF_TEXT: &str = "0\nSECTION\n2\nENTITIES\n0\nENDSEC\n0\nEOF\n";

    struct Scratch(PathBuf);

    impl Scratch {
        fn new(name: &str) -> Self {
            let nanos = SystemTime::now().duration_since(UNIX_EPOCH).map(|elapsed| elapsed.as_nanos()).unwrap_or(0);
            let path = std::env::temp_dir().join(format!("campanion-step-test-{name}-{}-{nanos}", std::process::id()));
            fs::create_dir(&path).expect("scratch dir");
            Self(path)
        }
    }

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    /// `true` when this process was spawned as a probe and must not spawn again.
    fn finish_probe(mode: &str, check: impl FnOnce()) -> bool {
        match std::env::var(PROBE) {
            Ok(actual) if actual == mode => {
                check();
                true
            }
            Ok(_) => true,
            Err(_) => false,
        }
    }

    fn probe(mode: &str, vars: &[(&str, &OsStr)]) {
        let _lock = ENV_LOCK.lock().expect("env lock");
        let exe = std::env::current_exe().expect("current_exe");
        let name = std::thread::current().name().expect("test thread name").to_owned();
        let mut cmd = Command::new(exe);
        cmd.arg("--exact").arg(&name).env(PROBE, mode);
        for (key, value) in vars {
            cmd.env(key, value);
        }
        let output = cmd.output().expect("spawn probe");
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "probe {mode} failed\n{stdout}\n{stderr}");
        assert!(stdout.contains("1 passed;"), "probe {mode} did not run one test\n{stdout}\n{stderr}");
    }

    #[test]
    fn find_cli_env_existing_file() {
        if finish_probe("file", || {
            let expect = std::env::var_os(EXPECT).expect("expect path");
            assert_eq!(find_cli().expect("cli"), PathBuf::from(expect));
        }) {
            return;
        }
        let scratch = Scratch::new("env-file");
        let cli = scratch.0.join("solidcraft-cli");
        fs::write(&cli, b"#!/bin/sh\nexit 0\n").expect("write cli");
        probe("file", &[(CLI_ENV, cli.as_os_str()), (EXPECT, cli.as_os_str())]);
    }

    #[test]
    fn find_cli_env_missing_file() {
        if finish_probe("missing", || {
            assert!(find_cli().is_none());
        }) {
            return;
        }
        let scratch = Scratch::new("env-missing");
        let missing = scratch.0.join("no-such-solidcraft-cli");
        probe("missing", &[(CLI_ENV, missing.as_os_str())]);
    }

    #[test]
    fn missing_cli_message() {
        assert_eq!(
            missing_cli_error().to_string(),
            "STEP import needs SolidCraft's solidcraft-cli: install SolidCraft or set CAMPANION_SOLIDCRAFT_CLI to the solidcraft-cli executable"
        );
    }

    #[test]
    fn face_to_dxf_rejects_non_finite() {
        let err = face_to_dxf(Path::new("solidcraft-cli"), Path::new("part.step"), [f64::NAN, 0.0, 1.0], UpAxis::Auto).expect_err("nan");
        assert!(err.to_string().contains("finite"), "{err}");
    }

    #[test]
    fn up_axis_parse_is_case_sensitive() {
        assert_eq!(UpAxis::parse("z").unwrap_or_else(|err| panic!("{err}")), UpAxis::Z);
        assert_eq!(UpAxis::parse("y").unwrap_or_else(|err| panic!("{err}")), UpAxis::Y);
        assert_eq!(UpAxis::parse("auto").unwrap_or_else(|err| panic!("{err}")), UpAxis::Auto);
        for bad in ["Z", "Y", "Auto", "AUTO", "", "x", "yz", " z", "y ", "auto\n"] {
            let err = UpAxis::parse(bad).expect_err(bad);
            match err {
                CliError::Usage(message) => assert!(message.contains("z, y or auto"), "{message}"),
                CliError::Run(message) => panic!("expected usage, got {message}"),
            }
        }
    }

    #[test]
    fn y_up_note_names_the_exporter() {
        let note = y_up_note(Some("SolidWorks"));
        assert_eq!(note, "note: STEP exported by SolidWorks (Y up); turned Y up to Z up. Use --up-axis z to keep the file's axes.");
        assert_eq!(y_up_note(None), "note: STEP exported by unknown (Y up); turned Y up to Z up. Use --up-axis z to keep the file's axes.");
        assert!(y_up_note(Some("has\nnewline")).contains("unknown"));
    }

    #[test]
    fn probe_reply_tolerates_unknown_fields_and_uses_the_first_object() {
        let text = r#"[{"bodies":["plate"],"component":1,"exporter":"SolidWorks","exporter_up":"y","system":"SolidWorks 2024","warnings":[],"stand_up":2,"future":{"keep":true}},{"exporter_up":"z"}]"#;
        let reply = probe_reply(text);
        assert!(reply.y_up);
        assert_eq!(reply.exporter.as_deref(), Some("SolidWorks"));
    }

    #[test]
    fn probe_reply_keeps_z_for_odd_output() {
        let z_cases = [
            r#"[{"exporter":"SolidCraft","exporter_up":"z","system":"SolidCraft","new_field":1}]"#,
            r#"[{"bodies":["plate"],"warnings":[]}]"#,
            r#"[{}]"#,
            r#"[{"exporter_up":"Y"}]"#,
            r#"[{"exporter_up":1,"exporter":"SolidWorks"}]"#,
            r#"["nope",{"exporter_up":"y","exporter":"SolidWorks"}]"#,
            r#"[null]"#,
            r#"[]"#,
            r#"{"exporter":"SolidWorks","exporter_up":"y"}"#,
            "not json",
            "",
            "   ",
            "null",
            "[] trailing",
        ];
        for text in z_cases {
            let reply = probe_reply(text);
            assert!(!reply.y_up, "{text}");
        }
        let named = probe_reply(r#"[{"exporter_up":"y","exporter":1,"extra":true}]"#);
        assert!(named.y_up);
        assert_eq!(named.exporter, None);
    }

    #[cfg(unix)]
    fn write_cli(dir: &Path, body: &str) -> PathBuf {
        use std::os::unix::fs::PermissionsExt;

        let path = dir.join("solidcraft-cli");
        fs::write(&path, body).expect("write fake cli");
        let mut perms = fs::metadata(&path).expect("cli metadata").permissions();
        perms.set_mode(0o755);
        fs::set_permissions(&path, perms).expect("chmod");
        path
    }

    #[cfg(unix)]
    fn shell_quote(path: &Path) -> String {
        let text = path.to_str().expect("utf-8 path");
        assert!(!text.contains('\''), "scratch path contains a single quote");
        format!("'{text}'")
    }

    #[cfg(unix)]
    fn success_cli(sentinel: &Path) -> String {
        format!(
            "#!/bin/sh\nset -eu\nscript=$2\nprintf '%s\\n' \"$(dirname -- \"$script\")\" > {sentinel}\ndxf=$(sed -n 's/.*\"path\":\"\\([^\"]*\\)\".*/\\1/p' \"$script\" | head -n 1)\ntest -n \"$dxf\"\nprintf '%s\\n' '0' 'SECTION' '2' 'ENTITIES' '0' 'ENDSEC' '0' 'EOF' > \"$dxf\"\n",
            sentinel = shell_quote(sentinel),
        )
    }

    #[cfg(unix)]
    fn boom_cli(sentinel: &Path) -> String {
        format!(
            "#!/bin/sh\nset -eu\nscript=$2\nprintf '%s\\n' \"$(dirname -- \"$script\")\" > {sentinel}\nprintf '%s\\n' 'boom' >&2\nexit 3\n",
            sentinel = shell_quote(sentinel),
        )
    }

    #[cfg(unix)]
    fn assert_temp_removed(sentinel: &Path) {
        let recorded = fs::read_to_string(sentinel).expect("sentinel");
        let temp = PathBuf::from(recorded.trim());
        let name = temp.file_name().and_then(|item| item.to_str()).expect("temp name");
        assert!(name.starts_with(&format!("campanion-step-{}-", std::process::id())), "{name}");
        assert!(!temp.exists(), "temp dir was not removed: {}", temp.display());
    }

    #[test]
    #[cfg(unix)]
    fn face_to_dxf_fake_cli_writes_dxf_and_removes_temp() {
        let scratch = Scratch::new("dxf-ok");
        let sentinel = scratch.0.join("temp-path");
        let cli = write_cli(&scratch.0, &success_cli(&sentinel));
        let step = scratch.0.join("part.step");
        fs::write(&step, b"ISO-10303-21;\n").expect("step");

        let text = face_to_dxf(&cli, &step, [1.5, -2.0, 3.25], UpAxis::Z).unwrap_or_else(|err| panic!("{err}"));
        assert_eq!(text, DXF_TEXT);
        assert_temp_removed(&sentinel);
    }

    #[test]
    #[cfg(unix)]
    fn face_to_dxf_fake_cli_stderr_on_failure() {
        let scratch = Scratch::new("dxf-boom");
        let sentinel = scratch.0.join("temp-path");
        let cli = write_cli(&scratch.0, &boom_cli(&sentinel));
        let step = scratch.0.join("part.step");
        fs::write(&step, b"ISO-10303-21;\n").expect("step");

        let err = face_to_dxf(&cli, &step, [0.0, 0.0, 0.0], UpAxis::Z).expect_err("status");
        let message = err.to_string();
        assert!(message.contains("boom"), "{message}");
        assert!(message.contains("solidcraft-cli failed on"), "{message}");
        assert_temp_removed(&sentinel);
    }

    #[cfg(unix)]
    fn shell_single(text: &str) -> String {
        format!("'{}'", text.replace('\'', "'\\''"))
    }

    /// `probe_body` is written to `--json-out`. `probe_error` is the probe's stderr and a non-zero
    /// exit. Neither means a probe invocation is a failure, so an explicit axis must not probe.
    #[cfg(unix)]
    fn recording_cli(log: &Path, probe_body: Option<&str>, probe_error: Option<&str>) -> String {
        let on_probe = if let Some(body) = probe_body {
            format!("cat > \"$json_out\" << 'ENDJSON'\n{body}\nENDJSON\nexit 0")
        } else if let Some(message) = probe_error {
            format!("printf '%s\\n' {} >&2\nexit 4", shell_single(message))
        } else {
            "printf '%s\\n' 'probe was not expected' >&2\nexit 9".to_string()
        };
        let template = r#"#!/bin/sh
set -eu
log=__LOG__
{
  printf '%s\n' 'CALL'
  printf '%s\n' "$@"
  printf '%s\n' 'ENDARGS'
  printf '%s\n' 'SCRIPT'
  cat -- "$2"
  printf '\n%s\n' 'ENDSCRIPT'
} >> "$log"
json_out=
prev=
for arg in "$@"; do
  if [ "$prev" = "--json-out" ]; then
    json_out=$arg
  fi
  prev=$arg
done
if [ -n "$json_out" ]; then
__ON_PROBE__
fi
dxf=$(sed -n 's/.*"path":"\([^"]*\)".*/\1/p' "$2" | tail -n 1)
test -n "$dxf"
printf '%s\n' '0' 'SECTION' '2' 'ENTITIES' '0' 'ENDSEC' '0' 'EOF' > "$dxf"
"#;
        template.replace("__LOG__", &shell_quote(log)).replace("__ON_PROBE__", &on_probe)
    }

    #[cfg(unix)]
    fn parse_calls(text: &str) -> Vec<(Vec<String>, String)> {
        let lines: Vec<&str> = text.lines().collect();
        let mut calls = Vec::new();
        let mut index = 0usize;
        while index < lines.len() {
            assert_eq!(lines.get(index).copied(), Some("CALL"), "{text}");
            index += 1;
            let mut args = Vec::new();
            while let Some(line) = lines.get(index).copied() {
                if line == "ENDARGS" {
                    break;
                }
                args.push(line.to_string());
                index += 1;
            }
            assert_eq!(lines.get(index).copied(), Some("ENDARGS"), "{text}");
            index += 1;
            assert_eq!(lines.get(index).copied(), Some("SCRIPT"), "{text}");
            index += 1;
            let mut script = Vec::new();
            while let Some(line) = lines.get(index).copied() {
                if line == "ENDSCRIPT" {
                    break;
                }
                script.push(line);
                index += 1;
            }
            assert_eq!(lines.get(index).copied(), Some("ENDSCRIPT"), "{text}");
            index += 1;
            calls.push((args, script.join("\n")));
        }
        calls
    }

    #[cfg(unix)]
    fn run_recorded(
        name: &str,
        up: UpAxis,
        probe_body: Option<&str>,
        probe_error: Option<&str>,
    ) -> (Result<String, CliError>, Vec<(Vec<String>, String)>) {
        let scratch = Scratch::new(name);
        let log = scratch.0.join("calls.log");
        let cli = write_cli(&scratch.0, &recording_cli(&log, probe_body, probe_error));
        let step = scratch.0.join("part.step");
        fs::write(&step, b"ISO-10303-21;\n").expect("step");
        let result = face_to_dxf(&cli, &step, [10.0, -3.5, 1.25], up);
        let text = fs::read_to_string(&log).unwrap_or_default();
        let calls = parse_calls(&text);
        // The private temp directory is gone, whichever way the run ended.
        if let Some((args, _)) = calls.first() {
            let script = PathBuf::from(args.get(1).map(String::as_str).unwrap_or(""));
            if let Some(temp) = script.parent() {
                assert!(!temp.exists(), "temp dir was not removed: {}", temp.display());
            }
        }
        (result, calls)
    }

    #[cfg(unix)]
    fn assert_probe_call(args: &[String]) {
        assert_eq!(args.first().map(String::as_str), Some("run"), "{args:?}");
        assert_eq!(args.get(2).map(String::as_str), Some("--results"), "{args:?}");
        assert_eq!(args.get(3).map(String::as_str), Some("--json-out"), "{args:?}");
        let out = args.get(4).map(String::as_str).unwrap_or("");
        assert!(out.ends_with("/probe.json"), "{out}");
        assert_eq!(args.get(5).map(String::as_str), Some("--quiet"), "{args:?}");
        assert_eq!(args.len(), 6, "{args:?}");
        assert!(!args.iter().any(|arg| arg == "--in"), "{args:?}");
    }

    #[cfg(unix)]
    fn assert_probe_script(script: &str, step_parent_marker: &str) {
        let value: serde_json::Value = serde_json::from_str(script).unwrap_or_else(|err| panic!("{err}\n{script}"));
        let commands = value.as_array().expect("probe array");
        assert_eq!(commands.len(), 1, "{script}");
        let open = commands.first().expect("doc.open");
        assert_eq!(open.get("command").and_then(|item| item.as_str()), Some("doc.open"));
        let params = open.get("params").and_then(|item| item.as_object()).expect("params");
        assert!(params.get("up").is_none(), "{params:?}");
        let path = params.get("path").and_then(|item| item.as_str()).expect("path");
        assert!(path.starts_with('/'), "{path}");
        assert!(path.ends_with("/part.step"), "{path}");
        assert!(path.contains(step_parent_marker), "{path}");
    }

    #[cfg(unix)]
    fn assert_in_call(args: &[String]) {
        assert_eq!(args.len(), 5, "{args:?}");
        assert_eq!(args.first().map(String::as_str), Some("run"));
        let script = args.get(1).map(String::as_str).unwrap_or("");
        assert!(script.ends_with("/script.json"), "{script}");
        assert_eq!(args.get(2).map(String::as_str), Some("--in"));
        let step = args.get(3).map(String::as_str).unwrap_or("");
        assert!(step.ends_with("/part.step"), "{step}");
        assert_eq!(args.get(4).map(String::as_str), Some("--quiet"));
    }

    #[cfg(unix)]
    fn assert_turn_call(args: &[String]) {
        assert_eq!(args.len(), 3, "{args:?}");
        assert_eq!(args.first().map(String::as_str), Some("run"));
        let script = args.get(1).map(String::as_str).unwrap_or("");
        assert!(script.ends_with("/script.json"), "{script}");
        assert_eq!(args.get(2).map(String::as_str), Some("--quiet"));
    }

    #[cfg(unix)]
    fn assert_turn_script(script: &str) {
        let value: serde_json::Value = serde_json::from_str(script).unwrap_or_else(|err| panic!("{err}\n{script}"));
        let commands = value.as_array().expect("script array");
        assert_eq!(commands.len(), 3, "{script}");
        let open = commands.first().expect("doc.open");
        assert_eq!(open.get("command").and_then(|item| item.as_str()), Some("doc.open"));
        assert_eq!(open.pointer("/params/up").and_then(|item| item.as_str()), Some("y"));
        let path = open.pointer("/params/path").and_then(|item| item.as_str()).expect("path");
        assert!(path.starts_with('/'), "{path}");
        assert!(path.ends_with("/part.step"), "{path}");
        assert_eq!(commands.get(1).and_then(|item| item.get("command")).and_then(|item| item.as_str()), Some("sketch.create"));
        let face = commands.get(1).and_then(|item| item.pointer("/params/plane/face")).and_then(|item| item.as_array()).expect("face");
        let coords: Vec<f64> = face.iter().map(|item| item.as_f64().expect("coord")).collect();
        assert_eq!(coords, vec![10.0, -3.5, 1.25]);
        assert_eq!(commands.get(2).and_then(|item| item.get("command")).and_then(|item| item.as_str()), Some("sketch.export_dxf"));
    }

    #[cfg(unix)]
    fn assert_plain_script(script: &str) {
        let value: serde_json::Value = serde_json::from_str(script).unwrap_or_else(|err| panic!("{err}\n{script}"));
        let commands = value.as_array().expect("script array");
        assert_eq!(commands.len(), 2, "{script}");
        assert_eq!(commands.first().and_then(|item| item.get("command")).and_then(|item| item.as_str()), Some("sketch.create"));
        assert!(!script.contains("doc.open"), "{script}");
        assert!(!script.contains("\"up\""), "{script}");
    }

    #[cfg(unix)]
    fn same_temp(calls: &[(Vec<String>, String)]) {
        let probe = PathBuf::from(calls.first().and_then(|(args, _)| args.get(1)).map(String::as_str).unwrap_or(""));
        let main = PathBuf::from(calls.get(1).and_then(|(args, _)| args.get(1)).map(String::as_str).unwrap_or(""));
        assert_eq!(probe.parent(), main.parent());
    }

    #[test]
    #[cfg(unix)]
    fn face_to_dxf_auto_y_opens_with_up_and_skips_in() {
        let reply = r#"[{"bodies":["plate"],"exporter":"SolidWorks","exporter_up":"y","system":"SolidWorks 2024","warnings":[],"future":{"n":1}},{"ignored":true}]"#;
        let (result, calls) = run_recorded("auto-y", UpAxis::Auto, Some(reply), None);
        let text = result.unwrap_or_else(|err| panic!("{err}"));
        assert_eq!(text, DXF_TEXT);
        assert_eq!(calls.len(), 2, "{calls:?}");
        assert_probe_call(&calls[0].0);
        assert_probe_script(&calls[0].1, "auto-y");
        assert_turn_call(&calls[1].0);
        assert_turn_script(&calls[1].1);
        same_temp(&calls);
    }

    #[test]
    #[cfg(unix)]
    fn face_to_dxf_auto_non_y_uses_in() {
        let replies = [
            r#"[{"exporter":"SolidCraft","exporter_up":"z","system":"SolidCraft","new_field":true}]"#,
            r#"[{"bodies":["plate"],"warnings":[]}]"#,
            r#"["nope",{"exporter_up":"y"}]"#,
            "[]",
            "this is not json",
        ];
        for (index, reply) in replies.iter().enumerate() {
            let (result, calls) = run_recorded(&format!("auto-z-{index}"), UpAxis::Auto, Some(reply), None);
            let text = result.unwrap_or_else(|err| panic!("{reply}: {err}"));
            assert_eq!(text, DXF_TEXT, "{reply}");
            assert_eq!(calls.len(), 2, "{reply} {calls:?}");
            assert_probe_call(&calls[0].0);
            assert_probe_script(&calls[0].1, &format!("auto-z-{index}"));
            assert_in_call(&calls[1].0);
            assert_plain_script(&calls[1].1);
            same_temp(&calls);
        }
    }

    #[test]
    #[cfg(unix)]
    fn face_to_dxf_auto_missing_probe_file_uses_in() {
        let scratch = Scratch::new("auto-missing");
        let log = scratch.0.join("calls.log");
        let body = recording_cli(&log, None, None).replace("printf '%s\\n' 'probe was not expected' >&2\nexit 9", "exit 0");
        let cli = write_cli(&scratch.0, &body);
        let step = scratch.0.join("part.step");
        fs::write(&step, b"ISO-10303-21;\n").expect("step");
        let text = face_to_dxf(&cli, &step, [10.0, -3.5, 1.25], UpAxis::Auto).unwrap_or_else(|err| panic!("{err}"));
        assert_eq!(text, DXF_TEXT);
        let calls = parse_calls(&fs::read_to_string(&log).expect("log"));
        assert_eq!(calls.len(), 2);
        assert_in_call(&calls[1].0);
        assert_plain_script(&calls[1].1);
    }

    #[test]
    #[cfg(unix)]
    fn face_to_dxf_explicit_z_does_not_probe() {
        let (result, calls) = run_recorded("explicit-z", UpAxis::Z, None, None);
        let text = result.unwrap_or_else(|err| panic!("{err}"));
        assert_eq!(text, DXF_TEXT);
        assert_eq!(calls.len(), 1, "{calls:?}");
        assert_in_call(&calls[0].0);
        assert_plain_script(&calls[0].1);
        assert!(!calls[0].0.iter().any(|arg| arg == "--results"));
    }

    #[test]
    #[cfg(unix)]
    fn face_to_dxf_explicit_y_does_not_probe() {
        let (result, calls) = run_recorded("explicit-y", UpAxis::Y, None, None);
        let text = result.unwrap_or_else(|err| panic!("{err}"));
        assert_eq!(text, DXF_TEXT);
        assert_eq!(calls.len(), 1, "{calls:?}");
        assert_turn_call(&calls[0].0);
        assert_turn_script(&calls[0].1);
        assert!(!calls[0].0.iter().any(|arg| arg == "--results" || arg == "--in"));
    }

    #[test]
    #[cfg(unix)]
    fn face_to_dxf_probe_failure_includes_stderr() {
        let (result, calls) = run_recorded("probe-fail", UpAxis::Auto, None, Some("probe broke"));
        let err = result.expect_err("probe status");
        let message = err.to_string();
        assert!(message.contains("probe broke"), "{message}");
        assert!(message.contains("solidcraft-cli failed on"), "{message}");
        assert!(message.contains("part.step"), "{message}");
        assert_eq!(calls.len(), 1, "{calls:?}");
        assert_probe_call(&calls[0].0);
    }
}
