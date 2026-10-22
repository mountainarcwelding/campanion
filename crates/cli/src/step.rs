//! Turn one planar face of a STEP part into DXF text.
//!
//! SolidCraft stays a separate program. This module runs its documented
//! `solidcraft-cli run` command and does not link or copy SolidCraft.

use std::fs::{self, File};
use std::io::{self, Read};
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
#[allow(dead_code)]
pub fn face_to_dxf(cli: &Path, step: &Path, face: [f64; 3]) -> Result<String, CliError> {
    let [x, y, z] = face;
    if ![x, y, z].iter().all(|coord| coord.is_finite()) {
        return Err(CliError::invalid("face point must be three finite millimetre coordinates"));
    }

    let temp = TempDir::create()?;
    let script = temp.path.join("script.json");
    let dxf = temp.path.join("face.dxf");
    write_script(&script, &dxf, x, y, z)?;
    run_cli(cli, &script, step, temp.path())?;
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

fn write_script(script: &Path, dxf: &Path, x: f64, y: f64, z: f64) -> Result<(), CliError> {
    let Some(dxf_path) = dxf.to_str() else {
        return Err(CliError::run(format!("temp path {} is not utf-8", dxf.display())));
    };
    let value = serde_json::json!([
        {
            "command": "sketch.create",
            "params": {
                "plane": { "face": [x, y, z] },
                "name": "campanion_face",
                "project_edges": true
            }
        },
        {
            "command": "sketch.export_dxf",
            "params": {
                "sketch": "campanion_face",
                "path": dxf_path
            }
        }
    ]);
    let text = serde_json::to_string(&value).map_err(|err| CliError::run(format!("script json: {err}")))?;
    fs::write(script, text).map_err(|err| CliError::run(format!("{}: {err}", script.display())))
}

struct Running {
    child: Child,
    /// Set once `try_wait` has reaped the child, so [`Drop`] does not kill a recycled pid.
    reaped: bool,
}

impl Running {
    fn spawn(cli: &Path, script: &Path, step: &Path, dir: &Path) -> Result<Self, CliError> {
        let stdout_path = dir.join("stdout.txt");
        let stderr_path = dir.join("stderr.txt");
        let stdout = File::create(&stdout_path).map_err(|err| CliError::run(format!("{}: {err}", stdout_path.display())))?;
        let stderr = File::create(&stderr_path).map_err(|err| CliError::run(format!("{}: {err}", stderr_path.display())))?;
        let child = Command::new(cli)
            .arg("run")
            .arg(script)
            .arg("--in")
            .arg(step)
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

fn run_cli(cli: &Path, script: &Path, step: &Path, dir: &Path) -> Result<(), CliError> {
    let mut running = Running::spawn(cli, script, step, dir)?;
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
        let err = face_to_dxf(Path::new("solidcraft-cli"), Path::new("part.step"), [f64::NAN, 0.0, 1.0]).expect_err("nan");
        assert!(err.to_string().contains("finite"), "{err}");
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

        let text = face_to_dxf(&cli, &step, [1.5, -2.0, 3.25]).unwrap_or_else(|err| panic!("{err}"));
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

        let err = face_to_dxf(&cli, &step, [0.0, 0.0, 0.0]).expect_err("status");
        let message = err.to_string();
        assert!(message.contains("boom"), "{message}");
        assert!(message.contains("solidcraft-cli failed on"), "{message}");
        assert_temp_removed(&sentinel);
    }
}
