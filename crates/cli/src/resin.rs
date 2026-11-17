//! `campanion resin` and `campanion resin-info`: Anycubic Photon Mono M7 Max `.pm7m` output.
//!
//! No exposure, lift or layer value has a default. Each one comes from a flag or from
//! `--params-from` (an existing `.pm7m`, such as a slicer export that is known to print).
//! Pass `--supports` (and the support flags, or `--support-params`) to generate a chamfered
//! raft and pillar supports before slicing.

use std::ffi::OsString;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use campanion_resin::pm7m::{LayerParams, M7_MAX, ResinSettings};
use campanion_resin::{Placement, build_pm7m, pw0, read_pm7m};

use crate::error::CliError;
use crate::run::read_bytes;
use crate::supports::{self, SupportCli};

const MAX_PM7M_BYTES: usize = 2 * 1024 * 1024 * 1024;

#[derive(Default)]
struct Args {
    model: Option<PathBuf>,
    out: Option<PathBuf>,
    params_from: Option<PathBuf>,
    keep_xy: bool,
    resin_name: Option<String>,
    layer_height: Option<f64>,
    bottom_layers: Option<u32>,
    bottom_exposure: Option<f64>,
    exposure: Option<f64>,
    bottom_lift_height: Option<f64>,
    bottom_lift_speed: Option<f64>,
    lift_height: Option<f64>,
    lift_speed: Option<f64>,
    support: SupportCli,
}

fn value<'a>(args: &'a [OsString], index: &mut usize, flag: &str) -> Result<&'a str, CliError> {
    let Some(v) = args.get(*index) else {
        return Err(CliError::usage(format!("{flag} needs a value")));
    };
    *index += 1;
    v.to_str().filter(|s| !s.is_empty()).ok_or_else(|| CliError::usage(format!("{flag} needs a value")))
}

fn number(text: &str, flag: &str) -> Result<f64, CliError> {
    text.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| CliError::usage(format!("{flag}: `{text}` is not a number")))
}

fn parse(args: &[OsString]) -> Result<Args, CliError> {
    let mut a = Args::default();
    let mut i = 0usize;
    while let Some(arg) = args.get(i) {
        i += 1;
        let Some(flag) = arg.to_str() else {
            return Err(CliError::usage("arguments must be UTF-8"));
        };
        let dup = |set: bool| if set { Err(CliError::usage(format!("{flag} is already set"))) } else { Ok(()) };
        match flag {
            "-o" => {
                dup(a.out.is_some())?;
                a.out = Some(PathBuf::from(value(args, &mut i, flag)?));
            }
            "--params-from" => {
                dup(a.params_from.is_some())?;
                a.params_from = Some(PathBuf::from(value(args, &mut i, flag)?));
            }
            "--keep-xy" => a.keep_xy = true,
            "--resin-name" => {
                dup(a.resin_name.is_some())?;
                a.resin_name = Some(value(args, &mut i, flag)?.to_string());
            }
            "--bottom-layers" => {
                dup(a.bottom_layers.is_some())?;
                let t = value(args, &mut i, flag)?;
                a.bottom_layers = Some(t.parse::<u32>().map_err(|_| CliError::usage(format!("{flag}: `{t}` is not a whole number")))?);
            }
            "--layer-height"
            | "--bottom-exposure"
            | "--exposure"
            | "--bottom-lift-height"
            | "--bottom-lift-speed"
            | "--lift-height"
            | "--lift-speed" => {
                let v = number(value(args, &mut i, flag)?, flag)?;
                let slot = match flag {
                    "--layer-height" => &mut a.layer_height,
                    "--bottom-exposure" => &mut a.bottom_exposure,
                    "--exposure" => &mut a.exposure,
                    "--bottom-lift-height" => &mut a.bottom_lift_height,
                    "--bottom-lift-speed" => &mut a.bottom_lift_speed,
                    "--lift-height" => &mut a.lift_height,
                    _ => &mut a.lift_speed,
                };
                dup(slot.is_some())?;
                *slot = Some(v);
            }
            other => {
                if crate::supports::apply_flag(other, args, &mut i, &mut a.support)? {
                    // support flag
                } else if other.starts_with('-') {
                    return Err(CliError::usage(format!("unknown argument `{other}`")));
                } else if a.model.is_some() {
                    return Err(CliError::usage("resin needs exactly one STL file"));
                } else {
                    a.model = Some(PathBuf::from(other));
                }
            }
        }
    }
    Ok(a)
}

fn load_sample(path: &Path) -> Result<ResinSettings, CliError> {
    let bytes = read_bytes(path)?;
    if bytes.len() > MAX_PM7M_BYTES {
        return Err(CliError::run("--params-from file is too large"));
    }
    let file = read_pm7m(&bytes).map_err(|err| CliError::run(format!("--params-from {}: {err}", path.display())))?;
    if file.machine_name != M7_MAX.name {
        return Err(CliError::run(format!("--params-from is for `{}`, not `{}`", file.machine_name, M7_MAX.name)));
    }
    file.settings().map_err(|err| CliError::run(format!("--params-from: {err}")))
}

fn settings(a: &Args) -> Result<ResinSettings, CliError> {
    let base = a.params_from.as_deref().map(load_sample).transpose()?;
    let mut missing: Vec<&str> = Vec::new();
    let mut pick = |flag: &'static str, given: Option<f64>, from: Option<f64>| -> f64 {
        match given.or(from) {
            Some(v) => v,
            None => {
                missing.push(flag);
                0.0
            }
        }
    };
    let b = base.as_ref();
    let layer_height_mm = pick("--layer-height", a.layer_height, b.map(|s| s.layer_height_mm));
    let bottom = LayerParams {
        exposure_s: pick("--bottom-exposure", a.bottom_exposure, b.map(|s| s.bottom.exposure_s)),
        lift_height_mm: pick("--bottom-lift-height", a.bottom_lift_height, b.map(|s| s.bottom.lift_height_mm)),
        lift_speed: pick("--bottom-lift-speed", a.bottom_lift_speed, b.map(|s| s.bottom.lift_speed)),
    };
    let normal = LayerParams {
        exposure_s: pick("--exposure", a.exposure, b.map(|s| s.normal.exposure_s)),
        lift_height_mm: pick("--lift-height", a.lift_height, b.map(|s| s.normal.lift_height_mm)),
        lift_speed: pick("--lift-speed", a.lift_speed, b.map(|s| s.normal.lift_speed)),
    };
    let bottom_layers = match a.bottom_layers.or(b.map(|s| s.bottom_layers)) {
        Some(v) => v,
        None => {
            missing.push("--bottom-layers");
            0
        }
    };
    if !missing.is_empty() {
        return Err(CliError::usage(format!("no defaults are assumed; give {} (or --params-from SAMPLE.pm7m)", missing.join(", "))));
    }
    let resin_name = a.resin_name.clone().or(b.map(|s| s.resin_name.clone())).unwrap_or_else(|| "campanion".to_string());
    let s = ResinSettings { layer_height_mm, bottom_layers, bottom, normal, resin_name };
    s.validate().map_err(|err| CliError::run(err.to_string()))?;
    Ok(s)
}

pub(crate) fn resin(args: &[OsString]) -> Result<ExitCode, CliError> {
    let a = parse(args)?;
    let Some(model) = a.model.clone() else {
        return Err(CliError::usage("resin needs an STL file"));
    };
    let Some(out) = a.out.clone() else {
        return Err(CliError::usage("resin needs -o OUT.pm7m"));
    };
    if out == model || a.params_from.as_ref() == Some(&out) {
        return Err(CliError::usage("-o must differ from the input files"));
    }
    let settings = settings(&a)?;
    let is_stl = model.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("stl"));
    if !is_stl {
        return Err(CliError::usage("resin reads .stl files"));
    }
    let bytes = read_bytes(&model)?;
    let mesh = campanion_mesh::parse_stl(&bytes).map_err(|err| CliError::run(format!("STL: {err}")))?;
    let (mesh, support_report) = if a.support.enabled {
        let (generated, params) = supports::generate(&model, &a.support)?;
        if a.support.supports_out.as_ref() == Some(&out) || a.support.preview.as_ref() == Some(&out) {
            return Err(CliError::usage("--supports-out and --preview must differ from -o"));
        }
        supports::write_outputs(&generated, &params, a.support.supports_out.as_deref(), a.support.preview.as_deref())?;
        (generated.mesh, Some(generated.points.len()))
    } else if a.support.points_only || a.support.support_params.is_some() || a.support.points.is_some() {
        return Err(CliError::usage("support flags need --supports"));
    } else {
        (mesh, None)
    };
    let placement = if a.keep_xy { Placement::Keep } else { Placement::Center };
    let (data, report) = build_pm7m(&mesh, &M7_MAX, &settings, placement).map_err(|err| CliError::run(err.to_string()))?;
    // Read the file back before it is written.
    let back = read_pm7m(&data).map_err(|err| CliError::run(format!("read-back failed: {err}")))?;
    if back.layers.len() != report.layers || back.machine_name != M7_MAX.name {
        return Err(CliError::run("read-back does not match what was built"));
    }
    std::fs::write(&out, &data)?;
    let mut err = io::stderr().lock();
    writeln!(
        err,
        "wrote {} for {}: {} layers of {} mm, {:.2} mm tall, {:.3} mL solid, {:.2} MB",
        out.display(),
        M7_MAX.name,
        report.layers,
        settings.layer_height_mm,
        report.height_mm,
        report.volume_ml,
        report.bytes as f64 / 1e6
    )?;
    if report.empty_layers > 0 {
        writeln!(err, "warning: {} layers have no exposed pixels (a gap in the model)", report.empty_layers)?;
    }
    if let Some(count) = support_report {
        writeln!(err, "sliced with a chamfered raft and {count} supports")?;
    } else {
        writeln!(err, "the model sits on the plate with no supports and no raft; pass --supports to add them")?;
    }
    writeln!(err, "NOT printer-tested: check the file on the printer screen and print a test before a real job")?;
    Ok(ExitCode::SUCCESS)
}

pub(crate) fn resin_info(args: &[OsString]) -> Result<ExitCode, CliError> {
    let [path] = args else {
        return Err(CliError::usage("resin-info needs one .pm7m file"));
    };
    let path = PathBuf::from(path);
    let bytes = read_bytes(&path)?;
    let file = read_pm7m(&bytes).map_err(|err| CliError::run(format!("{}: {err}", path.display())))?;
    let mut lit = 0u64;
    for image in &file.layer_images {
        let runs = pw0::decode_runs(image, u64::from(file.width) * u64::from(file.height)).map_err(|err| CliError::run(err.to_string()))?;
        lit += runs.iter().filter(|(v, _)| *v != 0).map(|(_, n)| u64::from(*n)).sum::<u64>();
    }
    let mut text = format!("machine: {}\nscreen: {} x {}\nlayers: {}\n", file.machine_name, file.width, file.height, file.layers.len());
    if let (Some(first), Some(last)) = (file.layers.first(), file.layers.last()) {
        text.push_str(&format!(
            "layer height: {} mm\nheight: {:.3} mm\nlayer 0: exposure {} s, lift {} at speed {}\nlast layer: exposure {} s, lift {} at speed {}\n",
            first.thickness,
            last.min_height + last.thickness,
            first.exposure_time,
            first.zup_height,
            first.zup_speed,
            last.exposure_time,
            last.zup_height,
            last.zup_speed
        ));
    }
    text.push_str(&format!("lit pixels (all layers): {lit}\nentries: {}\n", file.entry_names.len()));
    crate::write_stdout(&text)
}
