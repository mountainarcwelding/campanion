//! `campanion supports`: write a supported, chamfered-raft STL. Also used by `campanion resin --supports`.

use std::ffi::OsString;
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use campanion_mesh::write_stl_binary;
use campanion_resin::{
    InputPoint, SupportParams, SupportParamsOverlay, SupportedMesh, generate_supports, parse_points_text, parse_support_params_json,
    render_supported_png,
};

use crate::error::CliError;
use crate::run::read_bytes;

const MAX_TEXT_BYTES: usize = 32 * 1024 * 1024;

#[derive(Default)]
pub(crate) struct SupportCli {
    pub enabled: bool,
    pub overlay: SupportParamsOverlay,
    pub support_params: Option<PathBuf>,
    pub points: Option<PathBuf>,
    pub points_only: bool,
    pub preview: Option<PathBuf>,
    pub supports_out: Option<PathBuf>,
}

pub(crate) fn value<'a>(args: &'a [OsString], index: &mut usize, flag: &str) -> Result<&'a str, CliError> {
    let Some(v) = args.get(*index) else {
        return Err(CliError::usage(format!("{flag} needs a value")));
    };
    *index += 1;
    v.to_str().filter(|s| !s.is_empty()).ok_or_else(|| CliError::usage(format!("{flag} needs a value")))
}

pub(crate) fn number(text: &str, flag: &str) -> Result<f64, CliError> {
    text.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| CliError::usage(format!("{flag}: `{text}` is not a number")))
}

/// Handle one support-related flag. Returns `true` when the flag was recognised.
pub(crate) fn apply_flag(flag: &str, args: &[OsString], index: &mut usize, a: &mut SupportCli) -> Result<bool, CliError> {
    let dup = |set: bool| if set { Err(CliError::usage(format!("{flag} is already set"))) } else { Ok(()) };
    match flag {
        "--supports" => {
            a.enabled = true;
            Ok(true)
        }
        "--points-only" => {
            a.points_only = true;
            a.enabled = true;
            Ok(true)
        }
        "--support-params" => {
            dup(a.support_params.is_some())?;
            a.support_params = Some(PathBuf::from(value(args, index, flag)?));
            a.enabled = true;
            Ok(true)
        }
        "--points" => {
            dup(a.points.is_some())?;
            a.points = Some(PathBuf::from(value(args, index, flag)?));
            a.enabled = true;
            Ok(true)
        }
        "--preview" => {
            dup(a.preview.is_some())?;
            a.preview = Some(PathBuf::from(value(args, index, flag)?));
            a.enabled = true;
            Ok(true)
        }
        "--supports-out" => {
            dup(a.supports_out.is_some())?;
            a.supports_out = Some(PathBuf::from(value(args, index, flag)?));
            a.enabled = true;
            Ok(true)
        }
        "--raft-chamfer-angle"
        | "--raft-thickness"
        | "--raft-margin"
        | "--raft-corner-relief"
        | "--overhang-angle"
        | "--support-spacing"
        | "--tip-diameter"
        | "--tip-length"
        | "--pillar-diameter"
        | "--foot-diameter"
        | "--foot-height"
        | "--part-lift" => {
            let v = number(value(args, index, flag)?, flag)?;
            let slot = slot(&mut a.overlay, flag);
            dup(slot.is_some())?;
            *slot = Some(v);
            a.enabled = true;
            Ok(true)
        }
        _ => Ok(false),
    }
}

fn slot<'a>(overlay: &'a mut SupportParamsOverlay, flag: &str) -> &'a mut Option<f64> {
    match flag {
        "--raft-chamfer-angle" => &mut overlay.raft_chamfer_angle_deg,
        "--raft-thickness" => &mut overlay.raft_thickness_mm,
        "--raft-margin" => &mut overlay.raft_margin_mm,
        "--raft-corner-relief" => &mut overlay.raft_corner_relief_mm,
        "--overhang-angle" => &mut overlay.overhang_angle_deg,
        "--support-spacing" => &mut overlay.support_spacing_mm,
        "--tip-diameter" => &mut overlay.tip_diameter_mm,
        "--tip-length" => &mut overlay.tip_length_mm,
        "--pillar-diameter" => &mut overlay.pillar_diameter_mm,
        "--foot-diameter" => &mut overlay.foot_diameter_mm,
        "--foot-height" => &mut overlay.foot_height_mm,
        _ => &mut overlay.part_lift_mm,
    }
}

pub(crate) fn resolve_params(a: &SupportCli) -> Result<SupportParams, CliError> {
    let file = match &a.support_params {
        Some(path) => Some(load_params_file(path)?),
        None => None,
    };
    let merged = file.unwrap_or_default().merge(&a.overlay);
    merged.complete().map_err(|err| {
        let text = err.to_string();
        if text.contains("no defaults are assumed") { CliError::usage(text) } else { CliError::run(text) }
    })
}

fn load_params_file(path: &Path) -> Result<SupportParamsOverlay, CliError> {
    let bytes = read_bytes(path)?;
    if bytes.len() > MAX_TEXT_BYTES {
        return Err(CliError::run("--support-params file is too large"));
    }
    let text = std::str::from_utf8(&bytes).map_err(|_| CliError::run("--support-params file is not UTF-8"))?;
    parse_support_params_json(text).map_err(|err| CliError::run(err.to_string()))
}

pub(crate) fn load_points(path: &Path) -> Result<Vec<InputPoint>, CliError> {
    let bytes = read_bytes(path)?;
    if bytes.len() > MAX_TEXT_BYTES {
        return Err(CliError::run("--points file is too large"));
    }
    let text = std::str::from_utf8(&bytes).map_err(|_| CliError::run("--points file is not UTF-8"))?;
    parse_points_text(text).map_err(|err| CliError::run(err.to_string()))
}

pub(crate) fn generate(model: &Path, a: &SupportCli) -> Result<(SupportedMesh, SupportParams), CliError> {
    let params = resolve_params(a)?;
    let auto = !a.points_only;
    let extra = match &a.points {
        Some(path) => load_points(path)?,
        None => Vec::new(),
    };
    if a.points_only && extra.is_empty() {
        return Err(CliError::usage("--points-only needs --points FILE"));
    }
    let is_stl = model.extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("stl"));
    if !is_stl {
        return Err(CliError::usage("supports reads .stl files"));
    }
    let bytes = read_bytes(model)?;
    let mesh = campanion_mesh::parse_stl(&bytes).map_err(|err| CliError::run(format!("STL: {err}")))?;
    let result = generate_supports(&mesh, &params, &extra, auto).map_err(|err| CliError::run(err.to_string()))?;
    Ok((result, params))
}

pub(crate) fn write_outputs(result: &SupportedMesh, params: &SupportParams, stl_out: Option<&Path>, preview: Option<&Path>) -> Result<(), CliError> {
    if let Some(path) = stl_out {
        let bytes = write_stl_binary(&result.mesh);
        std::fs::write(path, bytes)?;
    }
    if let Some(path) = preview {
        let png = render_supported_png(result, 1280, 960).map_err(|err| CliError::run(err.to_string()))?;
        std::fs::write(path, png)?;
    }
    let mut err = io::stderr().lock();
    if let Some(path) = stl_out {
        writeln!(
            err,
            "wrote {} with a {:.3} mm raft, {}° chamfer, {} supports, part lifted {:.3} mm",
            path.display(),
            params.raft_thickness_mm,
            params.raft_chamfer_angle_deg,
            result.points.len(),
            params.part_lift_mm
        )?;
    }
    if let Some(path) = preview {
        writeln!(err, "wrote preview {}", path.display())?;
    }
    Ok(())
}

#[derive(Default)]
struct Args {
    model: Option<PathBuf>,
    out: Option<PathBuf>,
    support: SupportCli,
}

fn parse(args: &[OsString]) -> Result<Args, CliError> {
    let mut a = Args::default();
    let mut i = 0usize;
    while let Some(arg) = args.get(i) {
        i += 1;
        let Some(flag) = arg.to_str() else {
            return Err(CliError::usage("arguments must be UTF-8"));
        };
        if flag == "-o" {
            if a.out.is_some() {
                return Err(CliError::usage("-o is already set"));
            }
            a.out = Some(PathBuf::from(value(args, &mut i, flag)?));
            continue;
        }
        if apply_flag(flag, args, &mut i, &mut a.support)? {
            continue;
        }
        if flag.starts_with('-') {
            return Err(CliError::usage(format!("unknown argument `{flag}`")));
        }
        if a.model.is_some() {
            return Err(CliError::usage("supports needs exactly one STL file"));
        }
        a.model = Some(PathBuf::from(flag));
    }
    a.support.enabled = true;
    Ok(a)
}

pub(crate) fn supports_command(args: &[OsString]) -> Result<ExitCode, CliError> {
    let a = parse(args)?;
    let Some(model) = a.model.clone() else {
        return Err(CliError::usage("supports needs an STL file"));
    };
    let Some(out) = a.out.clone() else {
        return Err(CliError::usage("supports needs -o OUT.stl"));
    };
    if out == model
        || a.support.support_params.as_ref() == Some(&out)
        || a.support.points.as_ref() == Some(&out)
        || a.support.preview.as_ref() == Some(&out)
    {
        return Err(CliError::usage("-o must differ from the input files and from --preview"));
    }
    if a.support.preview.as_ref() == Some(&model) {
        return Err(CliError::usage("--preview must differ from the input files"));
    }
    let (result, params) = generate(&model, &a.support)?;
    write_outputs(&result, &params, Some(&out), a.support.preview.as_deref())?;
    let mut err = io::stderr().lock();
    writeln!(err, "NOT printer-tested: these supports and the chamfered raft have not been printed or physically tested")?;
    Ok(ExitCode::SUCCESS)
}
