//! Text form of a [`VerifyReport`]. One issue is one line.

use campanion_ir::{Issue, ProcessKind, RefKind, VerifyReport};

/// Summary, then one line per issue. The string ends with a newline.
pub fn format_report(report: &VerifyReport) -> String {
    let mut out = String::new();
    let status = if report.issues().is_empty() { "ok" } else { "errors" };
    line(&mut out, &format!("status {status}"));
    line(&mut out, &format!("issues {}", report.issues().len()));
    line(&mut out, &format!("length_mm {}", num(report.total_length())));
    line(&mut out, &format!("rapid_mm {}", num(report.rapid_length())));
    line(&mut out, &format!("time_s {}", num(report.time_s())));
    match report.bounds() {
        Some(bounds) => {
            let min = bounds.min();
            let max = bounds.max();
            line(&mut out, &format!("bounds_min_mm {} {} {}", num(min.x()), num(min.y()), num(min.z())));
            line(&mut out, &format!("bounds_max_mm {} {} {}", num(max.x()), num(max.y()), num(max.z())));
        }
        None => line(&mut out, "bounds none"),
    }
    for issue in report.issues() {
        line(&mut out, &format_issue(issue));
    }
    out
}

fn line(out: &mut String, text: &str) {
    out.push_str(text);
    out.push('\n');
}

fn num(value: f64) -> String {
    if value.is_finite() { format!("{value:.6}") } else { "non_finite".to_string() }
}

fn opt_id(value: Option<impl std::fmt::Display>) -> String {
    match value {
        Some(value) => value.to_string(),
        None => "-".to_string(),
    }
}

fn push_clean(out: &mut String, text: &str) {
    for ch in text.chars() {
        if ch.is_control() { out.push(' ') } else { out.push(ch) }
    }
}

fn process_name(process: &ProcessKind) -> String {
    match process {
        ProcessKind::Spindle => "spindle".to_string(),
        ProcessKind::Torch => "torch".to_string(),
        ProcessKind::Laser => "laser".to_string(),
        ProcessKind::Extruder => "extruder".to_string(),
        ProcessKind::Knife => "knife".to_string(),
        ProcessKind::Other(name) => {
            let mut out = String::from("other:");
            push_clean(&mut out, name);
            out
        }
    }
}

fn format_issue(issue: &Issue) -> String {
    match issue {
        Issue::AxisLimit { channel, axis, value, min, max } => {
            format!("axis_limit channel {} axis {axis} value {} min {} max {}", opt_id(*channel), num(*value), num(*min), num(*max))
        }
        Issue::Discontinuity { toolpath, block, gap, tolerance } => {
            format!("discontinuity toolpath {toolpath} block {block} gap_mm {} tolerance_mm {}", num(*gap), num(*tolerance))
        }
        Issue::ProcessNotClosed { toolpath, channel, process } => {
            format!("process_not_closed toolpath {} channel {} {}", opt_id(*toolpath), opt_id(*channel), process_name(process))
        }
        Issue::NonFinite { what } => {
            let mut out = String::from("non_finite ");
            push_clean(&mut out, what);
            out
        }
        Issue::BadFeed { toolpath, channel, block, feed } => {
            format!("bad_feed toolpath {} channel {} block {block} feed {}", opt_id(*toolpath), opt_id(*channel), num(*feed))
        }
        Issue::UnknownRef(RefKind::Axis(id)) => format!("unknown_ref axis {id}"),
        Issue::UnknownRef(RefKind::Channel(id)) => format!("unknown_ref channel {id}"),
    }
}
