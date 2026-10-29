//! Text form of a [`VerifyReport`]. One issue is one line.

use campanion_ir::{Issue, ProcessKind, RefKind, VerifyReport};

/// A mesh gouge to print with the verifier issues.
#[derive(Debug)]
pub struct GougeNote {
    /// Operation or toolpath name.
    pub operation: String,
    /// Tool tip, millimetres.
    pub x: f64,
    /// Tool tip, millimetres.
    pub y: f64,
    /// Tool tip, millimetres.
    pub z: f64,
    /// Millimetres below the drop-cutter contact.
    pub depth: f64,
}

/// Summary, then one line per issue. The string ends with a newline.
pub fn format_report(report: &VerifyReport) -> String {
    format_report_and_gouges(report, &[])
}

/// [`format_report`], then one line per gouge. Gouges count as verify errors.
pub fn format_report_and_gouges(report: &VerifyReport, gouges: &[GougeNote]) -> String {
    let mut out = String::new();
    let issue_count = report.issues().len().saturating_add(gouges.len());
    let status = if issue_count == 0 { "ok" } else { "errors" };
    line(&mut out, &format!("status {status}"));
    line(&mut out, &format!("issues {issue_count}"));
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
    for gouge in gouges {
        line(&mut out, &format_gouge(gouge));
    }
    out
}

fn format_gouge(gouge: &GougeNote) -> String {
    let mut name = String::new();
    push_clean(&mut name, &gouge.operation);
    format!("gouge operation {name} x {} y {} z {} depth {}", num(gouge.x), num(gouge.y), num(gouge.z), num(gouge.depth))
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
        Issue::AxisStep { toolpath, move_index, angle_deg } => {
            format!("tool_axis_step toolpath {toolpath} move {move_index} angle_deg {}", num(*angle_deg))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use campanion_ir::{Document, Program, verify};

    #[test]
    fn gouge_lines_count_as_verify_errors() {
        let program = Program::try_new(None, Vec::new(), None, Vec::new(), Vec::new(), Vec::new(), None).expect("program");
        let doc = Document::new(program).expect("document");
        let report = verify(doc.program());
        assert_eq!(format_report(&report), format_report_and_gouges(&report, &[]));
        let notes = [GougeNote { operation: "dome".into(), x: 1.0, y: 2.0, z: 3.5, depth: 0.5 }];
        let text = format_report_and_gouges(&report, &notes);
        assert!(text.contains("status errors\n"), "{text}");
        assert!(text.contains("issues 1\n"), "{text}");
        assert!(text.contains("gouge operation dome x 1.000000 y 2.000000 z 3.500000 depth 0.500000\n"), "{text}");
    }

    #[test]
    fn axis_step_names_the_toolpath_the_move_and_the_angle() {
        use campanion_ir::{ClBlock, ClMotion, ClPath, CompensationMode, Event, Feeds, MoveRole, Pose, Vec3};

        let pose_at = |deg: f64| {
            let radians = deg.to_radians();
            Pose::new(Vec3::new(1.0, 2.0, 3.0).expect("tip"), Vec3::new(radians.sin(), 0.0, radians.cos()).expect("axis")).expect("pose")
        };
        let feed = |deg: f64| ClBlock::Motion(ClMotion::Linear { to: pose_at(deg), feed: 100.0, role: MoveRole::Cut });
        let rapid = |deg: f64| ClBlock::Motion(ClMotion::Rapid { to: pose_at(deg), feed: None });
        let make = |blocks: Vec<ClBlock>| {
            ClPath::try_new(
                None,
                None,
                CompensationMode::Off,
                Feeds::try_new(100.0, None, None, None, None).expect("feeds"),
                None,
                Vec::new(),
                blocks,
            )
            .expect("path")
        };
        let indexed = make(vec![rapid(0.0), feed(40.0)]);
        let continuous = make(vec![ClBlock::Event(Event::Comment { text: "note".into() }), rapid(0.0), feed(5.0 + 1e-4)]).with_continuous(true);
        let program = Program::try_new(None, Vec::new(), None, vec![indexed, continuous], Vec::new(), Vec::new(), None).expect("program");
        let report = verify(&program);
        let text = format_report(&report);
        assert!(text.contains("tool_axis_step toolpath 1 move 1 angle_deg "), "{text}");
        assert!(!text.contains("tool_axis_step toolpath 0"), "{text}");
    }
}
