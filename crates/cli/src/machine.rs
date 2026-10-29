//! Limit profiles for three Haas lathes.
//!
//! The numbers are the standard configuration from the Haas spec page cited on each profile.
//! Optional equipment may change the machine's specifications (Haas).
//!
//! X and Z travel are an extent check of main-frame cutter-location tips only. The work offset
//! is not known, so a tip is not a position on the machine axis. Tip X is the radius stored on
//! the pose, not a diameter. Sub-spindle operations are in the sub frame and are excluded.
//!
//! Each profile records the main C-axis speed (60 rpm). That speed is not checked: a lathe job
//! does not program a C-axis rpm limit.

use serde_json::{Map, Value};

use crate::error::CliError;
use crate::job::{as_bool, as_str, check_keys, object, optional};

/// Secondary-spindle speeds from the Haas spec. `rpm` is the spindle cap. `sync_rpm` is the
/// synchronized speed used while the spindles are geared together.
#[derive(Clone, Copy)]
pub(crate) struct SubLimits {
    pub rpm: f64,
    pub sync_rpm: f64,
}

/// One cited machine. Lengths are millimetres. Speeds are revolutions per minute.
#[derive(Clone, Copy)]
pub(crate) struct MachineProfile {
    pub id: &'static str,
    /// Marketing name used in the option-compatibility error, for example `ST-20Y`.
    pub label: &'static str,
    pub x_travel: f64,
    pub y_min: f64,
    pub y_max: f64,
    pub z_travel: f64,
    pub main_rpm: f64,
    /// Main C-axis speed. Recorded from the spec and not checked.
    pub c_axis_rpm: f64,
    pub sub: Option<SubLimits>,
    /// True when the sub-spindle is standard equipment, so `machine.sub_spindle` defaults to true.
    pub sub_standard: bool,
    pub live_rpm: f64,
    /// True when a tailstock is an available option.
    pub tailstock_option: bool,
}

/// Haas ST-15Y.
///
/// Spec: <https://www.haascnc.com/machines/lathes/st/models/y-axis/st-15y.html>
///
/// The 6000 rpm live-tool upgrade is the ST Y-axis flyer:
/// <https://www.haascnc.com/content/dam/haascnc/assets/seriesflyers/21.07_st_yaxis_4_pager_39-20004b.pdf>
pub(crate) const HAAS_ST15Y: MachineProfile = MachineProfile {
    id: "haas_st15y",
    label: "ST-15Y",
    x_travel: 200.0,
    y_min: -51.0,
    y_max: 51.0,
    z_travel: 406.0,
    main_rpm: 4_000.0,
    c_axis_rpm: 60.0,
    sub: Some(SubLimits { rpm: 6_000.0, sync_rpm: 4_800.0 }),
    sub_standard: false,
    live_rpm: 6_000.0,
    tailstock_option: true,
};

/// Haas ST-20Y.
///
/// Spec: <https://www.haascnc.com/machines/lathes/st/models/y-axis/st-20y.html>
///
/// The 6000 rpm live-tool upgrade is the same ST Y-axis flyer:
/// <https://www.haascnc.com/content/dam/haascnc/assets/seriesflyers/21.07_st_yaxis_4_pager_39-20004b.pdf>
pub(crate) const HAAS_ST20Y: MachineProfile = MachineProfile {
    id: "haas_st20y",
    label: "ST-20Y",
    x_travel: 213.0,
    y_min: -51.0,
    y_max: 51.0,
    z_travel: 572.0,
    main_rpm: 4_000.0,
    c_axis_rpm: 60.0,
    sub: Some(SubLimits { rpm: 6_000.0, sync_rpm: 4_800.0 }),
    sub_standard: false,
    live_rpm: 6_000.0,
    tailstock_option: true,
};

/// Haas DS-30Y. The sub-spindle is standard. There is no tailstock: the B axis is the secondary spindle.
///
/// Spec: <https://www.haascnc.com/machines/lathes/dual-spindle/models/ds-30y.html>
pub(crate) const HAAS_DS30Y: MachineProfile = MachineProfile {
    id: "haas_ds30y",
    label: "DS-30Y",
    x_travel: 239.0,
    y_min: -51.0,
    y_max: 51.0,
    z_travel: 826.0,
    main_rpm: 4_500.0,
    c_axis_rpm: 60.0,
    sub: Some(SubLimits { rpm: 4_000.0, sync_rpm: 4_000.0 }),
    sub_standard: true,
    live_rpm: 6_000.0,
    tailstock_option: false,
};

const MODEL_LIST: &str = "machine.model must be haas_st15y, haas_st20y or haas_ds30y";

/// A job's `machine` block: the profile plus the options the program claims are fitted.
#[derive(Clone, Copy)]
pub(crate) struct MachineConfig {
    pub profile: &'static MachineProfile,
    pub live_tool_upgrade: bool,
    pub sub_spindle: bool,
    pub tailstock: bool,
}

impl MachineConfig {
    /// Live-tool rpm cap. `live_tool_upgrade` is accepted for older jobs and has no effect.
    pub(crate) fn live_limit(self) -> f64 {
        self.profile.live_rpm
    }
}

/// Speeds and head travel the job programs, compared with the profile when `machine` is present.
pub(crate) struct ProgrammedLimits<'a> {
    pub main_rpm: f64,
    /// `sub.max_rpm` when the job has a sub block and the machine is fitted with a sub-spindle.
    pub sub_rpm: Option<f64>,
    /// Transfer `rpm` values, in job order. Checked against the synchronized-spindle limit.
    pub sync: &'a [(&'a str, f64)],
    /// `(y_min, y_max)` when `head.y_axis` is true.
    pub head_y: Option<(f64, f64)>,
    /// `(tool id, max_rpm)` for live tools that set `max_rpm`.
    pub live_tools: &'a [(u32, f64)],
    /// `(operation name, rpm)` for live cycles.
    pub live_ops: &'a [(&'a str, f64)],
}

/// Range of main-frame tip positions. Empty until a tip is recorded.
///
/// This is an extent check only. The work offset is not known, so the range must fit in the
/// axis travel and is not placed at a machine coordinate.
#[derive(Clone, Copy)]
pub(crate) struct AxisExtent {
    x_min: f64,
    x_max: f64,
    z_min: f64,
    z_max: f64,
    saw: bool,
}

impl AxisExtent {
    pub(crate) const fn new() -> Self {
        Self { x_min: f64::INFINITY, x_max: f64::NEG_INFINITY, z_min: f64::INFINITY, z_max: f64::NEG_INFINITY, saw: false }
    }

    /// `x_radius` is the pose tip X. Turning stores a radius there, not a diameter.
    pub(crate) fn include_radius(&mut self, x_radius: f64, z: f64) {
        self.saw = true;
        self.x_min = self.x_min.min(x_radius);
        self.x_max = self.x_max.max(x_radius);
        self.z_min = self.z_min.min(z);
        self.z_max = self.z_max.max(z);
    }

    pub(crate) fn check(self, config: &MachineConfig) -> Result<(), CliError> {
        if !self.saw {
            return Ok(());
        }
        // Cited on the profile. A lathe job does not program a C-axis rpm, so it is not compared.
        let _c_axis_rpm = config.profile.c_axis_rpm;
        let x_extent = self.x_max - self.x_min;
        let z_extent = self.z_max - self.z_min;
        let profile = config.profile;
        if x_extent > profile.x_travel {
            return Err(fault(profile.id, format!("program X extent {x_extent:.3} exceeds the X travel {:.3}", profile.x_travel)));
        }
        if z_extent > profile.z_travel {
            return Err(fault(profile.id, format!("program Z extent {z_extent:.3} exceeds the Z travel {:.3}", profile.z_travel)));
        }
        Ok(())
    }
}

pub(crate) fn parse_machine(value: &Value) -> Result<MachineConfig, CliError> {
    let map = object(value, "machine")?;
    check_keys(map, &["model", "live_tool_upgrade", "sub_spindle", "tailstock"], "machine")?;
    let profile = profile_for(map)?;
    let live_tool_upgrade = flag(map, "live_tool_upgrade")?;
    let sub_spindle = match optional(map, "sub_spindle") {
        Some(value) => as_bool(value, "sub_spindle")?,
        None => profile.sub_standard,
    };
    let tailstock = flag(map, "tailstock")?;
    if tailstock && !profile.tailstock_option {
        return Err(no_tailstock(profile));
    }
    if sub_spindle && tailstock {
        return Err(incompatible_options(profile));
    }
    Ok(MachineConfig { profile, live_tool_upgrade, sub_spindle, tailstock })
}

pub(crate) fn no_tailstock(profile: &MachineProfile) -> CliError {
    fault(profile.id, format!("{} has no tailstock (the B axis is the secondary spindle)", profile.id))
}

pub(crate) fn check_programmed(config: &MachineConfig, limits: &ProgrammedLimits<'_>) -> Result<(), CliError> {
    let profile = config.profile;
    // The C-axis speed is part of the cited profile and is not a job limit.
    let _c_axis_rpm = profile.c_axis_rpm;
    if limits.main_rpm > profile.main_rpm {
        return Err(fault(profile.id, format!("main spindle max_rpm {} exceeds {}", limits.main_rpm, profile.main_rpm)));
    }
    if let (Some(actual), Some(sub)) = (limits.sub_rpm, profile.sub)
        && actual > sub.rpm
    {
        return Err(fault(profile.id, format!("sub spindle max_rpm {actual} exceeds {}", sub.rpm)));
    }
    if let Some(sub) = profile.sub {
        for (name, rpm) in limits.sync {
            if *rpm > sub.sync_rpm {
                return Err(fault(profile.id, format!("operation {name}: synchronized rpm {rpm} exceeds {}", sub.sync_rpm)));
            }
        }
    }
    if let Some((y_min, y_max)) = limits.head_y
        && (y_min < profile.y_min || y_max > profile.y_max)
    {
        return Err(fault(profile.id, format!("head Y travel [{y_min}, {y_max}] exceeds [{}, {}]", profile.y_min, profile.y_max)));
    }
    let live_limit = config.live_limit();
    for (id, rpm) in limits.live_tools {
        if *rpm > live_limit {
            return Err(fault(profile.id, format!("tool {id}: live tool max_rpm {rpm} exceeds {live_limit}")));
        }
    }
    for (name, rpm) in limits.live_ops {
        if *rpm > live_limit {
            return Err(fault(profile.id, format!("operation {name}: live tool rpm {rpm} exceeds {live_limit}")));
        }
    }
    Ok(())
}

fn profile_for(map: &Map<String, Value>) -> Result<&'static MachineProfile, CliError> {
    let Some(value) = optional(map, "model") else {
        return Err(CliError::invalid(MODEL_LIST));
    };
    let id = as_str(value, "machine.model")?;
    match id {
        "haas_st15y" => Ok(&HAAS_ST15Y),
        "haas_st20y" => Ok(&HAAS_ST20Y),
        "haas_ds30y" => Ok(&HAAS_DS30Y),
        _ => Err(CliError::invalid(MODEL_LIST)),
    }
}

fn flag(map: &Map<String, Value>, key: &str) -> Result<bool, CliError> {
    match optional(map, key) {
        Some(value) => as_bool(value, key),
        None => Ok(false),
    }
}

fn incompatible_options(profile: &MachineProfile) -> CliError {
    CliError::invalid(format!(
        "{}: the sub-spindle option is not compatible with the tailstock option (Haas {} spec page)",
        profile.id, profile.label
    ))
}

fn fault(model: &str, detail: impl std::fmt::Display) -> CliError {
    CliError::invalid(format!("machine {model}: {detail}"))
}

#[cfg(test)]
mod tests {
    use super::{AxisExtent, HAAS_DS30Y, HAAS_ST15Y, HAAS_ST20Y, MachineConfig, MachineProfile, ProgrammedLimits, check_programmed, parse_machine};

    fn fitted(profile: &'static MachineProfile, upgrade: bool, sub: bool, tailstock: bool) -> MachineConfig {
        MachineConfig { profile, live_tool_upgrade: upgrade, sub_spindle: sub, tailstock }
    }

    fn limits<'a>(
        main_rpm: f64,
        sub_rpm: Option<f64>,
        sync: &'a [(&'a str, f64)],
        head_y: Option<(f64, f64)>,
        live_tools: &'a [(u32, f64)],
        live_ops: &'a [(&'a str, f64)],
    ) -> ProgrammedLimits<'a> {
        ProgrammedLimits { main_rpm, sub_rpm, sync, head_y, live_tools, live_ops }
    }

    fn ok<T>(result: Result<T, crate::error::CliError>, what: &str) -> T {
        result.unwrap_or_else(|err| panic!("{what}: {err}"))
    }

    fn bad<T>(result: Result<T, crate::error::CliError>, what: &str) -> String {
        match result {
            Err(err) => err.to_string(),
            Ok(_) => panic!("{what} was accepted"),
        }
    }

    fn err(config: &MachineConfig, programmed: &ProgrammedLimits<'_>) -> String {
        bad(check_programmed(config, programmed), "limit")
    }

    #[test]
    fn c_axis_rpm_is_recorded_and_not_part_of_the_check() {
        for profile in [&HAAS_ST15Y, &HAAS_ST20Y, &HAAS_DS30Y] {
            assert_eq!(profile.c_axis_rpm, 60.0, "{}", profile.id);
        }
        let config = fitted(&HAAS_ST20Y, false, false, false);
        ok(check_programmed(&config, &limits(4_000.0, None, &[], None, &[], &[])), "c axis is not checked");
    }

    #[test]
    fn main_spindle_passes_at_the_limit_and_fails_just_above() {
        let st = fitted(&HAAS_ST20Y, false, false, false);
        ok(check_programmed(&st, &limits(4_000.0, None, &[], None, &[], &[])), "4000");
        let over = 4_000.001;
        let message = err(&st, &limits(over, None, &[], None, &[], &[]));
        assert!(message.contains(&format!("machine haas_st20y: main spindle max_rpm {over} exceeds 4000")), "{message}");

        let ds = fitted(&HAAS_DS30Y, false, true, false);
        ok(check_programmed(&ds, &limits(4_500.0, None, &[], None, &[], &[])), "4500");
        let over_ds = 4_500.001;
        let message = err(&ds, &limits(over_ds, None, &[], None, &[], &[]));
        assert!(message.contains(&format!("machine haas_ds30y: main spindle max_rpm {over_ds} exceeds 4500")), "{message}");
    }

    #[test]
    fn sub_spindle_and_sync_pass_at_the_limit_and_fail_just_above() {
        let st = fitted(&HAAS_ST20Y, false, true, false);
        ok(check_programmed(&st, &limits(4_000.0, Some(6_000.0), &[("handoff", 4_800.0)], None, &[], &[])), "limits");
        let sub_over = 6_000.001;
        let message = err(&st, &limits(4_000.0, Some(sub_over), &[], None, &[], &[]));
        assert!(message.contains(&format!("machine haas_st20y: sub spindle max_rpm {sub_over} exceeds 6000")), "{message}");
        let sync_over = 4_800.001;
        let message = err(&st, &limits(4_000.0, Some(6_000.0), &[("handoff", sync_over)], None, &[], &[]));
        assert!(message.contains(&format!("machine haas_st20y: operation handoff: synchronized rpm {sync_over} exceeds 4800")), "{message}");

        let ds = fitted(&HAAS_DS30Y, false, true, false);
        ok(check_programmed(&ds, &limits(4_500.0, Some(4_000.0), &[("handoff", 4_000.0)], None, &[], &[])), "ds limits");
        let sync_over = 4_000.001;
        let message = err(&ds, &limits(4_500.0, Some(4_000.0), &[("handoff", sync_over)], None, &[], &[]));
        assert!(message.contains(&format!("machine haas_ds30y: operation handoff: synchronized rpm {sync_over} exceeds 4000")), "{message}");
    }

    #[test]
    fn head_y_passes_at_the_profile_and_fails_just_outside() {
        let st = fitted(&HAAS_ST15Y, false, false, false);
        ok(check_programmed(&st, &limits(4_000.0, None, &[], Some((-51.0, 51.0)), &[], &[])), "exact");
        ok(check_programmed(&st, &limits(4_000.0, None, &[], None, &[], &[])), "no y axis");
        let low = -51.001;
        let message = err(&st, &limits(4_000.0, None, &[], Some((low, 51.0)), &[], &[]));
        assert!(message.contains(&format!("machine haas_st15y: head Y travel [{low}, 51] exceeds [-51, 51]")), "{message}");
        let high = 51.001;
        let message = err(&st, &limits(4_000.0, None, &[], Some((-51.0, high)), &[], &[]));
        assert!(message.contains(&format!("machine haas_st15y: head Y travel [-51, {high}] exceeds [-51, 51]")), "{message}");
    }

    #[test]
    fn live_tool_cap_is_6000_on_every_model_and_the_upgrade_flag_changes_nothing() {
        for (profile, main) in [(&HAAS_ST15Y, 4_000.0), (&HAAS_ST20Y, 4_000.0), (&HAAS_DS30Y, 4_500.0)] {
            for upgrade in [false, true] {
                let config = fitted(profile, upgrade, profile.sub_standard, false);
                assert_eq!(config.live_limit(), 6_000.0);
                ok(check_programmed(&config, &limits(main, None, &[], None, &[(2, 6_000.0)], &[("holes", 6_000.0)])), "6000");
                let over = 6_000.001;
                let tool = err(&config, &limits(main, None, &[], None, &[(2, over)], &[]));
                assert!(tool.contains(&format!("machine {}: tool 2: live tool max_rpm {over} exceeds 6000", profile.id)), "{tool}");
                let op = err(&config, &limits(main, None, &[], None, &[], &[("holes", over)]));
                assert!(op.contains(&format!("machine {}: operation holes: live tool rpm {over} exceeds 6000", profile.id)), "{op}");
            }
        }
    }

    #[test]
    fn travel_extent_passes_at_the_limit_and_fails_just_above() {
        let st = fitted(&HAAS_ST15Y, false, false, false);
        let mut exact = AxisExtent::new();
        exact.include_radius(0.0, 0.0);
        exact.include_radius(200.0, -406.0);
        ok(exact.check(&st), "exact travel");

        let mut x_over = AxisExtent::new();
        x_over.include_radius(0.0, 0.0);
        x_over.include_radius(200.001, -1.0);
        let message = bad(x_over.check(&st), "x");
        assert!(message.contains("machine haas_st15y: program X extent 200.001 exceeds the X travel 200.000"), "{message}");

        let mut z_over = AxisExtent::new();
        z_over.include_radius(0.0, 0.0);
        z_over.include_radius(10.0, -406.001);
        let message = bad(z_over.check(&st), "z");
        assert!(message.contains("machine haas_st15y: program Z extent 406.001 exceeds the Z travel 406.000"), "{message}");

        let st20 = fitted(&HAAS_ST20Y, false, false, false);
        let mut st20_exact = AxisExtent::new();
        st20_exact.include_radius(-1.0, 2.0);
        st20_exact.include_radius(212.0, 2.0 - 572.0);
        ok(st20_exact.check(&st20), "st20 exact");

        let ds = fitted(&HAAS_DS30Y, false, true, false);
        let mut ds_exact = AxisExtent::new();
        ds_exact.include_radius(0.0, 0.0);
        ds_exact.include_radius(239.0, -826.0);
        ok(ds_exact.check(&ds), "ds exact");
        let mut ds_over = AxisExtent::new();
        ds_over.include_radius(0.0, 0.0);
        ds_over.include_radius(239.001, 0.0);
        let message = bad(ds_over.check(&ds), "ds x");
        assert!(message.contains("machine haas_ds30y: program X extent 239.001 exceeds the X travel 239.000"), "{message}");

        ok(AxisExtent::new().check(&st), "no motion");
    }

    #[test]
    fn options_default_and_reject_unknown_models_the_upgrade_and_a_ds_tailstock() {
        let st = ok(parse_machine(&serde_json::json!({"model": "haas_st15y"})), "st");
        assert!(!st.live_tool_upgrade && !st.sub_spindle && !st.tailstock);

        let ds = ok(parse_machine(&serde_json::json!({"model": "haas_ds30y"})), "ds");
        assert!(ds.sub_spindle && !ds.tailstock && !ds.live_tool_upgrade, "sub-spindle is standard");

        let upgraded = ok(parse_machine(&serde_json::json!({"model": "haas_st20y", "live_tool_upgrade": true})), "upgrade");
        assert!(upgraded.live_tool_upgrade);
        assert_eq!(upgraded.live_limit(), 6_000.0);

        let missing = bad(parse_machine(&serde_json::json!({})), "model");
        assert!(missing.contains("machine.model must be haas_st15y, haas_st20y or haas_ds30y"), "{missing}");
        let unknown = bad(parse_machine(&serde_json::json!({"model": "haas_st10"})), "model");
        assert!(unknown.contains("machine.model must be haas_st15y, haas_st20y or haas_ds30y"), "{unknown}");
        let extra = bad(parse_machine(&serde_json::json!({"model": "haas_st15y", "serial": 1})), "field");
        assert!(extra.contains("unknown machine field `serial`"), "{extra}");

        let upgrade = parse_machine(&serde_json::json!({"model": "haas_ds30y", "live_tool_upgrade": true})).unwrap_or_else(|_| panic!("upgrade flag is accepted"));
        assert_eq!(upgrade.live_limit(), 6_000.0);

        let tail = bad(parse_machine(&serde_json::json!({"model": "haas_ds30y", "tailstock": true})), "tail");
        assert!(tail.contains("machine haas_ds30y: haas_ds30y has no tailstock (the B axis is the secondary spindle)"), "{tail}");

        let both = bad(parse_machine(&serde_json::json!({"model": "haas_st20y", "sub_spindle": true, "tailstock": true})), "both");
        assert!(both.contains("haas_st20y: the sub-spindle option is not compatible with the tailstock option (Haas ST-20Y spec page)"), "{both}");
        let both15 = bad(parse_machine(&serde_json::json!({"model": "haas_st15y", "sub_spindle": true, "tailstock": true})), "both15");
        assert!(
            both15.contains("haas_st15y: the sub-spindle option is not compatible with the tailstock option (Haas ST-15Y spec page)"),
            "{both15}"
        );

        let tail_only = ok(parse_machine(&serde_json::json!({"model": "haas_st15y", "tailstock": true})), "tailstock option");
        assert!(tail_only.tailstock && !tail_only.sub_spindle);
        let sub_only = ok(parse_machine(&serde_json::json!({"model": "haas_st20y", "sub_spindle": true})), "sub option");
        assert!(sub_only.sub_spindle && !sub_only.tailstock);
    }
}
