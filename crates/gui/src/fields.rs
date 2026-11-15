//! Which property fields are relevant to the active process and workspace.

use crate::session::ToolKind;
use crate::{SubProcess, Workspace};

/// Operation and tool fields the properties panel may show for one process.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProcessFields {
    /// Tool kinds the user may pick.
    pub tool_kinds: &'static [ToolKind],
    /// Depth of cut.
    pub depth: bool,
    /// Step down.
    pub step_down: bool,
    /// Cut feed.
    pub cut_feed: bool,
    /// Plunge feed (mills and routers).
    pub plunge_feed: bool,
    /// Process power fraction (laser).
    pub power: bool,
    /// Lead-in and lead-out.
    pub leads: bool,
    /// Climb / conventional / auto.
    pub direction: bool,
    /// Cutter compensation.
    pub compensation: bool,
    /// Pierce delay (plasma and oxy-fuel).
    pub pierce_delay: bool,
    /// Post-cut delay (plasma).
    pub post_delay: bool,
    /// Profile side.
    pub side: bool,
}

/// Fields that belong to `process`. When the process is unknown, `workspace` supplies a family default.
#[must_use]
pub fn fields_for(process: Option<SubProcess>, workspace: Workspace) -> ProcessFields {
    let process = process.or_else(|| workspace.sub_processes().first().copied());
    match process {
        Some(SubProcess::Laser) => ProcessFields {
            tool_kinds: &[ToolKind::Kerf],
            depth: true,
            step_down: true,
            cut_feed: true,
            plunge_feed: false,
            power: true,
            leads: true,
            direction: true,
            compensation: true,
            pierce_delay: false,
            post_delay: false,
            side: true,
        },
        Some(SubProcess::Plasma) => ProcessFields {
            tool_kinds: &[ToolKind::Kerf],
            depth: true,
            step_down: true,
            cut_feed: true,
            plunge_feed: false,
            power: false,
            leads: true,
            direction: true,
            compensation: true,
            pierce_delay: true,
            post_delay: true,
            side: true,
        },
        Some(SubProcess::Waterjet) => ProcessFields {
            tool_kinds: &[ToolKind::Kerf],
            depth: true,
            step_down: true,
            cut_feed: true,
            plunge_feed: false,
            power: false,
            leads: true,
            direction: true,
            compensation: true,
            pierce_delay: false,
            post_delay: false,
            side: true,
        },
        Some(SubProcess::OxyFuel) => ProcessFields {
            tool_kinds: &[ToolKind::Kerf],
            depth: true,
            step_down: true,
            cut_feed: true,
            plunge_feed: false,
            power: false,
            leads: true,
            direction: true,
            compensation: true,
            pierce_delay: true,
            post_delay: false,
            side: true,
        },
        Some(SubProcess::WireEdm) => ProcessFields {
            tool_kinds: &[ToolKind::Wire],
            depth: true,
            step_down: true,
            cut_feed: true,
            plunge_feed: false,
            power: false,
            leads: true,
            direction: true,
            compensation: true,
            pierce_delay: false,
            post_delay: false,
            side: true,
        },
        Some(SubProcess::Milling | SubProcess::Routing) | None => ProcessFields {
            tool_kinds: &[ToolKind::Diameter],
            depth: true,
            step_down: true,
            cut_feed: true,
            plunge_feed: true,
            power: false,
            leads: true,
            direction: true,
            compensation: true,
            pierce_delay: false,
            post_delay: false,
            side: true,
        },
        Some(SubProcess::Turning | SubProcess::Sla | SubProcess::Fdm) => ProcessFields {
            tool_kinds: &[ToolKind::Diameter],
            depth: false,
            step_down: false,
            cut_feed: true,
            plunge_feed: false,
            power: false,
            leads: false,
            direction: false,
            compensation: false,
            pierce_delay: false,
            post_delay: false,
            side: false,
        },
    }
}

/// True when `kind` is allowed for this process.
#[must_use]
pub fn tool_kind_allowed(fields: ProcessFields, kind: ToolKind) -> bool {
    fields.tool_kinds.contains(&kind)
}

#[cfg(test)]
mod tests {
    use super::{fields_for, tool_kind_allowed};
    use crate::session::ToolKind;
    use crate::{SubProcess, Workspace};

    #[test]
    fn laser_hides_wire_and_pierce() {
        let fields = fields_for(Some(SubProcess::Laser), Workspace::ThermalJet);
        assert_eq!(fields.tool_kinds, &[ToolKind::Kerf]);
        assert!(fields.power);
        assert!(!fields.pierce_delay);
        assert!(!fields.post_delay);
        assert!(!fields.plunge_feed);
        assert!(!tool_kind_allowed(fields, ToolKind::Wire));
        assert!(!tool_kind_allowed(fields, ToolKind::Diameter));
    }

    #[test]
    fn plasma_shows_pierce_not_power_or_wire() {
        let fields = fields_for(Some(SubProcess::Plasma), Workspace::ThermalJet);
        assert!(fields.pierce_delay);
        assert!(fields.post_delay);
        assert!(!fields.power);
        assert_eq!(fields.tool_kinds, &[ToolKind::Kerf]);
    }

    #[test]
    fn wire_edm_shows_wire_not_kerf_or_power() {
        let fields = fields_for(Some(SubProcess::WireEdm), Workspace::WireCutting);
        assert_eq!(fields.tool_kinds, &[ToolKind::Wire]);
        assert!(!fields.power);
        assert!(!fields.pierce_delay);
        assert!(!tool_kind_allowed(fields, ToolKind::Kerf));
    }

    #[test]
    fn mill_shows_plunge_and_diameter_only() {
        let fields = fields_for(Some(SubProcess::Milling), Workspace::MillRoute);
        assert_eq!(fields.tool_kinds, &[ToolKind::Diameter]);
        assert!(fields.plunge_feed);
        assert!(!fields.power);
        assert!(!fields.pierce_delay);
    }

    #[test]
    fn waterjet_and_oxyfuel_do_not_show_laser_power() {
        let water = fields_for(Some(SubProcess::Waterjet), Workspace::ThermalJet);
        let oxy = fields_for(Some(SubProcess::OxyFuel), Workspace::ThermalJet);
        assert!(!water.power && !water.pierce_delay);
        assert!(!oxy.power && oxy.pierce_delay);
        assert_eq!(water.tool_kinds, &[ToolKind::Kerf]);
        assert_eq!(oxy.tool_kinds, &[ToolKind::Kerf]);
    }
}
