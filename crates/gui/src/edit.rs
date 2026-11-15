//! Copy, paste, duplicate, delete and undo helpers for a [`JobSession`].

use crate::session::{JobSession, OpDraft, ToolDraft, TreeSelection, next_operation_name, next_tool_id};

/// In-memory clipboard for tools and operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SessionClipboard {
    /// A tool table row.
    Tool(ToolDraft),
    /// An operation.
    Operation(Box<OpDraft>),
}

/// Snapshot used by Ctrl+Z. The undo stack lives on the window, not on the session.
#[derive(Debug, Clone)]
pub struct SessionSnapshot {
    /// Session at the time of the edit.
    pub session: JobSession,
}

/// Copy the current tree selection into the clipboard.
pub fn copy_selection(session: &JobSession) -> Result<SessionClipboard, String> {
    match session.selected {
        Some(TreeSelection::Tool(index)) => {
            let tool = session.tools.get(index).ok_or_else(|| "no tool is selected".to_string())?;
            Ok(SessionClipboard::Tool(tool.clone()))
        }
        Some(TreeSelection::Operation(index)) => {
            let op = session.operations.get(index).ok_or_else(|| "no operation is selected".to_string())?;
            Ok(SessionClipboard::Operation(Box::new(op.clone())))
        }
        Some(TreeSelection::Source | TreeSelection::Layer(_)) => {
            Err("the imported part cannot be copied; copy a tool or operation instead".to_string())
        }
        None => Err("select a tool or operation to copy".to_string()),
    }
}

/// Paste `clipboard` into `session`, giving a unique name or tool id.
pub fn paste_clipboard(session: &mut JobSession, clipboard: &SessionClipboard) -> Result<String, String> {
    match clipboard {
        SessionClipboard::Tool(tool) => {
            let mut pasted = tool.clone();
            pasted.id = next_tool_id(&session.tools);
            if !pasted.name.is_empty() {
                pasted.name = unique_name(&pasted.name, &session.tools.iter().map(|item| item.name.as_str()).collect::<Vec<_>>());
            }
            session.tools.push(pasted);
            session.selected = Some(TreeSelection::Tool(session.tools.len() - 1));
            session.dirty = true;
            Ok("Pasted tool".to_string())
        }
        SessionClipboard::Operation(op) => {
            let mut pasted = (**op).clone();
            pasted.name = next_operation_name(&session.operations, &op.name);
            session.operations.push(pasted);
            session.selected = Some(TreeSelection::Operation(session.operations.len() - 1));
            session.dirty = true;
            Ok("Pasted operation".to_string())
        }
    }
}

/// Duplicate the current selection (copy + paste).
pub fn duplicate_selection(session: &mut JobSession) -> Result<String, String> {
    let clipboard = copy_selection(session)?;
    paste_clipboard(session, &clipboard)
}

/// Delete the current selection.
///
/// A tool still referenced by an operation is refused.
pub fn delete_selection(session: &mut JobSession) -> Result<String, String> {
    match session.selected {
        Some(TreeSelection::Tool(index)) => delete_tool(session, index),
        Some(TreeSelection::Operation(index)) => delete_operation(session, index),
        Some(TreeSelection::Source) => delete_source(session),
        Some(TreeSelection::Layer(ref name)) => {
            Err(format!("source layer `{name}` cannot be deleted from the part; remove the imported part instead"))
        }
        None => Err("select a tool, operation, or the imported part to delete".to_string()),
    }
}

fn delete_source(session: &mut JobSession) -> Result<String, String> {
    if session.source_path.is_none() && session.source_drawing.is_none() {
        return Err("no part is imported".to_string());
    }
    session.source_path = None;
    session.source_hash = None;
    session.source_kind = None;
    session.source_drawing = None;
    session.layers.clear();
    session.placement_x.clear();
    session.placement_y.clear();
    session.placement_rotate.clear();
    session.selected = None;
    session.dirty = true;
    Ok("Removed the imported part".to_string())
}

fn delete_tool(session: &mut JobSession, index: usize) -> Result<String, String> {
    let tool = session.tools.get(index).ok_or_else(|| "no tool is selected".to_string())?;
    let id = tool.id;
    if let Some(op) = session.operations.iter().find(|op| op.tool_id == id) {
        return Err(format!("Tool {id} is used by operation `{}`. Delete or reassign that operation first.", op.name));
    }
    let label = if tool.name.is_empty() { format!("tool {id}") } else { format!("tool {}", tool.name) };
    session.tools.remove(index);
    session.selected = None;
    session.dirty = true;
    Ok(format!("Deleted {label}"))
}

fn delete_operation(session: &mut JobSession, index: usize) -> Result<String, String> {
    let op = session.operations.get(index).ok_or_else(|| "no operation is selected".to_string())?;
    let name = op.name.clone();
    session.operations.remove(index);
    session.selected = None;
    session.dirty = true;
    Ok(format!("Deleted operation `{name}`"))
}

/// True when Delete can act on the current selection (a refusal still counts as enabled so the
/// user can see the reason).
#[must_use]
pub fn can_edit_selection(session: &JobSession) -> bool {
    matches!(session.selected, Some(TreeSelection::Source | TreeSelection::Tool(_) | TreeSelection::Operation(_)))
}

fn unique_name(base: &str, taken: &[&str]) -> String {
    if !taken.contains(&base) {
        return base.to_string();
    }
    for index in 2..10_000 {
        let candidate = format!("{base} {index}");
        if !taken.iter().any(|name| *name == candidate) {
            return candidate;
        }
    }
    format!("{base} copy")
}

#[cfg(test)]
mod tests {
    use super::{copy_selection, delete_selection, duplicate_selection, paste_clipboard};
    use crate::session::{JobSession, OpDraft, ToolDraft, ToolKind, TreeSelection};

    fn session_with_tool_and_op() -> JobSession {
        let mut session = JobSession::new();
        session.tools.push(ToolDraft { id: 1, name: "kerf".to_string(), kind: ToolKind::Kerf, size: "0.1".to_string() });
        session.operations.push(OpDraft::new_profile("outline", 1));
        session.selected = Some(TreeSelection::Operation(0));
        session
    }

    #[test]
    fn cannot_delete_a_tool_used_by_an_operation() {
        let mut session = session_with_tool_and_op();
        session.selected = Some(TreeSelection::Tool(0));
        let error = delete_selection(&mut session).expect_err("in use");
        assert!(error.contains("used by operation `outline`"), "{error}");
        assert_eq!(session.tools.len(), 1);
    }

    #[test]
    fn delete_operation_then_unused_tool() {
        let mut session = session_with_tool_and_op();
        session.selected = Some(TreeSelection::Operation(0));
        let status = delete_selection(&mut session).expect("op");
        assert!(status.contains("outline"), "{status}");
        assert!(session.operations.is_empty());
        session.selected = Some(TreeSelection::Tool(0));
        delete_selection(&mut session).expect("tool");
        assert!(session.tools.is_empty());
    }

    #[test]
    fn copy_paste_and_duplicate_operations_get_new_names() {
        let mut session = session_with_tool_and_op();
        session.selected = Some(TreeSelection::Operation(0));
        let clip = copy_selection(&session).expect("copy");
        paste_clipboard(&mut session, &clip).expect("paste");
        assert_eq!(session.operations.len(), 2);
        assert_ne!(session.operations[0].name, session.operations[1].name);
        duplicate_selection(&mut session).expect("dup");
        assert_eq!(session.operations.len(), 3);
        let names: Vec<_> = session.operations.iter().map(|op| op.name.as_str()).collect();
        assert_eq!(names.len(), 3);
        assert_eq!(names[0], "outline");
        assert!(names[1].starts_with("outline"));
        assert!(names[2].starts_with("outline"));
        assert_eq!(std::collections::BTreeSet::from_iter(names.iter()).len(), 3);
    }

    #[test]
    fn delete_imported_part_clears_source_and_keeps_tools() {
        let mut session = session_with_tool_and_op();
        session.source_path = Some(std::path::PathBuf::from("part.svg"));
        session.source_hash = Some("abc".to_string());
        session.layers.push(campanion_cli::LayerSummary { name: "0".to_string(), contour_count: 1 });
        session.placement_x = "10".to_string();
        session.selected = Some(TreeSelection::Source);
        let status = delete_selection(&mut session).expect("delete part");
        assert!(status.contains("Removed"), "{status}");
        assert!(session.source_path.is_none());
        assert!(session.source_drawing.is_none());
        assert!(session.layers.is_empty());
        assert!(session.placement_x.is_empty());
        assert_eq!(session.tools.len(), 1);
        assert_eq!(session.operations.len(), 1);
    }
}
