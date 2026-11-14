//! Posted example programs match the golden files and read back onto the IR.
//!
//! `sample`, `curves`, and `dome` are posted with every built-in that is not a lathe post.
//! `indexed` and `wrap` carry an A axis, so they are posted only with dialects whose `rotary.axes`
//! include that axis. `turn` and `turn_thread` are turning programs and are posted with every lathe post.

use std::path::PathBuf;

use campanion_ir::{Document, Program};
use campanion_post::{PostDef, PostOptions, audit, builtin, builtin_ids, compare, post_program, read_back};

const EXAMPLES: [&str; 3] = ["sample", "curves", "dome"];
const ROTARY_EXAMPLES: [&str; 2] = ["indexed", "wrap"];
const LATHE_EXAMPLES: [&str; 2] = ["turn", "turn_thread"];

fn manifest_path(parts: &[&str]) -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    for part in parts {
        path.push(part);
    }
    path
}

fn load_example(name: &str) -> Result<Document, String> {
    let path = manifest_path(&["tests", "data", &format!("{name}.ir.json")]);
    let text = std::fs::read_to_string(&path).map_err(|err| format!("{}: {err}", path.display()))?;
    Document::from_json(&text).map_err(|err| format!("{name}: {err}"))
}

fn posts_for(program: &Program, rotary_only: bool) -> Result<Vec<&'static str>, String> {
    let mut ids = Vec::new();
    for id in builtin_ids() {
        let post = builtin(id).ok_or_else(|| format!("missing {id}"))?;
        // waterjet and oxyfuel can post 3-axis jobs and optional continuous five-axis.
        // Keep the existing 3-axis goldens in this loop; linuxcnc_5ax stays skipped.
        if post.is_lathe() || (post.has_five_axis() && *id != "waterjet" && *id != "oxyfuel") {
            continue;
        }
        let include = if rotary_only {
            // Waterjet and oxy-fuel list A/B for tilting kerf heads, not mill wrap or 3+1 examples.
            if *id == "waterjet" || *id == "oxyfuel" {
                false
            } else {
                match program.rotary() {
                    Some(setup) => post.rotary_axes().contains(&setup.axis()),
                    None => false,
                }
            }
        } else {
            true
        };
        if include {
            ids.push(*id);
        }
    }
    if ids.is_empty() {
        return Err("no post supports this rotary program".into());
    }
    Ok(ids)
}

fn lathe_posts() -> Result<Vec<&'static str>, String> {
    let mut ids = Vec::new();
    for id in builtin_ids() {
        let post = builtin(id).ok_or_else(|| format!("missing {id}"))?;
        if post.is_lathe() {
            ids.push(*id);
        }
    }
    if ids.is_empty() {
        return Err("no lathe post".into());
    }
    Ok(ids)
}

fn text_has_s_above_zero(text: &str) -> bool {
    for line in text.lines() {
        let body = line.split('(').next().unwrap_or(line);
        for word in body.split_whitespace() {
            if let Some(rest) = word.strip_prefix('S')
                && let Ok(value) = rest.parse::<f64>()
                && value > 0.0
            {
                return true;
            }
        }
    }
    false
}

fn program_sets_power(program: &Program) -> bool {
    program.toolpaths().iter().any(|path| {
        path.blocks().iter().any(|block| match block {
            campanion_ir::ClBlock::Event(campanion_ir::Event::ProcessOn { params, .. }) => params.power_value().is_some(),
            campanion_ir::ClBlock::Motion(motion) => motion.power().is_some(),
            _ => false,
        })
    })
}

fn check_example(example: &str, document: &Document, ids: &[&str], update: bool, options: &PostOptions) -> Result<(), String> {
    let program = document.program();
    for id in ids {
        let post: PostDef = builtin(id).ok_or_else(|| format!("missing {id}"))?;
        if post.power_scale().is_some() && !program_sets_power(program) {
            let text = post_program(program, &post, options).map_err(|err| format!("{example} {id}: {err}"))?;
            if !text.contains("power was not set, using S0") {
                return Err(format!("{example} {id}: missing S0 default comment\n{text}"));
            }
            if text_has_s_above_zero(&text) {
                return Err(format!("{example} {id}: defaulted power wrote S above 0\n{text}"));
            }
            audit(&text, &post).map_err(|err| format!("{example} {id} audit: {err}"))?;
            continue;
        }
        let text = post_program(program, &post, options).map_err(|err| format!("{example} {id}: {err}"))?;
        if let Some(limit) = post.max_line_length() {
            for line in text.lines() {
                assert!(line.chars().count() <= limit, "{example} {id} exceeds max_line_length: {line}");
            }
        }
        let golden_path = manifest_path(&["tests", "golden", &format!("{example}.{id}.nc")]);
        if update {
            if let Some(parent) = golden_path.parent() {
                std::fs::create_dir_all(parent).map_err(|err| format!("{}: {err}", parent.display()))?;
            }
            std::fs::write(&golden_path, &text).map_err(|err| format!("{}: {err}", golden_path.display()))?;
        } else {
            let golden = std::fs::read_to_string(&golden_path).map_err(|err| format!("{}: {err}", golden_path.display()))?;
            assert_eq!(text, golden, "{example} {id}");
        }
        let moves = read_back(&text, &post).map_err(|err| format!("{example} {id} read: {err}"))?;
        compare(program, &moves, 1e-3, &post).map_err(|err| format!("{example} {id} compare: {err}"))?;
    }
    Ok(())
}

#[test]
fn examples_match_goldens_and_read_back() -> Result<(), String> {
    let update = std::env::var("UPDATE_GOLDEN").ok().as_deref() == Some("1");
    let options = PostOptions { arc_tolerance: 0.01 };
    for example in EXAMPLES {
        let document = load_example(example)?;
        let ids = posts_for(document.program(), false)?;
        check_example(example, &document, &ids, update, &options)?;
    }
    for example in ROTARY_EXAMPLES {
        let document = load_example(example)?;
        let ids = posts_for(document.program(), true)?;
        check_example(example, &document, &ids, update, &options)?;
    }
    for example in LATHE_EXAMPLES {
        let document = load_example(example)?;
        let ids = lathe_posts()?;
        check_example(example, &document, &ids, update, &options)?;
    }
    Ok(())
}
