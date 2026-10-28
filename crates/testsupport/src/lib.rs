//! Checks for IR JSON goldens.
//!
//! `f64` trigonometry follows the platform math library, so a toolpath written at full
//! precision can differ in the last bits. Parse both documents and compare them as JSON.
//! Numbers match when
//! `|actual - expected| <= 1e-9 + 1e-12 * max(|actual|, |expected|)`.
//! The absolute term is 1e-9 mm for linear coordinates and the same magnitude for every other
//! number. Strings, booleans, nulls, types, object keys and array lengths match exactly.
//! A mismatch names the JSON path and both values.

use serde_json::{Number, Value};

/// Absolute tolerance for an IR number. Linear coordinates are millimetres.
pub const ABS_TOL: f64 = 1e-9;

/// Relative tolerance for an IR number.
pub const REL_TOL: f64 = 1e-12;

/// Parse `actual` and `expected` as JSON and compare them with [`json_close`].
pub fn ir_json_close(actual: &str, expected: &str) -> Result<(), String> {
    let actual_value = parse_ir(actual, "actual")?;
    let expected_value = parse_ir(expected, "golden")?;
    json_close(&actual_value, &expected_value)
}

/// Compare JSON values. See the crate docs for the numeric tolerance.
pub fn json_close(actual: &Value, expected: &Value) -> Result<(), String> {
    compare(actual, expected, "$")
}

fn parse_ir(text: &str, label: &str) -> Result<Value, String> {
    serde_json::from_str(text).map_err(|err| format!("{label} IR is not JSON: {err}"))
}

fn compare(actual: &Value, expected: &Value, path: &str) -> Result<(), String> {
    match (actual, expected) {
        (Value::Number(actual_number), Value::Number(expected_number)) => {
            if numbers_close(actual_number, expected_number) {
                Ok(())
            } else {
                Err(mismatch(path, actual, expected))
            }
        }
        (Value::Array(actual_items), Value::Array(expected_items)) => {
            if actual_items.len() != expected_items.len() {
                return Err(mismatch(path, actual, expected));
            }
            for (index, (actual_item, expected_item)) in actual_items.iter().zip(expected_items.iter()).enumerate() {
                compare(actual_item, expected_item, &format!("{path}[{index}]"))?;
            }
            Ok(())
        }
        (Value::Object(actual_map), Value::Object(expected_map)) => {
            for (key, actual_value) in actual_map {
                if !expected_map.contains_key(key) {
                    return Err(mismatch_text(&join_key(path, key), &show(actual_value), "missing"));
                }
            }
            for (key, expected_value) in expected_map {
                match actual_map.get(key) {
                    Some(actual_value) => compare(actual_value, expected_value, &join_key(path, key))?,
                    None => return Err(mismatch_text(&join_key(path, key), "missing", &show(expected_value))),
                }
            }
            Ok(())
        }
        (Value::String(actual_text), Value::String(expected_text)) if actual_text == expected_text => Ok(()),
        (Value::Bool(actual_flag), Value::Bool(expected_flag)) if actual_flag == expected_flag => Ok(()),
        (Value::Null, Value::Null) => Ok(()),
        _ => Err(mismatch(path, actual, expected)),
    }
}

fn numbers_close(actual: &Number, expected: &Number) -> bool {
    if let (Some(actual_int), Some(expected_int)) = (actual.as_i64(), expected.as_i64()) {
        return actual_int == expected_int;
    }
    if let (Some(actual_int), Some(expected_int)) = (actual.as_u64(), expected.as_u64()) {
        return actual_int == expected_int;
    }
    match (actual.as_f64(), expected.as_f64()) {
        (Some(actual_float), Some(expected_float)) => floats_close(actual_float, expected_float),
        _ => false,
    }
}

fn floats_close(actual: f64, expected: f64) -> bool {
    if actual == expected {
        return true;
    }
    if !actual.is_finite() || !expected.is_finite() {
        return false;
    }
    let diff = (actual - expected).abs();
    let scale = actual.abs().max(expected.abs());
    diff <= ABS_TOL + REL_TOL * scale
}

fn mismatch(path: &str, actual: &Value, expected: &Value) -> String {
    mismatch_text(path, &show(actual), &show(expected))
}

fn mismatch_text(path: &str, actual: &str, expected: &str) -> String {
    format!("IR JSON mismatch at {path}: actual {actual}, expected {expected}")
}

fn show(value: &Value) -> String {
    let text = value.to_string();
    const MAX: usize = 180;
    if text.len() <= MAX {
        return text;
    }
    let mut end = MAX;
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    format!("{}...", &text[..end])
}

fn join_key(path: &str, key: &str) -> String {
    if is_plain_key(key) {
        format!("{path}.{key}")
    } else {
        let escaped = key.replace('\\', "\\\\").replace('"', "\\\"");
        format!("{path}[\"{escaped}\"]")
    }
}

fn is_plain_key(key: &str) -> bool {
    let mut chars = key.chars();
    match chars.next() {
        Some(first) if first.is_ascii_alphabetic() || first == '_' => chars.all(|ch| ch.is_ascii_alphanumeric() || ch == '_'),
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::ir_json_close;

    #[test]
    fn equal_within_tolerance_passes() {
        let actual = r#"{ "n": 1, "pose": { "z": 10.0000000004, "name": "finish" }, "ok": true, "items": [1, null] }"#;
        let expected = r#"{"ok":true,"items":[1,null],"pose":{"name":"finish","z":10.0},"n":1}"#;
        ir_json_close(actual, expected).expect("absolute tolerance");
        // 5e-5 is above the absolute tolerance and inside 1e-12 of 1e8.
        ir_json_close(r#"{"x":100000000.00005}"#, r#"{"x":100000000.0}"#).expect("relative tolerance");
    }

    #[test]
    fn difference_of_1e_6_fails_with_the_json_path() {
        let err = ir_json_close(r#"{"pose":{"z":10.0}}"#, r#"{"pose":{"z":10.000001}}"#).expect_err("1e-6 must fail");
        assert!(err.contains("$.pose.z"), "{err}");
        assert!(err.contains("10.0"), "{err}");
        assert!(err.contains("10.000001"), "{err}");
    }

    #[test]
    fn structural_difference_fails() {
        let err = ir_json_close(r#"{"moves":[1,2]}"#, r#"{"moves":[1]}"#).expect_err("array length");
        assert!(err.contains("$.moves"), "{err}");
        assert!(ir_json_close(r#"{"a":1}"#, r#"{"a":"1"}"#).is_err());
        assert!(ir_json_close(r#"{"a":1,"b":2}"#, r#"{"a":1}"#).is_err());
    }
}
