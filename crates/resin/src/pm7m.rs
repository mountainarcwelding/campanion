//! The `.pm7m` container for the Anycubic Photon Mono M7 Max.
//!
//! Layout derived from one Lychee Slicer 8.0.4 export. The file is a zip holding:
//!
//! - `anycubic_photon_resins.pwsp`: JSON with the machine (name, screen, bed) and a resin entry.
//! - `software_info.conf`, `print_info.json`: small JSON files.
//! - `preview_images/preview_0.png` (224x168) and `preview_1.png` (336x252).
//! - `scene.slice`: binary. Magic `ANYCUBIC-PWSZ`, then a layer count and one 64-byte record per
//!   layer. Only the layer count and each layer's top height were found to vary in the sample.
//! - `layers_controller.conf`: JSON with each layer's exposure, thickness and lift.
//! - `layer_images/layer_N.pw0Img`: one [`crate::pw0`] image per layer, N from 0.
//!
//! Unknown fields are copied as constants from the sample. See `docs/resin.md`.

use serde_json::{Map, Value, json};

use crate::error::{ResinError, Result};
use crate::pw0;
use crate::zip::{ZipWriter, read_zip};

const TEMPLATE: &str = include_str!("m7max_template.json");
const SCENE_MAGIC: &[u8] = b"ANYCUBIC-PWSZ";
const SCENE_COUNT_AT: usize = 100;
const SCENE_START_MARK_AT: usize = 388;
const SCENE_COUNT2_AT: usize = 392;
const SCENE_RECORDS_AT: usize = 396;
const SCENE_RECORD_LEN: usize = 64;
const SCENE_HEADER_VERSION_AT: usize = 80;
const SCENE_ONE_AT: usize = 84;
const SCENE_FLOAT_ONE_AT: usize = 96;
/// Offset inside a layer record of the constant `1` word.
const SCENE_RECORD_ONE_AT: usize = 24;

/// The target printer. The name must match exactly, including case.
#[derive(Debug, Clone, PartialEq)]
pub struct MachineSpec {
    pub name: &'static str,
    pub res_x: u32,
    pub res_y: u32,
    pub bed_x_mm: f64,
    pub bed_y_mm: f64,
    pub bed_z_mm: f64,
}

/// Anycubic Photon Mono M7 Max, as recorded in the sample's `machine_type`.
pub const M7_MAX: MachineSpec =
    MachineSpec { name: "Anycubic Photon Mono M7 Max", res_x: 6480, res_y: 3600, bed_x_mm: 297.5, bed_y_mm: 164.0, bed_z_mm: 300.0 };

impl MachineSpec {
    pub fn grid(&self) -> crate::slice::SliceGrid {
        crate::slice::SliceGrid {
            width: self.res_x,
            height: self.res_y,
            pitch_x_mm: self.bed_x_mm / f64::from(self.res_x),
            pitch_y_mm: self.bed_y_mm / f64::from(self.res_y),
        }
    }
}

/// Exposure and lift for one class of layer. All values come from the user or from a sample file.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LayerParams {
    pub exposure_s: f64,
    pub lift_height_mm: f64,
    pub lift_speed: f64,
}

/// Resin and print settings. No defaults: every value is user input or read from a sample.
#[derive(Debug, Clone, PartialEq)]
pub struct ResinSettings {
    pub layer_height_mm: f64,
    pub bottom_layers: u32,
    pub bottom: LayerParams,
    pub normal: LayerParams,
    pub resin_name: String,
}

impl ResinSettings {
    pub fn validate(&self) -> Result<()> {
        let ok = |v: f64| v.is_finite() && v > 0.0;
        if !(ok(self.layer_height_mm) && self.layer_height_mm <= 1.0) {
            return Err(ResinError::invalid("layer height must be above 0 and at most 1 mm"));
        }
        for (label, p) in [("bottom", &self.bottom), ("normal", &self.normal)] {
            if !ok(p.exposure_s) || p.exposure_s > 600.0 {
                return Err(ResinError::invalid(format!("{label} exposure must be above 0 and at most 600 s")));
            }
            if !(p.lift_height_mm.is_finite() && p.lift_height_mm >= 0.0 && p.lift_height_mm <= 100.0) {
                return Err(ResinError::invalid(format!("{label} lift height must be 0 to 100 mm")));
            }
            if !ok(p.lift_speed) || p.lift_speed > 200.0 {
                return Err(ResinError::invalid(format!("{label} lift speed must be above 0 and at most 200")));
            }
        }
        if self.resin_name.is_empty() || self.resin_name.len() > 64 || self.resin_name.chars().any(char::is_control) {
            return Err(ResinError::invalid("resin name must be 1 to 64 printable characters"));
        }
        Ok(())
    }

    fn params_for(&self, layer: usize) -> LayerParams {
        if (layer as u64) < u64::from(self.bottom_layers) { self.bottom } else { self.normal }
    }
}

/// Everything [`write_pm7m`] needs.
pub struct WriteInput<'a> {
    pub machine: &'a MachineSpec,
    pub settings: &'a ResinSettings,
    /// One encoded `.pw0Img` per layer, bottom first.
    pub layer_images: &'a [Vec<u8>],
    /// PNG bytes for `preview_0.png` (224x168) and `preview_1.png` (336x252).
    pub previews: (&'a [u8], &'a [u8]),
    /// Solid volume in millilitres, for `print_info.json`.
    pub volume_ml: f64,
}

/// One layer of `layers_controller.conf`.
#[derive(Debug, Clone, PartialEq)]
pub struct LayerRecord {
    pub index: u32,
    pub exposure_time: f64,
    pub min_height: f64,
    pub thickness: f64,
    pub zup_height: f64,
    pub zup_speed: f64,
}

/// A parsed `.pm7m`.
#[derive(Debug, Clone)]
pub struct Pm7mFile {
    pub machine_name: String,
    pub width: u32,
    pub height: u32,
    pub layers: Vec<LayerRecord>,
    /// Raw encoded layer images, indexed by layer.
    pub layer_images: Vec<Vec<u8>>,
    /// Top height of each layer from `scene.slice`.
    pub scene_heights: Vec<f32>,
    pub scene: Vec<u8>,
    pub pwsp: Value,
    pub print_info: Value,
    pub software_info: Value,
    pub entry_names: Vec<String>,
    pub preview_sizes: Vec<(String, u32, u32)>,
}

fn fmt_num(x: f64) -> String {
    if x.is_finite() && x.fract() == 0.0 && x.abs() < 1e15 { format!("{}", x as i64) } else { format!("{x}") }
}

/// A JSON number that prints without a trailing `.0` when whole.
pub fn num(x: f64) -> Value {
    if x.is_finite() && x.fract() == 0.0 && x.abs() < 1e15 {
        Value::from(x as i64)
    } else {
        serde_json::Number::from_f64(x).map_or(Value::Null, Value::Number)
    }
}

fn tab_json(value: &Value) -> Result<Vec<u8>> {
    use serde::Serialize as _;
    let mut out = Vec::new();
    let formatter = serde_json::ser::PrettyFormatter::with_indent(b"\t");
    let mut ser = serde_json::Serializer::with_formatter(&mut out, formatter);
    value.serialize(&mut ser).map_err(|err| ResinError::Format(err.to_string()))?;
    Ok(out)
}

fn layers_controller(settings: &ResinSettings, count: usize) -> String {
    let mut s = String::from("{\n\t\"count\": ");
    s.push_str(&count.to_string());
    s.push_str(",\n\t\"paras\": [");
    for i in 0..count {
        let p = settings.params_for(i);
        s.push_str(if i == 0 { "\n\t\t{\n" } else { ",\n\t\t{\n" });
        s.push_str(&format!("\t\t\t\"exposure_time\": {},\n", fmt_num(p.exposure_s)));
        s.push_str(&format!("\t\t\t\"layer_index\": {i},\n"));
        s.push_str(&format!("\t\t\t\"layer_minheight\": {},\n", fmt_num(i as f64 * settings.layer_height_mm)));
        s.push_str(&format!("\t\t\t\"layer_thickness\": {},\n", fmt_num(settings.layer_height_mm)));
        s.push_str(&format!("\t\t\t\"zup_height\": {},\n", fmt_num(p.lift_height_mm)));
        s.push_str(&format!("\t\t\t\"zup_speed\": {}\n\t\t}}", fmt_num(p.lift_speed)));
    }
    s.push_str("\n\t]\n}");
    s
}

fn scene_slice(settings: &ResinSettings, count: usize) -> Result<Vec<u8>> {
    let count32 = u32::try_from(count).map_err(|_| ResinError::invalid("too many layers"))?;
    let mut b = vec![0u8; SCENE_RECORDS_AT + SCENE_RECORD_LEN * count + 4];
    let mut put = |at: usize, bytes: &[u8]| {
        if let Some(dst) = b.get_mut(at..at + bytes.len()) {
            dst.copy_from_slice(bytes);
        }
    };
    put(0, SCENE_MAGIC);
    put(SCENE_HEADER_VERSION_AT, &3u32.to_le_bytes());
    put(SCENE_ONE_AT, &1u32.to_le_bytes());
    put(SCENE_FLOAT_ONE_AT, &1.0f32.to_le_bytes());
    put(SCENE_COUNT_AT, &count32.to_le_bytes());
    put(SCENE_START_MARK_AT, b"<---");
    put(SCENE_COUNT2_AT, &count32.to_le_bytes());
    for i in 0..count {
        let at = SCENE_RECORDS_AT + SCENE_RECORD_LEN * i;
        let top = ((i + 1) as f64 * settings.layer_height_mm) as f32;
        put(at, &top.to_le_bytes());
        put(at + SCENE_RECORD_ONE_AT, &1u32.to_le_bytes());
    }
    put(SCENE_RECORDS_AT + SCENE_RECORD_LEN * count, b"--->");
    Ok(b)
}

fn pwsp(machine: &MachineSpec, settings: &ResinSettings) -> Result<Value> {
    let mut doc: Value = serde_json::from_str(TEMPLATE).map_err(|err| ResinError::Format(format!("template: {err}")))?;
    let name = machine.name;
    let mt = doc.get_mut("machine_type").and_then(Value::as_object_mut).ok_or_else(|| ResinError::Format("template machine_type".into()))?;
    if mt.get("name").and_then(Value::as_str) != Some(name) {
        return Err(ResinError::Format("template machine name does not match the machine".into()));
    }
    let ext = doc.get_mut("machine_extern").and_then(Value::as_object_mut).ok_or_else(|| ResinError::Format("template machine_extern".into()))?;
    ext.insert("active_resins".into(), json!([settings.resin_name]));
    let resin = ext
        .get_mut("user_resins")
        .and_then(Value::as_array_mut)
        .and_then(|a| a.first_mut())
        .and_then(Value::as_object_mut)
        .ok_or_else(|| ResinError::Format("template user resin".into()))?;
    if let Some(prop) = resin.get_mut("property").and_then(Value::as_object_mut) {
        prop.insert("name".into(), json!(settings.resin_name));
    }
    if let Some(sp) = resin.get_mut("slicepara").and_then(Value::as_object_mut) {
        sp.insert("bott_layers".into(), json!(settings.bottom_layers));
        sp.insert("bott_time".into(), num(settings.bottom.exposure_s));
        sp.insert("exposure_time".into(), num(settings.normal.exposure_s));
        sp.insert("zthick".into(), num(settings.layer_height_mm));
        sp.insert("zup_height".into(), num(settings.normal.lift_height_mm));
        sp.insert("zup_speed".into(), num(settings.normal.lift_speed));
        // The sample's retract speed always equals its lift speed; layers_controller.conf has no
        // separate field for it.
        sp.insert("zdown_speed".into(), num(settings.normal.lift_speed));
    }
    if let Some(ms) = resin.get_mut("slice_extpara").and_then(|v| v.get_mut("multi_state_paras")).and_then(Value::as_object_mut) {
        for (key, p) in [("bott_0", settings.bottom), ("normal_0", settings.normal)] {
            let mut m = Map::new();
            m.insert("down_speed".into(), num(p.lift_speed));
            m.insert("height".into(), num(p.lift_height_mm));
            m.insert("up_speed".into(), num(p.lift_speed));
            ms.insert(key.into(), Value::Object(m));
        }
    }
    Ok(doc)
}

/// Build the `.pm7m` bytes.
pub fn write_pm7m(input: &WriteInput<'_>) -> Result<Vec<u8>> {
    input.settings.validate()?;
    let count = input.layer_images.len();
    if count == 0 {
        return Err(ResinError::invalid("no layers"));
    }
    let mut zip = ZipWriter::new();
    zip.add("anycubic_photon_resins.pwsp", &tab_json(&pwsp(input.machine, input.settings)?)?)?;
    let software = json!({"mark": "CAMpanion", "opengl": "", "os": std::env::consts::OS, "version": env!("CARGO_PKG_VERSION")});
    zip.add("software_info.conf", &tab_json(&software)?)?;
    let total_exposure: f64 = (0..count).map(|i| input.settings.params_for(i).exposure_s).sum();
    let info = json!({
        "cost": 0,
        "currency": "USD",
        "print_time": total_exposure.round() as i64,
        "volume": num((input.volume_ml * 1000.0).round() / 1000.0),
    });
    zip.add("print_info.json", &tab_json(&info)?)?;
    zip.add("preview_images/preview_1.png", input.previews.1)?;
    zip.add("preview_images/preview_0.png", input.previews.0)?;
    zip.add("scene.slice", &scene_slice(input.settings, count)?)?;
    zip.add("layers_controller.conf", layers_controller(input.settings, count).as_bytes())?;
    for (i, image) in input.layer_images.iter().enumerate() {
        zip.add(&format!("layer_images/layer_{i}.pw0Img"), image)?;
    }
    zip.finish()
}

/// Read print settings from a sample `.pm7m`. The file must be for [`M7_MAX`].
pub fn settings_from_pm7m_bytes(bytes: &[u8]) -> Result<ResinSettings> {
    let file = read_pm7m(bytes)?;
    if file.machine_name != M7_MAX.name {
        return Err(ResinError::invalid(format!("sample is for `{}`, not `{}`", file.machine_name, M7_MAX.name)));
    }
    file.settings()
}

/// Load print settings from a sample `.pm7m` path (the same file `campanion resin --params-from` uses).
pub fn load_print_settings(path: &std::path::Path) -> Result<ResinSettings> {
    let bytes = std::fs::read(path).map_err(|err| ResinError::invalid(format!("{}: {err}", path.display())))?;
    settings_from_pm7m_bytes(&bytes)
}

fn parse_json(entries: &[crate::zip::ZipEntry], name: &str) -> Result<Value> {
    let entry = entries.iter().find(|e| e.name == name).ok_or_else(|| ResinError::Format(format!("missing `{name}`")))?;
    serde_json::from_slice(&entry.data).map_err(|err| ResinError::Format(format!("`{name}`: {err}")))
}

fn png_size(data: &[u8]) -> Option<(u32, u32)> {
    let w = data.get(16..20)?;
    let h = data.get(20..24)?;
    Some((u32::from_be_bytes([w[0], w[1], w[2], w[3]]), u32::from_be_bytes([h[0], h[1], h[2], h[3]])))
}

/// Parse a `.pm7m`, checking its structure and every layer image's pixel total.
pub fn read_pm7m(bytes: &[u8]) -> Result<Pm7mFile> {
    let entries = read_zip(bytes)?;
    let pwsp = parse_json(&entries, "anycubic_photon_resins.pwsp")?;
    let print_info = parse_json(&entries, "print_info.json")?;
    let software_info = parse_json(&entries, "software_info.conf")?;
    let controller = parse_json(&entries, "layers_controller.conf")?;
    let mt = pwsp.get("machine_type").ok_or_else(|| ResinError::Format("pwsp has no machine_type".into()))?;
    let machine_name = mt.get("name").and_then(Value::as_str).ok_or_else(|| ResinError::Format("pwsp has no machine name".into()))?.to_string();
    let dim = |key: &str| -> Result<u32> {
        mt.get(key)
            .and_then(Value::as_u64)
            .and_then(|v| u32::try_from(v).ok())
            .filter(|v| *v > 0 && *v <= 20_000)
            .ok_or_else(|| ResinError::Format(format!("pwsp machine_type.{key}")))
    };
    let (width, height) = (dim("res_x")?, dim("res_y")?);
    let paras = controller.get("paras").and_then(Value::as_array).ok_or_else(|| ResinError::Format("layers_controller has no paras".into()))?;
    let declared = controller.get("count").and_then(Value::as_u64).unwrap_or(0);
    if declared != paras.len() as u64 {
        return Err(ResinError::Format("layers_controller count does not match its layer list".into()));
    }
    let mut layers = Vec::with_capacity(paras.len());
    for (i, p) in paras.iter().enumerate() {
        let f = |key: &str| p.get(key).and_then(Value::as_f64).ok_or_else(|| ResinError::Format(format!("layer {i}: missing `{key}`")));
        let index = f("layer_index")? as u32;
        if index as usize != i {
            return Err(ResinError::Format(format!("layer {i}: layer_index is {index}")));
        }
        layers.push(LayerRecord {
            index,
            exposure_time: f("exposure_time")?,
            min_height: f("layer_minheight")?,
            thickness: f("layer_thickness")?,
            zup_height: f("zup_height")?,
            zup_speed: f("zup_speed")?,
        });
    }
    let scene = entries.iter().find(|e| e.name == "scene.slice").ok_or_else(|| ResinError::Format("missing scene.slice".into()))?.data.clone();
    if scene.get(..SCENE_MAGIC.len()) != Some(SCENE_MAGIC) {
        return Err(ResinError::Format("scene.slice has the wrong magic".into()));
    }
    let word = |at: usize| -> Result<u32> {
        let b = scene.get(at..at + 4).ok_or_else(|| ResinError::Format("scene.slice is truncated".into()))?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    };
    let scene_count = word(SCENE_COUNT_AT)? as usize;
    if word(SCENE_COUNT2_AT)? as usize != scene_count || scene_count != layers.len() {
        return Err(ResinError::Format("scene.slice layer count does not match layers_controller".into()));
    }
    if scene.len() != SCENE_RECORDS_AT + SCENE_RECORD_LEN * scene_count + 4
        || scene.get(SCENE_START_MARK_AT..SCENE_START_MARK_AT + 4) != Some(b"<---")
        || scene.get(scene.len() - 4..) != Some(b"--->")
    {
        return Err(ResinError::Format("scene.slice has the wrong size or markers".into()));
    }
    let mut scene_heights = Vec::with_capacity(scene_count);
    for i in 0..scene_count {
        scene_heights.push(f32::from_bits(word(SCENE_RECORDS_AT + SCENE_RECORD_LEN * i)?));
    }
    let mut layer_images = Vec::with_capacity(layers.len());
    for i in 0..layers.len() {
        let name = format!("layer_images/layer_{i}.pw0Img");
        let entry = entries.iter().find(|e| e.name == name).ok_or_else(|| ResinError::Format(format!("missing `{name}`")))?;
        pw0::decode_runs(&entry.data, u64::from(width) * u64::from(height)).map_err(|err| ResinError::Format(format!("`{name}`: {err}")))?;
        layer_images.push(entry.data.clone());
    }
    let preview_sizes = entries
        .iter()
        .filter(|e| e.name.starts_with("preview_images/"))
        .filter_map(|e| png_size(&e.data).map(|(w, h)| (e.name.clone(), w, h)))
        .collect();
    Ok(Pm7mFile {
        machine_name,
        width,
        height,
        layers,
        layer_images,
        scene_heights,
        scene,
        pwsp,
        print_info,
        software_info,
        entry_names: entries.iter().map(|e| e.name.clone()).collect(),
        preview_sizes,
    })
}

impl Pm7mFile {
    /// Settings deduced from the layer list: bottom layers are the leading layers that share layer 0's
    /// exposure and lift. The normal values come from the last layer. Fails when the layers have fewer
    /// than two classes only if the list is empty.
    pub fn settings(&self) -> Result<ResinSettings> {
        let first = self.layers.first().ok_or_else(|| ResinError::invalid("sample has no layers"))?;
        let last = self.layers.last().ok_or_else(|| ResinError::invalid("sample has no layers"))?;
        let same =
            |a: &LayerRecord, b: &LayerRecord| a.exposure_time == b.exposure_time && a.zup_height == b.zup_height && a.zup_speed == b.zup_speed;
        let bottom_layers = self.layers.iter().take_while(|l| same(l, first)).count() as u32;
        let resin_name =
            self.pwsp.pointer("/machine_extern/active_resins/0").and_then(Value::as_str).filter(|s| !s.is_empty()).unwrap_or("resin").to_string();
        let to_params = |l: &LayerRecord| LayerParams { exposure_s: l.exposure_time, lift_height_mm: l.zup_height, lift_speed: l.zup_speed };
        let settings = ResinSettings {
            layer_height_mm: first.thickness,
            bottom_layers: if bottom_layers as usize == self.layers.len() { 0 } else { bottom_layers },
            bottom: to_params(first),
            normal: to_params(last),
            resin_name,
        };
        settings.validate()?;
        Ok(settings)
    }
}

/// Draw both thumbnails for `mesh` with the machine's preview colours from the sample.
pub fn make_previews(mesh: &campanion_mesh::Mesh) -> Result<(Vec<u8>, Vec<u8>)> {
    use crate::preview::{PreviewStyle, render_png};
    let doc: Value = serde_json::from_str(TEMPLATE).map_err(|err| ResinError::Format(format!("template: {err}")))?;
    let color = |ptr: &str| -> Result<[f64; 3]> {
        let a = doc.pointer(ptr).and_then(Value::as_array).ok_or_else(|| ResinError::Format(format!("template {ptr}")))?;
        let f = |i: usize| a.get(i).and_then(Value::as_f64).ok_or_else(|| ResinError::Format(format!("template {ptr}")));
        Ok([f(0)?, f(1)?, f(2)?])
    };
    let model = color("/machine_type/prev_model_color")?;
    let p0 = render_png(mesh, 224, 168, PreviewStyle { back: color("/machine_type/prev_back_color")?, model })?;
    let p1 = render_png(mesh, 336, 252, PreviewStyle { back: color("/machine_type/prev2_back_color")?, model })?;
    Ok((p0, p1))
}
