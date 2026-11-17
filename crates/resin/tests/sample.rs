#![allow(clippy::unwrap_used, clippy::expect_used)]
//! Checks against the Lychee 8.0.4 sample export. Set `CAMPANION_PM7M_SAMPLE` to its path; the
//! tests pass trivially when it is unset because the sample is Ryan's file and is not in the repo.

use campanion_resin::pm7m::{M7_MAX, WriteInput, read_pm7m, write_pm7m};
use campanion_resin::{pw0, zip};

fn sample() -> Option<Vec<u8>> {
    let path = std::env::var_os("CAMPANION_PM7M_SAMPLE")?;
    std::fs::read(path).ok()
}

#[test]
fn every_layer_image_round_trips_byte_for_byte() {
    let Some(bytes) = sample() else { return };
    let file = read_pm7m(&bytes).unwrap();
    assert_eq!(file.layers.len(), 592);
    for (i, image) in file.layer_images.iter().enumerate() {
        let dense = pw0::decode(image, file.width, file.height).unwrap();
        assert_eq!(&pw0::encode(&dense).unwrap(), image, "layer {i}");
    }
}

#[test]
fn rebuild_matches_sample() {
    let Some(bytes) = sample() else { return };
    let file = read_pm7m(&bytes).unwrap();
    assert_eq!(file.machine_name, M7_MAX.name);
    assert_eq!((file.width, file.height), (M7_MAX.res_x, M7_MAX.res_y));
    let settings = file.settings().unwrap();
    assert_eq!(settings.bottom_layers, 5);
    assert_eq!(settings.layer_height_mm, 0.05);
    let previews = [("preview_images/preview_0.png", 224, 168), ("preview_images/preview_1.png", 336, 252)];
    for (name, w, h) in previews {
        assert!(file.preview_sizes.contains(&(name.to_string(), w, h)), "{name}");
    }
    let orig = zip::read_zip(&bytes).unwrap();
    let get = |name: &str| orig.iter().find(|e| e.name == name).map(|e| e.data.clone()).unwrap();
    let rebuilt = write_pm7m(&WriteInput {
        machine: &M7_MAX,
        settings: &settings,
        layer_images: &file.layer_images,
        previews: (&get("preview_images/preview_0.png"), &get("preview_images/preview_1.png")),
        volume_ml: 3.0,
    })
    .unwrap();
    let new = zip::read_zip(&rebuilt).unwrap();
    // scene.slice is byte-identical.
    let find = |list: &[zip::ZipEntry], name: &str| list.iter().find(|e| e.name == name).map(|e| e.data.clone()).unwrap();
    assert_eq!(find(&new, "scene.slice"), find(&orig, "scene.slice"));
    // Layer images are byte-identical.
    for i in 0..592 {
        let n = format!("layer_images/layer_{i}.pw0Img");
        assert_eq!(find(&new, &n), find(&orig, &n), "{n}");
    }
    // layers_controller.conf has the same values (the sample's float rounding differs by 1 ulp).
    let a = read_pm7m(&rebuilt).unwrap();
    for (x, y) in a.layers.iter().zip(&file.layers) {
        assert_eq!(
            (x.index, x.exposure_time, x.thickness, x.zup_height, x.zup_speed),
            (y.index, y.exposure_time, y.thickness, y.zup_height, y.zup_speed)
        );
        assert!((x.min_height - y.min_height).abs() < 1e-9);
    }
    // Machine description is the same.
    let ours: serde_json::Value = serde_json::from_slice(&find(&new, "anycubic_photon_resins.pwsp")).unwrap();
    let theirs: serde_json::Value = serde_json::from_slice(&find(&orig, "anycubic_photon_resins.pwsp")).unwrap();
    assert_eq!(ours["machine_type"], theirs["machine_type"]);
    assert_eq!(ours["machine_extern"]["firmware_calc_print_time_paras"], theirs["machine_extern"]["firmware_calc_print_time_paras"]);
    // Entry order matches the sample apart from layer image order.
    let head: Vec<_> = orig.iter().take(6).map(|e| e.name.clone()).collect();
    let head_new: Vec<_> = new.iter().take(6).map(|e| e.name.clone()).collect();
    assert_eq!(head[..5], head_new[..5]);
    // Text entries with no per-run data match in key set.
    for name in ["print_info.json", "software_info.conf"] {
        let o: serde_json::Value = serde_json::from_slice(&find(&orig, name)).unwrap();
        let n: serde_json::Value = serde_json::from_slice(&find(&new, name)).unwrap();
        let ok: Vec<_> = o.as_object().unwrap().keys().collect();
        let nk: Vec<_> = n.as_object().unwrap().keys().collect();
        assert_eq!(ok, nk, "{name}");
    }
}
