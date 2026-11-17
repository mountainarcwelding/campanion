# Resin output: Anycubic Photon Mono M7 Max

**Status: not printer-tested.** Nothing written by `campanion resin` has been printed. The
format was derived from one file, so some fields are copied rather than understood. CAMpanion is
primarily command-line driven; there is no resin GUI yet.

## Commands

```
campanion resin MODEL.stl -o OUT.pm7m [--params-from SAMPLE.pm7m]
    [--layer-height MM] [--bottom-layers N]
    [--bottom-exposure S] [--bottom-lift-height MM] [--bottom-lift-speed V]
    [--exposure S] [--lift-height MM] [--lift-speed V]
    [--resin-name NAME] [--keep-xy]
campanion resin-info FILE.pm7m
```

- Every exposure, lift and layer value is required, from a flag or from `--params-from`. A flag
  wins over the sample. `--params-from` reads layer height from layer 0, treats the leading layers
  that share layer 0's exposure and lift as bottom layers, and takes the normal values from the last
  layer. Lift speed is stored as given; the sample does not name its unit.
- The model is moved so its lowest point is on the plate and its XY bounds are centred on the plate
  (`--keep-xy` leaves XY alone, read as millimetres from the plate centre). A model that does not fit
  the 297.5 x 164 x 300 mm volume is refused, never clipped.
- Layer `i` is sampled at `(i + 0.5) * layer height`. Fill is non-zero winding, so a body overlapping
  its own support shells merges. A pixel is fully on (15) or off (0).
- `resin-info` decodes every layer image and checks the layer counts and markers.
- The finished file is read back before it is written.

## What was derived from the sample

Machine: `Anycubic Photon Mono M7 Max` (exact case), screen 6480 x 3600, bed 297.5 x 164 x 300 mm.

`.pm7m` is a zip with, in this order: `anycubic_photon_resins.pwsp`, `software_info.conf`,
`print_info.json`, `preview_images/preview_1.png` (336x252), `preview_images/preview_0.png` (224x168),
`scene.slice`, `layers_controller.conf`, `layer_images/layer_N.pw0Img`.

`.pw0Img`: big-endian 16-bit words, top 4 bits gray (0 to 15), low 12 bits run length (1 to 4095),
row-major, runs continue across rows, long runs split greedily at 4095. The runs sum to 6480 x 3600.

`scene.slice`: 13-byte magic `ANYCUBIC-PWSZ`; u32 3 at offset 80; u32 1 at 84; f32 1.0 at 96;
the layer count as u32 at 100; `<---` at 388; the count again at 392; then one 64-byte record per
layer (f32 top height at +0, u32 1 at +24, zeros elsewhere); `--->` at the end. The other
bytes are zero in the sample.

`layers_controller.conf`: tab-indented JSON with `count` and `paras`, each layer carrying
`exposure_time`, `layer_index`, `layer_minheight`, `layer_thickness`, `zup_height`, `zup_speed`.

## Checks against the sample

With the sample's path in `CAMPANION_PM7M_SAMPLE`, `cargo test -p campanion-resin --test sample`:

- all 592 layer images decode and re-encode to identical bytes;
- `scene.slice` is rebuilt byte for byte from the settings read out of the sample;
- the rebuilt layer list matches (min heights agree within 1e-9; see below), `machine_type` and the
  firmware block of the pwsp are identical, and the thumbnails have the sample's sizes.

The sample is not committed (it is a user file). Without the variable, those tests do nothing.

## Assumptions and gaps

- **Image orientation is unverified.** +X is image right and +Y is image up. Whether the printer shows
  the image mirrored or flipped needs a printed asymmetric test part.
- **Copied constants.** `machine_extern` in the pwsp (including the sample's `alias` of
  `Anycubic Photon Mono M7 Pro`), the resin's density, price, depth-penetration curve and
  `multi_state_paras` entries other than `bott_0` and `normal_0`, and the unknown `scene.slice` words
  above are the sample's values. Which of these the printer actually reads is not known.
- **Resin metadata.** The pwsp resin entry carries the user's exposures, lift, bottom layers and layer
  height; its retract speed (`zdown_speed`) is set equal to the lift speed, as in the sample.
- **`layer_minheight`** is `i * layer_height`. The sample's values differ from that in the last digit
  for some layers (a rounding difference in whatever wrote it); the printer should not care.
- **`software_info.conf`** says `CAMpanion`, not the slicer name in the sample.
- **`print_info.json`**: `print_time` is the total exposure time only, a lower bound that leaves out lift
  and settle time; `cost` is 0; `volume` is the solid volume in mL.
- Previews are a simple shaded top view.
- Not done: anti-aliasing, supports and rafts, STL units other than millimetres, other Anycubic models,
  other resin printers, wait-before-lift and wait-after-lift times, per-layer overrides.
- No UVtools read-back was done (it was not installed on the build machine); the read-back is
  CAMpanion's own reader.
