# campanion-post

RS274/NGC posts. A post definition is JSON (`posts/linuxcnc.json` and the other built-ins). `post_program` writes a cutter-location program. `read_back` and `compare` check that text against the IR.

## Rotary axes

`rotary` is optional.

- `axes` — `"a"`, `"b"`, or both.
- `decimals` — digits on the A/B word. The angle is degrees, formatted with the same fixed-point rules as the other words.
- `feed_mode` — `inverse_time`, `degrees_per_minute`, or `surface`.
- `inverse_time_on` and `inverse_time_off` — required when `feed_mode` is `inverse_time` (LinuxCNC, Fanuc and Haas use `G93` and `G94`). Rejected for the other modes. Mach3 uses `degrees_per_minute`.

Without `rotary`, any pose that carries an angle is rejected: `post `<id>` does not support a rotary axis`. A program whose setup axis is not listed is rejected by name (`post `<id>` does not support rotary axis `b``).

A pose with an angle is written in the machine frame. `X` `Y` `Z` are `part_to_machine(pose, angle).tip()`. The A or B word is the continuous angle from the IR. Rapids include that word and do not change the feed mode.

### Feeds when the angle changes

The move length is the straight-line distance in millimetres between the part-frame tips. It is not the machine-frame distance, and it is not the arc length.

`inverse_time` emits `inverse_time_on` (LinuxCNC, Fanuc and Haas `G93`). `F = ir_feed / length`. `ir_feed` is millimetres per minute and `length` is millimetres, so `F` is `1 / minutes`. `F` is written on every one of those feed blocks. The next feed that keeps the angle emits `inverse_time_off` (`G94`) and goes back to millimetres per minute.

`degrees_per_minute` writes the IR feed number unchanged. Controllers that mix millimetres and degrees read that number as written.

`surface` writes the IR feed, millimetres per minute.

A feed that keeps the angle uses the normal millimetres-per-minute word.

### Arcs

An arc or helix whose start and end angles are equal is written as an arc in the machine frame when the rotated plane is still XY, ZX, or YZ. The centre is `rotate_point` of the part-frame centre. A reversed plane normal swaps G2 and G3. Any other rotary arc is linearised to the arc tolerance first, then each point is mapped with `part_to_machine`.

## Lathe posts

`lathe` is optional. A program with a turning setup is a lathe program: its poses are part-frame XZ. `tip.x()` is a radius in millimetres, `tip.y()` is 0 and `tip.z()` runs along the spindle. The post applies diameter mode. The IR does not.

A turning program posted without `lathe` is rejected: `post `<id>` is not a lathe post`. A lathe post given a mill program is rejected: `lathe post `<id>` needs a turning program`. A lathe program must not carry a rotary setup. A pose with `|y|` above `1e-6` is rejected.

After the header the post writes `mode_lines`, then `plane` (LinuxCNC `G18`), then `feed_mode` (`G94` on LinuxCNC, `G98` on Fanuc and Haas). Those three are written once.

Motion words are `X` and `Z` only. Y is never written. When `diameter_mode` is true, `X` is `2 * radius`. When it is false, `X` is the radius. Rapids are `G0`. Feeds are `G1` with `F`, in the same millimetres-per-minute number the IR stores. The linear decimals are the post's `decimals.linear`.

Arcs are in the ZX plane (`Plane::Zx`). In the IR picture, viewed from +Y with +Z to the right and +X up, counter-clockwise is `ArcSense::Ccw`. With `arc_flip` false that sense is `G3` and clockwise is `G2`, the same ZX sense this crate uses for mill arcs. `arc_flip` true swaps `G2` and `G3` for a front turret or an X-down drawing. `ijk` mode writes `I` and `K`. `I` is the X offset of the centre from the block start and is always a radius, even in diameter mode. `K` is the Z offset. `r` mode writes the geometric radius `R`. `lines` mode chords the arc to the arc tolerance. Read-back of a lathe post returns part-frame positions (`x` radius, `y` 0, `z`) and maps `G2`/`G3` back to the IR sense, so `compare` checks the arc in the ZX plane.

A toolpath whose binding has `speed` writes that speed on `ProcessOn` for the spindle, then `spindle_cw` or, when that spindle is reversed, `spindle_ccw`. Constant surface speed uses the `css` lines (`{surface}` metres per minute, `{max_rpm}`). Direct rpm uses the `rpm` lines (`{rpm}`). A whole number is an integer. Any other number uses the feed decimals. A binding with no speed uses the process-on speed parameter and the rpm lines. If neither is set, the post rejects the program. `ProcessOff` for the spindle writes `spindle_off`.

Tool changes use `lathe.tool_change` instead of the mill `tool_change` lines. `{tool}` is the tool id. `{tool2}` and `{offset2}` are that id in at least two digits. Coolant, comments, dwell, line numbers and the line-length limit are unchanged. `home` is written after the footer and before `program_end`. It may be empty. LinuxCNC uses `G53 G0 X0 Z0` for that machine-coordinate retract. `read_back` does not treat `G53` as a part move.
