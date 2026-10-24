# campanion-post

RS274/NGC posts. A post definition is JSON (`posts/linuxcnc.json` and the other built-ins). `post_program` writes a cutter-location program. `read_back` and `compare` check that text against the IR.

`decimal_point`, when true, keeps a decimal point on every dimensional word (`X` `Y` `Z`, `I` `J` `K`, `R`, rotary `A`/`B`, and lathe `X`/`Z`): a whole number is `52.` and zero is `0.`. Fanuc and Haas controls read a word written without a decimal point as least-input increments, so `X52` is 0.052 mm and `A90` is 0.09 degrees. Feed (`F`), spindle (`S`), dwell (`P`), line numbers, and tool and program numbers are left alone. LinuxCNC, Mach3, Grbl, and plasma omit the flag and read plain decimals.

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

`lathe` is optional. A program with a turning setup is a lathe program: its poses are part-frame. `tip.x()` is a radius in millimetres and `tip.z()` runs along the spindle. `tip.y()` is 0 unless the head carries Y and `lathe.y_axis` is set. The post applies diameter mode. The IR does not.

A turning program posted without `lathe` is rejected: `post `<id>` is not a lathe post`. A lathe post given a mill program is rejected: `lathe post `<id>` needs a turning program`. A lathe program must not carry a rotary setup. A pose with `|y|` above `1e-6` is rejected outside face and cylinder interpolation (see Mill-turn).

After the header the post writes `mode_lines`, then `plane` (`G18`), then `feed_mode` (`G94` on LinuxCNC and grblHAL, `G98` on Fanuc and Haas). Those three are written once. LinuxCNC and grblHAL select diameter mode with `G7`. Fanuc and Haas leave `mode_lines` empty: diameter mode is the control default.

Outside face and cylinder interpolation, motion words are `X` and `Z`, and `Y` when `lathe.y_axis` is set (see [Y axis](#y-axis)); Y is never doubled. Without `y_axis`, Y is never written there. When `diameter_mode` is true, `X` is `2 * radius`. When it is false, `X` is the radius. Rapids are `G0`. Feeds are `G1` with `F`, in the same millimetres-per-minute number the IR stores. The linear decimals are the post's `decimals.linear`. While a C axis is engaged, `G0` and `G1` also write the C word after Z. See Mill-turn below.

A synchronised thread move is written in full, never as a modal continuation: `G33 X.. Z.. K..` on LinuxCNC and grblHAL, `G32 X.. Z.. F..` on Fanuc and Haas. The lead uses the linear decimals and the post's decimal-point rule. The motion code is not left modal, so the next `G0` or `G1` is written explicitly. When the lead word is `F`, the modal feed is cleared and the next feed move writes `F` again. Threading needs a binding whose speed is direct rpm; constant surface speed, or a missing speed, is rejected with `threading needs a fixed rpm binding`. A mill post rejects the move. A lathe post with no `thread` block rejects it too. Read-back turns those blocks back into a thread motion, so `compare` checks the end point and the lead.

Outside face and cylinder interpolation (and Y-axis milling), arcs are in the ZX plane (`Plane::Zx`). In the IR picture, viewed from +Y with +Z to the right and +X up, counter-clockwise is `ArcSense::Ccw`. With `arc_flip` false that sense is `G3` and clockwise is `G2`, the same ZX sense this crate uses for mill arcs. `arc_flip` true swaps `G2` and `G3` for a front turret or an X-down drawing. `ijk` mode writes `I` and `K`. `I` is the X offset of the centre from the block start and is always a radius, even in diameter mode. `K` is the Z offset. `r` mode writes the geometric radius `R`. `lines` mode chords the arc to the arc tolerance. Read-back of a lathe post returns part-frame positions (`x` radius, `y` 0, `z`) and maps `G2`/`G3` back to the IR sense, so `compare` checks the arc in the ZX plane.

A toolpath whose binding has `speed` writes that speed on `ProcessOn` for the spindle, then `spindle_cw` or, when that spindle is reversed, `spindle_ccw`. Constant surface speed uses the `css` lines (`{surface}` metres per minute, `{max_rpm}`). Direct rpm uses the `rpm` lines (`{rpm}`). A whole number is an integer. Any other number uses the feed decimals. A binding with no speed uses the process-on speed parameter and the rpm lines. If neither is set, the post rejects the program. `ProcessOff` for the spindle writes `spindle_off`.

Tool changes use `lathe.tool_change` instead of the mill `tool_change` lines. `{tool}` is the tool id. `{tool2}` and `{offset2}` are that id in at least two digits. Coolant, comments, dwell, line numbers and the line-length limit are unchanged. `home` is written after the footer and before `program_end`. It may be empty. If face or cylinder interpolation is still on, the live tool is still running, the brake is still clamped, or the C axis is still engaged, the post closes them before `home`: interpolation off, live-tool stop, brake release, then C-axis disengage. LinuxCNC uses `G53 G0 X0 Z0` for that machine-coordinate retract. Haas uses `G53 G0 X0` and then `G53 G0 Z0`. Fanuc uses `G28 U0 W0`: `U` and `W` are incremental, and zero keeps the intermediate point at the current position so the control returns straight to the reference point. grblHAL writes no home lines. `read_back` does not treat `G53` or `G28` as a part move. On a lathe post, `U` and `W` are accepted only on a `G28` block. `G50` (the spindle-speed cap) and `G98` (Fanuc and Haas feed per minute) are not moves.

### Mill-turn (C axis and live tools)

A turning program can engage a spindle as a positioning C axis, clamp its brake, and run a live (driven) tool. The IR events are:

- `CAxis` `{ spindle, engage }`. `engage: true` uses that spindle as a C axis. `false` returns it to turning.
- `SpindleBrake` `{ spindle, clamp }`. `clamp: true` clamps the brake. `false` releases it.
- `LiveToolOn` `{ rpm, direction }`. `direction` is `forward` or `reverse`. `rpm` is finite, greater than 0, and at most 100000.
- `LiveToolOff` stops the live tool.
- `FaceInterpolation` `{ on }`. `on: true` selects Cartesian face interpolation. `false` leaves it.
- `CylinderInterpolation` `{ radius }`. A radius greater than 0 selects cylindrical interpolation. Absent or `null` leaves it.

In a turning program `Pose.rotary` is the C angle in degrees. It is finite, and its absolute value is at most 99999. A `CAxis` or `SpindleBrake` spindle must exist on the turning setup and have a C axis. The errors are `spindle NAME has no C axis` and `unknown spindle NAME`. `LiveToolOn` and `LiveToolOff` are allowed on a program that has no turning setup. A mill post rejects these mill-turn events as unsupported.

`lathe.c_axis`, `lathe.brake` and `lathe.live_tool` are optional. `c_axis.engage`, `c_axis.disengage`, `brake.clamp` and `brake.release` may be empty: an empty list writes nothing. `live_tool.forward`, `reverse` and `stop` each need at least one line. The top-level `references` list cites the codes (at most 32 entries, each 1 to 256 characters). Those strings are not written into the program.

`lathe.c_axis.letter` is one uppercase ASCII letter, and not `X`, `Y`, `Z`, `I`, `J`, `K`, `F`, `S`, `T`, `M`, `G`, `N`, `O`, or `P`. Haas uses `C`. `engage` and `disengage` take no placeholders. Engage writes `engage`. Disengage writes `disengage` and clears the modal C word, so the next engage writes the angle again. While the axis is engaged, a `G0` or `G1` whose pose carries `rotary` writes `<letter><angle>` after `Z`. The angle uses the linear decimals, including the post's decimal point. The word is modal: it is written only when that formatted text changes. A pose with a C angle while the axis is not engaged is `C angle needs the C axis engaged`. A post with no `c_axis` block is `post `<id>` has no C axis`. The missing block is reported before the not-engaged error. A turning spindle `ProcessOn` while the C axis is engaged is `turning spindle start while the C axis is engaged`.

Outside face and cylinder interpolation, arcs and helices stay ZX-plane arcs. They do not take a C word.

`FaceInterpolation` selects a Cartesian face frame. While it is on, a pose tip `(x, y, z)` is a point on the part face: `x` and `y` are Cartesian about the spindle axis and `z` is axial. The pose carries no rotary angle. Arcs may use `Plane::Xy`. `lathe.face_interp` is optional. `on` and `off` take no placeholders. `x_diameter` true writes X as a diameter. False writes the Cartesian X. The writer emits `on`, then each move as `X` `Y` `Z` (`Y` as stored). `G2` and `G3` in the XY plane use `I` and `J`. Those moves have no C word. `off` writes `off`. After that, modal X, Y and C are unknown, so the next move writes every axis.

`CylinderInterpolation` selects cylinder interpolation while `radius` is present and greater than 0. `tip.x` is the tool-tip radius. `tip.y` is the distance unrolled along the circumference at `radius` (C degrees = `y / radius` in radians). `z` is axial. Arcs may use `Plane::Yz`. `None` turns the mode off. `lathe.cylinder_interp` is optional. `on` may use `{radius}` and `{diameter}`, both formatted as linear numbers. `off` takes no placeholders. Motion is `X` `Y` `Z`. `X` follows `lathe.diameter_mode`. Arcs in the YZ plane use `J` and `K`.

Both events require the C axis to be engaged. The errors are `face interpolation needs the C axis engaged` and `cylinder interpolation needs the C axis engaged`. A post with neither the interpolation block nor `c_axis` reports `post `<id>` has no C axis` before the not-engaged error.

When the interpolation block is absent and `c_axis` is present, each pose is written as X and C. On a face, the radius is `hypot(x, y)` and C is `atan2(y, x)` in degrees, unwrapped so it stays within 180° of the previous C. On a cylinder, the radius is `tip.x` and C is `y / radius` in degrees. X follows `diameter_mode`. Arcs and long lines are split into `G1` moves so the polar chord stays within `PostOptions.arc_tolerance`.

A face move that passes within `1e-3` mm of the spindle axis (`x = y = 0`) is rejected in native face mode and in the fallback: `face path crosses the spindle centre; split it or use a post with face interpolation`.

Read-back of native face and cylinder blocks returns the IR frame, so `compare` checks those poses directly. In face mode, X stays a radius when `x_diameter` is false. Fallback output reads back as X and C. `compare` converts the IR pose with the same helper the writer uses.

`lathe.brake.clamp` and `lathe.brake.release` take no placeholders. `SpindleBrake` writes one of those lists. A post with no `brake` block is `post `<id>` has no spindle brake`. C may not change while the brake is clamped (release it, index, clamp again). A brake still clamped at program end is released before the C axis is disengaged.

`lathe.live_tool.forward` and `reverse` may use `{rpm}`. The value is rounded to a whole number and written with no decimal point, so 1500 becomes `P1500` on Haas. `stop` takes no placeholders. `feed_mode` is a bare word such as `G98`. When it is non-empty and different from `lathe.feed_mode`, it is written once, before the first live-tool start in the program. Haas `feed_mode` is already `G98`, so the live-tool block adds no extra feed word. `LiveToolOn` writes `forward` or `reverse`. `LiveToolOff` writes `stop`. A post with no `live_tool` block is `post `<id>` has no live tooling`.

`lathe.c_feed` is `per_minute` (the default) or `inverse_time`. `per_minute` writes the IR feed in millimetres per minute. Haas uses Setting 102 to turn that number into a C-axis rate. `inverse_time` writes `G93` before every `G1`, `G2` or `G3` that changes C, with `F = ir_feed / length` (`1 / minutes`). `length` is the straight-line part-frame cut, the same rule as a mill rotary post, not the arc length. A fallback chord uses those part-frame tips: Cartesian on a face, and the unrolled `(x, arc length, z)` point on a cylinder. An indexed move uses the chord after C rotates the tool tip about the spindle. `F` is written on every one of those blocks. The next move that does not change C (`G0`, `G1`, `G2` or `G3`) writes `lathe.feed_mode` (`G94` when that word is empty) and returns `F` to millimetres per minute. Read-back accepts `G93` and that off word. `compare` turns `F * length` back into millimetres per minute and allows 1%. A ZX arc still has no C word, so it does not change C. Fallback arcs are `G1` chords.

At the end of the program, interpolation is cancelled first, then a live tool that is still on is stopped, a clamped brake is released, and a C axis that is still engaged is disengaged. Those lines are written after the footer and before `lathe.home`.

On a lathe post that has `c_axis`, read-back keeps `letter` as the modal angle and returns it on rapid and linear moves. `compare` checks it against `Pose.rotary` with the same tolerance as a rotary word (`max(0.001 deg, one step of the linear decimals)`). `M` lines, `P` and `H` words, and `G28 H0.` are not moves. `G28 H0.` does not change the modal C word, because the axis letter is `C`.

Haas codes, from the Haas code table in `haas-facts.md`:

- `M133` / `M134` / `M135` with `P<rpm>`: live tool forward, reverse, and stop. <https://www.haascnc.com/service/codes-settings.type=mcode.machine=lathe.value=M133.html>
- `M14` / `M15`: main spindle brake clamp and release. Haas NGC Lathe Operator's Manual (96-8910), M-code list.
- `M154` / `M155`: C axis engage and disengage. `G28 H0.` unwinds C. <https://www.haascnc.com/service/service-content/guide-procedures/lathe---m-codes.html> and the NGC manual's G82/G83 live examples.
- C axis, `H` as incremental C, and Setting 102 (the diameter used for C feed): <https://www.haascnc.com/service/online-operator-s-manuals/lathe-operator-s-manual/lathe---options-programming.html>
- `G112` / `G113`: XY to XC interpolation, with `G17`. Under G112, X is a radius, `G98` is required, and the path must not cross the centreline. <https://www.haascnc.com/service/online-operator-s-manuals/lathe-operator-s-manual/lathe---options-programming.html>
- `G107 Y0 C0 R` or `Q`: cylindrical mapping in `G19`. A bare `G107` cancels. Under G107, X stays a diameter. The manual example is `G107 Y0 C0 Q4.` with `X3.5`. <https://www.haascnc.com/service/codes-settings.type=gcode.machine=lathe.value=G107.html>

On Haas NGC the C axis also engages automatically when commanded. CAMpanion writes `M154` explicitly as the manual's live-drilling example does.

`haas_lathe` carries these lines and cites them in `references`. `fanuc_lathe` and `grbl_lathe` leave `c_axis`, `brake`, `live_tool`, `face_interp`, `cylinder_interp` and `y_axis` unset.

### LinuxCNC

`linuxcnc_lathe` has no `face_interp` and no `cylinder_interp`. LinuxCNC has no `G112` or `G107`, so face and cylinder cycles use the writer's X/C fallback. `c_feed` is `inverse_time`: a feed that changes C is `G93`, and the next move that does not change C returns to `G94` (`lathe.feed_mode`).

The live tool is spindle 1. `forward` is `S{rpm} $1` then `M3 $1`. `reverse` is `S{rpm} $1` then `M4 $1`. `stop` is `M5 $1`. `feed_mode` is empty, because the lathe is already in `G94`. `$n` selects spindle n and is not a move.

```json
"c_axis": { "letter": "C", "engage": [], "disengage": [] },
"brake": { "clamp": [], "release": [] },
"live_tool": { "forward": ["S{rpm} $1", "M3 $1"], "reverse": ["S{rpm} $1", "M4 $1"], "stop": ["M5 $1"], "feed_mode": "" },
"y_axis": { "letter": "Y", "face_plane": "G17", "side_plane": "G19", "turn_plane": "G18" },
"c_feed": "inverse_time"
```

`c_axis.engage`, `c_axis.disengage`, `brake.clamp` and `brake.release` are empty. LinuxCNC switches the spindle to a C joint and clamps the brake in HAL, or in a user M-code (`M100`–`M199`). A custom post can fill those lists. Y uses `G17` for an axial tool and `G19` for a radial tool, then `G18` for turning. There is no Y activation code.

- `M3` / `M4` / `M5` with `$n` select spindle n. The live tool is spindle 1. <https://linuxcnc.org/docs/stable/html/gcode/m-code.html>
- `G93` is inverse time. `G94` is units per minute. <https://linuxcnc.org/docs/stable/html/gcode/g-code.html#gcode:g93-g94-g95>
- C-axis switching and the spindle brake are machine-specific (HAL or `M100`–`M199`). <https://linuxcnc.org/docs/stable/html/gcode/m-code.html#mcode:m100-m199>

### Y axis

`lathe.y_axis` is optional:

```json
{ "letter": "Y", "face_plane": "G17", "side_plane": "G19", "turn_plane": "G18" }
```

`letter` is one uppercase ASCII letter. It must not be `X`, `Z`, `I`, `J`, `K`, `F`, `S`, `T`, `M`, `G`, `N`, `O`, `P`, or the C-axis letter. `Y` is allowed. Each plane string is a non-empty bare word.

While Y is in use the writer puts `face_plane` on its own line before the first XY arc, or before the first Y move, of an axial tool (pose axis along Z). A radial tool (pose axis along X) gets `side_plane`. `turn_plane` is written when that work ends: on the next centreline move, and before C-axis disengage. A tool change does the same and parks the modal Y at 0, which is how a Haas turret returns to the centreline. An X or Z step that keeps Y off the centreline does not restore the turning plane. A move that changes Y back to 0 is still a Y move. The Y word is modal. The first centreline 0 is not written. C and Y may not change in one block (`C and Y in one move`). An XY or YZ arc counts as a Y move even when its endpoint Y is unchanged.

X stays on `diameter_mode`. Y is never doubled. Read-back keeps Y modal when `y_axis` is set and forces Y to 0 when it is not. `G17`, `G18` and `G19` select the plane and are not moves.

`haas_lathe` uses the object above. There is no Y activation code. Y axis, G17/G19 live milling planes, no activation code: Haas NGC Lathe Operator's Manual 6.12 / https://www.haascnc.com/service/online-operator-s-manuals/lathe-operator-s-manual/lathe---options-programming.html

`fanuc_lathe` is Fanuc G-code system A. The file is percent-delimited, like the Fanuc mill post: program `O0001`, sequence numbers from `N10` in steps of 10, and a maximum line length of 80. `G21` selects millimetres, `G40` cancels tool-nose compensation and `G80` cancels a canned cycle. `G18` selects the ZX plane. Feed per minute is `G98`. `G99` is left out, because that word is millimetres per revolution and the IR feed is millimetres per minute. `G90` is left out: on this control it is an outer and inner diameter cutting cycle, and `X` and `Z` are already absolute. Constant surface speed writes `G50 S{max_rpm}` and then `G96 S{surface}` (`S` is metres per minute under `G21`). Direct rpm is `G97 S{rpm}`. The spindle words are `M3`, `M4` and `M5`. The tool word is `T{tool2}{offset2}` (`T0101` for tool 1). Arcs are `I` and `K`, and `I` is a radius. The program ends with `M30` and `%`.

`haas_lathe` follows the Haas lathe operator manual and the same percent-delimited shape as the Haas mill post: program `O00001`, sequence numbers from `N10` in steps of 10, and a maximum line length of 80. The safety block is `G21 G18 G40 G80`. The lathe `plane` word writes `G18` again on the next line. Feed per minute is `G98`, written from `feed_mode`. `G99` is left out, because the IR feed is millimetres per minute. Constant surface speed is `G50 S{max_rpm}` then `G96 S{surface}`. Direct rpm is `G97 S{rpm}`. The tool word is `T{tool2}{offset2}`. Home moves X to the machine zero, then Z. The program ends with `M30` and `%`. Live tooling, the C axis, G112 face interpolation and G107 cylinder interpolation use the codes in Mill-turn above, and the post cites them in `references`. Face interpolation writes `G17`, `G112`, then `G113`, `G18`, with X as a radius. Cylinder interpolation writes `G19`, `M15`, `G107 Y0 C0 R{radius}`, then `G107`, `G18`, and X stays a diameter.

`grbl_lathe` is grblHAL in lathe mode. It has no line numbers and no program number. `G7` selects diameter mode, `G18` the ZX plane and `G94` feed per minute. Constant surface speed is `G96 D{max_rpm} S{surface}`. Direct rpm is `G97 S{rpm}`. Tool change is `T{tool} M6`. A manual tool change is expected. There is no home block. Comments are in parentheses, as in the Grbl mill post. The program ends with `M30`.
