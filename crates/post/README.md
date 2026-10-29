# campanion-post

RS274/NGC posts. A post definition is JSON (`posts/linuxcnc.json` and the other built-ins). `post_program` writes a cutter-location program. `read_back` and `compare` check that text against the IR.

`decimal_point`, when true, keeps a decimal point on every dimensional word (`X` `Y` `Z`, `I` `J` `K`, `R`, rotary `A`/`B`, and lathe `X`/`Z`): a whole number is `52.` and zero is `0.`. Fanuc and Haas controls read a word written without a decimal point as least-input increments, so `X52` is 0.052 mm and `A90` is 0.09 degrees. Feed (`F`), spindle (`S`), dwell (`P`), line numbers, and tool and program numbers are left alone. LinuxCNC, Mach3, Grbl, and plasma omit the flag and read plain decimals.

## Process cycle

`process_cycle` is `hold` or `per_cut`. Omit it for `hold`.

`hold` is the mill default. A spindle process-on that repeats the speed already running writes nothing, so one `M3` stays in effect across rapids and later operations. The matching `M5` is the process-off after the retract above the stock.

`per_cut` writes every process-on. When any process is still on and the next motion is a rapid (`G0`), the writer emits `thc.off` and that process's off lines before the rapid, then the on lines again before the next feed. Speed and power come from the process-on that is still in effect. A pierce dwell is written only when the IR already has a dwell at that cut. Plasma and `grbl_laser` set `per_cut`, so a torch or beam stays off through G0 travel even when the IR process is `spindle`.

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

## Five-axis 3+2 (five_axis)

`five_axis` is optional. Without it, a tilted tool axis is `post `<id>` has no tilted-plane output`. A program whose tool axis is tilted and that has no five-axis setup is `program has tilted tool axes but no kinematics`.

```json
{
  "mode": "work",
  "decimals": 3,
  "retract": ["G53 G00 Z0."],
  "index": ["G00 {rotaries}"],
  "on": ["G254"],
  "off": ["G255"]
}
```

- `mode` — `work` or `machine`.
- `decimals` — digits on each rotary word. The number style matches the post (`decimal_point`, `trailing_zeros`).
- `retract`, `on`, `off` — lines with no placeholders. `on` and `off` may be empty.
- `index` — every line contains `{rotaries}`. That expands to the rotary words in kinematics order, for example `B30. C45.`. The letter is `RotaryJoint.letter`.

`mode` `work` allows table joints only. A head joint is `post `<id>` work mode needs table rotaries only`. The control applies the rotary-centre offsets, so the post ignores pivots: each written point is the part tip rotated about the origin by the table angles, outer joint last. `haas_umc` uses this with G254. G254 is positioning only: index the rotaries, then turn it on; the first move is X and Y in one block and Z in the next; cancel with G255 before any later rotary move. G53 is not affected. The kinematics file must match the machine. This output has not been proven on a machine.

`mode` `machine` writes the gauge point from the kinematics inverse at tool length 0, so the pivots are in the coordinates and the tool length stays with G43. `linuxcnc_5ax` uses this. There is no TCP. The kinematics file's pivots must be measured from the G54 origin. `on` and `off` are empty.

The writer starts at rotary angles `[0, 0]` and tool axis +Z. Before the first motion of a toolpath, if that pose's axis differs by more than 1e-9 or this is the first toolpath of a five-axis program, it writes `off` when a frame is already active, then `retract`, solves `Kinematics::choose` for that pose at tool length 0, writes `index`, writes `on`, and marks the frame active. The first motion after an index is two rapids: X and Y, then Z. Later poses in that toolpath must keep the axis chosen at the start. A change is `toolpath N changes tool axis inside a 3+2 move`. At program end, an active frame writes `off`, then `retract`, then `index` with every angle 0, before `program_end`.

An arc that is cardinal in the written frame (XY, ZX, or YZ) stays an arc. Any other arc is chorded at the arc tolerance. With a zero tolerance that arc is `arc is not in a cardinal plane after the 3+2 rotation`. Read-back does not apply to a five-axis post.

## Lathe posts

`lathe` is optional. A program with a turning setup is a lathe program: its poses are part-frame. `tip.x()` is a radius in millimetres and `tip.z()` runs along the spindle. `tip.y()` is 0 unless the head carries Y and `lathe.y_axis` is set. The post applies diameter mode. The IR does not.

A turning program posted without `lathe` is rejected: `post `<id>` is not a lathe post`. A lathe post given a mill program is rejected: `lathe post `<id>` needs a turning program`. A lathe program must not carry a rotary setup. A pose with `|y|` above `1e-6` is rejected outside face and cylinder interpolation (see Mill-turn).

After the header the post writes `mode_lines`, then `plane` (`G18`), then `feed_mode` (`G94` on LinuxCNC and grblHAL, `G98` on Fanuc and Haas). Those three are written once. LinuxCNC and grblHAL select diameter mode with `G7`. Fanuc and Haas leave `mode_lines` empty: diameter mode is the control default.

Outside face and cylinder interpolation, motion words are `X` and `Z`, and `Y` when `lathe.y_axis` is set (see [Y axis](#y-axis)); Y is never doubled. Without `y_axis`, Y is never written there. When `diameter_mode` is true, `X` is `2 * radius`. When it is false, `X` is the radius. Rapids are `G0`. Feeds are `G1` with `F`, in the same millimetres-per-minute number the IR stores. The linear decimals are the post's `decimals.linear`. While a C axis is engaged, `G0` and `G1` also write the C word after Z. See Mill-turn below.

A synchronised thread move is written in full, never as a modal continuation: `G33 X.. Z.. K..` on LinuxCNC and grblHAL, `G32 X.. Z.. F..` on Fanuc and Haas. The lead uses the linear decimals and the post's decimal-point rule. The motion code is not left modal, so the next `G0` or `G1` is written explicitly. When the lead word is `F`, the modal feed is cleared and the next feed move writes `F` again. Threading needs a binding whose speed is direct rpm; constant surface speed, or a missing speed, is rejected with `threading needs a fixed rpm binding`. A mill post rejects the move. A lathe post with no `thread` block rejects it too. Read-back turns those blocks back into a thread motion, so `compare` checks the end point and the lead.

Outside face and cylinder interpolation (and Y-axis milling), arcs are in the ZX plane (`Plane::Zx`). In the IR picture, viewed from +Y with +Z to the right and +X up, counter-clockwise is `ArcSense::Ccw`. With `arc_flip` false that sense is `G3` and clockwise is `G2`, the same ZX sense this crate uses for mill arcs. `arc_flip` true swaps `G2` and `G3` for a front turret or an X-down drawing. `ijk` mode writes `I` and `K`. `I` is the X offset of the centre from the block start and is always a radius, even in diameter mode. `K` is the Z offset. `r` mode writes the geometric radius `R`. `lines` mode chords the arc to the arc tolerance. Read-back of a lathe post returns part-frame positions (`x` radius, `y` 0, `z`) and maps `G2`/`G3` back to the IR sense, so `compare` checks the arc in the ZX plane.

A toolpath whose binding has `speed` writes that speed on `ProcessOn` for the spindle, then `spindle_cw` or, when that spindle is reversed, `spindle_ccw`. Constant surface speed uses the `css` lines (`{surface}` metres per minute, `{max_rpm}`). Direct rpm uses the `rpm` lines (`{rpm}`). A whole number is an integer. Any other number uses the feed decimals. A binding with no speed uses the process-on speed parameter and the rpm lines. If neither is set, the post rejects the program. `ProcessOff` for the spindle writes `spindle_off`.

Tool changes use `lathe.tool_change` instead of the mill `tool_change` lines. `{tool}` is the tool id. `{tool2}` and `{offset2}` are that id in at least two digits. Coolant, comments, dwell, line numbers and the line-length limit are unchanged. `home` is written after the footer and before `program_end`. It may be empty. If face or cylinder interpolation is still on, the live tool is still running, the brake is still clamped, or the C axis is still engaged, the post closes them before `home`: interpolation off, live-tool stop, brake release, then C-axis disengage. LinuxCNC uses `G53 G0 X0 Z0` for that machine-coordinate retract. Haas uses `G53 G0 X0` and then `G53 G0 Z0`. Fanuc uses `G28 U0 W0`: `U` and `W` are incremental, and zero keeps the intermediate point at the current position so the control returns straight to the reference point. grblHAL writes no home lines. `read_back` does not treat `G53` or `G28` as a part move. On a lathe post, `U` and `W` are accepted only on a `G28` block. `G50` (the spindle-speed cap) and `G98` (Fanuc and Haas feed per minute) are not moves.

### Sub-spindle

`lathe.sub_spindle` is optional.

```json
{ "spindle": "sub", "select": ["G55", "G14"], "deselect": ["G15", "G54"], "mirror_z": false }
```

`spindle` is the IR spindle name these lines serve. `select` and `deselect` are bare lines with no placeholders, the same rule as `spindle_off`. `mirror_z` defaults to false.

The writer tracks the active spindle and starts at `main`. Before the first line of a toolpath whose binding names a different spindle, it stops the previous spindle when that spindle is still on, writes `deselect` when leaving the configured sub, and writes `select` when entering it. Those lines come before the toolpath comment and the tool change. If the program ends on the sub, `deselect` is written after the mill-turn shutdown and before `lathe.home`. A binding that is neither `main` nor `sub_spindle.spindle` is `post `<id>` has no spindle `<name>``.

`mirror_z` true negates Z on that spindle's moves, negates arc centre K, and swaps G2 and G3 on a ZX or YZ arc. `false` writes Z unchanged, which is what a control that mirrors under its select code expects. Read-back treats each select and deselect line as a pass-through modal line (the same class as `G54`) and, when `mirror_z` is true, un-mirrors Z, K and the arc sense so `compare` sees the part frame.

While the active spindle is the sub, `CAxis`, `SpindleBrake`, `LiveToolOn`, `FaceInterpolation` and `CylinderInterpolation` are `post `<id>`: live tooling and C axis on spindle `<name>` are not supported`. Speed lines (the G50 clamp, G96/G97, and M3/M4) are unchanged. They use the binding speed and that spindle's reversed flag.

`haas_lathe` selects with `G55` then `G14` and deselects with `G15` then `G54`. `mirror_z` is false: G14 mirrors Z on the control (Haas NGC lathe manual 96-8910 §7.1.8; G14 secondary spindle swap; cancelled by G15/M30/reset). G55 is the secondary-spindle work offset in the §7.1.72 example. `fanuc_lathe`, `linuxcnc_lathe` and `grbl_lathe` leave `sub_spindle` unset.

### Part transfer

`lathe.transfer` is optional. Without it, `ChuckClamp`, `SpindleSync`, `SpindleRun` and `SpindleMove` are `post `<id>` has no part transfer`.

```json
{
  "spindle": "sub",
  "clamp_main": ["M10"],
  "unclamp_main": ["M11"],
  "clamp_sub": ["M110"],
  "unclamp_sub": ["M111"],
  "sub_cw": ["M143 P{rpm}"],
  "sub_ccw": ["M144 P{rpm}"],
  "sub_stop": ["M145"],
  "sync_on": ["G199"],
  "sync_on_phase": ["G199 R{phase}"],
  "sync_off": ["G198"],
  "move_rapid": ["G00 B{z}"],
  "move_feed": ["G01 B{z} F{feed}"],
  "move_machine": ["G00 G53 B{z}"]
}
```

`spindle` is the IR name served by the secondary lines. `clamp_main` and `unclamp_main` serve spindle `main`. The other chuck, run and move lines serve `transfer.spindle`. A `ChuckClamp`, `SpindleMove` or `SpindleRun` that names any other spindle is `post `<id>` has no spindle `<name>``. The missing-transfer error wins when the post has no `transfer` block.

`{rpm}` on `sub_cw` and `sub_ccw`, `{phase}` on `sync_on_phase`, `{z}` on the move lines and `{feed}` on `move_feed` use the lathe number format: a whole number is an integer, and any other number uses the feed decimals. `{z}` may be negative. An empty list writes nothing. `SpindleRun` with `rpm` 0 writes `sub_stop` on the secondary spindle and `spindle_off` on `main`. A positive `rpm` on `main` writes the `rpm` lines (`G97 S{rpm}` on Haas) and then `spindle_cw` or `spindle_ccw` from the event's `reverse` flag, not from the binding. `SpindleSync` with `engage` true and a phase writes `sync_on_phase`. Without a phase it writes `sync_on`. `engage` false writes `sync_off`. `SpindleMove` with `machine` true writes `move_machine` even when a feed is set. A feed and `machine` false writes `move_feed`. Otherwise the move is `move_rapid`.

Read-back accepts these lines as pass-through. A housing move is not a part-frame point. `G53` on that move still does not change the modal motion. A `G00` or `G01` housing move does, so the next part rapid or feed stays in step with the reader.

`haas_lathe` uses the object above. M10/M11 are the main chuck, M110/M111 the secondary chuck (the spring ejector acts when those jaws open), M143/M144/M145 the secondary spindle with the speed on P, G199/G198 synchronous control with an optional R phase, and B the secondary housing. Those codes are cited in `references` from Haas NGC lathe manual 96-8910 §8.1.6, §8.1.35, §8.1.44, §7.1.71-72 and §6.34. `fanuc_lathe`, `linuxcnc_lathe` and `grbl_lathe` leave `transfer` unset. While the sub is selected (`sub_spindle.select`, Haas `G14`), a `SpindleRun` on the sub uses the main spindle lines (`M3`/`M4`/`M5`) instead, because Haas alarms on M143/M144/M145 under G14 (manual 96-8910 §7.1.8).

### Channels

`lathe.channels` is optional.

```json
{ "count": 2, "start": ["(CHANNEL {channel})"], "end": [], "wait": ["M{code}"], "wait_base": 100 }
```

`count` is at least 2. A smaller count fails to load with `channels.count must be at least 2`. `start` may use `{channel}`, written as the 1-based channel number. `wait` may use `{code}`, which is `wait_base` plus the sync id. `end` takes no placeholders.

A binding whose channel is not 0 needs this block. Without it the post is rejected: `post `<id>` is single-channel; the program uses channel N`. A channel at or above `count` is `post `<id>` has no channel N`.

When the block is present and the program uses another channel, the header and the lathe preamble (`mode_lines`, plane, feed mode) are written once. Each channel, from 0 up to `count - 1`, then gets its `start` lines, that channel's toolpaths in program order, and its `end` lines. Modal state, the position and the active spindle reset at each channel, so the first move of a channel writes its axes and motion word again. Line numbers continue. The footer, mill-turn shutdown, home and `program_end` are written once, after the last channel.

`Event::Sync` inside a channel section writes the `wait` lines. A program whose bindings are all channel 0 keeps today's sync comment, `(sync id)`, and does not open channel sections, even when the post defines `channels`.

Every channel that contains at least one toolpath must list the same sync ids in the same order. The first mismatch is `channel sync mismatch: channel A waits for [..], channel B for [..]`.

`channel_balance` times each stretch between syncs with the verify time model: feed moves are `length / feed * 60` seconds, dwells add seconds, and a rapid with no feed is free. The cursor carries across a sync. `idle_s` is the slowest channel in that segment minus this channel.

`generic_lathe_2ch` is a template copied from the Fanuc-style lathe post with the block above. Two-path controls use machine-specific wait M-codes and program layout; check the control manual. `haas_lathe` and `linuxcnc_lathe` have no `channels` block: they are single-channel.

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

`haas_lathe` carries these lines and cites them in `references`. `fanuc_lathe` and `grbl_lathe` leave `c_axis`, `brake`, `live_tool`, `face_interp`, `cylinder_interp`, `y_axis`, `sub_spindle` and `b_head` unset. `linuxcnc_lathe` also leaves `sub_spindle` unset. `haas_lathe` and `fanuc_lathe` leave `b_head` unset. No Haas lathe post has a tilting B head: on `haas_lathe` the letter B is the sub-spindle housing (`G00 B{z}`).

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

### Tilting B head

`lathe.b_head` is optional:

```json
{ "index": ["G0 B{b}"], "mode": "pivot", "pivot_length": 0 }
```

`index` is the lines for `Event::HeadIndex`. `{b}` is the angle in degrees, written with the signed lathe number format (a whole number is an integer, so 30 is `B30` and −30 is `B-30`). `mode` is `tip` or `pivot`. In `tip` mode, X and Z are the tool tip and the control compensates for the pivot. In `pivot` mode, while B is not 0, the post adds `pivot_length * ((cos B − 1), sin B)` to `(radius, Z)` of every point. Diameter mode doubles the radius part, the same way it doubles any other X. Read-back reverses that offset after diameter halving and Z un-mirroring. Arc `I` and `K` stay tip-frame offsets: the shift is constant while B is held, so it cancels. A `G0` or `G1` in the index lines updates the writer's modal motion. A post with no `b_head` block is `post `<id>` has no tilting B head`.

`linuxcnc_lathe` uses `mode` `pivot` and `index` `G0 B{b}`. Its `pivot_length` is 0. The B joint and pivot length are machine-specific; LinuxCNC has no TCP here, so pivot length must be set in the post. No Haas lathe post has a tilting B head.

### Y axis

`lathe.y_axis` is optional:

```json
{ "letter": "Y", "face_plane": "G17", "side_plane": "G19", "turn_plane": "G18" }
```

`letter` is one uppercase ASCII letter. It must not be `X`, `Z`, `I`, `J`, `K`, `F`, `S`, `T`, `M`, `G`, `N`, `O`, `P`, or the C-axis letter. `Y` is allowed. Each plane string is a non-empty bare word.

While Y is in use the writer puts `face_plane` on its own line before the first XY arc, or before the first Y move, of an axial tool (pose axis along Z). A radial tool (pose axis along X) gets `side_plane`. `turn_plane` is written when that work ends: on the next centreline move, and before C-axis disengage. A tool change does the same and parks the modal Y at 0, which is how a Haas turret returns to the centreline. An X or Z step that keeps Y off the centreline does not restore the turning plane. A move that changes Y back to 0 is still a Y move. The Y word is modal. The first centreline 0 is not written. C and Y may not change in one block (`C and Y in one move`). An XY or YZ arc counts as a Y move even when its endpoint Y is unchanged.

X stays on `diameter_mode`. Y is never doubled. Read-back keeps Y modal when `y_axis` is set and forces Y to 0 when it is not. `G17`, `G18` and `G19` select the plane and are not moves.

`haas_lathe` uses the object above. There is no Y activation code. Y axis, G17/G19 live milling planes, no activation code: Haas NGC Lathe Operator's Manual 6.12 / https://www.haascnc.com/service/online-operator-s-manuals/lathe-operator-s-manual/lathe---options-programming.html

`generic_lathe_2ch` uses the same words as `fanuc_lathe` and adds `channels`. It is a template: two-path controls use machine-specific wait M-codes and program layout; check the control manual. A single-channel program posted with it matches `fanuc_lathe`.

`fanuc_lathe` is Fanuc G-code system A. The file is percent-delimited, like the Fanuc mill post: program `O0001`, sequence numbers from `N10` in steps of 10, and a maximum line length of 80. `G21` selects millimetres, `G40` cancels tool-nose compensation and `G80` cancels a canned cycle. `G18` selects the ZX plane. Feed per minute is `G98`. `G99` is left out, because that word is millimetres per revolution and the IR feed is millimetres per minute. `G90` is left out: on this control it is an outer and inner diameter cutting cycle, and `X` and `Z` are already absolute. Constant surface speed writes `G50 S{max_rpm}` and then `G96 S{surface}` (`S` is metres per minute under `G21`). Direct rpm is `G97 S{rpm}`. The spindle words are `M3`, `M4` and `M5`. The tool word is `T{tool2}{offset2}` (`T0101` for tool 1). Arcs are `I` and `K`, and `I` is a radius. The program ends with `M30` and `%`.

`haas_lathe` follows the Haas lathe operator manual and the same percent-delimited shape as the Haas mill post: program `O00001`, sequence numbers from `N10` in steps of 10, and a maximum line length of 80. The safety block is `G21 G18 G40 G80`. The lathe `plane` word writes `G18` again on the next line. Feed per minute is `G98`, written from `feed_mode`. `G99` is left out, because the IR feed is millimetres per minute. Constant surface speed is `G50 S{max_rpm}` then `G96 S{surface}`. Direct rpm is `G97 S{rpm}`. The tool word is `T{tool2}{offset2}`. Home moves X to the machine zero, then Z. The program ends with `M30` and `%`. Live tooling, the C axis, G112 face interpolation and G107 cylinder interpolation use the codes in Mill-turn above, and the post cites them in `references`. Sub-spindle select is `G55` then `G14`; `G15` then `G54` returns to the main. `mirror_z` is false because G14 mirrors Z on the control. Part transfer uses M10/M11, M110/M111, M143/M144/M145, G199/G198 and B, cited in `references`. No Haas lathe post has a tilting B head; that B word moves the sub-spindle housing. Face interpolation writes `G17`, `G112`, then `G113`, `G18`, with X as a radius. Cylinder interpolation writes `G19`, `M15`, `G107 Y0 C0 R{radius}`, then `G107`, `G18`, and X stays a diameter.

`grbl_lathe` is grblHAL in lathe mode. It has no line numbers and no program number. `G7` selects diameter mode, `G18` the ZX plane and `G94` feed per minute. Constant surface speed is `G96 D{max_rpm} S{surface}`. Direct rpm is `G97 S{rpm}`. Tool change is `T{tool} M6`. A manual tool change is expected. There is no home block. Comments are in parentheses, as in the Grbl mill post. The program ends with `M30`.
