# Changelog

## [Unreleased]

### Fixed

- Mill `drill` no longer rapids to the hole bottom. Each hole is now a rapid to clearance and to the R plane (job retract height), a plunge feed to depth (pecking when `step_down` is less than `depth`), an optional `dwell`, and a rapid out. Every release from v0.1.0 to v0.8.0 rapided to depth; do not run mill drill output from those versions.

## 0.8.0 — 2026-10-24

Milestone M11 Part C, items 8–10: rotary kinematics as data, indexed 3+2 tilted work planes with Haas and LinuxCNC posts, and angled turning with a B head.

### Added

- Indexed 3+2 posts: `haas_umc` writes Haas G254/G255 dynamic work offsets, and `linuxcnc_5ax` writes machine coordinates from the kinematics inverse.
- Indexed 3+2: a job can name a kinematics file, a solution policy and a tilted work plane. Profile, pocket, drill, engrave, face and chamfer are planned in that plane and mapped into the part frame, and the IR program stores the five-axis setup.
- Angled turning and boring with a B head: a lathe job can set `head.b_axis` and `b_deg` on face, rough, finish, groove and part, and a lathe post can index B.
- Rotary kinematics as data: joint definitions, validation, forward kinematics from joint angles to a tool pose, inverse kinematics, and solution choice.

## 0.7.1 — 2026-10-24

Milestone M11 Part B: sub-spindle, part transfer and two-channel lathe programs, plus the LinuxCNC mill-turn output and `live_mill`.

### Added

- LinuxCNC mill-turn: `linuxcnc_lathe` runs the live tool as spindle 1 (`M3 $1`), and `lathe.c_feed` can be `inverse_time` (`G93`) for C moves. (#66)
- `live_mill`: front-face and OD-wrap milling from a drawing, through face or cylinder interpolation, or on the Y axis. (#68)
- Sub-spindle setups: a lathe post can select and deselect a second spindle (Haas G55/G14, then G15/G54), and a lathe job can name a sub block and bind operations to main or sub.
- Part transfer and eject on a lathe. Chuck clamp, spindle sync, a run of either spindle, and a move of the sub-spindle housing. Haas writes G199/G198, M10/M11, M110/M111, M143/M144/M145 and B-axis moves. A job `transfer` hands the part across, with an optional part-off, and `eject` opens the sub chuck.
- Two-channel lathe programs: a post can split one file into channel sections with sync waits, a job can put an operation on channel 0 or 1 and insert a wait, and posting a multichannel program prints a per-segment balance on stderr. `generic_lathe_2ch` is a template.
- Viewer: a two-lane spindle timeline under the playback slider for lathe programs with a sub spindle (chuck, rotation and sync per spindle, a cursor and a one-line state label), and a `sub` tick at the sub housing Z on the section view.
