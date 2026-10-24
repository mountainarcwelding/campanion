# Changelog

## [Unreleased]

## 0.7.1 — 2026-10-24

Milestone M11 Part B: sub-spindle, part transfer and two-channel lathe programs, plus the LinuxCNC mill-turn output and `live_mill`.

### Added

- LinuxCNC mill-turn: `linuxcnc_lathe` runs the live tool as spindle 1 (`M3 $1`), and `lathe.c_feed` can be `inverse_time` (`G93`) for C moves. (#66)
- `live_mill`: front-face and OD-wrap milling from a drawing, through face or cylinder interpolation, or on the Y axis. (#68)
- Sub-spindle setups: a lathe post can select and deselect a second spindle (Haas G55/G14, then G15/G54), and a lathe job can name a sub block and bind operations to main or sub.
- Part transfer and eject on a lathe. Chuck clamp, spindle sync, a run of either spindle, and a move of the sub-spindle housing. Haas writes G199/G198, M10/M11, M110/M111, M143/M144/M145 and B-axis moves. A job `transfer` hands the part across, with an optional part-off, and `eject` opens the sub chuck.
- Two-channel lathe programs: a post can split one file into channel sections with sync waits, a job can put an operation on channel 0 or 1 and insert a wait, and posting a multichannel program prints a per-segment balance on stderr. `generic_lathe_2ch` is a template.
- Viewer: a two-lane spindle timeline under the playback slider for lathe programs with a sub spindle (chuck, rotation and sync per spindle, a cursor and a one-line state label), and a `sub` tick at the sub housing Z on the section view.
