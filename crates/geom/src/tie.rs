//! Float-key choices that stay put when only libm noise changes.
//!
//! A minimum, maximum, or sort over raw `f64` keys follows the platform math library: `sin`,
//! `cos`, `atan2`, and `sqrt` differ by a few ulps, and a symmetric part makes two candidates
//! look equal until that noise picks a winner. Keys within
//! `1e-9 * max(1, |a|, |b|)` are the same key. The winner is then the lexicographically smaller
//! position after rounding each coordinate to 1e-6 mm, and after that the original index.
//! A key that is clearly smaller or larger still wins on its own.

use std::cmp::Ordering;

/// Relative band for a float ordering key. The absolute floor is 1e-9, in the key's own units.
pub const FLOAT_KEY_REL: f64 = 1e-9;

/// Millimetres represented by one step of the position tie-break.
const COORD_QUANTUM_MM: f64 = 1e-6;

/// `true` when `a` and `b` are the same ordering key, including exact equality.
pub fn float_keys_equal(a: f64, b: f64) -> bool {
    if a == b {
        return true;
    }
    if !a.is_finite() || !b.is_finite() {
        return false;
    }
    let scale = 1.0_f64.max(a.abs()).max(b.abs());
    (a - b).abs() <= FLOAT_KEY_REL * scale
}

/// Order two float keys. Values inside [`float_keys_equal`] compare equal.
pub fn cmp_float_key(a: f64, b: f64) -> Ordering {
    if float_keys_equal(a, b) { Ordering::Equal } else { a.total_cmp(&b) }
}

/// Order two positions after rounding each coordinate to 1e-6 mm.
pub fn cmp_xy_rounded(a: (f64, f64), b: (f64, f64)) -> Ordering {
    round_mm(a.0).cmp(&round_mm(b.0)).then(round_mm(a.1).cmp(&round_mm(b.1)))
}

/// `true` when `(key, xy, index)` should replace the current minimum.
pub fn min_beats(key: f64, xy: (f64, f64), index: usize, best_key: f64, best_xy: (f64, f64), best_index: usize) -> bool {
    cmp_float_key(key, best_key).then(cmp_xy_rounded(xy, best_xy)).then(index.cmp(&best_index)) == Ordering::Less
}

/// `true` when `(key, xy, index)` should replace the current maximum.
///
/// A tied key still prefers the smaller rounded position, then the smaller index.
pub fn max_beats(key: f64, xy: (f64, f64), index: usize, best_key: f64, best_xy: (f64, f64), best_index: usize) -> bool {
    cmp_float_key(best_key, key).then(cmp_xy_rounded(xy, best_xy)).then(index.cmp(&best_index)) == Ordering::Less
}

/// Index of the minimum `(key, x, y)`. Empty input returns `None`.
pub fn select_min_by_key(candidates: &[(f64, f64, f64)]) -> Option<usize> {
    select_by_key(candidates, true)
}

/// Index of the maximum `(key, x, y)`. Empty input returns `None`.
pub fn select_max_by_key(candidates: &[(f64, f64, f64)]) -> Option<usize> {
    select_by_key(candidates, false)
}

fn select_by_key(candidates: &[(f64, f64, f64)], minimum: bool) -> Option<usize> {
    let mut best: Option<usize> = None;
    for (index, &(key, x, y)) in candidates.iter().enumerate() {
        let take = match best {
            None => true,
            Some(best_index) => {
                let Some(&(best_key, best_x, best_y)) = candidates.get(best_index) else {
                    continue;
                };
                if minimum {
                    min_beats(key, (x, y), index, best_key, (best_x, best_y), best_index)
                } else {
                    max_beats(key, (x, y), index, best_key, (best_x, best_y), best_index)
                }
            }
        };
        if take {
            best = Some(index);
        }
    }
    best
}

fn round_mm(value: f64) -> i64 {
    if !value.is_finite() {
        return if value.is_nan() || value.is_sign_positive() { i64::MAX } else { i64::MIN };
    }
    let scaled = (value / COORD_QUANTUM_MM).round();
    if scaled >= i64::MAX as f64 {
        i64::MAX
    } else if scaled <= i64::MIN as f64 {
        i64::MIN
    } else {
        scaled as i64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bump_ulps(value: f64, ulps: i32) -> f64 {
        let mut out = value;
        for _ in 0..ulps.unsigned_abs() {
            out = if ulps > 0 { out.next_up() } else { out.next_down() };
        }
        out
    }

    #[test]
    fn clear_gap_ignores_position_and_a_few_ulps() {
        let base = [(0.0, 40.0, 40.0), (3.0, -40.0, -40.0), (6.0, 0.0, -1.0)];
        assert_eq!(select_min_by_key(&base), Some(0));
        assert_eq!(select_max_by_key(&base), Some(2));
        for ulps in -8..=8 {
            let noisy: Vec<(f64, f64, f64)> =
                base.iter().map(|&(key, x, y)| (bump_ulps(key, ulps), bump_ulps(x, ulps), bump_ulps(y, ulps))).collect();
            assert_eq!(select_min_by_key(&noisy), Some(0), "ulps {ulps}");
            assert_eq!(select_max_by_key(&noisy), Some(2), "ulps {ulps}");
        }
    }

    #[test]
    fn mirrored_ties_follow_rounded_position_under_key_noise() {
        // Four quadrant copies. Keys match; the low-x, then low-y, candidate wins.
        let at = |key: f64| {
            vec![(bump_ulps(key, 6), -9.06, -17.42), (bump_ulps(key, -5), -17.42, -9.06), (bump_ulps(key, 3), 9.06, 17.42), (key, 17.42, 9.06)]
        };
        let winner = select_min_by_key(&at(27.5)).expect("winner");
        assert_eq!(winner, 1);
        for ulps in -8..=8 {
            assert_eq!(select_min_by_key(&at(bump_ulps(27.5, ulps))), Some(winner), "ulps {ulps}");
            assert_eq!(select_max_by_key(&at(bump_ulps(27.5, ulps))), Some(winner), "ulps {ulps}");
        }
    }

    #[test]
    fn empty_selection_is_none_and_equal_rounded_points_keep_the_first_index() {
        assert_eq!(select_min_by_key(&[]), None);
        let tied = [(1.0, 0.0, 0.0), (bump_ulps(1.0, 4), bump_ulps(0.0, 3), bump_ulps(0.0, -2))];
        assert_eq!(select_min_by_key(&tied), Some(0));
        assert_eq!(select_max_by_key(&tied), Some(0));
    }

    #[test]
    fn proptest_clear_winner_survives_ulp_noise() {
        // Inline so the dev-dependency stays out of the type of this module when tests are off.
        run_clear_winner_property();
    }

    fn run_clear_winner_property() {
        use proptest::prelude::*;
        let mut runner = proptest::test_runner::TestRunner::new(ProptestConfig::with_cases(64));
        let strategy = (prop::collection::vec((-40.0..40.0f64, -40.0..40.0f64), 2..6), -8i32..=8);
        runner
            .run(&strategy, |(coords, ulps)| {
                let base: Vec<(f64, f64, f64)> = coords.iter().enumerate().map(|(index, &(x, y))| (index as f64 * 3.0, x, y)).collect();
                let noisy: Vec<(f64, f64, f64)> =
                    base.iter().map(|&(key, x, y)| (bump_ulps(key, ulps), bump_ulps(x, ulps), bump_ulps(y, ulps))).collect();
                prop_assert_eq!(select_min_by_key(&base), Some(0));
                prop_assert_eq!(select_min_by_key(&noisy), select_min_by_key(&base));
                prop_assert_eq!(select_max_by_key(&noisy), select_max_by_key(&base));
                Ok(())
            })
            .expect("clear winner property");
    }
}
