use core::{
    cmp::{Ordering, PartialOrd},
    ops::{Add, Neg, Sub},
};

/// This represents a duration based on the UWB device clock, which has a resolution of roughly 15
/// picoseconds.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Duration(i64);

impl Duration {
    /// Returns the duration in clock times, which have a unit duration of 1 / (128 * 499.2*10^6) seconds.
    pub fn as_ticks(&self) -> i64 {
        self.0
    }

    /// Converts a raw duration value to picoseconds.
    pub fn as_picos(&self) -> i64 {
        self.0 * 10000000 / 4992 / 128
    }

    pub fn from_picos(picoseconds: i64) -> Self {
        Duration(picoseconds * 4992 * 128 / 10000000)
    }

    /// Returns a duration corresponding to the time it takes for light to travel the given distance in meters.
    pub fn from_meters(meters: f64) -> Self {
        let picoseconds = meters / 0.000_299_792_458;
        Duration::from_picos(picoseconds as i64)
    }

    /// Returns the distance in meters that light travels in this duration.
    pub fn as_meters(&self) -> f64 {
        let picoseconds = self.as_picos();
        picoseconds as f64 * 0.000_299_792_458
    }
}

impl Neg for Duration {
    type Output = Duration;

    fn neg(self) -> Self::Output {
        Duration(-self.0)
    }
}

impl defmt::Format for Duration {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Duration({})", self.0);
    }
}

/// This represents a timestamp from the UWB device, which has a resolution of roughly 15
/// picoseconds, but rolls over about every 17.2 seconds.
///
/// Rollovers are accounted for in all operations, including subtraction and comparison. Operations
/// involving timestamps that are more than 8.6 seconds apart may yield incorrect results.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Timestamp(u64);

impl Timestamp {
    /// Returns the timestamp as a 40-bit clock value, in units of 1 / (128 * 499.2*10^6) seconds.
    pub fn as_ticks(&self) -> u64 {
        self.0
    }

    /// Creates a timestamp from a 40-bit clock value, in units of 1 / (128 * 499.2*10^6) seconds.
    pub fn from_ticks(raw: u64) -> Self {
        Timestamp(raw)
    }
}

impl defmt::Format for Timestamp {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Timestamp({})", self.0);
    }
}

impl Sub for Timestamp {
    type Output = Duration;

    fn sub(self, rhs: Self) -> Self::Output {
        // for now, assume this timestamp is after the other
        let d = if rhs.0 <= self.0 {
            self.0 - rhs.0
        } else {
            // rollover
            self.0 + (1 << 40) - rhs.0
        };

        // now, consider whether this timestamp is actually before the other timestamp
        if d < (1 << 39) {
            // this timestamp is after the other timestamp
            Duration(d as i64)
        } else {
            // this timestamp is before the other timestamp
            Duration((d as i64) - (1 << 40))
        }
    }
}

impl Add<Duration> for Timestamp {
    type Output = Timestamp;

    fn add(self, rhs: Duration) -> Self::Output {
        Timestamp((self.0 as i64 + rhs.0) as u64 & ((1u64 << 40) - 1))
    }
}

impl Sub<Duration> for Timestamp {
    type Output = Timestamp;

    fn sub(self, rhs: Duration) -> Self::Output {
        Timestamp((self.0 as i64 - rhs.0) as u64 & ((1u64 << 40) - 1))
    }
}

impl PartialOrd for Timestamp {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let d = (*self - *other).as_ticks();
        if d < 0 {
            Some(Ordering::Less)
        } else if d > 0 {
            Some(Ordering::Greater)
        } else {
            Some(Ordering::Equal)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Duration ─────────────────────────────────────────────────────────

    #[test]
    fn duration_neg() {
        assert_eq!((-Duration(0)).as_ticks(), 0);
        assert_eq!((-Duration(42)).as_ticks(), -42);
        assert_eq!((-Duration(-42)).as_ticks(), 42);
    }

    #[test]
    fn duration_as_ticks() {
        assert_eq!(Duration(0).as_ticks(), 0);
        assert_eq!(Duration(42).as_ticks(), 42);
        assert_eq!(Duration(-42).as_ticks(), -42);
    }

    #[test]
    fn duration_as_picos_known_values() {
        assert_eq!(Duration(0).as_picos(), 0);
        // 1 tick * 10_000_000 / 4992 / 128 = 10_000_000 / 4992 / 128 = 2003 / 128 = 15
        assert_eq!(Duration(1).as_picos(), 15);
        assert_eq!(Duration(-1).as_picos(), -15);
        // 638976 == 4992 * 128 ticks is exactly 10_000_000 ps
        assert_eq!(Duration(638976).as_picos(), 10_000_000);
        assert_eq!(Duration(-638976).as_picos(), -10_000_000);
    }

    #[test]
    fn duration_from_picos_known_values() {
        assert_eq!(Duration::from_picos(0).as_ticks(), 0);
        // Values below one-tick resolution (~15.65 ps) quantize to zero
        assert_eq!(Duration::from_picos(1).as_ticks(), 0);
        assert_eq!(Duration::from_picos(15).as_ticks(), 0);
        // 16 * 638976 / 10_000_000 = 1 (just above one-tick threshold)
        assert_eq!(Duration::from_picos(16).as_ticks(), 1);
        assert_eq!(Duration::from_picos(-16).as_ticks(), -1);
        // 10_000_000 * 638976 / 10_000_000 = 638976
        assert_eq!(Duration::from_picos(10_000_000).as_ticks(), 638976);
        assert_eq!(Duration::from_picos(-10_000_000).as_ticks(), -638976);
    }

    #[test]
    fn duration_picos_round_trip() {
        // Multiples of the tick period survive the round-trip exactly.
        assert_eq!(Duration::from_picos(10_000_000).as_picos(), 10_000_000);
        assert_eq!(Duration::from_picos(-10_000_000).as_picos(), -10_000_000);
        assert_eq!(Duration::from_picos(100_000_000).as_picos(), 100_000_000);
    }

    #[test]
    fn duration_from_meters_zero() {
        assert_eq!(Duration::from_meters(0.0).as_ticks(), 0);
    }

    #[test]
    fn duration_from_meters_one_meter() {
        // 1.0 / 0.000_299_792_458 ≈ 3335.6 ps → 3335 * 638976 / 10_000_000 = 213 ticks
        assert_eq!(Duration::from_meters(1.0).as_ticks(), 213);
    }

    #[test]
    fn duration_as_meters_round_trip() {
        // Integer quantization introduces sub-millimeter error at 100 m.
        let d = Duration::from_meters(100.0);
        let m = d.as_meters();
        assert!((m - 100.0).abs() < 0.01, "expected ~100 m, got {m} m");
    }

    #[test]
    fn duration_as_meters_negative() {
        let d = Duration::from_meters(-50.0);
        let m = d.as_meters();
        assert!((m + 50.0).abs() < 0.01, "expected ~-50 m, got {m} m");
    }

    // ── Timestamp ────────────────────────────────────────────────────────

    #[test]
    fn timestamp_from_and_as_ticks() {
        assert_eq!(Timestamp::from_ticks(0).as_ticks(), 0);
        assert_eq!(Timestamp::from_ticks(12345).as_ticks(), 12345);
        assert_eq!(
            Timestamp::from_ticks((1 << 40) - 1).as_ticks(),
            (1 << 40) - 1
        );
    }

    #[test]
    fn timestamp_sub_self_after_rhs() {
        let a = Timestamp::from_ticks(1000);
        let b = Timestamp::from_ticks(100);
        assert_eq!((a - b).as_ticks(), 900);
    }

    #[test]
    fn timestamp_sub_equal() {
        let a = Timestamp::from_ticks(500);
        assert_eq!((a - a).as_ticks(), 0);
    }

    #[test]
    fn timestamp_sub_self_before_rhs() {
        let a = Timestamp::from_ticks(100);
        let b = Timestamp::from_ticks(1000);
        assert_eq!((a - b).as_ticks(), -900);
    }

    #[test]
    fn timestamp_sub_rollover_self_just_after() {
        // self wrapped around; its raw value is smaller but it is temporally later.
        // elapsed = 100 + 100 = 200 ticks
        let a = Timestamp::from_ticks(100);
        let b = Timestamp::from_ticks((1 << 40) - 100);
        assert_eq!((a - b).as_ticks(), 200);
    }

    #[test]
    fn timestamp_sub_rollover_self_just_before() {
        // rhs wrapped around; self's raw value is larger but it is temporally earlier.
        let a = Timestamp::from_ticks((1 << 40) - 100);
        let b = Timestamp::from_ticks(100);
        assert_eq!((a - b).as_ticks(), -200);
    }

    #[test]
    fn timestamp_sub_from_zero() {
        let a = Timestamp::from_ticks(500);
        let b = Timestamp::from_ticks(0);
        assert_eq!((a - b).as_ticks(), 500);
    }

    #[test]
    fn timestamp_partial_cmp_equal() {
        let a = Timestamp::from_ticks(500);
        assert_eq!(a.partial_cmp(&a), Some(Ordering::Equal));
    }

    #[test]
    fn timestamp_partial_cmp_greater() {
        let a = Timestamp::from_ticks(1000);
        let b = Timestamp::from_ticks(500);
        assert_eq!(a.partial_cmp(&b), Some(Ordering::Greater));
    }

    #[test]
    fn timestamp_partial_cmp_less() {
        let a = Timestamp::from_ticks(500);
        let b = Timestamp::from_ticks(1000);
        assert_eq!(a.partial_cmp(&b), Some(Ordering::Less));
    }

    #[test]
    fn timestamp_partial_cmp_rollover_greater() {
        // a rolled over; its raw value is smaller but it is temporally later.
        let a = Timestamp::from_ticks(100);
        let b = Timestamp::from_ticks((1 << 40) - 100);
        assert_eq!(a.partial_cmp(&b), Some(Ordering::Greater));
    }

    #[test]
    fn timestamp_partial_cmp_rollover_less() {
        // b rolled over; a's raw value is larger but it is temporally earlier.
        let a = Timestamp::from_ticks((1 << 40) - 100);
        let b = Timestamp::from_ticks(100);
        assert_eq!(a.partial_cmp(&b), Some(Ordering::Less));
    }

    // ── Timestamp + Duration / Timestamp - Duration ───────────────────────

    #[test]
    fn timestamp_add_duration_zero() {
        let t = Timestamp::from_ticks(1000);
        assert_eq!((t + Duration(0)).as_ticks(), 1000);
    }

    #[test]
    fn timestamp_add_duration_positive() {
        let t = Timestamp::from_ticks(1000);
        assert_eq!((t + Duration(500)).as_ticks(), 1500);
    }

    #[test]
    fn timestamp_add_duration_negative() {
        let t = Timestamp::from_ticks(1000);
        assert_eq!((t + Duration(-400)).as_ticks(), 600);
    }

    #[test]
    fn timestamp_add_duration_rollover() {
        // Adding a duration that pushes the timestamp past the 40-bit boundary.
        let t = Timestamp::from_ticks((1 << 40) - 100);
        assert_eq!((t + Duration(200)).as_ticks(), 100);
    }

    #[test]
    fn timestamp_add_duration_negative_underflow() {
        // Adding a negative duration that wraps below zero.
        let t = Timestamp::from_ticks(100);
        assert_eq!((t + Duration(-200)).as_ticks(), (1u64 << 40) - 100);
    }

    #[test]
    fn timestamp_sub_duration_zero() {
        let t = Timestamp::from_ticks(1000);
        assert_eq!((t - Duration(0)).as_ticks(), 1000);
    }

    #[test]
    fn timestamp_sub_duration_positive() {
        let t = Timestamp::from_ticks(1000);
        assert_eq!((t - Duration(400)).as_ticks(), 600);
    }

    #[test]
    fn timestamp_sub_duration_negative() {
        let t = Timestamp::from_ticks(1000);
        assert_eq!((t - Duration(-500)).as_ticks(), 1500);
    }

    #[test]
    fn timestamp_sub_duration_underflow() {
        // Subtracting a duration that wraps below zero.
        let t = Timestamp::from_ticks(100);
        assert_eq!((t - Duration(200)).as_ticks(), (1u64 << 40) - 100);
    }

    #[test]
    fn timestamp_sub_duration_rollover() {
        // Subtracting a negative duration that pushes past the 40-bit boundary.
        let t = Timestamp::from_ticks((1 << 40) - 100);
        assert_eq!((t - Duration(-200)).as_ticks(), 100);
    }

    #[test]
    fn timestamp_add_sub_duration_round_trip() {
        let t = Timestamp::from_ticks(500_000);
        let d = Duration(12_345);
        assert_eq!(((t + d) - d).as_ticks(), t.as_ticks());
    }
}
