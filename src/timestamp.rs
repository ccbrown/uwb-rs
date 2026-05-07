/// This type represents a timestamp from the UWB device, which has a resolution of roughly 15
/// picoseconds, but rolls over about every 17.2 seconds.
#[derive(Clone, Copy)]
pub struct Timestamp(u64);

impl Timestamp {
    /// Returns the timestamp as a raw 40-bit value, in units of 1 / (128 * 499.2*10^6) seconds.
    pub fn as_raw(&self) -> u64 {
        self.0
    }

    /// Creates a timestamp from a raw 40-bit value, in units of 1 / (128 * 499.2*10^6) seconds.
    pub fn from_raw(raw: u64) -> Self {
        Timestamp(raw)
    }

    /// Converts a raw timestamp value to picoseconds.
    pub fn raw_to_picoseconds(raw: u64) -> u64 {
        raw * 10000000 / 4992 / 128
    }

    /// Returns the timestamp in picoseconds.
    pub fn as_picoseconds(&self) -> u64 {
        Self::raw_to_picoseconds(self.0)
    }

    /// Returns the number of picoseconds elapsed between this timestamp and another timestamp,
    /// accounting for rollover.
    ///
    /// Note that this method assumes that the two timestamps are at most one rollover apart.
    pub fn picoseconds_since(&self, other: Timestamp) -> u64 {
        let elapsed = if other.0 < self.0 {
            self.0 - other.0
        } else {
            // rollover
            self.0 + (1 << 40) - other.0
        };
        Self::raw_to_picoseconds(elapsed)
    }
}

impl defmt::Format for Timestamp {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Timestamp({})", self.0);
    }
}
