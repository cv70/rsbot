#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Millis(pub u32);

impl Millis {
    pub const ZERO: Self = Self(0);

    pub const fn from_millis(millis: u32) -> Self {
        Self(millis)
    }

    pub const fn as_millis(self) -> u32 {
        self.0
    }

    pub const fn saturating_sub(self, earlier: Self) -> Self {
        Self(self.0.saturating_sub(earlier.0))
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ControlTime {
    pub tick: u32,
    pub elapsed_ms: u16,
}

impl ControlTime {
    pub const fn new(tick: u32, elapsed_ms: u16) -> Self {
        Self { tick, elapsed_ms }
    }

    pub const fn elapsed_secs(self) -> f32 {
        self.elapsed_ms as f32 / 1000.0
    }
}
