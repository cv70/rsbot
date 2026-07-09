use crate::chassis::Twist;
use crate::time::ControlTime;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccelerationLimits {
    pub max_linear_mps2: f32,
    pub max_angular_radps2: f32,
}

impl AccelerationLimits {
    pub const fn new(max_linear_mps2: f32, max_angular_radps2: f32) -> Self {
        Self {
            max_linear_mps2,
            max_angular_radps2,
        }
    }

    pub fn limit(&self, current: Twist, target: Twist, elapsed_secs: f32) -> Twist {
        if elapsed_secs <= 0.0 {
            return current;
        }

        Twist {
            linear_mps: step_toward(
                current.linear_mps,
                target.linear_mps,
                self.max_linear_mps2 * elapsed_secs,
            ),
            angular_radps: step_toward(
                current.angular_radps,
                target.angular_radps,
                self.max_angular_radps2 * elapsed_secs,
            ),
        }
    }
}

impl Default for AccelerationLimits {
    fn default() -> Self {
        Self::new(1.0, 6.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionLimiter {
    limits: AccelerationLimits,
    current: Twist,
}

impl MotionLimiter {
    pub const fn new(limits: AccelerationLimits) -> Self {
        Self {
            limits,
            current: Twist::stop(),
        }
    }

    pub const fn current(&self) -> Twist {
        self.current
    }

    pub const fn limits(&self) -> AccelerationLimits {
        self.limits
    }

    pub fn reset(&mut self, twist: Twist) {
        self.current = twist;
    }

    pub fn apply(&mut self, target: Twist, elapsed_secs: f32) -> Twist {
        self.current = self.limits.limit(self.current, target, elapsed_secs);
        self.current
    }

    pub fn apply_time(&mut self, target: Twist, time: ControlTime) -> Twist {
        self.current = self.limits.limit(self.current, target, time.elapsed_secs());
        self.current
    }
}

fn step_toward(current: f32, target: f32, max_delta: f32) -> f32 {
    if max_delta <= 0.0 {
        return current;
    }

    let delta = target - current;

    if delta.abs() <= max_delta {
        target
    } else {
        current + delta.signum() * max_delta
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn acceleration_limits_step_toward_target() {
        let limits = AccelerationLimits::new(1.0, 2.0);
        let limited = limits.limit(Twist::stop(), Twist::new(10.0, -10.0), 0.1);

        assert_eq!(limited, Twist::new(0.1, -0.2));
    }

    #[test]
    fn acceleration_limits_reach_target_when_delta_is_small() {
        let limits = AccelerationLimits::new(1.0, 2.0);
        let limited = limits.limit(Twist::new(0.9, -0.9), Twist::new(1.0, -1.0), 1.0);

        assert_eq!(limited, Twist::new(1.0, -1.0));
    }
}
