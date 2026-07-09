use core::fmt::{Display, Formatter};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Twist {
    pub linear_mps: f32,
    pub angular_radps: f32,
}

impl Twist {
    pub const fn new(linear_mps: f32, angular_radps: f32) -> Self {
        Self {
            linear_mps,
            angular_radps,
        }
    }

    pub const fn stop() -> Self {
        Self::new(0.0, 0.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelVelocities {
    pub left_mps: f32,
    pub right_mps: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelAngularVelocities {
    pub left_radps: f32,
    pub right_radps: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveLimits {
    pub max_linear_mps: f32,
    pub max_angular_radps: f32,
}

impl DriveLimits {
    pub const fn new(max_linear_mps: f32, max_angular_radps: f32) -> Self {
        Self {
            max_linear_mps,
            max_angular_radps,
        }
    }

    pub fn clamp(&self, twist: Twist) -> Twist {
        Twist {
            linear_mps: twist
                .linear_mps
                .clamp(-self.max_linear_mps, self.max_linear_mps),
            angular_radps: twist
                .angular_radps
                .clamp(-self.max_angular_radps, self.max_angular_radps),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferentialDrive {
    pub wheel_radius_m: f32,
    pub track_width_m: f32,
    pub limits: DriveLimits,
}

impl DifferentialDrive {
    pub const fn new(wheel_radius_m: f32, track_width_m: f32, limits: DriveLimits) -> Self {
        Self {
            wheel_radius_m,
            track_width_m,
            limits,
        }
    }

    pub fn wheel_velocities(&self, twist: Twist) -> WheelVelocities {
        let twist = self.limits.clamp(twist);
        let half_track = self.track_width_m / 2.0;

        WheelVelocities {
            left_mps: twist.linear_mps - twist.angular_radps * half_track,
            right_mps: twist.linear_mps + twist.angular_radps * half_track,
        }
    }

    pub fn wheel_angular_velocities(&self, twist: Twist) -> WheelAngularVelocities {
        let velocities = self.wheel_velocities(twist);

        WheelAngularVelocities {
            left_radps: velocities.left_mps / self.wheel_radius_m,
            right_radps: velocities.right_mps / self.wheel_radius_m,
        }
    }
}

impl Default for DifferentialDrive {
    fn default() -> Self {
        Self::new(0.033, 0.160, DriveLimits::new(0.4, 2.5))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChassisError {
    CommandRejected,
    EmergencyStopActive,
    FaultActive,
    CommunicationFault,
    LowBattery,
}

impl Display for ChassisError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::CommandRejected => formatter.write_str("chassis command rejected"),
            Self::EmergencyStopActive => formatter.write_str("emergency stop is active"),
            Self::FaultActive => formatter.write_str("chassis fault is active"),
            Self::CommunicationFault => formatter.write_str("chassis communication fault"),
            Self::LowBattery => formatter.write_str("battery voltage is below the safe limit"),
        }
    }
}

pub trait Chassis {
    fn drive(&mut self, twist: Twist) -> Result<(), ChassisError>;
    fn stop(&mut self) -> Result<(), ChassisError> {
        self.drive(Twist::stop())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.0001;

    #[test]
    fn differential_drive_converts_forward_motion_to_equal_wheel_speeds() {
        let drive = DifferentialDrive::new(0.05, 0.2, DriveLimits::new(1.0, 3.0));
        let wheels = drive.wheel_angular_velocities(Twist::new(0.5, 0.0));

        assert!((wheels.left_radps - 10.0).abs() < EPSILON);
        assert!((wheels.right_radps - 10.0).abs() < EPSILON);
    }

    #[test]
    fn differential_drive_converts_turning_motion_to_opposite_wheel_speeds() {
        let drive = DifferentialDrive::new(0.05, 0.2, DriveLimits::new(1.0, 3.0));
        let wheels = drive.wheel_angular_velocities(Twist::new(0.0, 1.0));

        assert!((wheels.left_radps + 2.0).abs() < EPSILON);
        assert!((wheels.right_radps - 2.0).abs() < EPSILON);
    }

    #[test]
    fn drive_limits_clamp_unsafe_commands() {
        let limits = DriveLimits::new(0.4, 2.5);
        let twist = limits.clamp(Twist::new(2.0, -4.0));

        assert_eq!(twist, Twist::new(0.4, -2.5));
    }
}
