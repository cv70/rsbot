use crate::chassis::{Twist, WheelAngularVelocities};
use crate::drivers::mock::MockChassis;
use crate::safety::FaultCode;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChassisTelemetry {
    pub sequence_id: u32,
    pub commanded_twist: Twist,
    pub wheel_velocities: WheelAngularVelocities,
    pub battery_voltage: Option<f32>,
    pub emergency_stop: bool,
    pub command_timed_out: bool,
    pub fault: Option<FaultCode>,
}

impl ChassisTelemetry {
    pub const fn new(
        sequence_id: u32,
        commanded_twist: Twist,
        wheel_velocities: WheelAngularVelocities,
    ) -> Self {
        Self {
            sequence_id,
            commanded_twist,
            wheel_velocities,
            battery_voltage: None,
            emergency_stop: false,
            command_timed_out: false,
            fault: None,
        }
    }

    pub const fn stopped(sequence_id: u32) -> Self {
        Self::new(
            sequence_id,
            Twist::stop(),
            WheelAngularVelocities {
                left_radps: 0.0,
                right_radps: 0.0,
            },
        )
    }
}

impl From<&MockChassis> for ChassisTelemetry {
    fn from(chassis: &MockChassis) -> Self {
        Self::new(
            chassis.sequence_id(),
            chassis.last_twist(),
            chassis.last_wheels(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chassis::{Chassis, DifferentialDrive};

    #[test]
    fn mock_chassis_can_be_reported_as_telemetry() {
        let mut chassis = MockChassis::new(DifferentialDrive::default());

        chassis.drive(Twist::new(0.2, 0.0)).unwrap();
        let telemetry = ChassisTelemetry::from(&chassis);

        assert_eq!(telemetry.sequence_id, 1);
        assert_eq!(telemetry.commanded_twist, Twist::new(0.2, 0.0));
        assert_eq!(telemetry.fault, None);
    }
}
