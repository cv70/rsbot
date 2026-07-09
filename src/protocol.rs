use crate::chassis::{Twist, WheelAngularVelocities};
use crate::safety::FaultCode;
use crate::telemetry::ChassisTelemetry;
use crate::time::Millis;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveCommand {
    pub sequence_id: u32,
    pub linear_mps: f32,
    pub angular_radps: f32,
    pub timeout_ms: u16,
}

impl DriveCommand {
    pub fn new(sequence_id: u32, twist: Twist, timeout: Millis) -> Self {
        Self {
            sequence_id,
            linear_mps: twist.linear_mps,
            angular_radps: twist.angular_radps,
            timeout_ms: timeout.as_millis().min(u16::MAX as u32) as u16,
        }
    }

    pub const fn twist(&self) -> Twist {
        Twist::new(self.linear_mps, self.angular_radps)
    }

    pub const fn timeout(&self) -> Millis {
        Millis::from_millis(self.timeout_ms as u32)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControllerTelemetry {
    pub sequence_id: u32,
    pub left_wheel_radps: f32,
    pub right_wheel_radps: f32,
    pub battery_voltage: f32,
    pub emergency_stop: bool,
    pub fault_code: u16,
}

impl ControllerTelemetry {
    pub const fn new(
        sequence_id: u32,
        wheel_velocities: WheelAngularVelocities,
        battery_voltage: f32,
        emergency_stop: bool,
        fault: Option<FaultCode>,
    ) -> Self {
        Self {
            sequence_id,
            left_wheel_radps: wheel_velocities.left_radps,
            right_wheel_radps: wheel_velocities.right_radps,
            battery_voltage,
            emergency_stop,
            fault_code: match fault {
                Some(fault) => fault.as_u16(),
                None => 0,
            },
        }
    }

    pub const fn wheel_velocities(&self) -> WheelAngularVelocities {
        WheelAngularVelocities {
            left_radps: self.left_wheel_radps,
            right_radps: self.right_wheel_radps,
        }
    }

    pub const fn fault(&self) -> Option<FaultCode> {
        FaultCode::from_u16(self.fault_code)
    }
}

impl From<ChassisTelemetry> for ControllerTelemetry {
    fn from(telemetry: ChassisTelemetry) -> Self {
        Self::new(
            telemetry.sequence_id,
            telemetry.wheel_velocities,
            telemetry.battery_voltage.unwrap_or(0.0),
            telemetry.emergency_stop,
            telemetry.fault,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Millis;

    #[test]
    fn drive_command_preserves_twist_and_clamps_timeout_to_u16() {
        let command = DriveCommand::new(
            7,
            Twist::new(0.2, -0.4),
            Millis::from_millis(u16::MAX as u32 + 10),
        );

        assert_eq!(command.sequence_id, 7);
        assert_eq!(command.twist(), Twist::new(0.2, -0.4));
        assert_eq!(command.timeout_ms, u16::MAX);
    }

    #[test]
    fn controller_telemetry_maps_fault_codes() {
        let telemetry = ControllerTelemetry::new(
            1,
            WheelAngularVelocities {
                left_radps: 1.0,
                right_radps: 2.0,
            },
            12.0,
            false,
            Some(FaultCode::LowBattery),
        );

        assert_eq!(telemetry.fault(), Some(FaultCode::LowBattery));
    }
}
