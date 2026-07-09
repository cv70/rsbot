pub mod mock;

use crate::chassis::{Chassis, ChassisError, DifferentialDrive, Twist};
use crate::protocol::{ControllerTelemetry, DriveCommand};
use crate::time::Millis;

pub trait MotorControllerTransport {
    fn send_drive_command(&mut self, command: DriveCommand) -> Result<(), ChassisError>;
    fn read_telemetry(&mut self) -> Result<Option<ControllerTelemetry>, ChassisError>;
}

#[derive(Debug)]
pub struct TransportChassis<T> {
    transport: T,
    drive: DifferentialDrive,
    command_timeout: Millis,
    next_sequence_id: u32,
    last_telemetry: Option<ControllerTelemetry>,
}

impl<T> TransportChassis<T> {
    pub const fn new(transport: T, drive: DifferentialDrive, command_timeout: Millis) -> Self {
        Self {
            transport,
            drive,
            command_timeout,
            next_sequence_id: 1,
            last_telemetry: None,
        }
    }

    pub const fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn into_transport(self) -> T {
        self.transport
    }

    pub const fn last_telemetry(&self) -> Option<ControllerTelemetry> {
        self.last_telemetry
    }
}

impl<T> TransportChassis<T>
where
    T: MotorControllerTransport,
{
    pub fn poll_telemetry(&mut self) -> Result<Option<ControllerTelemetry>, ChassisError> {
        let telemetry = self.transport.read_telemetry()?;

        if telemetry.is_some() {
            self.last_telemetry = telemetry;
        }

        Ok(telemetry)
    }
}

impl<T> Chassis for TransportChassis<T>
where
    T: MotorControllerTransport,
{
    fn drive(&mut self, twist: Twist) -> Result<(), ChassisError> {
        let clamped = self.drive.limits.clamp(twist);
        let command = DriveCommand::new(self.next_sequence_id, clamped, self.command_timeout);

        self.transport.send_drive_command(command)?;
        self.next_sequence_id = self.next_sequence_id.wrapping_add(1);

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chassis::DriveLimits;
    use crate::time::Millis;

    #[derive(Debug, Default)]
    struct FakeTransport {
        last_command: Option<DriveCommand>,
        telemetry: Option<ControllerTelemetry>,
    }

    impl MotorControllerTransport for FakeTransport {
        fn send_drive_command(&mut self, command: DriveCommand) -> Result<(), ChassisError> {
            self.last_command = Some(command);
            Ok(())
        }

        fn read_telemetry(&mut self) -> Result<Option<ControllerTelemetry>, ChassisError> {
            Ok(self.telemetry)
        }
    }

    #[test]
    fn transport_chassis_sends_clamped_drive_command() {
        let transport = FakeTransport::default();
        let drive = DifferentialDrive::new(0.05, 0.2, DriveLimits::new(0.4, 2.0));
        let mut chassis = TransportChassis::new(transport, drive, Millis::from_millis(250));

        chassis.drive(Twist::new(2.0, -4.0)).unwrap();
        let command = chassis.transport().last_command.unwrap();

        assert_eq!(command.sequence_id, 1);
        assert_eq!(command.twist(), Twist::new(0.4, -2.0));
        assert_eq!(command.timeout(), Millis::from_millis(250));
    }

    #[test]
    fn transport_chassis_caches_latest_telemetry() {
        let transport = FakeTransport {
            telemetry: Some(ControllerTelemetry::new(
                1,
                crate::chassis::WheelAngularVelocities {
                    left_radps: 1.0,
                    right_radps: 1.0,
                },
                12.0,
                false,
                None,
            )),
            ..FakeTransport::default()
        };
        let drive = DifferentialDrive::default();
        let mut chassis = TransportChassis::new(transport, drive, Millis::from_millis(250));

        let telemetry = chassis.poll_telemetry().unwrap();

        assert_eq!(telemetry, chassis.last_telemetry());
    }
}
