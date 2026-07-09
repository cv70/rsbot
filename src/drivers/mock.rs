use crate::board::{
    BatteryReader, BatterySample, EncoderCounts, EncoderReader, ImuReader, MotorCommand, MotorPwm,
    MotorSide,
};
use crate::chassis::{Chassis, ChassisError, DifferentialDrive, Twist, WheelAngularVelocities};
use crate::imu::ImuSample;

#[derive(Debug, Clone)]
pub struct MockChassis {
    drive: DifferentialDrive,
    sequence_id: u32,
    last_twist: Twist,
    last_wheels: WheelAngularVelocities,
}

impl MockChassis {
    pub fn new(drive: DifferentialDrive) -> Self {
        Self {
            drive,
            sequence_id: 0,
            last_twist: Twist::stop(),
            last_wheels: WheelAngularVelocities {
                left_radps: 0.0,
                right_radps: 0.0,
            },
        }
    }

    pub const fn last_twist(&self) -> Twist {
        self.last_twist
    }

    pub const fn last_wheels(&self) -> WheelAngularVelocities {
        self.last_wheels
    }

    pub const fn sequence_id(&self) -> u32 {
        self.sequence_id
    }
}

impl Chassis for MockChassis {
    fn drive(&mut self, twist: Twist) -> Result<(), ChassisError> {
        let clamped = self.drive.limits.clamp(twist);
        self.sequence_id = self.sequence_id.wrapping_add(1);
        self.last_twist = clamped;
        self.last_wheels = self.drive.wheel_angular_velocities(clamped);

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MockBoard {
    pub encoder_counts: EncoderCounts,
    pub imu: ImuSample,
    pub battery: BatterySample,
    pub left_motor: MotorCommand,
    pub right_motor: MotorCommand,
}

impl Default for MockBoard {
    fn default() -> Self {
        Self {
            encoder_counts: EncoderCounts::default(),
            imu: ImuSample::default(),
            battery: BatterySample { voltage: 12.0 },
            left_motor: MotorCommand::stop(),
            right_motor: MotorCommand::stop(),
        }
    }
}

impl MockBoard {
    pub const fn new(
        encoder_counts: EncoderCounts,
        imu: ImuSample,
        battery: BatterySample,
    ) -> Self {
        Self {
            encoder_counts,
            imu,
            battery,
            left_motor: MotorCommand::stop(),
            right_motor: MotorCommand::stop(),
        }
    }

    pub fn set_encoder_counts(&mut self, left: i32, right: i32) {
        self.encoder_counts = EncoderCounts { left, right };
    }

    pub fn set_yaw(&mut self, yaw_rad: f32) {
        self.imu.yaw_rad = yaw_rad;
    }

    pub fn set_battery_voltage(&mut self, voltage: f32) {
        self.battery.voltage = voltage;
    }
}

impl MotorPwm for MockBoard {
    fn set_motor(&mut self, side: MotorSide, command: MotorCommand) -> Result<(), ChassisError> {
        match side {
            MotorSide::Left => self.left_motor = command,
            MotorSide::Right => self.right_motor = command,
        }

        Ok(())
    }
}

impl EncoderReader for MockBoard {
    fn read_counts(&mut self) -> Result<EncoderCounts, ChassisError> {
        Ok(self.encoder_counts)
    }
}

impl ImuReader for MockBoard {
    fn read_imu(&mut self) -> Result<ImuSample, ChassisError> {
        Ok(self.imu)
    }
}

impl BatteryReader for MockBoard {
    fn read_battery(&mut self) -> Result<BatterySample, ChassisError> {
        Ok(self.battery)
    }
}
