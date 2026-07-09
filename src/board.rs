use crate::chassis::ChassisError;
use crate::imu::ImuSample;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorSide {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorCommand {
    pub duty: f32,
}

impl MotorCommand {
    pub fn new(duty: f32) -> Self {
        Self {
            duty: duty.clamp(-1.0, 1.0),
        }
    }

    pub const fn stop() -> Self {
        Self { duty: 0.0 }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EncoderCounts {
    pub left: i32,
    pub right: i32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatterySample {
    pub voltage: f32,
}

pub trait MotorPwm {
    fn set_motor(&mut self, side: MotorSide, command: MotorCommand) -> Result<(), ChassisError>;

    fn stop_all(&mut self) -> Result<(), ChassisError> {
        self.set_motor(MotorSide::Left, MotorCommand::stop())?;
        self.set_motor(MotorSide::Right, MotorCommand::stop())
    }
}

pub trait EncoderReader {
    fn read_counts(&mut self) -> Result<EncoderCounts, ChassisError>;
}

pub trait ImuReader {
    fn read_imu(&mut self) -> Result<ImuSample, ChassisError>;
}

pub trait BatteryReader {
    fn read_battery(&mut self) -> Result<BatterySample, ChassisError>;
}
