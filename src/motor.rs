use crate::board::{MotorCommand, MotorPwm, MotorSide};
use crate::chassis::{ChassisError, WheelAngularVelocities};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSpeedControlConfig {
    pub left_kp: f32,
    pub right_kp: f32,
    pub max_duty: f32,
    pub deadband_radps: f32,
}

impl Default for WheelSpeedControlConfig {
    fn default() -> Self {
        Self {
            left_kp: 0.08,
            right_kp: 0.08,
            max_duty: 0.8,
            deadband_radps: 0.05,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSpeedController {
    config: WheelSpeedControlConfig,
    last_left: MotorCommand,
    last_right: MotorCommand,
}

impl WheelSpeedController {
    pub const fn new(config: WheelSpeedControlConfig) -> Self {
        Self {
            config,
            last_left: MotorCommand::stop(),
            last_right: MotorCommand::stop(),
        }
    }

    pub const fn last_left(&self) -> MotorCommand {
        self.last_left
    }

    pub const fn last_right(&self) -> MotorCommand {
        self.last_right
    }

    pub fn update(
        &mut self,
        target: WheelAngularVelocities,
        measured: WheelAngularVelocities,
    ) -> (MotorCommand, MotorCommand) {
        self.last_left = control_wheel(
            target.left_radps,
            measured.left_radps,
            self.config.left_kp,
            self.config.max_duty,
            self.config.deadband_radps,
        );
        self.last_right = control_wheel(
            target.right_radps,
            measured.right_radps,
            self.config.right_kp,
            self.config.max_duty,
            self.config.deadband_radps,
        );

        (self.last_left, self.last_right)
    }

    pub fn stop(&mut self) -> (MotorCommand, MotorCommand) {
        self.last_left = MotorCommand::stop();
        self.last_right = MotorCommand::stop();
        (self.last_left, self.last_right)
    }

    pub fn write_to<P>(&self, pwm: &mut P) -> Result<(), ChassisError>
    where
        P: MotorPwm,
    {
        pwm.set_motor(MotorSide::Left, self.last_left)?;
        pwm.set_motor(MotorSide::Right, self.last_right)
    }
}

fn control_wheel(
    target_radps: f32,
    measured_radps: f32,
    kp: f32,
    max_duty: f32,
    deadband_radps: f32,
) -> MotorCommand {
    if target_radps.abs() <= deadband_radps && measured_radps.abs() <= deadband_radps {
        return MotorCommand::stop();
    }

    MotorCommand::new(((target_radps - measured_radps) * kp).clamp(-max_duty, max_duty))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wheel_speed_controller_outputs_positive_duty_for_slow_wheel() {
        let mut controller = WheelSpeedController::new(WheelSpeedControlConfig::default());

        let (left, right) = controller.update(
            WheelAngularVelocities {
                left_radps: 5.0,
                right_radps: 5.0,
            },
            WheelAngularVelocities {
                left_radps: 1.0,
                right_radps: 6.0,
            },
        );

        assert!(left.duty > 0.0);
        assert!(right.duty < 0.0);
    }

    #[test]
    fn wheel_speed_controller_stops_inside_deadband() {
        let mut controller = WheelSpeedController::new(WheelSpeedControlConfig::default());

        let (left, right) = controller.update(
            WheelAngularVelocities {
                left_radps: 0.01,
                right_radps: -0.01,
            },
            WheelAngularVelocities {
                left_radps: 0.01,
                right_radps: -0.01,
            },
        );

        assert_eq!(left, MotorCommand::stop());
        assert_eq!(right, MotorCommand::stop());
    }
}
