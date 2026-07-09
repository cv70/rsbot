use crate::chassis::Twist;
use crate::imu::normalize_angle;
use crate::math;
use crate::odometry::Pose2d;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RobotMode {
    Idle,
    Manual,
    Navigate,
    Fault,
    EmergencyStop,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavigationCommand {
    Stop,
    Velocity(Twist),
    FaceHeading { yaw_rad: f32 },
    GoTo { target: Pose2d },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavigationConfig {
    pub max_linear_mps: f32,
    pub max_angular_radps: f32,
    pub position_tolerance_m: f32,
    pub yaw_tolerance_rad: f32,
    pub linear_gain: f32,
    pub angular_gain: f32,
}

impl Default for NavigationConfig {
    fn default() -> Self {
        Self {
            max_linear_mps: 0.3,
            max_angular_radps: 1.5,
            position_tolerance_m: 0.05,
            yaw_tolerance_rad: 0.05,
            linear_gain: 0.8,
            angular_gain: 2.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavigationController {
    config: NavigationConfig,
    mode: RobotMode,
    command: NavigationCommand,
}

impl NavigationController {
    pub const fn new(config: NavigationConfig) -> Self {
        Self {
            config,
            mode: RobotMode::Idle,
            command: NavigationCommand::Stop,
        }
    }

    pub const fn mode(&self) -> RobotMode {
        self.mode
    }

    pub const fn command(&self) -> NavigationCommand {
        self.command
    }

    pub fn set_command(&mut self, command: NavigationCommand) {
        self.command = command;
        self.mode = match command {
            NavigationCommand::Stop => RobotMode::Idle,
            NavigationCommand::Velocity(_) => RobotMode::Manual,
            NavigationCommand::FaceHeading { .. } | NavigationCommand::GoTo { .. } => {
                RobotMode::Navigate
            }
        };
    }

    pub fn set_fault(&mut self) {
        self.mode = RobotMode::Fault;
        self.command = NavigationCommand::Stop;
    }

    pub fn set_emergency_stop(&mut self) {
        self.mode = RobotMode::EmergencyStop;
        self.command = NavigationCommand::Stop;
    }

    pub fn update(&mut self, pose: Pose2d) -> Twist {
        match (self.mode, self.command) {
            (RobotMode::Fault | RobotMode::EmergencyStop | RobotMode::Idle, _) => Twist::stop(),
            (_, NavigationCommand::Stop) => {
                self.mode = RobotMode::Idle;
                Twist::stop()
            }
            (_, NavigationCommand::Velocity(twist)) => twist,
            (_, NavigationCommand::FaceHeading { yaw_rad }) => self.face_heading(pose, yaw_rad),
            (_, NavigationCommand::GoTo { target }) => self.go_to(pose, target),
        }
    }

    fn face_heading(&mut self, pose: Pose2d, target_yaw_rad: f32) -> Twist {
        let yaw_error = normalize_angle(target_yaw_rad - pose.yaw_rad);

        if yaw_error.abs() <= self.config.yaw_tolerance_rad {
            self.mode = RobotMode::Idle;
            self.command = NavigationCommand::Stop;
            return Twist::stop();
        }

        Twist::new(
            0.0,
            (yaw_error * self.config.angular_gain).clamp(
                -self.config.max_angular_radps,
                self.config.max_angular_radps,
            ),
        )
    }

    fn go_to(&mut self, pose: Pose2d, target: Pose2d) -> Twist {
        let dx = target.x_m - pose.x_m;
        let dy = target.y_m - pose.y_m;
        let distance = math::sqrt(dx * dx + dy * dy);

        if distance <= self.config.position_tolerance_m {
            return self.face_heading(pose, target.yaw_rad);
        }

        let target_heading = math::atan2(dy, dx);
        let heading_error = normalize_angle(target_heading - pose.yaw_rad);
        let linear_scale = math::cos(heading_error).max(0.0);
        let linear = (distance * self.config.linear_gain * linear_scale)
            .clamp(0.0, self.config.max_linear_mps);
        let angular = (heading_error * self.config.angular_gain).clamp(
            -self.config.max_angular_radps,
            self.config.max_angular_radps,
        );

        Twist::new(linear, angular)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn go_to_outputs_forward_motion_for_target_ahead() {
        let mut controller = NavigationController::new(NavigationConfig::default());
        controller.set_command(NavigationCommand::GoTo {
            target: Pose2d::new(1.0, 0.0, 0.0),
        });

        let twist = controller.update(Pose2d::default());

        assert!(twist.linear_mps > 0.0);
        assert!(twist.angular_radps.abs() < 0.0001);
    }

    #[test]
    fn go_to_stops_when_position_and_heading_are_reached() {
        let mut controller = NavigationController::new(NavigationConfig::default());
        controller.set_command(NavigationCommand::GoTo {
            target: Pose2d::new(0.01, 0.0, 0.01),
        });

        let twist = controller.update(Pose2d::default());

        assert_eq!(twist, Twist::stop());
        assert_eq!(controller.mode(), RobotMode::Idle);
    }

    #[test]
    fn emergency_stop_forces_zero_command() {
        let mut controller = NavigationController::new(NavigationConfig::default());
        controller.set_command(NavigationCommand::Velocity(Twist::new(0.2, 0.0)));
        controller.set_emergency_stop();

        assert_eq!(controller.update(Pose2d::default()), Twist::stop());
    }
}
