use crate::board::{BatteryReader, EncoderCounts, EncoderReader, ImuReader, MotorPwm};
use crate::chassis::{ChassisError, DifferentialDrive, Twist};
use crate::control::MotionLimiter;
use crate::motor::{WheelSpeedControlConfig, WheelSpeedController};
use crate::navigation::{NavigationCommand, NavigationConfig, NavigationController, RobotMode};
use crate::odometry::{OdometryConfig, OdometryState, Pose2d, WheelEncoderSample};
use crate::safety::{FaultCode, SafetyConfig, SafetyStatus};
use crate::telemetry::ChassisTelemetry;
use crate::time::Millis;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RuntimeConfig {
    pub drive: DifferentialDrive,
    pub safety: SafetyConfig,
    pub navigation: NavigationConfig,
    pub wheel_control: WheelSpeedControlConfig,
    pub ticks_per_wheel_rev: i32,
    pub imu_yaw_weight: f32,
}

impl RuntimeConfig {
    pub const fn new(
        drive: DifferentialDrive,
        safety: SafetyConfig,
        navigation: NavigationConfig,
        wheel_control: WheelSpeedControlConfig,
        ticks_per_wheel_rev: i32,
        imu_yaw_weight: f32,
    ) -> Self {
        Self {
            drive,
            safety,
            navigation,
            wheel_control,
            ticks_per_wheel_rev,
            imu_yaw_weight,
        }
    }
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self::new(
            DifferentialDrive::default(),
            SafetyConfig::default(),
            NavigationConfig::default(),
            WheelSpeedControlConfig::default(),
            1024,
            0.1,
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RuntimeStatus {
    pub mode: RobotMode,
    pub pose: Pose2d,
    pub target_twist: Twist,
    pub safety: SafetyStatus,
}

#[derive(Debug)]
pub struct RobotRuntime<B> {
    board: B,
    config: RuntimeConfig,
    navigation: NavigationController,
    odometry: OdometryState,
    wheel_control: WheelSpeedController,
    motion_limiter: Option<MotionLimiter>,
    safety: SafetyStatus,
    last_command_at: Option<Millis>,
    last_update_at: Option<Millis>,
    sequence_id: u32,
    target_twist: Twist,
}

impl<B> RobotRuntime<B> {
    pub fn new(board: B, config: RuntimeConfig) -> Self {
        Self {
            board,
            config,
            navigation: NavigationController::new(config.navigation),
            odometry: OdometryState::new(OdometryConfig {
                drive: config.drive,
                ticks_per_wheel_rev: config.ticks_per_wheel_rev,
                imu_yaw_weight: config.imu_yaw_weight,
            }),
            wheel_control: WheelSpeedController::new(config.wheel_control),
            motion_limiter: config.safety.acceleration_limits.map(MotionLimiter::new),
            safety: SafetyStatus::default(),
            last_command_at: None,
            last_update_at: None,
            sequence_id: 0,
            target_twist: Twist::stop(),
        }
    }

    pub const fn board(&self) -> &B {
        &self.board
    }

    pub fn board_mut(&mut self) -> &mut B {
        &mut self.board
    }

    pub fn into_board(self) -> B {
        self.board
    }

    pub const fn status(&self) -> RuntimeStatus {
        RuntimeStatus {
            mode: self.navigation.mode(),
            pose: self.odometry.pose(),
            target_twist: self.target_twist,
            safety: self.safety,
        }
    }

    pub fn set_command(&mut self, command: NavigationCommand, now: Millis) {
        self.navigation.set_command(command);
        self.last_command_at = Some(now);
        self.safety.command_timed_out = false;
        if self.safety.fault == Some(FaultCode::CommandTimeout) {
            self.safety.fault = None;
        }
    }

    pub fn set_emergency_stop(&mut self, active: bool) {
        self.safety.emergency_stop = active;

        if active {
            self.safety.fault = Some(FaultCode::EmergencyStop);
            self.navigation.set_emergency_stop();
        } else if self.safety.fault == Some(FaultCode::EmergencyStop) {
            self.safety.fault = None;
        }
    }

    pub fn set_fault(&mut self, fault: Option<FaultCode>) {
        self.safety.fault = fault;
        if fault.is_some() {
            self.navigation.set_fault();
        }
    }

    pub fn reset_odometry(&mut self, pose: Pose2d, encoder: WheelEncoderSample) {
        self.odometry.reset(pose, encoder);
    }
}

impl<B> RobotRuntime<B>
where
    B: MotorPwm + EncoderReader + ImuReader + BatteryReader,
{
    pub fn tick(&mut self, now: Millis) -> Result<RuntimeStatus, ChassisError> {
        let elapsed_secs = self.elapsed_secs(now);
        self.update_battery()?;

        if self.command_timed_out(now) {
            self.safety.command_timed_out = true;
            self.safety.fault = Some(FaultCode::CommandTimeout);
            self.navigation.set_fault();
        }

        let encoder = self.read_encoder_sample()?;
        let imu = self.board.read_imu()?;
        let pose = self.odometry.update(encoder, Some(imu), elapsed_secs);

        self.target_twist = self.safe_twist(pose, elapsed_secs);
        let target_wheels = self
            .config
            .drive
            .wheel_angular_velocities(self.target_twist);
        let measured_wheels = self.odometry.wheel_velocities();
        self.wheel_control.update(target_wheels, measured_wheels);

        if self.target_twist == Twist::stop() {
            self.wheel_control.stop();
        }

        self.wheel_control.write_to(&mut self.board)?;
        self.sequence_id = self.sequence_id.wrapping_add(1);
        self.last_update_at = Some(now);

        Ok(self.status())
    }

    pub fn stop(&mut self, now: Millis) -> Result<(), ChassisError> {
        self.navigation.set_command(NavigationCommand::Stop);
        self.target_twist = Twist::stop();
        if let Some(limiter) = &mut self.motion_limiter {
            limiter.reset(Twist::stop());
        }
        self.wheel_control.stop();
        self.wheel_control.write_to(&mut self.board)?;
        self.last_command_at = Some(now);
        self.safety.command_timed_out = false;
        Ok(())
    }

    pub fn telemetry(&self) -> ChassisTelemetry {
        let mut telemetry = ChassisTelemetry::new(
            self.sequence_id,
            self.target_twist,
            self.odometry.wheel_velocities(),
        );
        telemetry.battery_voltage = self.safety.battery_voltage;
        telemetry.emergency_stop = self.safety.emergency_stop;
        telemetry.command_timed_out = self.safety.command_timed_out;
        telemetry.fault = self.safety.fault;
        telemetry
    }

    fn elapsed_secs(&self, now: Millis) -> f32 {
        self.last_update_at
            .map(|last| now.saturating_sub(last).as_millis() as f32 / 1000.0)
            .filter(|elapsed| *elapsed > 0.0)
            .unwrap_or(0.0)
    }

    fn update_battery(&mut self) -> Result<(), ChassisError> {
        let battery = self.board.read_battery()?;
        self.safety.battery_voltage = Some(battery.voltage);

        if self
            .config
            .safety
            .min_battery_voltage
            .is_some_and(|minimum| battery.voltage < minimum)
        {
            self.safety.fault = Some(FaultCode::LowBattery);
            self.navigation.set_fault();
        }

        Ok(())
    }

    fn command_timed_out(&self, now: Millis) -> bool {
        self.last_command_at.is_some_and(|last_command_at| {
            now.saturating_sub(last_command_at) > self.config.safety.command_timeout
        })
    }

    fn read_encoder_sample(&mut self) -> Result<WheelEncoderSample, ChassisError> {
        let EncoderCounts { left, right } = self.board.read_counts()?;
        Ok(WheelEncoderSample::new(left, right))
    }

    fn safe_twist(&mut self, pose: Pose2d, elapsed_secs: f32) -> Twist {
        if self.safety.emergency_stop || self.safety.fault.is_some() {
            return Twist::stop();
        }

        let target = self.config.drive.limits.clamp(self.navigation.update(pose));

        let Some(limiter) = &mut self.motion_limiter else {
            return target;
        };

        limiter.apply(target, elapsed_secs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{
        BatterySample, EncoderCounts, EncoderReader, ImuReader, MotorCommand, MotorPwm, MotorSide,
    };
    use crate::chassis::DriveLimits;
    use crate::control::AccelerationLimits;
    use crate::imu::ImuSample;

    #[derive(Debug, Clone, Copy)]
    struct FakeBoard {
        counts: EncoderCounts,
        imu: ImuSample,
        battery: BatterySample,
        left: MotorCommand,
        right: MotorCommand,
    }

    impl Default for FakeBoard {
        fn default() -> Self {
            Self {
                counts: EncoderCounts::default(),
                imu: ImuSample::default(),
                battery: BatterySample { voltage: 12.0 },
                left: MotorCommand::stop(),
                right: MotorCommand::stop(),
            }
        }
    }

    impl MotorPwm for FakeBoard {
        fn set_motor(
            &mut self,
            side: MotorSide,
            command: MotorCommand,
        ) -> Result<(), ChassisError> {
            match side {
                MotorSide::Left => self.left = command,
                MotorSide::Right => self.right = command,
            }
            Ok(())
        }
    }

    impl EncoderReader for FakeBoard {
        fn read_counts(&mut self) -> Result<EncoderCounts, ChassisError> {
            Ok(self.counts)
        }
    }

    impl ImuReader for FakeBoard {
        fn read_imu(&mut self) -> Result<ImuSample, ChassisError> {
            Ok(self.imu)
        }
    }

    impl BatteryReader for FakeBoard {
        fn read_battery(&mut self) -> Result<BatterySample, ChassisError> {
            Ok(self.battery)
        }
    }

    fn test_config() -> RuntimeConfig {
        RuntimeConfig {
            drive: DifferentialDrive::new(0.05, 0.2, DriveLimits::new(0.5, 2.0)),
            ticks_per_wheel_rev: 100,
            imu_yaw_weight: 0.0,
            ..RuntimeConfig::default()
        }
    }

    #[test]
    fn runtime_drives_motors_for_waypoint_command() {
        let board = FakeBoard::default();
        let mut runtime = RobotRuntime::new(board, test_config());

        runtime.set_command(
            NavigationCommand::GoTo {
                target: Pose2d::new(1.0, 0.0, 0.0),
            },
            Millis::from_millis(0),
        );
        runtime.tick(Millis::from_millis(10)).unwrap();

        assert!(runtime.board().left.duty > 0.0);
        assert!(runtime.board().right.duty > 0.0);
        assert_eq!(runtime.status().mode, RobotMode::Navigate);
    }

    #[test]
    fn runtime_stops_motors_on_command_timeout() {
        let board = FakeBoard::default();
        let mut runtime = RobotRuntime::new(board, test_config());

        runtime.set_command(
            NavigationCommand::Velocity(Twist::new(0.2, 0.0)),
            Millis::from_millis(0),
        );
        runtime.tick(Millis::from_millis(10)).unwrap();
        runtime.tick(Millis::from_millis(400)).unwrap();

        assert_eq!(runtime.board().left, MotorCommand::stop());
        assert_eq!(runtime.board().right, MotorCommand::stop());
        assert_eq!(
            runtime.status().safety.fault,
            Some(FaultCode::CommandTimeout)
        );
    }

    #[test]
    fn runtime_stops_motors_on_low_battery() {
        let mut board = FakeBoard::default();
        board.battery.voltage = 9.0;
        let mut config = test_config();
        config.safety.min_battery_voltage = Some(10.0);
        let mut runtime = RobotRuntime::new(board, config);

        runtime.set_command(
            NavigationCommand::Velocity(Twist::new(0.2, 0.0)),
            Millis::from_millis(0),
        );
        runtime.tick(Millis::from_millis(10)).unwrap();

        assert_eq!(runtime.board().left, MotorCommand::stop());
        assert_eq!(runtime.status().safety.fault, Some(FaultCode::LowBattery));
    }

    #[test]
    fn runtime_applies_acceleration_limits_to_target_twist() {
        let board = FakeBoard::default();
        let mut config = test_config();
        config.safety.acceleration_limits = Some(AccelerationLimits::new(1.0, 2.0));
        let mut runtime = RobotRuntime::new(board, config);

        runtime.set_command(
            NavigationCommand::Velocity(Twist::new(0.5, 2.0)),
            Millis::from_millis(0),
        );
        runtime.tick(Millis::from_millis(0)).unwrap();
        runtime.tick(Millis::from_millis(100)).unwrap();

        assert_eq!(runtime.status().target_twist, Twist::new(0.1, 0.2));
    }
}
