use crate::chassis::{DifferentialDrive, WheelAngularVelocities};
use crate::imu::{ImuSample, normalize_angle};
use crate::math;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pose2d {
    pub x_m: f32,
    pub y_m: f32,
    pub yaw_rad: f32,
}

impl Pose2d {
    pub const fn new(x_m: f32, y_m: f32, yaw_rad: f32) -> Self {
        Self { x_m, y_m, yaw_rad }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct WheelEncoderSample {
    pub left_ticks: i32,
    pub right_ticks: i32,
}

impl WheelEncoderSample {
    pub const fn new(left_ticks: i32, right_ticks: i32) -> Self {
        Self {
            left_ticks,
            right_ticks,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryConfig {
    pub drive: DifferentialDrive,
    pub ticks_per_wheel_rev: i32,
    pub imu_yaw_weight: f32,
}

impl OdometryConfig {
    pub fn meters_per_tick(&self) -> f32 {
        core::f32::consts::TAU * self.drive.wheel_radius_m / self.ticks_per_wheel_rev as f32
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OdometryState {
    config: OdometryConfig,
    pose: Pose2d,
    last_encoder: Option<WheelEncoderSample>,
    wheel_velocities: WheelAngularVelocities,
}

impl OdometryState {
    pub const fn new(config: OdometryConfig) -> Self {
        Self {
            config,
            pose: Pose2d::new(0.0, 0.0, 0.0),
            last_encoder: None,
            wheel_velocities: WheelAngularVelocities {
                left_radps: 0.0,
                right_radps: 0.0,
            },
        }
    }

    pub const fn pose(&self) -> Pose2d {
        self.pose
    }

    pub const fn wheel_velocities(&self) -> WheelAngularVelocities {
        self.wheel_velocities
    }

    pub fn reset(&mut self, pose: Pose2d, encoder: WheelEncoderSample) {
        self.pose = pose;
        self.last_encoder = Some(encoder);
        self.wheel_velocities = WheelAngularVelocities {
            left_radps: 0.0,
            right_radps: 0.0,
        };
    }

    pub fn update(
        &mut self,
        encoder: WheelEncoderSample,
        imu: Option<ImuSample>,
        elapsed_secs: f32,
    ) -> Pose2d {
        let Some(last_encoder) = self.last_encoder else {
            self.last_encoder = Some(encoder);
            return self.pose;
        };

        let left_delta_ticks = encoder.left_ticks - last_encoder.left_ticks;
        let right_delta_ticks = encoder.right_ticks - last_encoder.right_ticks;
        let meters_per_tick = self.config.meters_per_tick();
        let left_delta_m = left_delta_ticks as f32 * meters_per_tick;
        let right_delta_m = right_delta_ticks as f32 * meters_per_tick;

        let distance_m = (left_delta_m + right_delta_m) / 2.0;
        let delta_yaw_rad = (right_delta_m - left_delta_m) / self.config.drive.track_width_m;
        let yaw_mid = self.pose.yaw_rad + delta_yaw_rad / 2.0;

        self.pose.x_m += distance_m * math::cos(yaw_mid);
        self.pose.y_m += distance_m * math::sin(yaw_mid);
        self.pose.yaw_rad = normalize_angle(self.pose.yaw_rad + delta_yaw_rad);

        if let Some(imu) = imu {
            let imu_weight = self.config.imu_yaw_weight.clamp(0.0, 1.0);
            self.pose.yaw_rad =
                normalize_angle(self.pose.yaw_rad * (1.0 - imu_weight) + imu.yaw_rad * imu_weight);
        }

        if elapsed_secs > 0.0 {
            self.wheel_velocities = WheelAngularVelocities {
                left_radps: left_delta_m / self.config.drive.wheel_radius_m / elapsed_secs,
                right_radps: right_delta_m / self.config.drive.wheel_radius_m / elapsed_secs,
            };
        }

        self.last_encoder = Some(encoder);
        self.pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chassis::{DifferentialDrive, DriveLimits};

    fn test_config() -> OdometryConfig {
        OdometryConfig {
            drive: DifferentialDrive::new(0.05, 0.2, DriveLimits::new(1.0, 4.0)),
            ticks_per_wheel_rev: 100,
            imu_yaw_weight: 0.0,
        }
    }

    #[test]
    fn odometry_integrates_straight_motion() {
        let mut odometry = OdometryState::new(test_config());
        odometry.reset(Pose2d::default(), WheelEncoderSample::new(0, 0));

        let pose = odometry.update(WheelEncoderSample::new(100, 100), None, 1.0);

        assert!((pose.x_m - core::f32::consts::TAU * 0.05).abs() < 0.0001);
        assert!(pose.y_m.abs() < 0.0001);
        assert!(pose.yaw_rad.abs() < 0.0001);
    }

    #[test]
    fn odometry_integrates_in_place_rotation() {
        let mut odometry = OdometryState::new(test_config());
        odometry.reset(Pose2d::default(), WheelEncoderSample::new(0, 0));

        let pose = odometry.update(WheelEncoderSample::new(-50, 50), None, 1.0);

        assert!(pose.x_m.abs() < 0.0001);
        assert!(pose.y_m.abs() < 0.0001);
        assert!((pose.yaw_rad - core::f32::consts::FRAC_PI_2).abs() < 0.0001);
    }
}
