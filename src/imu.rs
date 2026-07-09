#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ImuSample {
    pub yaw_rad: f32,
    pub yaw_rate_radps: f32,
    pub accel_x_mps2: f32,
    pub accel_y_mps2: f32,
}

impl ImuSample {
    pub const fn new(
        yaw_rad: f32,
        yaw_rate_radps: f32,
        accel_x_mps2: f32,
        accel_y_mps2: f32,
    ) -> Self {
        Self {
            yaw_rad,
            yaw_rate_radps,
            accel_x_mps2,
            accel_y_mps2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeadingFusion {
    imu_weight: f32,
}

impl HeadingFusion {
    pub fn new(imu_weight: f32) -> Self {
        Self {
            imu_weight: imu_weight.clamp(0.0, 1.0),
        }
    }

    pub fn fuse(&self, odometry_yaw_rad: f32, imu_yaw_rad: f32) -> f32 {
        math::normalize_angle(
            odometry_yaw_rad * (1.0 - self.imu_weight) + imu_yaw_rad * self.imu_weight,
        )
    }
}

impl Default for HeadingFusion {
    fn default() -> Self {
        Self::new(0.1)
    }
}

pub use crate::math::normalize_angle;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalizes_angle_to_signed_pi_range() {
        let angle = normalize_angle(core::f32::consts::PI + 0.2);

        assert!((angle + core::f32::consts::PI - 0.2).abs() < 0.0001);
    }

    #[test]
    fn heading_fusion_blends_odometry_and_imu() {
        let fusion = HeadingFusion::new(0.25);

        assert!((fusion.fuse(1.0, 2.0) - 1.25).abs() < 0.0001);
    }
}
use crate::math;
