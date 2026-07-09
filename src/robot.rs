use crate::chassis::{Chassis, ChassisError, Twist};

#[derive(Debug)]
pub struct Robot<C> {
    chassis: C,
}

impl<C> Robot<C>
where
    C: Chassis,
{
    pub const fn new(chassis: C) -> Self {
        Self { chassis }
    }

    pub fn drive(&mut self, linear_mps: f32, angular_radps: f32) -> Result<(), ChassisError> {
        self.chassis.drive(Twist::new(linear_mps, angular_radps))
    }

    pub fn stop(&mut self) -> Result<(), ChassisError> {
        self.chassis.stop()
    }

    pub fn into_chassis(self) -> C {
        self.chassis
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chassis::DifferentialDrive;
    use crate::drivers::mock::MockChassis;

    #[test]
    fn robot_forwards_motion_commands_to_chassis() {
        let chassis = MockChassis::new(DifferentialDrive::default());
        let mut robot = Robot::new(chassis);

        robot.drive(0.2, 0.5).unwrap();
        let chassis = robot.into_chassis();

        assert_eq!(chassis.last_twist(), Twist::new(0.2, 0.5));
    }
}
