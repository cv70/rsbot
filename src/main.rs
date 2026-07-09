use rsbot::chassis::{ChassisError, DifferentialDrive, DriveLimits};
use rsbot::drivers::mock::MockBoard;
use rsbot::navigation::NavigationCommand;
use rsbot::odometry::Pose2d;
use rsbot::runtime::{RobotRuntime, RuntimeConfig};
use rsbot::time::Millis;

fn main() -> Result<(), ChassisError> {
    let drive = DifferentialDrive::new(0.033, 0.160, DriveLimits::new(0.4, 2.5));
    let config = RuntimeConfig {
        drive,
        ticks_per_wheel_rev: 1024,
        ..RuntimeConfig::default()
    };
    let mut robot = RobotRuntime::new(MockBoard::default(), config);

    robot.set_command(
        NavigationCommand::GoTo {
            target: Pose2d::new(0.5, 0.0, 0.0),
        },
        Millis::from_millis(0),
    );

    for tick in 1..=5 {
        robot.tick(Millis::from_millis(tick * 20))?;
    }

    robot.stop(Millis::from_millis(120))?;

    Ok(())
}
