use crate::chassis::{Chassis, ChassisError, Twist};
use crate::control::{AccelerationLimits, MotionLimiter};
use crate::time::Millis;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultCode {
    CommandTimeout,
    EmergencyStop,
    LowBattery,
    CommunicationFault,
    MotorFault(u16),
    ControllerFault(u16),
}

impl FaultCode {
    pub const fn as_u16(self) -> u16 {
        match self {
            Self::CommandTimeout => 1,
            Self::EmergencyStop => 2,
            Self::LowBattery => 3,
            Self::CommunicationFault => 4,
            Self::MotorFault(code) => 1000 + code,
            Self::ControllerFault(code) => code,
        }
    }

    pub const fn from_u16(code: u16) -> Option<Self> {
        match code {
            0 => None,
            1 => Some(Self::CommandTimeout),
            2 => Some(Self::EmergencyStop),
            3 => Some(Self::LowBattery),
            4 => Some(Self::CommunicationFault),
            1000..=1999 => Some(Self::MotorFault(code - 1000)),
            other => Some(Self::ControllerFault(other)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SafetyConfig {
    pub command_timeout: Millis,
    pub acceleration_limits: Option<AccelerationLimits>,
    pub min_battery_voltage: Option<f32>,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            command_timeout: Millis::from_millis(300),
            acceleration_limits: None,
            min_battery_voltage: None,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SafetyStatus {
    pub emergency_stop: bool,
    pub command_timed_out: bool,
    pub battery_voltage: Option<f32>,
    pub fault: Option<FaultCode>,
}

#[derive(Debug)]
pub struct SafeChassis<C> {
    inner: C,
    config: SafetyConfig,
    status: SafetyStatus,
    limiter: Option<MotionLimiter>,
    last_command_at: Option<Millis>,
    last_update_at: Option<Millis>,
}

impl<C> SafeChassis<C> {
    pub fn new(inner: C, config: SafetyConfig) -> Self {
        Self {
            inner,
            config,
            status: SafetyStatus::default(),
            limiter: config.acceleration_limits.map(MotionLimiter::new),
            last_command_at: None,
            last_update_at: None,
        }
    }

    pub const fn inner(&self) -> &C {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut C {
        &mut self.inner
    }

    pub fn into_inner(self) -> C {
        self.inner
    }

    pub const fn config(&self) -> SafetyConfig {
        self.config
    }

    pub const fn status(&self) -> SafetyStatus {
        self.status
    }

    pub fn set_emergency_stop(&mut self, active: bool) {
        self.status.emergency_stop = active;

        if active {
            self.status.fault = Some(FaultCode::EmergencyStop);
        } else if self.status.fault == Some(FaultCode::EmergencyStop) {
            self.status.fault = None;
        }
    }

    pub fn set_fault(&mut self, fault: Option<FaultCode>) {
        self.status.fault = fault;
    }

    pub fn update_battery_voltage(&mut self, voltage: f32) {
        self.status.battery_voltage = Some(voltage);

        if self
            .config
            .min_battery_voltage
            .is_some_and(|minimum| voltage < minimum)
        {
            self.status.fault = Some(FaultCode::LowBattery);
        }
    }
}

impl<C> SafeChassis<C>
where
    C: Chassis,
{
    pub fn drive_at(&mut self, twist: Twist, now: Millis) -> Result<(), ChassisError> {
        self.stop_if_timed_out(now)?;
        self.ensure_can_drive()?;

        let target = self.apply_acceleration_limit(twist, now);
        self.inner.drive(target)?;

        self.status.command_timed_out = false;
        self.last_command_at = Some(now);
        self.last_update_at = Some(now);

        Ok(())
    }

    pub fn stop_at(&mut self, now: Millis) -> Result<(), ChassisError> {
        if let Some(limiter) = &mut self.limiter {
            limiter.reset(Twist::stop());
        }

        self.inner.stop()?;
        self.status.command_timed_out = false;
        self.last_command_at = Some(now);
        self.last_update_at = Some(now);

        Ok(())
    }

    pub fn check_timeout_at(&mut self, now: Millis) -> Result<(), ChassisError> {
        self.stop_if_timed_out(now)
    }

    fn ensure_can_drive(&self) -> Result<(), ChassisError> {
        if self.status.emergency_stop {
            return Err(ChassisError::EmergencyStopActive);
        }

        match self.status.fault {
            Some(FaultCode::LowBattery) => Err(ChassisError::LowBattery),
            Some(_) => Err(ChassisError::FaultActive),
            None => Ok(()),
        }
    }

    fn apply_acceleration_limit(&mut self, target: Twist, now: Millis) -> Twist {
        let Some(limiter) = &mut self.limiter else {
            return target;
        };

        let Some(last_update_at) = self.last_update_at else {
            limiter.reset(target);
            return target;
        };

        limiter.apply(
            target,
            now.saturating_sub(last_update_at).as_millis() as f32 / 1000.0,
        )
    }

    fn stop_if_timed_out(&mut self, now: Millis) -> Result<(), ChassisError> {
        let Some(last_command_at) = self.last_command_at else {
            return Ok(());
        };

        if now.saturating_sub(last_command_at) <= self.config.command_timeout {
            return Ok(());
        }

        if !self.status.command_timed_out {
            self.inner.stop()?;
            if let Some(limiter) = &mut self.limiter {
                limiter.reset(Twist::stop());
            }
            self.last_update_at = Some(now);
        }

        self.status.command_timed_out = true;
        Ok(())
    }
}

impl<C> Chassis for SafeChassis<C>
where
    C: Chassis,
{
    fn drive(&mut self, twist: Twist) -> Result<(), ChassisError> {
        self.drive_at(twist, Millis::ZERO)
    }

    fn stop(&mut self) -> Result<(), ChassisError> {
        self.stop_at(Millis::ZERO)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chassis::DifferentialDrive;
    use crate::drivers::mock::MockChassis;

    #[test]
    fn safe_chassis_stops_inner_chassis_after_command_timeout() {
        let start = Millis::from_millis(1_000);
        let inner = MockChassis::new(DifferentialDrive::default());
        let mut chassis = SafeChassis::new(
            inner,
            SafetyConfig {
                command_timeout: Millis::from_millis(100),
                ..SafetyConfig::default()
            },
        );

        chassis.drive_at(Twist::new(0.2, 0.0), start).unwrap();
        chassis
            .check_timeout_at(Millis::from_millis(1_101))
            .unwrap();

        assert!(chassis.status().command_timed_out);
        assert_eq!(chassis.inner().last_twist(), Twist::stop());
    }

    #[test]
    fn safe_chassis_rejects_motion_while_emergency_stop_is_active() {
        let inner = MockChassis::new(DifferentialDrive::default());
        let mut chassis = SafeChassis::new(inner, SafetyConfig::default());

        chassis.set_emergency_stop(true);

        assert_eq!(
            chassis.drive(Twist::new(0.1, 0.0)),
            Err(ChassisError::EmergencyStopActive)
        );
    }

    #[test]
    fn clearing_emergency_stop_preserves_unrelated_faults() {
        let inner = MockChassis::new(DifferentialDrive::default());
        let mut chassis = SafeChassis::new(inner, SafetyConfig::default());

        chassis.set_fault(Some(FaultCode::LowBattery));
        chassis.set_emergency_stop(false);

        assert_eq!(chassis.status().fault, Some(FaultCode::LowBattery));
    }

    #[test]
    fn safe_chassis_applies_acceleration_limits_after_first_command() {
        let start = Millis::from_millis(1_000);
        let inner = MockChassis::new(DifferentialDrive::new(
            0.05,
            0.2,
            crate::chassis::DriveLimits::new(10.0, 10.0),
        ));
        let mut chassis = SafeChassis::new(
            inner,
            SafetyConfig {
                acceleration_limits: Some(AccelerationLimits::new(1.0, 2.0)),
                ..SafetyConfig::default()
            },
        );

        chassis.drive_at(Twist::stop(), start).unwrap();
        chassis
            .drive_at(Twist::new(10.0, 10.0), Millis::from_millis(1_100))
            .unwrap();

        assert_eq!(chassis.inner().last_twist(), Twist::new(0.1, 0.2));
    }
}
