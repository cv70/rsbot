pub const PI: f32 = core::f32::consts::PI;
pub const TAU: f32 = core::f32::consts::TAU;
pub const FRAC_PI_2: f32 = core::f32::consts::FRAC_PI_2;

pub fn normalize_angle(angle_rad: f32) -> f32 {
    let mut angle = angle_rad;

    while angle > PI {
        angle -= TAU;
    }

    while angle < -PI {
        angle += TAU;
    }

    angle
}

pub fn sqrt(value: f32) -> f32 {
    if value <= 0.0 {
        return 0.0;
    }

    let mut estimate = if value >= 1.0 { value } else { 1.0 };

    for _ in 0..8 {
        estimate = 0.5 * (estimate + value / estimate);
    }

    estimate
}

pub fn sin(angle_rad: f32) -> f32 {
    let x = normalize_angle(angle_rad);
    let x2 = x * x;

    x * (1.0 - x2 / 6.0 + x2 * x2 / 120.0 - x2 * x2 * x2 / 5040.0)
}

pub fn cos(angle_rad: f32) -> f32 {
    sin(angle_rad + FRAC_PI_2)
}

pub fn atan2(y: f32, x: f32) -> f32 {
    if x == 0.0 {
        return if y > 0.0 {
            FRAC_PI_2
        } else if y < 0.0 {
            -FRAC_PI_2
        } else {
            0.0
        };
    }

    let abs_y = y.abs() + 0.000_000_1;
    let angle = if x >= 0.0 {
        let r = (x - abs_y) / (x + abs_y);
        FRAC_PI_2 / 2.0 - FRAC_PI_2 / 2.0 * r
    } else {
        let r = (x + abs_y) / (abs_y - x);
        3.0 * FRAC_PI_2 / 2.0 - FRAC_PI_2 / 2.0 * r
    };

    if y < 0.0 { -angle } else { angle }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sqrt_estimates_square_root() {
        assert!((sqrt(4.0) - 2.0).abs() < 0.0001);
        assert!((sqrt(0.25) - 0.5).abs() < 0.0001);
    }

    #[test]
    fn trig_estimates_cardinal_angles() {
        assert!(sin(0.0).abs() < 0.0001);
        assert!((sin(FRAC_PI_2) - 1.0).abs() < 0.001);
        assert!((cos(0.0) - 1.0).abs() < 0.001);
    }

    #[test]
    fn atan2_estimates_cardinal_angles() {
        assert!(atan2(0.0, 1.0).abs() < 0.0001);
        assert!((atan2(1.0, 0.0) - FRAC_PI_2).abs() < 0.0001);
    }
}
