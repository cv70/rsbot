# rsbot

Rust 2024 edition robot project starter for a wheeled MCU-controlled robot.

The first supported body plan is a differential-drive lower body controlled by
an ESP32-class MCU. The core library is `no_std`-friendly: high-level code sends
linear/angular velocity or waypoint commands, while the chassis, safety,
odometry, and navigation layers keep hardware details isolated.

## Run

```sh
cargo run
```

## Test

```sh
cargo test
```

## Structure

- `src/chassis.rs`: differential-drive geometry, velocity limits, and chassis trait.
- `src/control.rs`: acceleration limits and motion smoothing primitives.
- `src/safety.rs`: safety wrapper for emergency stop, timeout, low battery, and faults.
- `src/odometry.rs`: wheel-encoder and IMU-assisted pose estimation.
- `src/navigation.rs`: robot modes and waypoint/heading navigation controller.
- `src/motor.rs`: wheel-speed controller that converts wheel speed error to PWM duty.
- `src/runtime.rs`: complete MCU tick loop tying board I/O, odometry, navigation, safety, and motors.
- `src/board.rs`: board-level traits for motors, encoders, IMU, and battery.
- `src/time.rs`: MCU-friendly time types.
- `src/telemetry.rs`: structured chassis telemetry.
- `src/protocol.rs`: host-to-controller command and telemetry message types.
- `src/drivers/`: mock board/chassis and generic motor-controller transport boundary.
- `src/robot.rs`: high-level robot API over any chassis implementation.
- `src/main.rs`: runnable demo using `MockBoard` and the full runtime loop.
- `docs/architecture.md`: system architecture for the ESP32 MCU target.

## Next Hardware Step

Implement the board traits for the selected ESP32 HAL: PWM/direction GPIO for
the motors, encoder counts, IMU samples, battery voltage, and emergency stop
input. The runtime loop is already in place and can run against `MockBoard`.
