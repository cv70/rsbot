# rsbot 系统架构设计

## 1. 目标

rsbot 是一个基于 Rust 2024 edition 的机器人项目。当前架构目标是
ESP32 纯 MCU 控制系统，而不是 Linux 主机加 MCU 的分布式系统。

第一版机器人形态：

- 下半身：两轮差速小车底盘，加一个或多个万向轮。
- 主控：ESP32。
- 调度模型：RTIC 风格的静态任务、优先级和固定周期调度。
- 实现方式：使用 ESP32 可落地的 HAL、定时器和中断机制，不强制依赖
  RTIC crate。
- 电机接口：PWM + 方向 GPIO + AB 相编码器。
- 导航传感器：编码器 + IMU。
- 导航能力：航迹推算、航向控制、短距离航点移动。

第一版不包含 SLAM、全局地图、视觉导航、复杂避障或上位机规划。

## 2. 当前代码状态

当前核心库已经按 MCU 迁移方向组织：

```text
src/
  lib.rs          no_std 核心库入口。
  time.rs         MCU 友好的 Millis 和 ControlTime。
  math.rs         no_std 浮点近似数学函数。
  chassis.rs      Twist、差速运动学、底盘 trait 和错误类型。
  control.rs      加速度限制和速度平滑。
  safety.rs       急停、命令超时、低电压和故障状态。
  odometry.rs     编码器/IMU 里程计。
  imu.rs          IMU 数据模型和航向融合入口。
  navigation.rs   机器人模式、导航命令和航点控制器。
  motor.rs        左右轮速度 P 控制和 PWM duty 输出。
  runtime.rs      导航、里程计、安全、电机输出的完整 MCU tick 闭环。
  board.rs        PWM、电机、编码器、IMU、电池的板级 trait。
  protocol.rs     速度命令和遥测协议模型。
  telemetry.rs    底盘遥测模型。
  drivers/        mock 和 transport 边界。
  main.rs         桌面 demo，使用 mock 底盘。
```

`src/lib.rs` 使用 `#![no_std]`。桌面 demo 仍然可以通过 `cargo run` 运行，
但核心控制逻辑不依赖 `std::time`、`println!` 或 `std::error::Error`。

## 3. 分层架构

```text
Application / Mission
  - idle
  - manual velocity control
  - waypoint navigation
  - fault and emergency-stop handling
          |
          v
Navigation
  - NavigationCommand
  - RobotMode
  - NavigationController -> Twist
          |
          v
Safety and Control
  - command timeout
  - emergency stop
  - fault lockout
  - velocity and acceleration limits
          |
          v
Chassis and Kinematics
  - Chassis trait
  - Twist
  - DifferentialDrive
  - wheel velocity conversion
          |
          v
Wheel Speed Control
  - target wheel angular velocity
  - measured wheel angular velocity
  - P controller -> motor duty
          |
          v
Board / HAL Boundary
  - MotorPwm
  - EncoderReader
  - ImuReader
  - BatteryReader
          |
          v
ESP32 Hardware
  - PWM and direction GPIO
  - AB phase encoder counters
  - I2C/SPI IMU
  - ADC battery measurement
  - hardware emergency stop input
```

核心约束：

- 高层只发送 `Twist` 或 `NavigationCommand`。
- 高层不直接设置左右轮 PWM。
- 安全层必须靠近底盘输出边界。
- `RobotRuntime` 串联传感器读取、里程计、导航、安全、轮速控制和 PWM 输出。
- 板级外设通过 trait 注入，不让控制算法绑定具体 ESP32 crate。
- 核心算法保持固定容量、无堆优先、非阻塞、可测试。

## 4. 关键数据模型

### 4.1 速度命令

```rust
pub struct Twist {
    pub linear_mps: f32,
    pub angular_radps: f32,
}
```

`Twist` 是底盘移动的统一接口。差速底盘转换公式：

```text
left_mps  = linear_mps - angular_radps * track_width_m / 2
right_mps = linear_mps + angular_radps * track_width_m / 2
wheel_radps = wheel_mps / wheel_radius_m
```

### 4.2 位姿和里程计

```rust
pub struct Pose2d {
    pub x_m: f32,
    pub y_m: f32,
    pub yaw_rad: f32,
}
```

里程计使用左右轮编码器增量积分位姿，并可使用 IMU yaw 做低权重融合。

### 4.3 导航命令

```rust
pub enum NavigationCommand {
    Stop,
    Velocity(Twist),
    FaceHeading { yaw_rad: f32 },
    GoTo { target: Pose2d },
}
```

导航控制器输入当前 `Pose2d`，输出 `Twist`。第一版导航只做局部航点控制，
不维护地图。

### 4.4 安全状态

安全层统一处理：

- 急停。
- 命令超时。
- 低电压。
- 通信故障。
- 电机故障。
- 控制器故障。

故障激活时，导航层应输出停止，底盘层应拒绝继续运动。

### 4.5 运行时闭环

`RobotRuntime` 是 MCU 主控闭环的核心编排器：

```text
tick(now)
  -> read battery
  -> check command timeout and faults
  -> read encoders
  -> read IMU
  -> update odometry
  -> navigation.update(pose) -> Twist
  -> safety clamp / acceleration limit / stop
  -> DifferentialDrive -> target wheel rad/s
  -> WheelSpeedController -> left/right MotorCommand
  -> MotorPwm output
```

桌面 demo 使用 `MockBoard` 运行相同闭环，真实 ESP32 只需要实现 board trait。

## 5. MCU 运行模型

目标运行模型采用固定周期任务。具体实现可用 ESP32 定时器、中断和 HAL
组合完成。

建议初始频率：

```text
1 kHz   motor task
        - 更新 PWM 输出
        - 读取编码器增量
        - 处理硬件急停输入

200 Hz  imu task
        - 读取 IMU yaw / yaw rate
        - 检测 IMU 采样异常

100 Hz  odometry/control task
        - 编码器 + IMU 里程计积分
        - 速度限制和加速度限制
        - 生成左右轮目标

50 Hz   navigation task
        - 执行 NavigationController
        - 将目标航点转换为 Twist

10 Hz   telemetry task
        - 输出电池、电机、故障、位姿和速度状态
```

任务之间应传递小型值类型或固定容量状态，不在实时路径上分配堆内存。

## 6. ESP32 板级边界

核心库只定义 trait：

- `MotorPwm`: 设置左右电机 duty，范围 `-1.0..=1.0`。
- `EncoderReader`: 读取左右编码器累计计数。
- `ImuReader`: 读取 IMU 样本。
- `BatteryReader`: 读取电池电压。

ESP32 具体实现后续应放在 board/HAL 适配层。该层负责：

- 选择 PWM 通道和方向 GPIO。
- 配置编码器计数外设或 GPIO 中断。
- 配置 I2C/SPI IMU。
- 配置 ADC 电池采样。
- 把硬件错误转换为 `ChassisError` 或 `FaultCode`。

当前已有 `MockBoard`，用于在桌面环境验证完整闭环。ESP32 适配层应实现同样
的 trait，而不是改动导航、里程计或安全算法。

## 7. 安全策略

必须从第一版硬件开始实现：

- 急停输入优先级最高，触发后一个控制周期内停止 PWM。
- 命令超时默认 300 ms，超时后输出停止。
- 低电压进入故障状态。
- 编码器或 IMU 数据异常进入故障状态。
- 故障未清除前不能恢复运动。
- 加速度限制用于避免打滑、冲击电机或使结构失稳。

安全策略应同时存在于导航状态机和底盘输出边界。导航负责不再生成运动
意图，底盘负责最终拒绝危险命令。

## 8. 后续实现路线

### Phase 1: 核心算法和桌面仿真

- 保持 `no_std` 核心库可编译。
- 使用 mock 底盘和 mock board 测试运动学、安全、里程计、导航和电机输出。
- 在桌面 demo 中串联导航、里程计、安全控制和 PWM 输出。

### Phase 2: ESP32 板级适配

- 创建 ESP32 board crate 或 board 模块。
- 实现 PWM、方向 GPIO、编码器、IMU、电池读取。
- 接入硬件急停。
- 用低 duty 验证左右电机方向和编码器符号。

### Phase 3: 闭环底盘控制

- 当前已有第一版左右轮 P 控制器。
- 后续根据真实电机响应替换为 PI/PID 控制器。
- 标定轮半径、轮距、ticks per revolution。
- 验证 1 m 直行和 360 度旋转误差。

### Phase 4: 导航 v1

- 使用航迹推算执行短距离 `GoTo`。
- 支持 `FaceHeading` 和停止阈值。
- 加入故障注入测试和低电压/急停验收。

## 9. 验收标准

- `cargo test` 通过所有核心算法测试。
- 核心库保持 `#![no_std]`。
- 急停触发后一个控制周期内停止电机输出。
- 命令超时后自动停车。
- 低速直行 1 m 的里程计误差可测量并可校准。
- 原地旋转 360 度的航向误差可测量并可校准。
- 机器人能执行一个短距离航点移动并在目标附近停止。
