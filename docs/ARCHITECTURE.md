# System Architecture

This document describes the layered software/firmware architecture for both robots and the communication contracts between components.

---

## 1. Design principles

| Principle | How it is applied |
|---|---|
| Separation of concerns | Perception, planning/control, and actuation are independent ROS nodes (R1) or modular functions (R2) |
| Shared autonomy | Driver commands velocity + heading target; firmware/ROS closes the heading loop |
| Deterministic I/O | Encoder sampling is FreeRTOS-pinned (R1) or bare-metal polled (R2) |
| Fail-safe actuators | `motor_driver.py` zeros all outputs on ROS shutdown |

---

## 2. R1 Offense — layered stack

```mermaid
flowchart TB
    subgraph L4["Layer 4 — Operator"]
        Driver["Human driver + PS4 DualShock 4"]
    end

    subgraph L3["Layer 3 — Application / ROS"]
        JB["joystick_bridge.py"]
        IP["imu_pipeline.py"]
        EB["encoder_bridge.py"]
        MC["motion_controller.py"]
        MD["motor_driver.py"]
    end

    subgraph L2["Layer 2 — Middleware MCU"]
        Odom["ESP32 FreeRTOS odometry"]
        Upper["ESP32 upper_control"]
    end

    subgraph L1["Layer 1 — Hardware"]
        Motors["4× brushed DC"]
        BNO["BNO055 IMU"]
        Enc["3× quadrature encoders"]
        Act["BLDC · Stepper · Pneumatics"]
    end

    Driver --> JB
    JB --> MC
    BNO --> IP --> MC
    Enc --> Odom --> EB --> MC
    MC --> MD --> Motors
    Driver --> MD
    MD --> Act
    Upper --> Act
```

### 2.1 ROS topic contract

| Topic | Type | Publisher | Subscriber | Payload |
|---|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | `joy` node | `joystick_bridge`, `motor_driver` | Raw axes/buttons |
| `/ps4_data` | `geometry_msgs/Twist` | `joystick_bridge` | `motion_controller` | `linear.x/y` velocity, `angular.z` heading target |
| `/bno055_data` | `std_msgs/Float32` | `imu_pipeline` | `motion_controller` | Filtered yaw (degrees) |
| `/encoder_distance` | `std_msgs/Float32MultiArray` | `encoder_bridge` | `motion_controller` | `[x_cm, y_cm]` |
| `/motor_value` | `std_msgs/Int32MultiArray` | `motion_controller` | `motor_driver` | 4 signed wheel speeds |
| `/encoder` | `std_msgs/Float32MultiArray` | `motion_controller` | (odometry consumers) | Corrected world pose |

### 2.2 Timing

```mermaid
sequenceDiagram
    participant Joy as joy @ ~60 Hz
    participant JB as joystick_bridge
    participant IMU as imu_pipeline @ 50 Hz
    participant MC as motion_controller @ 50 Hz
    participant MD as motor_driver
    participant ESP as ESP32 odom @ 100 Hz
    participant EB as encoder_bridge @ 50 Hz

    Joy->>JB: /joy
    JB->>MC: /ps4_data
    IMU->>MC: /bno055_data
    ESP->>EB: USART packet
    EB->>MC: /encoder_distance
    MC->>MD: /motor_value
    Note over MC: field-centric transform → PI heading → 4-wheel IK
```

### 2.3 USART — ESP32 odometry → Pi

Binary EasyTransfer struct from firmware (`esp32_odometry_rtos.ino`):

```c
struct SEND_READING {
    double currentDist_x;  // world-frame X (cm)
    double currentDist_y;  // world-frame Y (cm)
    double angle;          // heading (rad)
};
```

The ROS `encoder_bridge.py` also documents an ASCII fallback form `X:<int>, Y:<int>` for debug/serial parsing at 115200 baud on `/dev/serial0`.

### 2.4 Serial — Upper control commands

| Command | Example | Effect |
|---|---|---|
| `T <us>` | `T 1500` | ESC throttle (1000–2000 µs) |
| `A <deg>` | `A 30.5` | Stepper target angle |
| `D` | `D` | Pneumatic dribble sequence |

---

## 3. R2 Defense — bare-metal architecture

```mermaid
flowchart TB
    PS4["PS4 DualShock"] --> ESP["ESP32 Bluetooth receiver"]
    ESP -->|"USART EasyTransfer"| Due["Arduino Due main loop"]

    Due --> Block1["Deadzone + scale sticks"]
    Block1 --> Block2["Heading lock P-control"]
    Block2 --> Block3["3-wheel inverse kinematics"]
    Block3 --> Block4["Proportional PWM scaling"]
    Block4 --> M["Motors M1–M3"]

    Due --> Net["Triangle rising-edge → net deploy"]
    Net --> Enc["Stop at 700 encoder ticks"]
    Due --> Pne["Pneumatic valves"]
```

### 3.1 EasyTransfer PS4 packet

```c
struct PS4_DATA {
    int  x;        // left stick X (±100)
    int  y;        // left stick Y
    int  buttons;  // packed button flags
    float angle;   // gyro heading for lock
};
```

### 3.2 Why bare-metal on R2?

R2’s control path is strictly sequential: *receive → decide → actuate*. There is no concurrent sensor fusion pipeline that benefits from an RTOS. Bare-metal on the Due yields lower latency and simpler timing for match-critical motor response.

---

## 4. Cross-robot comparison

```mermaid
flowchart LR
    subgraph Shared["Shared design"]
        A["PS4 teleop"]
        B["Active heading lock"]
        C["Omni IK + PWM scaling"]
        D["EasyTransfer USART"]
    end

    subgraph Diff["Divergent stack"]
        E["R1: ROS + FreeRTOS + field-centric"]
        F["R2: bare-metal Due + net mechanism"]
    end

    Shared --- Diff
```

---

## Related documents

- [CONTROL_SYSTEMS.md](CONTROL_SYSTEMS.md) — PID, filters, IK equations  
- [HARDWARE.md](HARDWARE.md) — sensors, actuators, pin maps  
- [../README.md](../README.md) — abstract and quick start  
