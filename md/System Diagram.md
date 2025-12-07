# Self-Balancing Robot System Diagram

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    SELF-BALANCING ROBOT "ANTER"                 │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────┐
│   Power Supply   │
│    (Battery)     │
│    7.4V LiPo     │
└────────┬─────────┘
         │
         ├──────────────────┐
         │                  │
         ▼                  ▼
┌─────────────────┐  ┌─────────────────┐
│  Arduino UNO    │  │  Motor Driver   │
│   (ATmega328)   │  │    L298N        │
│                 │  │                 │
│  Processing:    │  │  Power:         │
│  - PID Control  │  │  - 2 DC Motors  │
│  - Sensor Read  │  │  - PWM Control  │
│  - Bluetooth    │  │                 │
└────────┬────────┘  └────────┬────────┘
         │                    │
         │                    │
    ┌────┴────┐         ┌────┴────┐
    │         │         │         │
    ▼         ▼         ▼         ▼
```

---

## Component Interconnection Diagram

```
                         ┌─────────────────────┐
                         │   ARDUINO UNO       │
                         │                     │
    ┌───────────────────►│ Pin 2 (INT0)        │
    │                    │ SDA (A4)            │◄────┐
    │                    │ SCL (A5)            │◄────┤
    │                    │                     │     │
    │   ┌───────────────►│ Pin 5  (ENA)        │     │
    │   │ ┌─────────────►│ Pin 6  (IN1)        │     │
    │   │ │ ┌───────────►│ Pin 7  (IN2)        │     │
    │   │ │ │ ┌─────────►│ Pin 8  (IN4)        │     │
    │   │ │ │ │ ┌───────►│ Pin 9  (IN3)        │     │
    │   │ │ │ │ │ ┌─────►│ Pin 10 (ENB)        │     │
    │   │ │ │ │ │ │      │                     │     │
    │   │ │ │ │ │ │      │ Pin 11 (BUZZER)     │──┐  │
    │   │ │ │ │ │ │      │ Pin 12 (BT_RX)      │──┼──┼──┐
    │   │ │ │ │ │ │      │ Pin 13 (BT_TX)      │──┼──┼──┼──┐
    │   │ │ │ │ │ │      │                     │  │  │  │  │
    │   │ │ │ │ │ │      └─────────────────────┘  │  │  │  │
    │   │ │ │ │ │ │                               │  │  │  │
    │   │ │ │ │ │ │      ┌─────────────────────┐  │  │  │  │
    │   │ │ │ │ │ └─────►│ L298N Motor Driver  │  │  │  │  │
    │   │ │ │ │ └───────►│                     │  │  │  │  │
    │   │ │ │ └─────────►│ Controls:           │  │  │  │  │
    │   │ │ └───────────►│ - Left Motor Speed  │  │  │  │  │
    │   │ └─────────────►│ - Right Motor Speed │  │  │  │  │
    │   └───────────────►│ - Direction Control │  │  │  │  │
    │                    │                     │  │  │  │  │
    │                    │ OUT1 ────────┐      │  │  │  │  │
    │                    │ OUT2 ────────┼──┐   │  │  │  │  │
    │                    │ OUT3 ────────┼──┼─┐ │  │  │  │  │
    │                    │ OUT4 ────────┼──┼─┼─┤ │  │  │  │
    │                    └──────────────┼──┼─┼─┘ │  │  │  │
    │                                   │  │ │   │  │  │  │
    │  ┌────────────────────────────────┘  │ │   │  │  │  │
    │  │  ┌─────────────────────────────────┘ │   │  │  │  │
    │  │  │  ┌──────────────────────────────────┘  │  │  │  │
    │  │  │  │                                     │  │  │  │
    │  ▼  ▼  ▼                                     │  │  │  │
    │ ┌──────────┐        ┌──────────┐             │  │  │  │
    │ │ DC MOTOR │        │ DC MOTOR │             │  │  │  │
    │ │  (LEFT)  │        │ (RIGHT)  │             │  │  │  │
    │ │          │        │          │             │  │  │  │
    │ │  Wheel   │        │  Wheel   │             │  │  │  │
    │ └──────────┘        └──────────┘             │  │  │  │
    │                                              │  │  │  │
    │ ┌────────────────────────────────────────┐   │  │  │  │
    │ │         MPU6050 (Gyroscope)            │   │  │  │  │
    ├─│  - 6-Axis Motion Sensor                │   │  │  │  │
    │ │  - Accelerometer + Gyroscope           │   │  │  │  │
      │  - DMP (Digital Motion Processor)      │   │  │  │  │
      │  - I2C Communication (SDA, SCL)        │   │  │  │  │
      │  - Interrupt Pin → Arduino Pin 2       │   │  │  │  │
      └────────────────┬───────────────────────┘   │  │  │  │
                       │                           │  │  │  │
                       └───────────────────────────┘  │  │  │
                                                      │  │  │
      ┌───────────────────────────────────────────────┘  │  │
      │  LCD Display (16x2) I2C                          │  │
      │  - Address: 0x27                                 │  │
      │  - Shows: "Hi I'm ANTER" + "Ready!"              │  │
      │  - I2C Communication (SDA, SCL) ─────────────────┘  │
      └──────────────────────────────────────────────────────┘

      ┌─────────────────────────────────────────────────────┐
      │  HC-05 Bluetooth Module                             │
      │  - RX → Arduino Pin 12                              │
      │  - TX → Arduino Pin 13                              │
      │  - Baud Rate: 9600                                  │
      │  - Commands: '1' (Forward), '2' (Backward),         │
      │              '3' (Right), '4' (Left)                │
      └─────────────────────────────────────────────────────┘

      ┌─────────────────────────────────────────────────────┐
      │  Buzzer (Pin 11)                                    │
      │  - Alert when tilt angle > 30 degrees               │
      └─────────────────────────────────────────────────────┘
```

---

## Control Flow Diagram

```
        START
          │
          ▼
    ┌──────────┐
    │ Initialize│
    │ Hardware  │
    └─────┬────┘
          │
          ▼
    ┌─────────────┐
    │  Setup MPU  │
    │  Calibrate  │
    │  DMP Ready  │
    └─────┬───────┘
          │
          ▼
    ┌─────────────┐
    │  Setup PID  │
    │  Parameters │
    └─────┬───────┘
          │
    ┌─────▼──────┐
    │ MAIN LOOP  │◄────────────────┐
    └─────┬──────┘                 │
          │                        │
          ▼                        │
    ┌──────────────┐               │
    │ Wait for MPU │               │
    │  Interrupt   │               │
    └─────┬────────┘               │
          │                        │
    ┌─────▼────────┐               │
    │ Read MPU Data│               │
    │ (Angle)      │               │
    └─────┬────────┘               │
          │                        │
    ┌─────▼────────┐               │
    │ Check        │               │
    │ Bluetooth    │               │
    │ Command?     │               │
    └─────┬────────┘               │
          │                        │
      Yes │ No                     │
    ┌─────▼────────┐               │
    │ Process      │               │
    │ Command      │               │
    │ (1,2,3,4)    │               │
    └─────┬────────┘               │
          │                        │
    ┌─────▼────────┐               │
    │ Auto-Stop    │               │
    │ After 500ms? │               │
    └─────┬────────┘               │
          │                        │
    ┌─────▼────────┐               │
    │ Smooth       │               │
    │ Transition   │               │
    │ Setpoint     │               │
    └─────┬────────┘               │
          │                        │
    ┌─────▼────────┐               │
    │ PID Compute  │               │
    │ (Calculate   │               │
    │  Motor Speed)│               │
    └─────┬────────┘               │
          │                        │
    ┌─────▼────────┐               │
    │ Apply Turn   │               │
    │ Offset for   │               │
    │ Left/Right   │               │
    └─────┬────────┘               │
          │                        │
    ┌─────▼────────┐               │
    │ Move Motors  │               │
    │ (L298N)      │               │
    └─────┬────────┘               │
          │                        │
    ┌─────▼────────┐               │
    │ Check Angle  │               │
    │ Threshold    │               │
    └─────┬────────┘               │
          │                        │
      Yes │ No                     │
    ┌─────▼────────┐               │
    │ Activate     │               │
    │ Buzzer       │               │
    └─────┬────────┘               │
          │                        │
          └────────────────────────┘
```

---

## PID Control System

```
                   ┌─────────────────────────────────────┐
                   │       PID CONTROLLER                │
                   │                                     │
Setpoint (175°) ──►│ ┌─────────┐                        │
                   │ │  Error  │  Kp = 22               │
    ┌──────────────┼─│Calculate│  Ki = 140              │
    │              │ └────┬────┘  Kd = 1.5              │
    │              │      │                              │
    │              │      ├──► P (Proportional)          │
    │              │      │                              │
    │              │      ├──► I (Integral)    ────┐     │
    │              │      │                        │     │
    │              │      └──► D (Derivative) ─────┤     │
    │              │                               │     │
    │              │      ┌────────────────────────▼───┐ │
    │              │      │    Sum & Limit             │ │
    │              │      │    Output: -255 to +255    │ │
    │              │      └────────────┬───────────────┘ │
    │              └───────────────────┼─────────────────┘
    │                                  │
    │                                  ▼
    │                         ┌─────────────────┐
    │                         │  Motor Speed    │
    │                         │   Controller    │
    │                         └────────┬────────┘
    │                                  │
    │                         ┌────────▼────────┐
    │                         │  DC Motors      │
    │                         │  Rotate Wheels  │
    │                         └────────┬────────┘
    │                                  │
    │                         ┌────────▼────────┐
    │                         │ Robot Moves     │
    │                         │ Angle Changes   │
    │                         └────────┬────────┘
    │                                  │
    │                         ┌────────▼────────┐
    │                         │   MPU6050       │
    │                         │ Measures Angle  │
    │                         └────────┬────────┘
    │                                  │
    └──────────────────────────────────┘
            (Feedback Loop)
```

---

## Bluetooth Command Processing

```
┌─────────────────┐
│ Bluetooth RX    │
│  (Pin 12/13)    │
└────────┬────────┘
         │
         ▼
    ┌─────────┐
    │ Read    │
    │ Command │
    └────┬────┘
         │
         ├────► '1' Forward  ──► targetSetpoint = 170 (tilt forward)
         │                      targetTurnOffset = 0
         │                      Start Timer
         │
         ├────► '2' Backward ──► targetSetpoint = 180 (tilt backward)
         │                      targetTurnOffset = 0
         │                      Start Timer
         │
         ├────► '3' Right    ──► targetSetpoint = 175 (no tilt)
         │                      targetTurnOffset = +30 (right slower)
         │                      Start Timer
         │
         ├────► '4' Left     ──► targetSetpoint = 175 (no tilt)
         │                      targetTurnOffset = -30 (left slower)
         │                      Start Timer
         │
         └────► Other        ──► Stop (return to balance)
                                 targetSetpoint = 175
                                 targetTurnOffset = 0
         
         ▼
    ┌──────────────┐
    │ Wait 500ms   │
    └──────┬───────┘
           │
           ▼
    ┌──────────────┐
    │ Auto-Stop    │
    │ Return to    │
    │ Balance      │
    └──────────────┘
```

---

## Pin Connection Summary

| Component | Arduino Pin | Description |
|-----------|-------------|-------------|
| **MPU6050** | SDA (A4) | I2C Data |
| | SCL (A5) | I2C Clock |
| | INT | Pin 2 (Interrupt) |
| **L298N Motor Driver** | Pin 5 | ENA (Left Motor Speed) |
| | Pin 6 | IN1 (Left Motor Dir 1) |
| | Pin 7 | IN2 (Left Motor Dir 2) |
| | Pin 8 | IN4 (Right Motor Dir 2) |
| | Pin 9 | IN3 (Right Motor Dir 1) |
| | Pin 10 | ENB (Right Motor Speed) |
| **LCD 16x2** | SDA (A4) | I2C Data (shared with MPU) |
| | SCL (A5) | I2C Clock (shared with MPU) |
| **HC-05 Bluetooth** | Pin 12 | RX (Receive from BT) |
| | Pin 13 | TX (Transmit to BT) |
| **Buzzer** | Pin 11 | Alert Output |

---

## Power Distribution

```
┌────────────────┐
│  Battery       │
│  7.4V LiPo     │
│  (2S)          │
└───────┬────────┘
        │
        ├──────────────┐
        │              │
        ▼              ▼
┌────────────┐  ┌──────────────┐
│ L298N      │  │ Arduino UNO  │
│ (Motors)   │  │ 5V Regulator │
│ Raw Power  │  │ (via Vin)    │
└────────────┘  └──────┬───────┘
                       │
                ┌──────┴───────┐
                │              │
                ▼              ▼
        ┌────────────┐  ┌────────────┐
        │ MPU6050    │  │ HC-05      │
        │ (3.3V/5V)  │  │ (5V/3.3V)  │
        └────────────┘  └────────────┘
                │
                ▼
        ┌────────────┐
        │ LCD I2C    │
        │ (5V)       │
        └────────────┘
```

---

## System Specifications

| Parameter | Value |
|-----------|-------|
| **Control Loop Rate** | ~100 Hz (10ms) |
| **PID Sample Time** | 10 ms |
| **Bluetooth Baud Rate** | 9600 |
| **I2C Bus Speed** | 400 kHz (Fast Mode) |
| **Motor PWM Frequency** | ~490 Hz (Arduino default) |
| **Movement Duration** | 500 ms (auto-stop) |
| **Balance Angle** | 175° (calibrated) |
| **Buzzer Threshold** | ±30° from setpoint |

---

## System Features

1. **Self-Balancing**: Uses MPU6050 + PID for automatic balance
2. **Wireless Control**: 4-direction movement via Bluetooth
3. **Auto-Stop**: Stops automatically after 500ms
4. **Smooth Motion**: Gradual acceleration/deceleration
5. **Visual Feedback**: LCD shows robot name and status
6. **Audio Alert**: Buzzer warns of excessive tilt
7. **Differential Drive**: Independent control of left/right motors

---

This diagram represents the complete system architecture of the self-balancing robot "ANTER".
