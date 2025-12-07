# Self-Balancing Robot - عنتر

A two-wheeled self-balancing robot controlled by an MPU6050 gyroscope sensor and PID controller, with Bluetooth remote control capabilities.

![Robot Image](images/)

## 📋 Table of Contents
- [Self-Balancing Robot - عنتر](#self-balancing-robot---عنتر)
  - [📋 Table of Contents](#-table-of-contents)
  - [🤖 Overview](#-overview)
  - [🔧 Hardware Components](#-hardware-components)
  - [📚 Software Dependencies](#-software-dependencies)
  - [🔌 Pin Configuration](#-pin-configuration)
  - [✨ Features](#-features)
  - [🚀 Installation](#-installation)
  - [📖 Usage](#-usage)
    - [First Time Setup](#first-time-setup)
    - [Bluetooth Commands](#bluetooth-commands)
  - [⚙️ PID Tuning](#️-pid-tuning)
  - [📁 Project Structure](#-project-structure)
  - [🛠️ Troubleshooting](#️-troubleshooting)
  - [📄 License](#-license)
  - [👨‍💻 Author](#-author)

## 🤖 Overview

This project implements a self-balancing robot that maintains its upright position using feedback from an MPU6050 sensor and a PID control algorithm. The robot can be controlled remotely via Bluetooth and displays real-time status information on an LCD screen.

## 🔧 Hardware Components

- **Microcontroller**: Arduino (Uno/Mega)
- **Motion Sensor**: MPU6050 (6-axis gyroscope + accelerometer)
- **Motor Driver**: L298N H-Bridge
- **Motors**: 2x DC Motors
- **Display**: 16x2 I2C LCD (Address: 0x27)
- **Bluetooth Module**: HC-05 (or compatible)
- **Buzzer**: Warning buzzer
- **Power Supply**: Battery pack (recommended 7-12V)

## 📚 Software Dependencies

The following libraries are required (included in the `Libraries/` folder):

- **PID_v1**: PID controller implementation
- **LMotorController**: Motor control library
- **I2Cdev**: I2C communication library
- **MPU6050**: Motion sensor library with DMP support
- **LiquidCrystal_I2C**: LCD display library (install via Arduino Library Manager)
- **SoftwareSerial**: Bluetooth communication (built-in Arduino library)

## 🔌 Pin Configuration

| Component | Arduino Pin | Description |
|-----------|-------------|-------------|
| MPU6050 | SDA, SCL | I2C communication (A4, A5 on Uno) |
| LCD | SDA, SCL | I2C communication (A4, A5 on Uno) |
| Bluetooth RX | D12 | Serial receive |
| Bluetooth TX | D13 | Serial transmit |
| Buzzer | D11 | Warning buzzer output |
| Motor A (ENA) | D5 | PWM speed control |
| Motor A (IN1) | D6 | Direction control |
| Motor A (IN2) | D7 | Direction control |
| Motor B (IN3) | D8 | Direction control |
| Motor B (IN4) | D9 | Direction control |
| Motor B (ENB) | D10 | PWM speed control |

## ✨ Features

- **Self-Balancing**: Automatic balance maintenance using PID control
- **Bluetooth Control**: Remote control via smartphone app
- **Real-time Display**: LCD shows angle, state, and commands
- **Auto-Stop**: Automatic stop after 2 seconds of inactivity
- **Warning System**: Buzzer alerts when tilt exceeds threshold (30°)
- **Smooth Movements**: Gradual transitions for setpoint and turning
- **I2C Scanner**: Built-in test for detecting I2C devices

## 🚀 Installation

1. **Clone or download** this repository
2. **Install Arduino IDE** (version 1.8.x or higher)
3. **Install required libraries**:
   - Copy the `Libraries/` folder contents to your Arduino libraries directory
   - Or add them via Arduino IDE: Sketch → Include Library → Add .ZIP Library
   - Install `LiquidCrystal_I2C` via Library Manager

4. **Hardware setup**:
   - Connect components according to the pin configuration table
   - Ensure MPU6050 is securely mounted and level
   - Double-check motor driver connections

5. **Upload the code**:
   - Open `Self_balancing_robot.ino` in Arduino IDE
   - Select your board and port
   - Upload the sketch

## 📖 Usage

### First Time Setup

1. **I2C Scanner Test**:
   - Upload `test/I2C_Scanner/I2C_Scanner.ino` to verify MPU6050 and LCD addresses
   - Open Serial Monitor (9600 baud) to see detected devices

2. **MPU6050 Calibration**:
   - Upload `test/MPU6050_Test/MPU6050_Test.ino`
   - Place robot on level surface
   - Note the angle reading when balanced (adjust `originalSetpoint` if needed)

3. **Main Program**:
   - Upload `Self_balancing_robot.ino`
   - Place robot upright and power on
   - Wait for LCD to show "Ready"

### Bluetooth Commands

Connect via Bluetooth (default pairing code: 1234 or 0000):

| Command | Action |
|---------|--------|
| `F` | Move forward |
| `B` | Move backward |
| `L` | Turn left |
| `R` | Turn right |
| `0` | Stop |

## ⚙️ PID Tuning

Default PID values (in code):
```cpp
Kp = 22;    // Proportional gain
Kd = 1.5;   // Derivative gain
Ki = 140;   // Integral gain
```

**Tuning Tips**:
- Start with `Ki = 0` and `Kd = 0`
- Increase `Kp` until robot oscillates
- Add `Kd` to reduce oscillation
- Add `Ki` to eliminate steady-state error
- Adjust `originalSetpoint` (around 175) for perfect balance

## 📁 Project Structure

```
Self_balancing_robot/
├── Self_balancing_robot.ino    # Main program
├── Libraries/                   # Custom libraries
│   ├── I2Cdev/
│   ├── LMotorController/
│   ├── MPU6050/
│   └── PID_v1/
├── test/                        # Test sketches
│   ├── I2C_Scanner/
│   ├── MPU6050_Raw_Test/
│   └── MPU6050_Test/
├── md/                          # Documentation
│   ├── System Diagram.md
│   └── System Explain.md        # Detailed explanation (Arabic)
├── images/                      # Project images
└── README.md                    # This file
```

## 🛠️ Troubleshooting

**Robot won't balance:**
- Check MPU6050 connection and I2C address
- Verify motor connections (may need to swap wires)
- Adjust PID values
- Ensure battery is fully charged

**LCD shows nothing:**
- Check I2C address (0x27 or 0x3F)
- Verify I2C wiring (SDA, SCL)
- Adjust LCD contrast potentiometer

**Bluetooth not connecting:**
- Check RX/TX connections (may be swapped)
- Verify baud rate (9600)
- Ensure Bluetooth module is powered

**MPU6050 initialization fails:**
- Check I2C wiring
- Try different I2C address (0x68 or 0x69)
- Ensure proper power supply (3.3V or 5V)

## 📄 License

This project is open source and available for educational purposes.

## 👨‍💻 Author

Ahmed - Self-Balancing Robot Project

---

**Note**: For detailed technical explanation in Arabic, see `md/System Explain.md`
