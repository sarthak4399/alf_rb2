# Line Following Robot with PID Control

## Overview

This project implements a **Line Following Robot** that uses PID control for precise navigation along a predefined path. The robot has four wheels and eight line sensors, with motor control handled via an Arduino-compatible microcontroller. The bot's performance is optimized using PID tuning and the Ziegler-Nichols method.

## Features

- **PID Control**: Ensures accurate navigation by dynamically adjusting motor speeds based on sensor feedback.
- **Oscillation Tuning**: Automated tuning of PID constants using the Ziegler-Nichols method.
- **Sensor Feedback**: Eight line sensors detect the position of the bot relative to the line.
- **Four-Wheel Drive**: Each wheel is individually controlled for better maneuverability.
- **Stop Conditions**: Special conditions halt the robot based on sensor input (e.g., a detected stop signal).
- **Failsafe Behavior**: Implements corrective measures when the bot loses the line.

## Components

### Hardware
1. **Arduino-Compatible Microcontroller**
2. **8 Line Sensors**
3. **4 Motors**
4. **Motor Driver Module**
5. **Power Supply**
6. **Chassis with Wheels**
7. **Miscellaneous Components**: Jumper wires, resistors, etc.

### Software
- Arduino IDE (for programming)
- Serial Monitor (for debugging and tuning)

## Pin Configuration

| Component       | Arduino Pin |
|----------------|-------------|
| Motor 1 (ENA)  | 5           |
| Motor 1 (IN1)  | 6           |
| Motor 1 (IN2)  | 7           |
| Motor 2 (ENB)  | 10          |
| Motor 2 (IN3)  | 8           |
| Motor 2 (IN4)  | 9           |
| Motor 3 (ENC)  | 11          |
| Motor 3 (IN5)  | 12          |
| Motor 3 (IN6)  | 13          |
| Motor 4 (END)  | 3           |
| Motor 4 (IN7)  | 4           |
| Motor 4 (IN8)  | 2           |
| Line Sensors   | 11-4        |
| Stop Sensor (TIR)| 4         |

## Working Principle

1. **Line Detection**  
   - The bot calculates its position relative to the line using readings from the eight sensors.
   - A weighted average determines the bot's position.

2. **PID Control**  
   - **Proportional**: Corrects immediate errors.
   - **Integral**: Adjusts for accumulated errors.
   - **Derivative**: Accounts for changes in error.
   - The control algorithm adjusts motor speeds to keep the bot on the line.

3. **Stop Conditions**  
   - The bot halts when specific sensor patterns are detected.

4. **Failsafe**  
   - If no line is detected, the bot gradually reduces speed and attempts to recover.

## Code Highlights

- **PID Tuning**:
  Automatically calculates optimal `Kp`, `Ki`, and `Kd` values using the Ziegler-Nichols method.

- **Motor Control**:
  The `move()` function handles the direction and speed for each motor.

- **Line Position**:
  The `botPosition()` function determines the bot's position relative to the line.

## Setup Instructions

1. **Hardware Assembly**:
   - Connect motors to the motor driver.
   - Connect sensors to the microcontroller.
   - Ensure proper power connections.

2. **Software Setup**:
   - Install the Arduino IDE.
   - Upload the provided code to the microcontroller.

3. **Testing**:
   - Place the robot on a test track.
   - Use the Serial Monitor to tune the PID constants, if necessary.

## Usage

1. Power on the robot and place it on the line.
2. After an initial delay, the robot will begin following the line.
3. Observe the behavior and fine-tune PID constants if required.

## Tuning Parameters

- **Kp**: Proportional constant (default: `0.21`).
- **Ki**: Integral constant (default: `0`).
- **Kd**: Derivative constant (default: `0`).
- Adjust these values for smoother and more accurate line following.

## Future Improvements

1. Add support for curved tracks with dynamic PID adjustments.
2. Implement obstacle detection using ultrasonic sensors.
3. Optimize power usage for longer operation.

## Author

Sarthak Sanjiv Khandare 
sarthakkhandare21@gmail.com

## License

This project is open-source and available under the [MIT License](https://opensource.org/licenses/MIT)
