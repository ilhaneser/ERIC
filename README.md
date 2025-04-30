# Stepper Motor Control System

## Overview
This Arduino-based system provides precise control for stepper motors with position tracking in millimeters. It features commands for motor control, movement operations, and position tracking with support for homing and load positioning.

## Features
- Motor control (on/off, direction)
- Precise movement in pulses or millimeters
- Position tracking in millimeters
- Homing functionality with kill switch detection
- Load position automation with safety checks
- Configurable pulse frequency

## Hardware Requirements
- Arduino board (Uno, Mega, etc.)
- Stepper motor driver
- Stepper motor
- Kill switch (limit switch)
- Power supply appropriate for your motor

## Pin Connections
| Arduino Pin | Connection         | Description                              |
|-------------|-------------------|------------------------------------------|
| 10          | MOTOR_ON_PIN      | Motor enable pin                         |
| 5           | DIRECTION_PIN     | Direction control pin                    |
| 2           | JOG_PIN           | Jog pulse pin                           |
| 13          | KILL_SWITCH_PIN   | Kill switch input pin                    |

## Kill Switch Wiring 
- Connect one wire to GND (ground)
- Connect other wire to pin 13

## Calibration
- 1200 pulses = 10mm (1cm)
- 120 pulses = 1mm

## Position Values
- HOME position: 0.0mm
- LOAD position: 240.0mm

## Available Commands

| Command      | Description                                    | Example        |
|--------------|------------------------------------------------|----------------|
| ON           | Turn motor on                                  | `ON`           |
| OFF          | Turn motor off                                 | `OFF`          |
| FWD          | Set direction forward                          | `FWD`          |
| REV          | Set direction backward                         | `REV`          |
| FREQ:        | Set pulse frequency                            | `FREQ:1000`    |
| MOVE:        | Generate specified number of pulses            | `MOVE:1000`    |
| MOVE_MM:     | Move specified distance in millimeters         | `MOVE_MM:10`   |
| HOME         | Home the motor using kill switch               | `HOME`         |
| LOAD         | Move to load position (240mm)                  | `LOAD`         |
| POS          | Show current position                          | `POS`          |
| STOP         | Stop pulse generation                          | `STOP`         |
| STATUS       | Get system status                              | `STATUS`       |
| TEST_KILL    | Test the kill switch state                     | `TEST_KILL`    |
| ZERO         | Set current position to zero                   | `ZERO`         |

## Important Safety Features
- The LOAD command can only be executed when the position is exactly at 0.0mm (HOME position)
- If position is greater than 0mm, the LOAD command will display an error message
- The system automatically tracks whether the LOAD command is allowed based on current position

## Homing Process
1. Moving backward until kill switch is activated
2. Moving forward until kill switch is deactivated
3. Position is set to 0.0mm (HOME position)
4. LOAD command is enabled

## Status Reporting
Use the `STATUS` command to see detailed information about:
- Motor state (ON/OFF)
- Direction
- Frequency
- Running state
- Position in mm and pulses
- Kill switch state
- LOAD command availability

## Example Usage Sequence
1. `ON` - Turn on the motor
2. `HOME` - Home the motor
3. `LOAD` - Move to the load position (only works at HOME position)
4. `POS` - Check current position
5. `HOME` - Return to home position
6. `OFF` - Turn off the motor

## Troubleshooting
- If the motor doesn't move, check if it's enabled with the `ON` command
- If position tracking seems off, use the `HOME` command to recalibrate
- If the kill switch is not working properly, use `TEST_KILL` to check its status
- If the LOAD command returns an error, check that you're at the HOME position (0.0mm) using the `POS` command
