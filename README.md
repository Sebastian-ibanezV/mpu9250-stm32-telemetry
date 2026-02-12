# MPU9250 STM32 Telemetry System

Real-time IMU telemetry system built from scratch using an STM32F411 (Blackpill) and the MPU9250 inertial measurement unit.  
Includes a custom I2C driver and a Python real-time visualization tool for debugging sensors and control systems.

---

## Project Overview

This project implements a complete embedded telemetry pipeline:

STM32 → MPU9250 → UART → PC → Python Visualizer → Live Graphs

The firmware reads accelerometer and gyroscope data at 100Hz and streams it over UART using a CSV protocol.  
A Python program receives and plots angular velocity in real-time, making it useful for debugging sensor noise, calibration, and control algorithms.

This project is part of a custom quadcopter flight controller development.

---

## Features

- Custom MPU9250 I2C driver (STM32 HAL)
- Automatic device detection (0x68 / 0x69)
- WHO_AM_I validation
- Gyroscope moving average filtering
- Gyroscope calibration
- Accelerometer calibration
- 100 Hz telemetry streaming
- CSV communication protocol
- Real-time Python visualization

---

## Hardware

- STM32F411 Blackpill
- MPU9250 IMU
- I2C @ 400 kHz
- UART @ 115200 baud

I2C pins:
- PB6 → SCL
- PB7 → SDA

Pull-up resistors: 4.7kΩ recommended to 3.3V.

---

## Telemetry Format

t_ms,ax,ay,az,gx,gy,gz,acc_ok

## Quick Start

### Firmware
- Board: STM32F411 Blackpill
- I2C1: PB6=SCL, PB7=SDA (400 kHz) or any I2C pin.
- UART2: 115200 baud

Flash the firmware and open a serial monitor.
You should see the CSV header:
`t_ms,ax,ay,az,gx,gy,gz,acc_ok`

### Python Visualizer
```bash
pip install -r tools/python-visualizer/requirements.txt
python tools/python-visualizer/serial_plotter_robust.py COM7

