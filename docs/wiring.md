# Wiring (STM32F411 Blackpill + MPU9250)

## Power
- MPU9250 VCC: **3.3V**
- GND: common ground with STM32

## I2C
- STM32 **PB6** -> SCL
- STM32 **PB7** -> SDA
- I2C speed: 400 kHz

### Pull-ups
If your MPU9250 module does not include pull-ups, add:
- SDA -> 3.3V via **4.7kΩ**
- SCL -> 3.3V via **4.7kΩ**

## UART (Telemetry)
- UART2: 115200 baud
- Used to stream CSV lines to the PC

## Notes
- For clean IMU testing, try powering without ESC/motors connected (noise can affect I2C/ground).
- If WHO_AM_I fails: check I2C wiring, pull-ups, and address (0x68/0x69).
