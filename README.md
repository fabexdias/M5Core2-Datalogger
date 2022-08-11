# M5Core2 Datalogger
This data logger is made for a UAV and it's based on M5Stack (Core 2 model). The UAV HFE International motor from 2020 emits data (telemetry) from a Serial Port that is processed (parsed) and displayed by the M5Core2. It's used an HSB-9380th servo for controlling the air that is fed to the engine heads with a PID controller. The temperature of the motor is measured with a MCP9700 sensor. The data is time-stamped (using RTC) and stored in the SDcard.

## Features

- Serial Communication
- Graphic interface with touch buttons
- SDcard data storing
- RTC setup
- Servo PID controller
- ADC reads

## ArduinoIDE
The code is ArduinoIDE compatible, after importing ESP32Time, ESP32Servo, and M5Core2 libraries as explained on the M5Stack website.