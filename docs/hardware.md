# Hardware Setup and Wiring

This document details the hardware components and wiring for the CanoeDash project.

## Components

*   Raspberry Pi Pico 2 (with RP2350)
*   SparkFun RGB LED Rotary Encoder
*   Honda BF5 Outboard Engine (for RPM sensing)
*   12V Marine Battery
*   Voltage Divider Circuit
*   Various sensors, LEDs, and a bilge pump

## Pinout and Wiring

| Component                 | Pico Pin (GP) | Description                                         |
| ------------------------- | ------------- | --------------------------------------------------- |
| **Rotary Encoder**        |               |                                                     |
| Encoder A                 | GP2           | Rotational input A                                  |
| Encoder B                 | GP3           | Rotational input B                                  |
| Switch                    | GP4           | Push-button switch                                  |
| **RGB LED (on Encoder)**  |               |                                                     |
| Red Cathode               | GP5 (PWM)     |                                                     |
| Green Cathode             | GP6 (PWM)     |                                                     |
| Blue Cathode              | GP7 (PWM)     |                                                     |
| **Sensors**               |               |                                                     |
| RPM Pulse                 | GP8 (PIO)     | Connect to spark plug wire via an inductive pickup  |
| Bilge Status              | GP9           | High/Low input from bilge float switch              |
| Oil Level                 | GP26 (ADC0)   | Analog input from oil level sensor                  |
| Battery Voltage           | GP27 (ADC1)   | Analog input from voltage divider                   |
| **Actuators**             |               |                                                     |
| Bilge Pump Control        | GP16          | High/Low output to relay controlling bilge pump     |
| **Lighting Circuits**     |               |                                                     |
| Light Circuit 1           | GP10 (PWM)    | PWM output to MOSFET/driver for light circuit 1     |
| Light Circuit 2           | GP11 (PWM)    | ...                                                 |
| Light Circuit 3           | GP12 (PWM)    | ...                                                 |
| Light Circuit 4           | GP13 (PWM)    | ...                                                 |
| Light Circuit 5           | GP14 (PWM)    | ...                                                 |
| Light Circuit 6           | GP15 (PWM)    | ...                                                 |

## Voltage Divider for Battery Sensing

To safely measure the battery voltage (which can be up to 14.4V) with the Pico's 3.3V ADC, a voltage divider is required.

*   **R1**: 10kΩ resistor connected between the battery's positive terminal and GP27.
*   **R2**: 3.3kΩ resistor connected between GP27 and GND.

This creates a voltage ratio of `3.3k / (10k + 3.3k) = 0.248`. The voltage at the ADC pin will be `V_battery * 0.248`. The firmware must account for this scaling factor.

## RPM Sensing

The RPM is sensed by wrapping a wire around the spark plug lead of the Honda BF5 engine. This wire acts as an inductive pickup, generating a small pulse of voltage every time the spark plug fires. This pulse is then fed into a signal conditioning circuit (e.g., a simple diode clamp and Schmitt trigger) to produce a clean digital pulse for the Pico's PIO on GP8.

The Honda BF5 is a single-cylinder, four-stroke engine, which means it fires once every two revolutions. Therefore, the engine RPM is calculated as `RPM = (1 / pulse_interval_seconds) * 60 / 2 = 30 / pulse_interval_seconds`.

**Note:** A level shifter may be required for some components if they operate at a different voltage level than the Pico's 3.3V logic.
