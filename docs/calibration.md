# Sensor Calibration

This document provides guidance on calibrating the sensors for the CanoeDash project.

## Voltage Sensor

The voltage sensor uses a voltage divider circuit. To calibrate:

1.  Measure the actual battery voltage using a reliable multimeter.
2.  Read the raw ADC value from the Pico (0-4095).
3.  Calculate the voltage per ADC unit: `V_per_ADC = V_actual / ADC_raw`.
4.  Update the firmware with this calibration factor. The formula in the firmware should be `V_battery = ADC_raw * V_per_ADC`.

The theoretical scaling factor is based on the resistor values, but in practice, resistor tolerances will require fine-tuning.

## RPM Sensor

The RPM calculation depends on the engine's characteristics. The Honda BF5 is a single-cylinder, four-stroke engine, firing once every two revolutions.

*   **Formula**: `RPM = 30 / pulse_interval_seconds`

The main calibration for the RPM sensor involves ensuring a clean pulse from the spark plug lead. This may require adjusting the signal conditioning circuit (e.g., the threshold of a Schmitt trigger) to reject noise and reliably detect the spark event.

## Oil Level Sensor

The oil level sensor is connected to an ADC pin. The calibration will depend on the type of sensor used.

*   **If using a float sensor with a variable resistor**: Measure the ADC values at different known oil levels (e.g., empty, full, half). Create a lookup table or a linear mapping in the firmware to convert ADC values to oil percentage.
*   **If using a simple float switch (high/low)**: No calibration is needed, but the firmware should interpret the high/low signal correctly.

## Light Level Sensor

The light level sensor (part of the rotary encoder) is not used in the current implementation, but if it were, it would need to be calibrated against a known light source to provide meaningful lux values.
