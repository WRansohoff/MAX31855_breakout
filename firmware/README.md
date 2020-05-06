# MAX31855 Thermocouple Amplifier Test Program

This is a simple application demonstrating how to read temperature data from a thermocouple using a `MAX31855` chip using an `STM32L432KC` microcontroller:

https://datasheets.maximintegrated.com/en/ds/MAX31855.pdf

The `MAX31855` outputs data over `SPI`, but it doesn't have a 'data in' pin. It outputs 32 bits of data when its `CS` pin is pulled low and the clock pin is toggled:

| Bit(s) |              Meaning              |
|:------:|:---------------------------------:|
| 31     | Thermocouple temperature sign bit |
| 18-30  | Thermocouple temperature data     |
| 17     | Reserved                          |
| 16     | 0 = reading okay, 1 = fault       |
| 15     | Internal temperature sign bit     |
| 4-14   | Internal temperature data         |
| 3      | Reserved                          |
| 2      | 1 = short to `VCC` fault          |
| 1      | 1 = short to `GND` fault          |
| 0      | 1 = open circuit fault            |

The last 16 bits contain diagnostics data, so you don't need to read them. This program only retrieves the first 16 bits to report temperature data and check that the chip didn't encounter a fault.

Be sure to use the right version of chip for your thermocouple! The `MAX31855KASA+` goes with K-type thermocouples, the `MAX31855NASA+` with N-type thermocouples, etc.
