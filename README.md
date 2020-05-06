# MAX31855 Thermocouple Amplifier Board

This is a simple breakout board for a `MAX31855` thermocouple amplifier. It is an easy-to-use chip which handles all of the fancy math and analog conversion for calculating the temperature output from a thermocouple.

Need to measure temperatures higher than a couple hundred degrees C? Give one of these a try.

See `max31855_BOM.csv` for a bill of materials, but the only oddly-sized part is the screw terminal block.

See `firmware/` for an example STM32 program which reads the currently detected temperature value out of the chip if one is connected.
