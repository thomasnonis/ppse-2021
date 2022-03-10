# Packages

## Crystal

- 3225

## Resistors

- Standard: 0603
- Mid power: 0805
- Shunts: 1210

## Capacitors

- 100n: 0603
- 1u: 0603
- \>10u: 0805 (_In order to distinguish them from the smaller ones_)

## Flash

- SO8

## USB

- Either B or micro B

# To research

- RMII (Ethernet)
- Search where to buy the RT5150B Buck Boost on the Rpi Pico

# Peripherals

- GPS
  -  SAM8Q (SMD) or PAM7Q (THT)
  -  Connect UART as predisposition
  -  Extend pads for manual soldering
  -  Supply control with pMOS and PDTC114YT. No direct drive to MOSFET, since there could be a current throught the internal protection diode of the GPIO

- RGB LED

# Power

- USB 5V
- 3.3V Battery
- 24V input

- MP2322 Buck concverter from supply to 3V3 (just as exercise, it does not support 24V)
- JST 2.5 01x02 for battery
- Screw connector for 24V (5,04)