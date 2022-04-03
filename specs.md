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
- Search where to buy the RT6150B Buck Boost on the Rpi Pico

Modello non disponibile da nessuna parte. Esempi di alternative su farnell:

[Farnell](https://it.farnell.com/w/c/semiconduttori-circuiti-integrati/circuiti-integrati-di-gestione-di-potenza-pmic/regolatori-di-tensione/regolatori-switching-fissi-dc-dc?tensione-di-ingresso-min=3v|3.3v&tensione-di-ingresso-max=36v|45v&tensione-di-uscita-nom-=3.3v&range=inc-in-stock)

| Device | In range | Out range | Out current | URL |
|---|---|---|---|---|
| MPQ4313 | 3.3 - 45 | 3.3V, 5V fixed | 3A | [datasheet](https://www.farnell.com/datasheets/3183543.pdf) | 
| MAX2040x | 3.0 - 36 | 0.8 - 10 | 4-8A | [datasheet](https://www.farnell.com/datasheets/3497518.pdf) | 
| MAX20004/6/8 | 3.0 - 36 | 1 - 5 | 4/6/8A | [datasheet](https://www.farnell.com/datasheets/3108373.pdf) | 
| MPQ9840 | 3.3 - 36 | ? | 3.5A | [datasheet](https://www.farnell.com/datasheets/3380075.pdf) | 

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