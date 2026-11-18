# Smart Traffic Junction Controller

Bare-metal ATmega328P traffic controller written in AVR Assembly. The
implementation provides:

- an 8-state normal vehicle cycle with 2-second all-red clearance states;
- a falling-edge `INT0` pedestrian request that safely starts a red
  pedestrian phase;
- a descending 7-segment countdown from 5 to 0;
- an ADC0 ambient-light sensor with night-mode flashing and a 5-second
  recovery hysteresis; and
- a 1 Hz Timer1 schedule.

## Firmware

The assembly entry point is [`assembly/main.asm`](assembly/main.asm).

## Pin usage

| Function | ATmega328P pin |
|---|---|
| Pedestrian button / INT0 | PD2 |
| NS yellow | PD3 |
| NS green | PD4 |
| EW red | PD5 |
| EW yellow | PD6 |
| EW green | PD7 |
| 7-segment segments | PB0-PB5 |
| LDR / ADC0 | PC0 |

The project report also shows D2-D7 for traffic LEDs and PC2/ADC2 in its
alternate pin table. This implementation follows the ADC0 pin from the
project overview and requires the design to avoid sharing PD2 between the
pedestrian interrupt and a traffic output.

## Build

Use Microchip Studio or an AVR assembler such as `avr-as` with the
ATmega328P register definitions:

```sh
avr-as -mmcu=atmega328p assembly/main.asm -o build/main.o
avr-objcopy -O ihex build/main.o build/main.hex
```
