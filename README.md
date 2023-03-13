<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `OpenCyphalPicoBase-firmware`
===========================================
<a href="https://opencyphal.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/opencyphal.svg" width="25%"></a>
[![General Formatting Checks](https://github.com/107-systems/OpenCyphalPicoBase-firmware/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/OpenCyphalPicoBase-firmware/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/OpenCyphalPicoBase-firmware/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/OpenCyphalPicoBase-firmware/actions?workflow=Spell+Check)
[![Compile Examples](https://github.com/107-systems/OpenCyphalPicoBase-firmware/workflows/Compile/badge.svg)](https://github.com/107-systems/OpenCyphalPicoBase-firmware/actions?workflow=Compile)

Firmware for the [OpenCyphalPicoBase](https://github.com/generationmake/OpenCyphalPicoBase) board.

## How-to-build/upload
```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v .
arduino-cli upload -b rp2040:rp2040:rpipico -v . -p /dev/ttyACM0
```
**or**
```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v . --build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git describe --always)"
```
Adding argument `--build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git describe --always)"` allows to feed the Git hash of the current software version to [107-Arduino-Cyphal](https://github.com/107-systems/107-Arduino-Cyphal) stack from where it can be retrieved via i.e. [yakut](https://github.com/opencyphal/yakut).

### Node-ID
The Node-ID is stored in the eeprom. If no eeprom is found, the Node-ID is 101.

### Subject-ID
Some Subject-IDs are the same as with the leg controller. The host can differentiate between them by their Node-IDs.

| **Subject-ID** | **Direction** | **Name**             | **Type**    |
|:--------------:|:-------------:|:--------------------:|:-----------:|
| heartbeat      | pub           | heartbeat            | heartbeat   |
| 1001           | pub           | input-voltage        | Real32      |
| 1005           | sub           | LED1                 | Bit         |
| 1010           | pub           | internal Temperature | Real32      |
| 2000           | pub           | INPUT 0              | Bit         |
| 2001           | pub           | INPUT 1              | Bit         |
| 2002           | pub           | INPUT 2              | Bit         |
| 2003           | pub           | INPUT 3              | Bit         |
| 2004           | sub           | OUTPUT 0             | Bit         |
| 2005           | sub           | OUTPUT 1             | Bit         |
| 2006           | sub           | SERVO 0              | Integer16   |
| 2007           | sub           | SERVO 1              | Integer16   |
| 2008           | pub           | ANALOG INPUT 0       | Integer16   |
| 2009           | pub           | ANALOG INPUT 1       | Integer16   |
| 2010           | sub           | light mode           | Integer8    |

### light mode

| **value** | **function**        |
|:---------:|:-------------------:|
| 1         | red static          |
| 2         | green static        |
| 3         | blue static         |
| 4         | white static        |
| 5         | amber static        |
| 11        | red blinking        |
| 12        | green blinking      |
| 13        | blue blinking       |
| 14        | white blinking      |
| 15        | amber blinking      |
| 101       | red running light   |
| 102       | green running light |
| 103       | blue running light  |
| 104       | white running light |
| 105       | amber running light |
| other     | all LEDs off        |

## pin usage

### Raspberry Pi Pico

| **Pin** | **Pin Name** | **Signal**    | **Description**                  |
|:-------:|:------------:|:-------------:|:--------------------------------:|
| 1       | GP0          | UART0_TX      | reserved for future use (GNSS)   |
| 2       | GP1          | UART0_RX      | reserved for future use (GNSS)   |
| 3       | GND          | GND           |                                  |
| 4       | GP2          |               |                                  |
| 5       | GP3          |               |                                  |
| 6       | GP4          | I2C0_SDA      | for eeprom and qwiic connector   |
| 7       | GP5          | I2C0_SCL      | for eeprom and qwiic connector   |
| 8       | GND          | GND           |                                  |
| 9       | GP6          | GPI0          | input 0                          |
| 10      | GP7          | GPI1          | input 1                          |
| 11      | GP8          | GPI2          | input 2                          |
| 12      | GP9          | GPI3          | input 3                          |
| 13      | GND          | GND           |                                  |
| 14      | GP10         | GPO0          | output 0                         |
| 15      | GP11         | GPO1          | output 1                         |
| 16      | GP12         | reserved      | Neopixel                         |
| 17      | GP13         | reserved      | radiation detector               |
| 18      | GND          | GND           |                                  |
| 19      | GP14         | SERVO0        | servo 0                          |
| 20      | GP15         | SERVO1        | servo 1                          |
| 21      | GP16         | SPI_MISO      | SPI for MCP2515                  |
| 22      | GP17         | MCP2515_CS    | SPI for MCP2515                  |
| 23      | GND          | GND           |                                  |
| 24      | GP18         | SPI_CLK       | SPI for MCP2515                  |
| 25      | GP19         | SPI_MOSI      | SPI for MCP2515                  |
| 26      | GP20         | MCP2515_INT   | interrupt for MCP2515            |
| 27      | GP21         | STATUS_LED2   | internal status LED 2            |
| 28      | GND          | GND           |                                  |
| 29      | GP22         | STATUS_LED3   | internal status LED 3            |
| 30      | RUN          | RESET         | Reset for Board                  |
| 31      | GP26         | INPUT_VOLTAGE | measure input voltage            |
| 32      | GP27         | ANALOG_INPUT0 | analog input                     |
| 33      | GND          | GND           |                                  |
| 34      | GP28         | ANALOG_INPUT1 | analog input                     |
| 35      | ADC_VREF     |               |                                  |
| 36      | 3V3 (OUT)    | 3V3-rail      | supply voltage for board         |
| 37      | 3V3_EN       |               |                                  |
| 38      | GND          | GND           |                                  |
| 39      | VSYS         |               |                                  |
| 40      | VBUS         | 5V-rail       | supply voltage for board         |
