<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `CyphalPicoBase/CAN-firmware`
===========================================
<a href="https://opencyphal.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/opencyphal.svg" width="25%"></a>
[![General Formatting Checks](https://github.com/107-systems/CyphalPicoBase-CAN-firmware/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/CyphalPicoBase-CAN-firmware/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/CyphalPicoBase-CAN-firmware/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/CyphalPicoBase-CAN-firmware/actions?workflow=Spell+Check)
[![Compile Examples](https://github.com/107-systems/CyphalPicoBase-CAN-firmware/workflows/Compile/badge.svg)](https://github.com/107-systems/CyphalPicoBase-CAN-firmware/actions?workflow=Compile)

Firmware for the [CyphalPicoBase/CAN](https://github.com/generationmake/CyphalPicoBase-CAN) board. You can buy one [here](https://cyphal.store/products/lxrobotics-cyphalpicobase-can) 💸.

## How-to-build/upload

This firmware relies on the excellent work of Earle F. Philhower, III to create an arduino integration with the Raspberry Pi Pico SDK (as opposed to the Mbed-based integration).
You'll need to install this first to build. Instructions can be found [in the arduino-pico documentation](https://arduino-pico.readthedocs.io/en/latest/install.html#installing-via-arduino-cli) but the TLDR is:

```bash
arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040
```

Next you'll need to install the libraries this firmware depends on:

```bash
arduino-cli lib update-index
arduino-cli lib install 107-Arduino-Debug 107-Arduino-MCP2515 107-Arduino-UniqueId 107-Arduino-24LCxx 107-Arduino-littlefs
arduino-cli lib install 107-Arduino-Cyphal-Support 107-Arduino-Cyphal
arduino-cli lib install "Adafruit NeoPixel"
```

Now you should be able to build.

```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v .
arduino-cli upload -b rp2040:rp2040:rpipico -v . -p /dev/ttyACM0
```
**or**
```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v . --build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"
```
Adding argument `--build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"` allows to feed the Git hash of the current software version to [107-Arduino-Cyphal](https://github.com/107-systems/107-Arduino-Cyphal) stack from where it can be retrieved via i.e. [yakut](https://github.com/opencyphal/yakut).

### Platform.io

This repository also allows you to build using platform.io. The platformio.ini has all the details and it should "just work™".

### How-to-`yakut`
[Install](https://github.com/OpenCyphal/yakut) and configure `yakut`:
```bash
. setup_yakut.sh
```
Obtain value of digital input 0 using `yakut` (`cyphal.pub.input0.id` = `1001`):
```bash
y sub 1001:uavcan.primitive.scalar.Bit.1.0 --with-metadata
```

## Quickstart
### How-to-configure the Node-ID
Set `cyphal.node.id`to the desired value, for example 30.
```bash
y r 0 cyphal.node.id 30
```
Store settings to eeprom and restart controller.
```bash
y cmd 0 store
y cmd 0 restart
```
### How-to-control the built-in LED
How to control the built-in LED on the Raspberry Pi Pico.

Set `cyphal.sub.led1.id`to a value different than 65535, for example 100.
```bash
y r 30 cyphal.sub.led1.id 100
```
Store settings to eeprom and restart controller.
```bash
y cmd 30 store
y cmd 30 restart
```
Turn LED on by publishing to Subject-ID 100
```bash
y pub -N 1 100:uavcan.primitive.scalar.Bit.1.0 true
```
Turn LED off by publishing to Subject-ID 100
```bash
y pub -N 1 100:uavcan.primitive.scalar.Bit.1.0 false
```

### How-to-control a digital output
How to control digital output 0 and digital output1 of the CyphalPicoBase-CAN.

Set `cyphal.sub.output0.id`to a value different than 65535, for example 200.
```bash
y r 30 cyphal.sub.output0.id 200
```
Store settings to eeprom and restart controller.
```bash
y cmd 30 store
y cmd 30 restart
```
Turn digital output 0 on by publishing to Subject-ID 200
```bash
y pub -N 1 200:uavcan.primitive.scalar.Bit.1.0 true
```
Turn digital output 0 off by publishing to Subject-ID 200
```bash
y pub -N 1 200:uavcan.primitive.scalar.Bit.1.0 false
```

### How-to-control a servo PWM output
How to control the servo PWM outputs 0 and 1 of the CyphalPicoBase-CAN.

Set `cyphal.sub.servo0.id`to a value different than 65535, for example 300.
```bash
y r 30 cyphal.sub.servo0.id 300
```
Store settings to eeprom and restart controller.
```bash
y cmd 30 store
y cmd 30 restart
```
Turn servo to a dedicated position by publishing to Subject-ID 300. Possible values are between 800 and 2200. Others will be dismissed.
```bash
y pub -N 1 300:uavcan.primitive.scalar.Integer16.1.0 1500
```

## Register list
| **Name**                                  | **Type** | **Default Value**                     | **Description**                               |
|-------------------------------------------|----------|---------------------------------------|-----------------------------------------------|
| cyphal.node.description                   | rw       | CyphalPicoBase-CAN                    | Node description                              |
| cyphal.node.id                            | rw       | 0                                     | Node id (max 127)                             |
| cyphal.pub.analoginput0.id                | rw       | 65535                                 | Subject-ID (publish) for analog input 0       |
| cyphal.pub.analoginput0.type              | ro       | cyphal.primitive.scalar.Integer16.1.0 |                                               |
| cyphal.pub.analoginput1.id                | rw       | 65535                                 | Subject-ID (publish) for analog input 1       |
| cyphal.pub.analoginput1.type              | ro       | cyphal.primitive.scalar.Integer16.1.0 |                                               |
| cyphal.pub.input0.id                      | rw       | 65535                                 | Subject-ID (publish) for digital input 0      |
| cyphal.pub.input0.type                    | ro       | cyphal.primitive.scalar.Bit.1.0       |                                               |
| cyphal.pub.input1.id                      | rw       | 65535                                 | Subject-ID (publish) for digital input 1      |
| cyphal.pub.input1.type                    | ro       | cyphal.primitive.scalar.Bit.1.0       |                                               |
| cyphal.pub.input2.id                      | rw       | 65535                                 | Subject-ID (publish) for digital input 2      |
| cyphal.pub.input2.type                    | ro       | cyphal.primitive.scalar.Bit.1.0       |                                               |
| cyphal.pub.input3.id                      | rw       | 65535                                 | Subject-ID (publish) for digital input 3      |
| cyphal.pub.input3.type                    | ro       | cyphal.primitive.scalar.Bit.1.0       |                                               |
| cyphal.pub.inputvoltage.id                | rw       | 65535                                 | Subject-ID (publish) for input voltage        |
| cyphal.pub.inputvoltage.type              | ro       | cyphal.primitive.scalar.Real32.1.0    |                                               |
| cyphal.pub.internaltemperature.id         | rw       | 65535                                 | Subject-ID (publish) for internal temp        |
| cyphal.pub.internaltemperature.type       | ro       | cyphal.primitive.scalar.Real32.1.0    |                                               |
| cyphal.sub.led1.id                        | rw       | 65535                                 | Subject-ID (subscribe) for led1               |
| cyphal.sub.led1.type                      | ro       | cyphal.primitive.scalar.Bit.1.0       |                                               |
| cyphal.sub.lightmode.id                   | rw       | 65535                                 | Subject-ID (subscribe) for light mode         |
| cyphal.sub.lightmode.type                 | ro       | cyphal.primitive.scalar.Integer8.1.0  |                                               |
| cyphal.sub.output0.id                     | rw       | 65535                                 | Subject-ID (subscribe) for digital output 0   |
| cyphal.sub.output0.type                   | ro       | cyphal.primitive.scalar.Bit.1.0       |                                               |
| cyphal.sub.output1.id                     | rw       | 65535                                 | Subject-ID (subscribe) for digital output 1   |
| cyphal.sub.output1.type                   | ro       | cyphal.primitive.scalar.Bit.1.0       |                                               |
| cyphal.sub.servo0.id                      | rw       | 65535                                 | Subject-ID (subscribe) for servo PWM output 0 |
| cyphal.sub.servo0.type                    | ro       | cyphal.primitive.scalar.Integer16.1.0 |                                               |
| cyphal.sub.servo1.id                      | rw       | 65535                                 | Subject-ID (subscribe) for servo PWM output 1 |
| cyphal.sub.servo1.type                    | ro       | cyphal.primitive.scalar.Integer16.1.0 |                                               |
| pico.update_period_ms.analoginput0        | rw       | 500                                   | Update period for analog input 0              |
| pico.update_period_ms.analoginput1        | rw       | 500                                   | Update period for analog input 1              |
| pico.update_period_ms.input0              | rw       | 500                                   | Update period for digital input 0             |
| pico.update_period_ms.input1              | rw       | 500                                   | Update period for digital input 1             |
| pico.update_period_ms.input2              | rw       | 500                                   | Update period for digital input 2             |
| pico.update_period_ms.input3              | rw       | 500                                   | Update period for digital input 3             |
| pico.update_period_ms.inputvoltage        | rw       | 3000                                  | Update period for input voltage               |
| pico.update_period_ms.internaltemperature | rw       | 10000                                 | Update period for internal temperature        |
| pico.update_period_ms.light               | rw       | 250                                   | Update period for light functions             |
