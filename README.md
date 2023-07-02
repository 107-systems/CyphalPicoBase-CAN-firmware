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
arduino-cli compile -b rp2040:rp2040:rpipico -v . --build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"
```
Adding argument `--build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"` allows to feed the Git hash of the current software version to [107-Arduino-Cyphal](https://github.com/107-systems/107-Arduino-Cyphal) stack from where it can be retrieved via i.e. [yakut](https://github.com/opencyphal/yakut).

## Register list

| **name**                                  | **type** | **default value**                     | **Description**                        |
|:-----------------------------------------:|:--------:|:-------------------------------------:|:--------------------------------------:|
| cyphal.node.description                   | rw       | L3X-Z AUX_CONTROLLER                  | node description                       |
| cyphal.node.id                            | rw       | 0                                     | node id (max 127)                      |
| cyphal.pub.analoginput0.id                | rw       | 65535                                 | subject-id (publish) for analoginput0  |
| cyphal.pub.analoginput0.type              | ro       | cyphal.primitive.scalar.Integer16.1.0 |                                        |
| cyphal.pub.analoginput1.id                | rw       | 65535                                 | subject-id (publish) for analoginput1  |
| cyphal.pub.analoginput1.type              | ro       | cyphal.primitive.scalar.Integer16.1.0 |                                        |
| cyphal.pub.input0.id                      | rw       | 65535                                 | subject-id (publish) for input0        |
| cyphal.pub.input0.type                    | ro       | cyphal.primitive.scalar.Bit.1.0       |                                        |
| cyphal.pub.input1.id                      | rw       | 65535                                 | subject-id (publish) for input1        |
| cyphal.pub.input1.type                    | ro       | cyphal.primitive.scalar.Bit.1.0       |                                        |
| cyphal.pub.input2.id                      | rw       | 65535                                 | subject-id (publish) for input2        |
| cyphal.pub.input2.type                    | ro       | cyphal.primitive.scalar.Bit.1.0       |                                        |
| cyphal.pub.input3.id                      | rw       | 65535                                 | subject-id (publish) for input3        |
| cyphal.pub.input3.type                    | ro       | cyphal.primitive.scalar.Bit.1.0       |                                        |
| cyphal.pub.inputvoltage.id                | rw       | 65535                                 | subject-id (publish) for input voltage |
| cyphal.pub.inputvoltage.type              | ro       | cyphal.primitive.scalar.Real32.1.0    |                                        |
| cyphal.pub.internaltemperature.id         | rw       | 65535                                 | subject-id (publish) for internal temp |
| cyphal.pub.internaltemperature.type       | ro       | cyphal.primitive.scalar.Real32.1.0    |                                        |
| cyphal.sub.led1.id                        | rw       | 65535                                 | subject-id (subscribe) for led1        |
| cyphal.sub.led1.type                      | ro       | cyphal.primitive.scalar.Bit.1.0       |                                        |
| cyphal.sub.lightmode.id                   | rw       | 65535                                 | subject-id (subscribe) for lightmode   |
| cyphal.sub.lightmode.type                 | ro       | cyphal.primitive.scalar.Integer8.1.0  |                                        |
| cyphal.sub.output0.id                     | rw       | 65535                                 | subject-id (subscribe) for output0     |
| cyphal.sub.output0.type                   | ro       | cyphal.primitive.scalar.Bit.1.0       |                                        |
| cyphal.sub.output1.id                     | rw       | 65535                                 | subject-id (subscribe) for output1     |
| cyphal.sub.output1.type                   | ro       | cyphal.primitive.scalar.Bit.1.0       |                                        |
| cyphal.sub.servo0.id                      | rw       | 65535                                 | subject-id (subscribe) for servo0      |
| cyphal.sub.servo0.type                    | ro       | cyphal.primitive.scalar.Integer16.1.0 |                                        |
| cyphal.sub.servo1.id                      | rw       | 65535                                 | subject-id (subscribe) for servo1      |
| cyphal.sub.servo1.type                    | ro       | cyphal.primitive.scalar.Integer16.1.0 |                                        |
| pico.update_period_ms.analoginput0        | rw       | 500                                   | update period for analoginput0         |
| pico.update_period_ms.analoginput1        | rw       | 500                                   | update period for analoginput1         |
| pico.update_period_ms.input0              | rw       | 500                                   | update period for input0               |
| pico.update_period_ms.input1              | rw       | 500                                   | update period for input1               |
| pico.update_period_ms.input2              | rw       | 500                                   | update period for input2               |
| pico.update_period_ms.input3              | rw       | 500                                   | update period for input3               |
| pico.update_period_ms.inputvoltage        | rw       | 3000                                  | update period for input voltage        |
| pico.update_period_ms.internaltemperature | rw       | 10000                                 | update period for internal temperature |
| pico.update_period_ms.light               | rw       | 250                                   | update period for light functions      |


## How-to-build/upload
```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v .
arduino-cli upload -b rp2040:rp2040:rpipico -v . -p /dev/ttyACM0
```
**or**
```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v . --build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"
```
Adding argument `--build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"` allows to feed the Git hash of the current software version to [107-Arduino-Cyphal](https://github.com/107-systems/107-Arduino-Cyphal) stack from where it can be retrieved via i.e. [yakut](https://github.com/opencyphal/yakut).
