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

### How-to-`yakut`
[Install](https://github.com/OpenCyphal/yakut) and configure `yakut`:
```bash
. setup_yakut.sh
```
Obtain value of digital input 0 using `yakut` (`cyphal.pub.input0.id` = `1001`):
```bash
y sub 1001:uavcan.primitive.scalar.Bit.1.0 --with-metadata
```

## Register list

| **name**                                  | **type** | **default value**                     | **Description**                        |
| ----------------------------------------- | -------- | ------------------------------------- | -------------------------------------- |
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

## functions
### Node-ID

Set `cyphal.node.id`to the desired value, for example 30.

```bash
y r 0 cyphal.node.id 30
```

Store settings to eeprom and restart controller.

```bash
y cmd 0 store
y cmd 0 restart
```

### LED
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

### Output
How to control output0 and output1 of the OpenCyphalPicoBase.

Set `cyphal.sub.output0.id`to a value different than 65535, for example 200.

```bash
y r 30 cyphal.sub.output0.id 200
```

Store settings to eeprom and restart controller.

```bash
y cmd 30 store
y cmd 30 restart
```

Turn output0 on by publishing to Subject-ID 200

```bash
y pub -N 1 200:uavcan.primitive.scalar.Bit.1.0 true
```

Turn output0 off by publishing to Subject-ID 200

```bash
y pub -N 1 200:uavcan.primitive.scalar.Bit.1.0 false
```

### Servo
How to control the servo outputs (servo0 and servo1) of the OpenCyphalPicoBase.

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
