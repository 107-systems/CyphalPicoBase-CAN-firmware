# l3xz-fw_aux-controller
Firmware for the auxiliary controller (alarm LEDs and emergency stop)

[![Check Arduino](https://github.com/107-systems/l3xz-fw_aux-controller/actions/workflows/check-arduino.yml/badge.svg)](https://github.com/107-systems/l3xz-fw_aux-controller/actions/workflows/check-arduino.yml)
[![Check keywords.txt](https://github.com/107-systems/l3xz-fw_aux-controller/actions/workflows/check-keywords-txt.yml/badge.svg)](https://github.com/107-systems/l3xz-fw_aux-controller/actions/workflows/check-keywords-txt.yml)
[![General Formatting Checks](https://github.com/107-systems/l3xz-fw_aux-controller/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/l3xz-fw_aux-controller/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/l3xz-fw_aux-controller/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/l3xz-fw_aux-controller/actions?workflow=Spell+Check)
[![Compile Examples](https://github.com/107-systems/l3xz-fw_aux-controller/workflows/Compile/badge.svg)](https://github.com/107-systems/l3xz-fw_aux-controller/actions?workflow=Compile)

## uavcan settings

specific seetings for the L3X-Z Hexapod can be found here: https://github.com/107-systems/l3xz-hw#node-ids

### Node-ID

every leg controller needs to have its own Node-ID. The Node-ID is stored in the eeprom. If no eeprom is found, the Node-ID is 101.

### Subject-ID

some Subject-IDs are the same as with the leg controller. The host can differentiate between them by their Node-IDs.

| **Subject-ID** | **direction** | **name**          | **type**    |
|:--------------:|:-------------:|:-----------------:|:-----------:|
| heartbeat      | pub           | heartbeat         | heartbeat   |
| 1001           | pub           | input-voltage     | Real32      |
| 1005           | sub           | LED1              | Bit         |
| 2001           | pub           | emergency stop    | Bit         |
| 2002           | sub           | light mode        | Integer8    |
| 1010           | sub           | update_interval   | Integer16   |

### light mode

| **value** | **function**        |
|:---------:|:-------------------:|
| 1         | red static          |
| 2         | green static        |
| 3         | blue static         |
| 4         | white static        |
| 5         | amber static        |
| 100       | amber running light |
| other     | all LEDs off        |

## related repositories
* https://github.com/107-systems/107-Arduino-MCP2515
* https://github.com/107-systems/107-Arduino-UAVCAN
