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
