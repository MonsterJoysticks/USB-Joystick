# USB-Joystick-Adapter
Firmware and schematics for the Monster Joysticks USB joystick adapter.

The Firmware for this adapter has been heavily based on https://github.com/raphnet code for his USB adapters, it uses the V-USB libraies for interfacing with AVR microcontrollers and specifically the atMega8 for this device https://www.obdev.at/products/vusb/index.html.

The Pre-built and flashed devices sold on MonsterJoysticks.com have the BootLoaderHID https://www.obdev.at/products/vusb/bootloadhid.html pre-flashed and new firmware can then be flashed via USB rather then the use of a USBAsp device.
