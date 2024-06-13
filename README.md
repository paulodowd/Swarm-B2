# Swarm-B2
IR Communication board for Pololu 3Pi+ robots that also accommodates an M5 Stack Core2.  

## IR Communication board fully assembled


## IR Communication board Programming / Updating
The IR Communication board is built around an Arduino Nano, so you should follow the normal steps to program one.  To program the IR Communication board with the software, or to update it with your own software, it is necessary to first remove the jumper situated on the back of the circuit board labelled `RX_break`.  If you do not do this, the Arduino IDE will report that the programmer cannot sync with the arduino device (or similar error).  Remember to replace this jumper, because it connects the IR receiver modules electronically to the Arduino. 

## Using the IR Communication board with other devices
This IR communication board can be used as a general purpose communication board.  It should be possible to use the IR Communication board in it's current design and format with any other device that can operate the I2C protocol.  To do so, you simply need to connect the `5v`, `GND`, `SCL` and `SDA` pins appropriately to your operating device.  These physical pins are labelled on the underside of the circuit board as `+RED` (5v), `-BLK` (GND), `SDA` and `SCL`, or as `5V`, `GND`, `SCL`, `SDA` on the  topside of the circuit board.

Because the IR Communication board is designed around an Arduino Nano, it should also be relatively simple to adjust the nano software to operate as a stand-alone device.

## Minor Modifications to the Pololu 3Pi+
The IR Communication board has been designed to work with the Atmega32u4 variant of the Pololu 3Pi+ robot.  To use the IR Communication board with the Pololu 3Pi+ robot it is necessary to install some pin headers.  

## BOM, Bill of Materials


## Fabrication, Gerber files
Navigate to folder `pcb` to find the last known gerber files sent for manufacturing, `Gerbers.zip`.  

## Schematic
The schematic of the communication board is available in picture format in the `images` folder and within the pcb folder as KiCad files.
<img src="https://raw.githubusercontent.com/paulodowd/Swarm-B2/main/images/schematic.svg">

## Notes on Using a M5Stack Core2

- The docs for the M5Stack Core2 are at this <a href="https://docs.m5stack.com/en/core/core2">link</a>.
- Follow the guides at <a href="https://docs.m5stack.com/en/arduino/arduino_ide">this link</a> to setup the Arduino IDE.
