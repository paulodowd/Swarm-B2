
# Swarm-B2
Infra-red (IR) Communication board for Pololu 3Pi+ robots that also has a pin header to interface with an M5 Stack Core2.  I've designed this board to conduct research into specific aspects of IR Communication in a swarm robotics context.  This is the second major design of such a board, hence the name (S)warm (B)oard 2 - SwarmB2.

The board functions as an i2c device, so it should be quite easy to integrate into other projects using 5v, ground, SCK and SDA connections.  The board can communicate with other SwarmB2 boards up to about 2.5m.  

## Features
- 32 bytes per message.
- 9600 or 4800 baudrate (device dependent)
- Receive or transmit, cannot do both concurrently.
- Receiving is polled across 4 receivers.
- Transmission power set by a potentiometer (0-2.5m)
- Message framing with start delimiter and message length embedded.
- A 16 bit CRC to detect transmission errors.
- A COBS/SLIP style byte-stuffing/escaping.
- UART level Frame Error tracking.
- Error statistics at message level.
- Performance statitics (timings, etc).
- Bearing estimation.
- 3x LDR and 2x Sharp IR Range Finder integration.
- Reconfigurable communication strategies/priorities.
- I2C API to reconfigure firmware.
- Header pins to attach to a Pololu 3Pi+ robot.
- Header pins for an M5Stack Core2 device interfacing.

## Things I wish I had included
At some point one has to stop tinkering to achieve a larger goal.  If I was to revise this board again I would:
- Include an LED next to each IR Demodulator, so I could easily see when/where messages are being received from.
- Fix IR LED resistance, and wire the supply of the each LED to the Nano digital pins.  Note that, with 5k ohm resistance in series with the 8 IR LEDs, transmission can still occur over 2cm.  So it seems possible to simply drive each IR LED from the Nano directly (very low current) for a reasonable transmission distance (30cm?), and this would allow for directional transmission.
- Or, find a digital potentiometer to allow for the transmission power to be set from the device, rather than a potentiometer.
- Include a reset button on the top side of the board for the Nano device.
- Move away from the Arduino Nano, to something like the Teensy 4.0/1 which has 8 (!) hardware UART interfaces - which would remove the need to poll receivers, and provide continuous directional receiving.   

## SwarmB2 fully assembled
<p align="center">
<br>
<img src="https://github.com/paulodowd/Swarm-B2/blob/main/images/top_3pi_m5.jpg?raw=true" width="250"></img>
<img src="https://github.com/paulodowd/Swarm-B2/blob/main/images/on3pi_45.jpg?raw=true" width="250"></img>
<br>
</p>

## SwarmB2 Programming / Updating
SwarmB2 is built around an Arduino Nano, so you should follow the normal steps to program one.  To program the IR Communication board with the software, or to update it with your own software, it is necessary to first remove the jumper situated on the back of the circuit board labelled `RX_break`.  If you do not do this, the Arduino IDE will report that the programmer cannot sync with the arduino device (or similar error).  Remember to replace this jumper, because it connects the IR receiver modules electronically to the Arduino. Without the jumper, your board won't receive any IR messages.

The current firmware includes a modified copy of the library <a href="https://github.com/SlashDevin/NeoHWSerial">NeoHWSerial originally by SlashDevin</a>.  My modification allows for UART frame errors to be caught, counted, reported and reset.  This seems like a very niche requirement, I'm not sure how portable the modification is - so I've opted not to fork/make a pull request.  The original attribution and licensing information are included within the sub-directory `Swarm-B2/firmware/nano_neo/NeoHWSerial_Modded/`. 

## SwarmB2 in other Projects
This IR communication board can be used as a general purpose communication board.  The firmware on this github page is written as an i2c slave device with address `0x11`.  It should be possible to use the IR Communication board in it's current design and format with any other device that can operate the I2C protocol.  To do so, you simply need to connect the `5v`, `GND`, `SCL` and `SDA` pins appropriately to your operating device.  These physical pins are labelled on the underside of the circuit board as `+RED` (5v), `-BLK` (GND), `SDA` and `SCL`, or as `5V`, `GND`, `SCL`, `SDA` on the  topside of the circuit board.

## 38Khz or 56Khz?
This board is designed around the use of either the TSDP34138 or the TSDP34156, which demodulate a 38.4Khz or 57.6Khz carrier frequency respectively.  The firmware needs to be configured (config.h - comment/uncomment as necessary) for which TSDP341xx device you've populated on your SwarmB2 board.  

Please note that all IR Demodulators like this are not created equally!  These two in particular are designed by Vishay for "continuous" data rates, which is important for the IR communication boards.  

## SwarmB2 Configuration

The SwarmB2 board has a lot of possible configuration. I expect that different tasks will require different configuration.  The firmware has been written to accept various configuration and with an order of precedence for which settings are applied and when.  The complexity comes from the necessity to poll receivers, and that receive and transmit must happen independently (not at the same time).  This sets up some interesting variations such as (a) transmit after checking each receiver (b) transmit periodically, independent of receivers.

## Minor Modifications to the Pololu 3Pi+
The IR Communication board has been designed to work with the Atmega32u4 variant of the Pololu 3Pi+ robot.  To use the IR Communication board with the Pololu 3Pi+ robot it is necessary to install some pin headers.  

## BOM, Bill of Materials


## Fabrication, Gerber files
Navigate to folder `pcb` to find the last known gerber files sent for manufacturing, `Gerbers.zip`.  

## Schematic
The schematic of the communication board is available in picture format in the `images` folder and within the pcb folder as KiCad files.
<img src="https://raw.githubusercontent.com/paulodowd/Swarm-B2/main/images/schematic.svg">

## Interesting Information

IR communication is implemented using the standard Serial (hardware UART) functionality common to Arduino (e.g. `Serial.print()`), and so the IR circuit is electronically connected to  the Arduino TX and RX pins.  The detection (receiving) circuit uses a Demodulator chip that is tuned for a carrier frequency of 38.4Khz or 57.6Khz (more details below).  Therefore, the Arduino Nano generates a 38Khz or 58Khz signal via an internal timer interrupt to provide the transmitting carrier, output on digital pin 4.  

The carrier signal is combined electronically with the TX pin state through an OR gate (74xx32). The logic (OR) of this operation is defined by the gates sinking the current through the tranmitting IR LEDs.  Therefore, the LEDs are not driven by the 74xx32 (or the Arduino Nano, to be clear), and are considered active low.  A nice unintended side effect of this is that when TX is inactive (LOW, not transmitting), the 74xx32 output is HIGH despite the continued 38Khz signal generation - which means the IR LEDs are off and not saturating the environment.

In this circuit design, a single gate is used to sink current.  It might be important to note that the <a href="https://www.ti.com/lit/ds/symlink/sn54hc32-sp.pdf">datasheet</a> for the logic gates specifies a max current sink of 50ma through ground - and here we are potentially sinking upto 200ma from the 4 IR LEDs.    So far, I've not seen any significant changes to performance and this may be because I am not sinking current continuously, instead intermitently at 38Khz. All 4 OR gates within the 74xx32 are ganged, but the internal ground connection may still be a limitation. In either case, I haven't yet seen any problems or difference in performance.  It might be unwise to active the IR LEDs continuously (i.e. not modulated).

### Important Considerations

- **IR Demodulator:** I tried a few IR demodulator chips and they are not all created equally.  In particular, those designed to receive bursts of transmission appear poorly suited.  When using ones for burst transmission, I observed that I had to wave the transmitter or receiver around to get a successful transmission (perhaps some inner circuitry reaches a saturation?).  Instead it is better to use those designed for continuous transmission. The Vishay **TSDP34138** (<a href="https://www.farnell.com/datasheets/2245004.pdf">datasheet</a>) seems to fit this and works well / reliably.  The datasheet specifies upto 4800 baud for a 38.4Khz carrier frequency.  If you want to reproduce this board, I recommend you use the same device, or be ready for some trials of different devices.
- **Self-Receiving:** I did have an issue of a single board receiving it's own IR transmissions, and not being able to receive transmission from neighbours.  I believe this is because the UART implementation (on the Atmega328 at least) operates send (TX) and receive (RX) independently.  When using IR light, this means it will detect it's own transmission. After attempting a lot of different solutions, it was as simple as disabling the RX functionality of the UART by a register operation.  I'm glad this was available!  You can review the details of this on page 171 of the Atmega328 datasheet (<a href="https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf">here</a>).  
- **Programming the Arduino Nano:** It is important to note that the programming of the Arduino Nano will most often fail if the IR Demodulator is electronically connected.  Therefore, the circuit design has a jumper (labelled, "RX Break") intended for a connecting removable sleeve, to allow the connection to be broken for programming.  Importantly, no IR messages can be received if this jumper is not closed with the sleeve.  
- **Buffer Sizes:** The main limitation on the size of a string that can be sent over IR is actually the number of bytes that can be transferred on the I2C bus.  I think this is dependent on the Arduino Wire library and could be modifed. Or, strings to transmit could be broken into parts to transfer over I2C.  A second consideration is that the Arduino UART device has a buffer of 64 bytes.  For simplicity, the software current has buffers of 32 bytes.

## Notes on Using a M5Stack Core2

- The docs for the M5Stack Core2 are at this <a href="https://docs.m5stack.com/en/core/core2">link</a>.
- Follow the guides at <a href="https://docs.m5stack.com/en/arduino/arduino_ide">this link</a> to setup the Arduino IDE.

## Links and resources I have found useful
- Example code and discussion for different CRC algorithms: <a href="https://www.sunshine2k.de/articles/coding/crc/understanding_crc.html">link</a>
- Interesting Arduino forum post debugging CRC: <a href="https://forum.arduino.cc/t/crc-8-i2c-cyclic-redundancy-check/644812/4">link</a>
- Microchip Application Note with CRC: <a href="https://ww1.microchip.com/downloads/en/AppNotes/00730a.pdf">link</a>
- Discussion on CRC and data length: <a href="https://stackoverflow.com/questions/2321676/data-length-vs-crc-length">link</a>
- Some information on UART Baudrates: <a href="https://lucidar.me/en/serialib/most-used-baud-rates-table/">link</a>
- Discussion on start/stop bit and errors with UART: <a href="https://electronics.stackexchange.com/questions/91450/what-exactly-is-the-start-bit-error-in-uart">link</a>
- Disucssion on catching frame data overrun in UART: <a href="https://forum.arduino.cc/t/rs232-uart-frame-data-overrun-error-handling/193266/12">link</a>
- Table of ASCII values: <a href="https://www.ascii-code.com/">link</a>

