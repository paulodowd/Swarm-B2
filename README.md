# Swarm-B2
IR Communication board for Pololu 3Pi+ robots that also accommodates an M5 Stack Core2.  
<p align="center">
<br>
<img src="https://github.com/paulodowd/Swarm-B2/blob/main/images/top_3pi_m5.jpg?raw=true"></img>
<img src="https://github.com/paulodowd/Swarm-B2/blob/main/images/on3pi_45.jpg?raw=true"></img>
<br>
</p>

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

## Background Information

A pcb design to create infra-red (IR) communication between Pololu 3Pi+ robots. The design utilises an Arduino Nano as an I2C slave device.  The pro mini handles all transmit and receive operations over IR. The master device (intended as the 3Pi+, but could be any device with I2C) can upload a string to transmit, and also request the latest strings received, if any.  This can get a little confusing to discuss: I2C is used to transfer messages between Master and Slave, and Serial/IR is used to tranmit messages between slave devices.  

IR communication is implemented using the standard Serial functionality common to Arduino (e.g. `Serial.print()`), and so the IR circuit is electronically connected to  the Arduino TX and RX pins.  The detection (receiving) circuit uses a Demodulator chip that is tuned for a carrier frequency of 38.4Khz (more details below).  Therefore, the Arduino Nano generates a 38Khz signal via an internal timer interrupt to provide the transmitting carrier, output on digital pin 4.  

The 38Khz carrier signal is combined electronically with the TX pin state through an OR gate (74xx32). The logic (OR) of this operation is defined by the gates sinking the current through the tranmitting IR LEDs.  Therefore, the LEDs are not driven by the 74xx32 (or the pro mini, to be clear), and are considered active low.  A nice unintended side effect of this is that when TX is inactive (LOW, not transmitting), the 74xx32 output is HIGH despite the continued 38Khz signal generation - which means the IR LEDs are off and not saturating the environment.

In this circuit design, a single gate is used to sink current.  It might be important to note that the <a href="https://www.ti.com/lit/ds/symlink/sn54hc32-sp.pdf">datasheet</a> for the logic gates specifies a max current sink of 50ma through ground - and here we are potentially sinking upto 250ma from the 4 IR LEDs.    So far, I've not seen any significant changes to performance and this may be because I am not sinking current continuously, instead intermitently at 38Khz. All 4 OR gates within the 74xx32 are ganged, but the internal ground connection may still be a limitation. In either case, I haven't yet seen any problems or difference in performance.  It might be unwise to active the IR LEDs continuously.

Some messages transmitted over IR do get garbled.  As a quick solution, the firmware uses a CRC checksum.  I've used the same implementation as detailed on the RepRap wiki for GCode (<a href="https://reprap.org/wiki/G-code#Checking">here</a>).  Prior to transmitting, the Arduino Nano generates a single byte checksum value, and appends this to the message string.  Therefore, when a message is received, the receiving Arduino Nano completes the same CRC check on the received message for itself, and then compares if the checksum values match.  If not, some data must be corrupt and the message is discarded.  The firmware keeps track of how many messages are received successfully or not.

The message strings transmitted between the Arduino Nano devices would look something like:

`*message string@f!`

Here, the `*` is used to identify the start of a message, the `@` is used to identify the end of the message and that the next byte is the checksum value.  The `!` symbol identifies the end of the message string.  In the example above, I haven't used the correct checksum value, it is just an illustrative example.  

The communication board has 4 receivers spaced equally around the radius of the board. Each receiver can be independently enabled or disabled.  This means it is possible to determine from which direction a message was received by sequentially activating a receiver for a period of time.  However, this does mean the receivers must be polled, and it is not obvious what the optimum period of time is to wait to receive any messages.  The communication board will store 1 message for each receiver, which is over-written by any more recently received messages.  When the master device (i.e. the 3Pi+ robot) requests a message from the slave (the communication board), once the message has been sent over I2C it is deleted on the slave (the communication board).

When the master sets a string for the slave to transmit over IR, this should be sent over I2C with no terminating character (e.g., no newline, or anything specific, just the string you wish to transmit).  including `*`, `\n` or `!` will break functionality!  The the slave/Arduino Nano will append the tokens, checksum value etc.  

### Important Considerations

- **IR Demodulator:** I tried a few IR demodulator chips and they are not all created equally.  In particular, those designed to receive bursts of transmission appear poorly suited.  When using ones for burst transmission, I observed that I had to wave the transmitter or receiver around to get a successful transmission (perhaps some inner circuitry reaches a saturation?).  Instead it is better to use those designed for continuous transmission. The Vishay **TSDP34138** (<a href="https://www.farnell.com/datasheets/2245004.pdf">datasheet</a>) seems to fit this and works well / reliably.  The datasheet specifies upto 4800 baud for a 38.4Khz carrier frequency.  If you want to reproduce this board, I recommend you use the same device, or be ready for some trials of different devices.
- **Self-Receiving:** I did have an issue of a single board receiving it's own IR transmissions, and not being able to receive transmission from neighbours.  I believe this is because the UART implementation (on the Atmega328 at least) operates send (TX) and receive (RX) independently.  When using IR light, this means it will detect it's own transmission. After attempting a lot of different solutions, it was as simple as disabling the RX functionality of the UART by a register operation.  I'm glad this was available!  You can review the details of this on page 171 of the Atmega328 datasheet (<a href="https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf">here</a>).  
- **Programming the Arduino Nano:** It is important to note that the programming of the Pro Mini will most often fail if the IR Demodulator is electronically connected.  Therefore, the circuit design has a jumper (labelled, "RX Break") intended for a connecting removable sleeve, to allow the connection to be broken for programming.  Importantly, no IR messages can be received if this jumper is not closed with the sleeve.  
- **Buffer Sizes:** The main limitation on the size of a string that can be sent over IR is actually the number of bytes that can be transferred on the I2C bus.  I think this is dependent on the Arduino Wire library and could be modifed. Or, strings to transmit could be broken into parts to transfer over I2C.  A second consideration is that the Arduino UART device has a buffer of 64 bytes.  For simplicity, the software current has buffers of 32 bytes.  Importantly, 3 bytes are reserved for the tokens `*`, `@` and `!`.- 

## Notes on Using a M5Stack Core2

- The docs for the M5Stack Core2 are at this <a href="https://docs.m5stack.com/en/core/core2">link</a>.
- Follow the guides at <a href="https://docs.m5stack.com/en/arduino/arduino_ide">this link</a> to setup the Arduino IDE.
