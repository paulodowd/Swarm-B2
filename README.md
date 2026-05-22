<p align="center">
<br>
<img src="https://github.com/paulodowd/Swarm-B2/blob/main/images/top_3pi_m5.jpg?raw=true" width="250"></img>
<img src="https://github.com/paulodowd/Swarm-B2/blob/main/images/on3pi_45.jpg?raw=true" width="250"></img>
<br>
</p>

# Swarm-B2
Infra-red (IR) Communication board for Pololu 3Pi+ robots that also has a pin header to interface with an M5 Stack Core2.  This is the second major design of such a board, hence the name (S)warm (B)oard 2 - SwarmB2.  A third version is under-development with the intention to supercede this design ([SwarmB3](https://github.com/paulodowd/SwarmB3)). 

The board functions as an i2c device, so it should be relatively easy to integrate into other projects using 5v, ground, SCK and SDA connections.  The board can communicate with other SwarmB2 boards up to about 2.5m max, or down to less than a centimetre.  The firmware for the board has been written with the intention that all (or most) configuration parameters can be set at runtime, so reprogramming firmware shouldn't be necessary.

## Features
- 32 bytes per message.
- SLIP-style encoding/escaping.
- 9600 or 4800 baudrate (device dependent)
- Receive or transmit, cannot do both concurrently.
- Receiving is polled across 4 receivers.
- Transmission power set by a potentiometer (0-2.5m)
- Message framing with start delimiter and message length embedded.
- A 16 bit CRC to detect transmission errors.
- Error statistics at message level.
- Performance statitics (timings, etc).
- Bearing estimation.
- 3x LDR and 2x Sharp IR Range Finder integration on PCB.
- Reconfigurable communication strategies/priorities.
- Header pins to attach to a Pololu 3Pi+ robot.
- Header pins for an M5Stack Core2 device interfacing.


## Getting Started

The best way to get started is to download and install the example <a href="https://github.com/paulodowd/Swarm-B2/tree/main/examples/M5Visualiser">M5Visualiser</a> written for the M5Stack Core2.  This example demonstrates how to make function calls through the `SwarmB2_c` class. The class has function calls for all functionality available on the Swarm-B2, returning data types specified in the file `ircomm_i2c.h`.  

## Working with the M5Stack Core2

To get started with the M5Stack Core2, it is best to follow the manufacturers documentation to configure the Arduino IDE.  Make sure you follow the instructions for both:
- <a href="https://docs.m5stack.com/en/arduino/arduino_board">Board Management Installation</a>
- <a href="https://docs.m5stack.com/en/arduino/arduino_library">Arduino Library Installation</a>

The <a href="https://github.com/paulodowd/Swarm-B2/tree/main/examples/M5Visualiser">M5Visualiser</a> example was written against the M5Unified library - installing or using a different M5Stack library is likely to cause compilation errors for the provided example.

## Important Characteristics

### Message Size
The Swarm-B2 can receive a message of maximum length 32 bytes to transmit.  This limitation is currently defined by the Arduino i2c implementation.  The Swarm-B2 can accept a message of upto 32bytes, and it will then encode the message using a further number of bytes.   It is important to consider that when transmitting a long length message, the probability that the message will become corrupted (or similar) increases.  

### Polling
<p align="center">
  <img src="https://github.com/paulodowd/Swarm-B2/blob/main/images/polling.png?raw=true", width="200">
</p>
The Swarm-B2 board can only read one receiver at a time. The board is configured by default to switch which receiver is enabled as soon as one message is successfully received.  The board is also configured by default to switch receiver if it detects no activity.  Otherwise, the board is configured by default to listen for a period of time before switching receiver. 

### Transmitting
The Swarm-B2 must disable receiving to be able to transmit, otherwise it will immediately receive it's own transmission.  Therefore, the board is configured to periodically transmit messages, and in the interval period poll the receivers to receive messages.  

### Transmitting Power
The power of the IR LEDs which transmit messages can be varied via the blue potentiometer on the back of the Swarm-B2 circuit board.  The blue potentiometer has a small silver dot: rotating the white arrow indicator to point at this silver dot will produce maximum power, or maximum range of transmission.  Swarm-B2 can transmit upto 3m distance.  However, this is not very useful in a swarm/multi-robot context.  If the transmit power is set high, it is likely that the environment will become saturated with IR emissions, effectively blocking transmissions between all other robots.  The recommended position for the potentiometer is indicated in the below image with a pink dot.  This position sets the communication distance to approximately 20cm, or two robot-body lengths.

<p align="center">
<img src="https://github.com/paulodowd/Swarm-B2/blob/main/images/SwarmB2_Back.jpg?raw=true" width="300">
</p>


### Other Configurations
It is possible to change the Swarm-B2 configuration via i2c transactions.  For example, it is possible to setup a Swarm-B2 to only transmit continously.  Or, it is possible to setup the Swarm-B2 to only receive on 1 receiver continuously.  

## FAQ


## Using the Swarm-B2 board in other Projects
This IR communication board can be used as a general purpose communication board.  The firmware on this github page is written as an i2c slave device with address `0x11`.  It should be possible to use the IR Communication board in it's current design and format with any other device that can operate the I2C protocol.  To do so, you simply need to connect the `5v`, `GND`, `SCL` and `SDA` pins appropriately to your operating device.  These physical pins are labelled on the underside of the circuit board as `+RED` (5v), `-BLK` (GND), `SDA` and `SCL`, or as `5V`, `GND`, `SCL`, `SDA` on the  topside of the circuit board.

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

