# Swarm-B2
IR Communication board for Pololu 3Pi+ robots that also accommodates an M5 Stack Core2

## IR Communication board Programming / Updating
To program the IR Communication board with the software, or to update it with your own software, it is necessary to first remove the jumper situated on the back of the circuit board labelled `RX_break`.  If you do not do this, the Arduino IDE will report that the programmer cannot sync with the arduino device (or similar error).  Remember to replace this jumper, because it connects the IR receiver modules electronically to the Arduino. 

## Fabrication, Gerber files
Navigate to folder `pcb` to find the last known gerber files sent for manufacturing, `Gerbers.zip`.  

## Schematic
The schematic of the communication board is available in picture format in the `images` folder and within the pcb folder as KiCad files.
<img src="https://raw.githubusercontent.com/paulodowd/Swarm-B2/main/images/schematic.svg">
