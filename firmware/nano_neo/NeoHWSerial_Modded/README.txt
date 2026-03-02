This library ("NeoHWSerial_Modded") is a version of NeoHWSerial by SlashDevin, original details below.
This version was modified by Paul O'Dowd March 2026.
The modification allows for UART frame errors to be caught, counted, reported and reset.

Modifications within NeoHWSerial.h file to add:
- Line 112: volatile uint32_t frame_error_count; 
- Line 139: uint32_t getFrameErrorCount();
- Line 130: void resetFrameErrorCount();
- Line 160: code to catch UART frame error and count them.

Modifications within NeoHWSerial.cpp file to add:
- Line 144: getFrameErrorCount() {} to report count
- Line 151: resetFrameErrorCount() {} to reset count
