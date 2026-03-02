/*************************************

  Example to demonstrate BRK detection from within custom Rx-ISR handler

  Setup:
    - use Arduino Mega and connect Tx1 (=sender) with Rx2 (=receiver)

*************************************/

////////////////
// INCLUDE LIBRARIES
////////////////

#include <NeoHWSerial.h>


////////////////
// INCLUDE MACROS
////////////////

// comment out for handling only in ISR
//#define STORE_BUFFER


////////////////
// GLOBAL VARIABLES
////////////////

// flag to indicate a BRK (set in rx_handler())
bool BRK = false;


////////////////
// GLOBAL FUNCTIONS
////////////////

/////////////////////
// custom Serial receive interrupt handler
/////////////////////
bool rx_handler(uint8_t byte, uint8_t status)
{
  // avoid compiler warnings
  (void)(byte);
  (void)(status);

  // check for framing error
  BRK = (bool) (status & (1<<FE0));
    
  NeoSerial.print("ISR: Rx=0x");
  NeoSerial.print(byte, HEX);
  if (BRK)
    NeoSerial.print(", BRK");
  NeoSerial.println();

  // return true -> byte is stored in Serial buffer
	#if defined(STORE_BUFFER)
    return true;
  
  // return false -> byte is not stored in Serial buffer
	#else
    return false;
  #endif

} // rx_handler()


/////////////////////
// initialization
/////////////////////
void setup()
{
  // serial console
  NeoSerial.begin(115200);

  // sender
  NeoSerial1.begin(19200);

  // receiver
  NeoSerial2.begin(19200);
  NeoSerial2.attachInterrupt(rx_handler);

} // setup()


/////////////////////
// main loop
/////////////////////
void loop() {

  uint8_t  c;
  
  // send BRK via Serial1 (0x00 w/o stop bit)
  NeoSerial1.begin(9600);
  NeoSerial1.write(0x00);
  NeoSerial1.flush();
  
  // receive BRK via Serial2
  if (NeoSerial2.available())
  {
    c = NeoSerial2.read();
    NeoSerial.print("loop: Rx=0x");
    NeoSerial.print(c, HEX);
    if (BRK)
      NeoSerial.print(", BRK");
    NeoSerial.println();
  }
  BRK = false;

  // send SYNC via Serial1 (0x55)
  NeoSerial1.begin(19200);
  NeoSerial1.write(0x55);
  NeoSerial1.flush();
  
  // receive BRK via Serial2
  if (NeoSerial2.available())
  {
    c = NeoSerial2.read();
    NeoSerial.print("loop: Rx=0x");
    NeoSerial.print(c, HEX);
    if (BRK)
      NeoSerial.print(", BRK");
    NeoSerial.println();
  }
  BRK = false;

  // wait some time
  delay(2000);
  NeoSerial.println();

} // loop()
