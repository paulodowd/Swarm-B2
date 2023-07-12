
/*
 * 
 *  SENDER
 * 
 */

#include <avr/io.h>
#include "ircomm.h"

// Used to control the IR communication.
IRComm_c ircomm;

float count;  // We will send a floating point number as a test

#define COUNT_UPDATE_MS 64     // how often to update count?
unsigned long count_update_ts; // timestamp

void setup() {

  // Start the IR communication board.
  ircomm.init();
  ircomm.enableTx();

  // We will use "count" to transmit
  // a changing value
  count = 0.0;

}




void loop() {

  // "Slowly" update the count
  if( millis() - count_update_ts > COUNT_UPDATE_MS ) {
    count_update_ts = millis();
    count += 0.01;
  }

  
  // Tell the board to transmit a floating point
  // value 
  ircomm.transmitFloat( count );

  // This line must be called to process new
  // received messages and transmit new messages
  // Note that, IRComm_c class will manage how often
  // it will transmit messages itself.  Take a look
  // at the setTxDelay() function.
  // It means we still call this update at a relatively
  // high frequency.
  ircomm.update();

  
  delay(2);
}

// Experiment to see if reading the LSB
// of an analog read can generate a
// useful random seed.  Nothing connected
// to A0.  I think the LSB fluctuates
// randomly(?)
void initRandomSeed() {
  pinMode(A0, INPUT);
  byte r = 0x00;
  for ( int i = 0; i < 8; i++ ) {
    byte b = (byte)analogRead(A0);
    
    b = b & 0x01;
    r |= (b << i);
    delayMicroseconds(2);
  }
  //Serial.println(r, DEC);
  randomSeed( r );
}
