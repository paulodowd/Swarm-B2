
/*
 * 
 *  RECEIVER
 * 
 */

#include <avr/io.h>
#include "ircomm.h"

// Used to control the IR communication.
IRComm_c ircomm;

unsigned long cycle_ts;
#define CYCLE_MS 100      // how often do we change receiver?

void setup() {
  
  // Start the IR communication board.
  ircomm.init();

  // This program will be receiving messages
  // only.  So we disable transmit (Tx) 
  // functionality.
  ircomm.stopTx();
  ircomm.disableTx();

  // Register start time to cycle which
  // IR receiver is used
  cycle_ts = millis();
}




void loop() {

  // Rotate which IR receiver is active
  if( millis() - cycle_ts > CYCLE_MS ) {
    cycle_ts = millis();
    ircomm.cyclePowerRx();
  }

  // This line must be called to process new
  // received messages and/or transmit new messages
  ircomm.update();

  // Check if a message has been received.
  // Note that, this will only report true if
  // a message was received correctly (no errors)
  if( ircomm.hasMsg() ) {

    // If you want to use Serial.print() to print your
    // debug statements, you need to tell the board to 
    // stop receiving temporarily. Othewise, you will
    // receive your own debug statements!
    ircomm.disableRx();

    // Let's look at what was received.
    // Assuming a floating point number has been sent.
    Serial.print("Receiver  ");
    Serial.print( ircomm.getActiveRx() );      // which recevier?
    Serial.print(": ");
    Serial.println( ircomm.getFloatValue() ); // message content

    // Make sure we finish sending these debug
    // comments before we begin the receiving again.
    Serial.flush();                 

    // If you don't delete this received message, it will
    // stay for 1 second and then be automatically deleted.
    // This will delete the message immediately
    ircomm.clearRxMsg();

    // Don't forget to re-enable receiving messages after
    // you are done printing your debug statements.
    ircomm.enableRx();

  } else {

    //Serial.println("No Message");

  }


  delay(10);
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
