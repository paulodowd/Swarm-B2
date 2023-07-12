
#include <avr/io.h>
#include "ircomm.h"

// Used to control the IR communication.
IRComm_c ircomm;

unsigned long cycle_ts;
#define CYCLE_MS 100

void setup() {
  // Set random seed.
  initRandomSeed();

  // Start the IR communication board.
  ircomm.init();

  cycle_ts = millis();

}




void loop() {


  if( millis() - cycle_ts > CYCLE_MS ) {
    cycle_ts = millis();
    ircomm.cyclePowerRx();
  }


  // If you don't want your board to transmit,
  // you can use the following command.
  ircomm.stopTx();
  ircomm.disableTx();
  
  // This line must be called to process new
  // received messages and transmit new messages
  ircomm.update();

  // Tell your board to transmit a floating point
  // value like this:
  // Transmission becomes enabled by default.
  //ircomm.transmitFloat(99.2);

  // Check if a message has been received:
  if( ircomm.hasMsg() ) {


    // If you want to use Serial.print() to print your
    // debug statements, you need to tell the board to 
    // stop receiving temporarily. Othewise, you will
    // receive your own debug statements!
    ircomm.disableRx();

    // Let's look at what was received.
    // Assuming a floating point number has been sent.
    Serial.print("Receiver  ");
    Serial.print( ircomm.rx_pwr_index );
    Serial.print(": ");
    Serial.println( ircomm.getFloatValue() );
    Serial.flush();

    // If you don't delete this received message, it will
    // stay for 1 second and then be automatically deleted.
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
