
/*
 * 
 *  RECEIVER
 * 
 */

#include <avr/io.h>
#include "ircomm.h"

// Used to control the IR communication.
IRComm_c ircomm;


float number_of_messages_received = 0;

unsigned long cycle_ts;
#define CYCLE_MS 20      // how often do we change receiver?

int count[5] = {0,0,0,0,0};

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

  Serial.println("Reset");

  ircomm.powerOnRx(0);
}



// How long do we collect messages for?
void loop() {

  // Rotate which IR receiver is active
  if( millis() - cycle_ts > CYCLE_MS ) {
    cycle_ts = millis();
    //ircomm.cyclePowerRx();
  }

  // This line must be called to process new
  // received messages and/or transmit new messages
  ircomm.update();

  
    //Serial.print("Active receiver NOW: ");
    //Serial.println( ircomm.rx_pwr_index );

  // Check if a message has been received.
  // Note that, this will only report true if
  // a message was received correctly (no errors)
  int which_has_msg = ircomm.hasMsg();
  if( which_has_msg >= 0 ) { // -1 no message

    ircomm.disableRx();
    Serial.print("Has message index = ");
    Serial.println( which_has_msg );
    Serial.println("Message buffers: ");
    for( int i = 0; i < 5; i++ ) {
      
      
      // A blank buffer starts with !
      // != '!' means a message is there
      if( ircomm.rx_msg[i][0] != '!' ) {
        count[i]++;
      }
      //Serial.print( ircomm.rx_msg[i]);
      //Serial.print(" : " );
      //Serial.println( ircomm.getFloatValue( i ) );
    }
    Serial.println( ircomm.getFloatValue( 0 ) );
    Serial.flush(); 

    ircomm.clearRxMsg( which_has_msg );
    ircomm.enableRx();
    /*

    // If you want to use Serial.print() to print your
    // debug statements, you need to tell the board to 
    // stop receiving temporarily. Othewise, you will
    // receive your own debug statements!
    ircomm.disableRx();

    // Let's look at what was received.
    // Assuming a floating point number has been sent.
    Serial.print("Active Receiver  ");

    int which_receiver = ircomm.getActiveRx();
    
    Serial.println( which_receiver );      // which recevier?

    Serial.print("Has message in buffer: " );
    Serial.println( which_has_msg );
    Serial.print(": ");
    Serial.println( ircomm.getFloatValue(which_has_msg) ); // message content

    // Make sure we finish sending these debug
    // comments before we begin the receiving again.
    Serial.flush();                 

    // If you don't delete this received message, it will
    // stay for 1 second and then be automatically deleted.
    // This will delete the message immediately
    ircomm.clearRxMsg( which_has_msgreceiver );

    // Don't forget to re-enable receiving messages after
    // you are done printing your debug statements.
    ircomm.enableRx();
    */
  } else {

    //Serial.println("No Message");

  }

  //Serial.println("Results:");
  //for( int i = 0; i < 5; i++ ) {
  //  Serial.print( count[i] );
  //  Serial.print(",");
  //}
  //Serial.println("");

  delay(1);
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
