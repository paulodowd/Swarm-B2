
#include <avr/io.h>
#include <Wire.h>
#include "ircomm.h"
#include "ircomm_data.h"

#define I2C_ADDR  8

// Used to control the IR communication.
IRComm_c ircomm;

// How frequently to update the general
// status of the board, e.g., the LDR
// readings.
unsigned long status_ts;
#define STATUS_MS 250

// Cycle for which IR receiver is being
// used.
unsigned long cycle_ts;
#define CYCLE_MS 70

i2c_mode_t last_mode;
i2c_status_t status;

void i2c_recv( int len ) {


  if ( len == 1 ) { // receiving a mode change.
    i2c_mode_t new_mode;
    Wire.readBytes( (byte*)&new_mode, sizeof( new_mode ) );
    // Check we are setting to a valid mode.
    if( new_mode.mode < MAX_MODE && new_mode.mode > 0 ) {
      last_mode.mode = new_mode.mode;
    }
  } else { // Receiving a new message to transmit.

    char buf[MAX_MSG]; // temp buffer
    memset(buf, 0, sizeof(buf));
    int count = 0;
    while ( Wire.available() && count < MAX_MSG ) {
      char c = Wire.read();
      buf[count] = c;
      count++;
    }

    // If it is a malformed message from the robot,
    // or the ! symbol
    // we clear the tx_buf and so stop sending messages
    if ( count <= 1 || buf[1] == '!' ) {
      //Serial.println(" Cleared");

      // CLEAR TX BUF, DISABLE TX
      //memset( tx_buf, 0, sizeof( tx_buf ) );
      ircomm.clearTxBuf();
    }
    //Serial.print("I2C Received:" );
    //Serial.println(buf);
    ircomm.transmitString( buf, strlen(buf));
  }

}

// When the Core2 calls an i2c request, this function
// is executed.  Sends robot status to Core2.
void i2c_send() {
  if ( last_mode.mode == MODE_REPORT_STATUS ) {

    Wire.write( (byte*)&status, sizeof( status ) );
  } else if ( last_mode.mode == MODE_REPORT_LDR0 ) {
    Wire.write((byte*)&status.ldr[0], sizeof( status.ldr[0] ));
    
  } else if ( last_mode.mode == MODE_REPORT_MSG0 ) {
    if ( ircomm.rx_msg[0] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.rx_msg[0], strlen(ircomm.rx_msg[0]) );
    }
    
  } else if ( last_mode.mode == MODE_REPORT_MSG1 ) {
    if ( ircomm.rx_msg[1] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.rx_msg[1], strlen(ircomm.rx_msg[1]) );
    }
    
  } else if ( last_mode.mode == MODE_REPORT_MSG2 ) {
    if ( ircomm.rx_msg[2] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.rx_msg[2], strlen(ircomm.rx_msg[2]) );
    }
    
  } else if ( last_mode.mode == MODE_REPORT_MSG3 ) {
    if ( ircomm.rx_msg[3] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.rx_msg[3], strlen(ircomm.rx_msg[3]) );
    }
    
  }
}




void setup() {
  // Set random seed.
  initRandomSeed();

  // Pinmodes
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);


  // Begin I2C as a slave device.
  Wire.begin( I2C_ADDR );
  Wire.onReceive( i2c_recv );
  Wire.onRequest( i2c_send );

  // Start the IR communication board.
  ircomm.init();

  ircomm.powerOnRx(2);

  last_mode.mode = MODE_REPORT_STATUS;

  status.mode = 0;
  status.ldr[0] = 0;
  status.ldr[1] = 0;
  status.ldr[2] = 0;
  status.msg_count[0] = 0;
  status.msg_count[1] = 0;
  status.msg_count[2] = 0;
  status.msg_count[3] = 0;


  cycle_ts = millis();
  status_ts = millis();

}




void loop() {

  if ( millis() - status_ts > STATUS_MS ) {
    status_ts = millis();

    status.ldr[0] = analogRead( A0 );
    status.ldr[1] = analogRead( A1 );
    status.ldr[2] = analogRead( A2 );
    status.mode = (uint8_t)ircomm.rx_pwr_index;
    //Serial.println("Status Update");
  }


  if ( millis() - cycle_ts > CYCLE_MS ) {
    cycle_ts = millis();
    ircomm.cyclePowerRx();
  }


  // If you don't want your board to transmit,
  // you can use the following command.
  //ircomm.stopTx();
  //ircomm.disableTx();

  // This line must be called to process new
  // received messages and transmit new messages
  ircomm.update();


  // Check if a message has been received:
  if ( ircomm.hasMsg(ircomm.rx_pwr_index) ) {

    
    // If you want to use Serial.print() to print your
    // debug statements, you need to tell the board to
    // stop receiving temporarily. Othewise, you will
    // receive your own debug statements!
    //ircomm.disableRx();

    if ( ircomm.rx_pwr_index < RX_PWR_N ) {
      status.msg_count[ ircomm.rx_pwr_index ]++;
    }

    // Let's look at what was received.
    // Assuming a floating point number has been sent.
//    Serial.print("Receiver  ");
//    Serial.print( ircomm.rx_pwr_index );
//    Serial.print(": ");
//    Serial.println( ircomm.getFloatValue(ircomm.rx_pwr_index) );
//    Serial.flush();

    // If you don't delete this received message, it will
    // stay for TTL time (ircomm.h) and then be automatically deleted.
    //ircomm.clearRxMsg(ircomm.rx_pwr_index);

    // Don't forget to re-enable receiving messages after
    // you are done printing your debug statements.
    //ircomm.enableRx();
    
  } else {

    //Serial.println("No Message");
    //Serial.println("Status:");
    //Serial.print( status.ldr[0] ); Serial.print(",");
    //Serial.print( status.ldr[1] ); Serial.print(",");
    //Serial.print( status.ldr[2] ); Serial.print("\n");
    //Serial.print( status.msg_count[0] ); Serial.print(",");
    //Serial.print( status.msg_count[1] ); Serial.print(",");
    //Serial.print( status.msg_count[2] ); Serial.print("\n");

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
