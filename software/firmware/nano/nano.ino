/*

   Currently getting 2.93ms per byte for IR transmission.
   I2C limits the string to max 32 bytes.  However, using
   4 bytes as start, token, checkbyte and terminal.
   Serial UART can buffer 64bytes.
*/


#include <avr/io.h>
#include <Wire.h>
#include "ircomm.h"
#include "ircomm_data.h"

#define I2C_ADDR  8

// Test/Debug modes
#define TEST_DISABLED 0
#define TEST_TX 1
#define TEST_RX 2

#define SELF_TEST_MODE TEST_DISABLED
//#define SELF_TEST_MODE TEST_TX
//#define SELF_TEST_MODE TEST_RX

// A timestamp used only to configure various
// testing/debugging activities.
unsigned long test_ts;
#define TEST_MS 10 // Good for test tx
//#define TEST_MS 300 // good for test rx


// Pin definitions for extra sensors
// on the communication board, if present
#define LDR0_PIN  A0
#define LDR1_PIN  A1
#define LDR2_PIN  A2
#define PROX0_PIN A6
#define PROX1_PIN A7

// Used to control the IR communication.
IRComm_c ircomm;


i2c_mode_t last_mode;
i2c_status_t status;


// Function to receive commands from the master device.
// This will typically be in the format of a mode change
// and then any appropriate data.
void i2c_recv( int len ) {


  if ( len == 1 ) { // receiving a mode change.
    i2c_mode_t new_mode;
    Wire.readBytes( (byte*)&new_mode, sizeof( new_mode ) );


    // Check we are setting to a valid mode.
    if ( new_mode.mode < MAX_MODE && new_mode.mode >= 0 ) {
      last_mode.mode = new_mode.mode;
    }

    // For instant mode changes
    // Should we set a last_mode state after this?
    if ( new_mode.mode == MODE_STOP_TX ) {

      // Clearing tx  buff will stop transmission of messages
      ircomm.clearTxBuf();

    } else if ( new_mode.mode == MODE_RESET_COUNTS ) {
      for ( int i = 0; i < 4; i++ ) {
        status.fail_count[i] = 0;
        status.pass_count[i] = 0;
      }

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
    ircomm.formatString( buf, strlen(buf));
  }

}

// When the 3Pi or Core2 calls an i2c request, this function
// is executed.
void i2c_send() {

  if ( last_mode.mode == MODE_REPORT_STATUS ) {


    for ( int i = 0; i < 4; i++ ) {
      status.pass_count[ i ] = ircomm.pass_count[i];
      status.fail_count[ i ] = ircomm.fail_count[i];
    }

    Wire.write( (byte*)&status, sizeof( status ) );

    //
    // Below are request for message size from the
    // master device.
    //
  } else if ( last_mode.mode == MODE_SIZE_MSG0 ) {
    i2c_mode_t msg_status;
    msg_status.mode = strlen( ircomm.rx_msg[0] );
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG1 ) {
    i2c_mode_t msg_status;
    msg_status.mode = strlen( ircomm.rx_msg[1] );
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG2 ) {
    i2c_mode_t msg_status;
    msg_status.mode = strlen( ircomm.rx_msg[2] );
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG3 ) {
    i2c_mode_t msg_status;
    msg_status.mode = strlen( ircomm.rx_msg[3] );
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


    //
    // Below are requests for actual message to be
    // reported to the master device
    //
  } else if ( last_mode.mode == MODE_REPORT_MSG0 ) {
    if ( ircomm.rx_msg[0] == 0 ) {
      Wire.write("!");  // Error token
    } else {
      Wire.write( ircomm.rx_msg[0], strlen(ircomm.rx_msg[0]) );
      // Delete message
      ircomm.clearRxMsg( 0 );
    }

  } else if ( last_mode.mode == MODE_REPORT_MSG1 ) {
    if ( ircomm.rx_msg[1] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.rx_msg[1], strlen(ircomm.rx_msg[1]) );
      // Delete message
      ircomm.clearRxMsg( 1 );
    }

  } else if ( last_mode.mode == MODE_REPORT_MSG2 ) {
    if ( ircomm.rx_msg[2] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.rx_msg[2], strlen(ircomm.rx_msg[2]) );

      // Delete message
      ircomm.clearRxMsg( 2 );
    }


  } else if ( last_mode.mode == MODE_REPORT_MSG3 ) {
    if ( ircomm.rx_msg[3] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.rx_msg[3], strlen(ircomm.rx_msg[3]) );

      // Delete message
      ircomm.clearRxMsg( 3 );
    }
  } else if ( last_mode.mode == MODE_REPORT_RX_ACTIVITY ) {
    i2c_activity_t activity;
    for ( int i = 0; i < 4; i++ ) {
      activity.rx[i] = ircomm.rx_activity[i];
    }
    // Transmit
    Wire.write( (byte*)&activity, sizeof( activity ) );

  } else if (  last_mode.mode == MODE_REPORT_RX_DIRECTION ) {

    i2c_bearing_t bearing;

    // Here, we treat each receiver as contributing to
    // either x or y, because they are aligned to the x
    // and y axis in their placement on the circuit board.
    // Therefore, rx0 is +y, rx1 is +x,
    // rx2 is -y, and rx3 is -x. We then treat them as
    // vectors, using atan2 to find the resultant
    // direction.  This is relative (local) to the robot.
    bearing.mag = ircomm.rx_activity[0];
    bearing.mag += ircomm.rx_activity[1];
    bearing.mag += ircomm.rx_activity[2];
    bearing.mag += ircomm.rx_activity[3];
    float x = (ircomm.rx_activity[0] - ircomm.rx_activity[2]);
    float y = (ircomm.rx_activity[1] - ircomm.rx_activity[3]);
    bearing.theta = atan2( y, x );

    // Transmit
    Wire.write( (byte*)&bearing, sizeof( bearing ) );

  } else if ( last_mode.mode == MODE_REPORT_SENSORS ) {

    // Data struct
    i2c_sensors_t sensors;

    // Collect sensor readings - should be very quick
    sensors.ldr[0] = (int16_t)analogRead( LDR0_PIN );
    sensors.ldr[1] = (int16_t)analogRead( LDR1_PIN );
    sensors.ldr[2] = (int16_t)analogRead( LDR2_PIN );
    sensors.prox[0] = (int16_t)analogRead( PROX0_PIN );
    sensors.prox[1] = (int16_t)analogRead( PROX1_PIN );

    // Transmit
    Wire.write( (byte*)&sensors, sizeof( sensors ) );
  }
}




void setup() {



  // Set random seed.
  initRandomSeed();

  // Pinmodes for LDR
  pinMode(LDR0_PIN, INPUT);
  pinMode(LDR1_PIN, INPUT);
  pinMode(LDR2_PIN, INPUT);
  pinMode(PROX0_PIN, INPUT);
  pinMode(PROX1_PIN, INPUT);

  // Debug LED
  pinMode( 13, OUTPUT );

  // Begin I2C as a slave device.
  Wire.begin( I2C_ADDR );
  Wire.onReceive( i2c_recv );
  Wire.onRequest( i2c_send );

  // Start the IR communication board.
  ircomm.init();

  ircomm.powerOnRx(2);

  last_mode.mode = MODE_REPORT_STATUS;

  status.mode = MODE_REPORT_STATUS;
  for ( int i = 0; i < 4; i++ ) {
    status.pass_count[i] = 0;
    status.fail_count[i] = 0;
  }

  test_ts = millis();

 

}




void loop() {


  // Periodic update of other board status
  if ( millis() - test_ts > TEST_MS ) {
    test_ts = millis();
    if ( SELF_TEST_MODE == TEST_TX ) {
      // Create a test string up to 29 chars long
      int max_chars = 19;
      char buf[ max_chars ];

      memset( buf, 0, sizeof( buf ) );

      sprintf( buf, "%lu", millis() );
      int ms_len = strlen( buf );
      buf[ms_len] = ':';
      ms_len++;
      for ( int i = ms_len; i < max_chars; i++ ) {
        buf[i] = (byte)(65 + i);
      }


      ircomm.formatString(buf, strlen(buf) );
    } else if ( SELF_TEST_MODE == TEST_RX ) {
      ircomm.disableRx();
      for ( int i = 0; i < 4; i++ ) {
        Serial.print( ircomm.pass_count[i] );
        Serial.print(",");
      }
      Serial.println();
      Serial.flush();
      ircomm.enableRx();
    }


  }





  // If you don't want your board to transmit,
  // you can use the following command.
  //ircomm.stopTx();
  //ircomm.disableTx();

  // This line must be called to process new
  // received messages and transmit new messages
  ircomm.update();

  // If you want to use Serial.print() to print your
  // debug statements, you need to tell the board to
  // stop receiving temporarily. Othewise, you will
  // receive your own debug statements!
  //ircomm.disableRx();


  // I'm copying like this because in the future I think we
  // can use this to detect new messages by comparing counts
  //  for( int i = 0; i < 4; i++ ) status.msg_count[ i ] = ircomm.msg_dt[i];//pass_count[ircomm.rx_pwr_index];



}

// Experiment to see if reading the LSB
// of an analog read can generate a
// useful random seed.  Nothing connected
// to A3, so it is floating and picking up
// emf.
void initRandomSeed() {
  pinMode(A3, INPUT);
  byte r = 0x00;
  for ( int i = 0; i < 8; i++ ) {
    byte b = (byte)analogRead(A3);
    b = b & 0x01;
    //Serial.println(b, BIN);

    r |= (b << i);
    delayMicroseconds(10);
  }
  //Serial.println(r, BIN);
  randomSeed( r );
}
