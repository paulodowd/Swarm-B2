/*



*/


#include <avr/io.h>
#include <Wire.h>
#include "ircomm.h"
#include "ircomm_i2c.h"


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
#define TEST_MS 50 // Good for test tx
#define TEST_MS 1000 // good for test rx

// Pin definitions for extra sensors
// on the communication board, if present
#define LDR0_PIN  A0
#define LDR1_PIN  A1
#define LDR2_PIN  A2
#define PROX0_PIN A6
#define PROX1_PIN A7

// Used to control the IR communication.
IRComm_c ircomm;

// Used to track requests made to
// the board over i2c
volatile i2c_mode_t last_mode;
volatile i2c_status_t status;


// Function to receive commands from the master device.
// This will typically be in the format of a mode change
// and then any appropriate data.
void i2c_receive( int len ) {


  // Mode changes are always 1 byte long.  We use these to
  // ready the board to send back information or to ask the
  // board to complete specific actions (e.g, delete a
  // message).
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

    } else if ( new_mode.mode == MODE_RESET_STATUS ) {
      memset( &status, 0, sizeof( status ) );
      for ( int i = 0; i < 4; i++ ) {
        ircomm.pass_count[i] = 0;
        ircomm.fail_count[i] = 0;
        ircomm.activity[i] = 0;
      }
      ircomm.rx_cycles = 0;
      ircomm.tx_count = 0;


      for ( int i = 0; i < 4; i++ ) {
        for ( int j = 0; j < 4; j++ ) {
          ircomm.error_type[i][j] = 0;
        }
      }


    } else if ( new_mode.mode == MODE_CLEAR_MSG0 ) {
      // Delete message
      ircomm.clearRxMsg( 0 );

    } else if ( new_mode.mode == MODE_CLEAR_MSG1 ) {
      // Delete message
      ircomm.clearRxMsg( 1 );

    } else if ( new_mode.mode == MODE_CLEAR_MSG2 ) {
      // Delete message
      ircomm.clearRxMsg( 2 );

    } else if ( new_mode.mode == MODE_CLEAR_MSG3 ) {
      // Delete message
      ircomm.clearRxMsg( 3 );

    } else if ( new_mode.mode == MODE_CLEAR_HIST ) {
      ircomm.hist[0] = 0;
      ircomm.hist[1] = 0;
      ircomm.hist[2] = 0;
      ircomm.hist[3] = 0;
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
    if ( count < 1 ) {
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
void i2c_request() {

  if ( last_mode.mode == MODE_REPORT_STATUS ) {


    for ( int i = 0; i < 4; i++ ) {
      status.pass_count[ i ] = (uint16_t)ircomm.pass_count[i];
      status.fail_count[ i ] = (uint16_t)ircomm.fail_count[i];
      status.activity[i] = (uint16_t)ircomm.activity[i];
    }

    Wire.write( (byte*)&status, sizeof( status ) );

    //
    // Below are request for message size from the
    // master device.
    //
  } else if ( last_mode.mode == MODE_REPORT_TIMINGS ) {

    i2c_msg_timings_t msg_timings;
    for ( int i = 0; i < 4; i++ ) {
      msg_timings.msg_dt[i] = (uint16_t)ircomm.msg_dt[i];
      msg_timings.msg_t[i] = (uint16_t)ircomm.msg_t[i];
    }
    msg_timings.rx_timeout = (uint16_t)ircomm.ir_config.rx_timeout;
    msg_timings.tx_period = (uint16_t)ircomm.ir_config.tx_period;
    Wire.write( (byte*)&msg_timings, sizeof( msg_timings ) );

  }  else if ( last_mode.mode == MODE_SIZE_MSG0 ) {

    i2c_mode_t msg_status;
    msg_status.mode = strlen( ircomm.i2c_msg[0] );
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG1 ) {
    i2c_mode_t msg_status;
    msg_status.mode = strlen( ircomm.i2c_msg[1] );
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG2 ) {
    i2c_mode_t msg_status;
    msg_status.mode = strlen( ircomm.i2c_msg[2] );
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG3 ) {
    i2c_mode_t msg_status;
    msg_status.mode = strlen( ircomm.i2c_msg[3] );
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


    //
    // Below are requests for actual message to be
    // reported to the master device
    //
  } else if ( last_mode.mode == MODE_REPORT_MSG0 ) {
    if ( ircomm.i2c_msg[0] == 0 ) {
      Wire.write("!");  // Error token
    } else {
      Wire.write( ircomm.i2c_msg[0], strlen(ircomm.i2c_msg[0]) );
      // Delete message
      ircomm.clearRxMsg( 0 );
    }

  } else if ( last_mode.mode == MODE_REPORT_MSG1 ) {
    if ( ircomm.i2c_msg[1] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.i2c_msg[1], strlen(ircomm.i2c_msg[1]) );
      // Delete message
      ircomm.clearRxMsg( 1 );
    }

  } else if ( last_mode.mode == MODE_REPORT_MSG2 ) {
    if ( ircomm.i2c_msg[2] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.i2c_msg[2], strlen(ircomm.i2c_msg[2]) );

      // Delete message
      ircomm.clearRxMsg( 2 );
    }


  } else if ( last_mode.mode == MODE_REPORT_MSG3 ) {
    if ( ircomm.i2c_msg[3] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.i2c_msg[3], strlen(ircomm.i2c_msg[3]) );

      // Delete message
      ircomm.clearRxMsg( 3 );
    }
  } else if ( last_mode.mode == MODE_REPORT_CYCLES ) {
    i2c_cycles_t cycles;
    cycles.tx_count = ircomm.tx_count;
    cycles.rx_cycles = ircomm.rx_cycles;

    // Transmit
    Wire.write( (byte*)&cycles, sizeof( cycles ) );

  } else if ( last_mode.mode == MODE_REPORT_ERRORS ) {
    i2c_errors_t errors;

    for ( int i = 0; i < 4; i++ ) {
      for ( int j = 0; j < 4; j++ ) {
        errors.error_type[i][j] = ircomm.error_type[i][j];
      }
    }

    // Transmit
    Wire.write( (byte*)&errors, sizeof( errors ) );

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

  } else if ( last_mode.mode == MODE_REPORT_HIST ) {

    i2c_id_hist_t hist;
    hist.id[0] = ircomm.hist[0];
    hist.id[1] = ircomm.hist[1];
    hist.id[2] = ircomm.hist[2];
    hist.id[3] = ircomm.hist[3];
    Wire.write( (byte*)&hist, sizeof( hist ) );

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
  Wire.begin( IRCOMM_I2C_ADDR );
  Wire.onReceive( i2c_receive );
  Wire.onRequest( i2c_request );

  // Start the IR communication board.
  ircomm.init();

  last_mode.mode = MODE_REPORT_STATUS;

  for ( int i = 0; i < 4; i++ ) {
    status.pass_count[i] = 0;
    status.fail_count[i] = 0;
  }

  test_ts = millis();

// Paul: I was using this to test
//  setRandomMsg(8);
}


// Construct a message of length len out
// of random ascii characters, avoiding
// * and @
void setRandomMsg(int len) {
  // Let's test variable message lengths
  int max_chars = len;
  char buf[ MAX_MSG ];

  memset( buf, 0, sizeof( buf ) );

  //sprintf( buf, "%lu", millis() );
  //int ms_len = strlen( buf );
  //buf[ms_len] = ':';
  //ms_len++;
  for ( int i = 0; i < max_chars; i++ ) {
    int r = random( 0, 62 );
    buf[i] = (byte)(65 + r); // 65+ avoids * and @
  }
  ircomm.formatString(buf, strlen(buf) );

}

void loop() {
//
//  if ( millis() - test_ts > 100 ) {
//    test_ts = millis();
//    setRandomMsg(8);
//  }


  // This line must be called to process new
  // received messages and transmit new messages
  ircomm.update();

}

// Enabled/disabled with the #define TEST_ at
// the top of the code.
void testFunctions() {
  // Periodic update of other board status
  if ( millis() - test_ts > TEST_MS ) {
    test_ts = millis();

    if ( SELF_TEST_MODE == TEST_TX ) {
      // Create a test string up to 29 chars long
      setRandomMsg(8);



    } else if ( SELF_TEST_MODE == TEST_RX ) {

      ircomm.disableRx();

      // What is rx delay being set to?
      //      Serial.print( ircomm.rx_delay );
      //      Serial.print(",");
      //      for ( int i = 0; i < 4; i++ ) {
      //        Serial.print( i );
      //        Serial.print(":");
      //        Serial.print( ircomm.msg_dt[i] );
      //        Serial.print(",");
      //        Serial.print( ircomm.msg_t[i] );
      //        Serial.print(",");
      //        Serial.print( ircomm.pass_count[i] );
      //        Serial.print(",");
      //        Serial.print( ircomm.fail_count[i] );
      //        Serial.println();
      //      }
      //
      //      for ( int i = 0; i < 4; i++ ) {
      //        Serial.print( ircomm.pass_count[i] );
      //        Serial.print(",");
      //      }
      //      for ( int i = 0; i < 4; i++ ) {
      //        Serial.print( 0 - (int)ircomm.fail_count[i] );
      //        Serial.print(",");
      //      }
      //      for ( int i = 0; i < 4; i++ ) {
      //        Serial.print( ircomm.error_type[i] );
      //        Serial.print(",");
      //      }
      //      Serial.println();
      // what type of errors on receive are we getting?
      //      for ( int i = 0; i < 4; i++ ) {
      //        Serial.print( ircomm.error_type[i] );
      //        Serial.print(",");
      //      }
      //      for( int i = 0; i < 4; i++ ) {
      //        Serial.print( ircomm.rx_buf[i] );
      //        Serial.print(", ");
      //      }
      //      Serial.println();
      //      for( int i = 0; i < 4; i++ ) {
      //        Serial.print( ircomm.rx_vectors[i] );
      //        Serial.print(", ");
      //      }
      //      Serial.println();

      //      float x = (ircomm.rx_vectors[0] - ircomm.rx_vectors[2]);
      //    float y = (ircomm.rx_vectors[1] - ircomm.rx_vectors[3]);
      //    float theta = atan2( y, x );
      //    Serial.println( theta,4 );


      //ircomm.reportConfiguration();

      Serial.flush();
      ircomm.enableRx();
    }


  }

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
