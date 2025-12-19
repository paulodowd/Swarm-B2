#include <NeoHWSerial.h>
#include <NeoHWSerial_private.h>

/*



*/


#include <avr/io.h>
#include <Wire.h>
#include "ircomm.h"
#include "ircomm_i2c.h"

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
volatile ir_mode_t last_mode;

volatile boolean full_reset;

// Function to receive commands from the master device.
// This will typically be in the format of a mode change
// and then any appropriate data.
void i2c_receive( int len ) {


  // Mode changes are always 1 byte long.  We use these to
  // ready the board to send back information or to ask the
  // board to complete specific actions (e.g, delete a
  // message).
  if ( len == 1 && last_mode.mode != MODE_SET_MSG ) { // receiving a mode change.
    ir_mode_t new_mode;
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
      full_reset = true;
    } else if ( new_mode.mode == MODE_STOP_RX ) {
      ircomm.disabled = true;

    } else if ( new_mode.mode == MODE_START_RX ) {

      ircomm.disabled = false;

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
      //      ircomm.hist[0] = 0;
      //      ircomm.hist[1] = 0;
      //      ircomm.hist[2] = 0;
      //      ircomm.hist[3] = 0;
    } else if ( new_mode.mode == MODE_SET_MSG ) {
      last_mode.mode = MODE_SET_MSG;
    }

  } else if ( last_mode.mode == MODE_SET_MSG ) {

    // This should be impossible...
    if ( len > MAX_MSG || len < 1 ) {
      //ircomm.clearTxBuf();
      // clear i2c buffer
      while ( Wire.available() ) Wire.read();
    } else {

      // Transfer bytes, encode with parser
      byte buf[MAX_MSG];
      memset( buf, 0, sizeof( buf ));
      Wire.readBytes( buf, len );
      ircomm.config.tx.len = ircomm.parser.formatIRMessage( ircomm.tx_buf, buf, len );
    }

    last_mode.mode = MODE_NOT_SET;

  } else if ( last_mode.mode == MODE_SET_RX ) {

    if ( len == sizeof( ir_rx_params_t ) ) {

      Wire.readBytes( (uint8_t*)&ircomm.config.rx, sizeof( ircomm.config.rx ));

      // User may have switched Rx index!
      // This also resets the parser & UART
      ircomm.powerOnRx( ircomm.config.rx.index );

    } else {

      // Something has gone wrong. Just
      // read in the bytes to clear the
      // data
      while ( Wire.available() ) Wire.read();
    }
    last_mode.mode = MODE_NOT_SET;

  } else if ( last_mode.mode == MODE_SET_TX ) {
    if ( len == sizeof( ir_tx_params_t ) ) {
      Wire.readBytes( (uint8_t*)&ircomm.config.tx, sizeof( ircomm.config.tx ));
    } else {

      // Something has gone wrong. Just
      // read in the bytes to clear the
      // data
      while ( Wire.available() ) Wire.read();
    }

    last_mode.mode = MODE_NOT_SET;

  } else {

    // Unhandled
  }

}

// When the 3Pi or Core2 calls an i2c request, this function
// is executed.
void i2c_request() {

if( last_mode.mode == MODE_REPORT_CRC ) {

    Wire.write( (byte*)&ircomm.metrics.crc, sizeof(ircomm.metrics.crc) );

  } else if( last_mode.mode == MODE_REPORT_ACTIVITY ) {

    Wire.write( (byte*)&ircomm.metrics.activity, sizeof(ircomm.metrics.activity) );

  } else if( last_mode.mode == MODE_REPORT_SATURATION ) {

    Wire.write( (byte*)&ircomm.metrics.saturation, sizeof(ircomm.metrics.saturation) );

  } else if( last_mode.mode == MODE_REPORT_SKIPS ) {

    Wire.write( (byte*)&ircomm.metrics.skips, sizeof(ircomm.metrics.skips) );

  } else if ( last_mode.mode == MODE_REPORT_MSG_TIMINGS ) {

    Wire.write( (byte*)&ircomm.metrics.msg_timings, sizeof( ircomm.metrics.msg_timings ) );

  }  else if ( last_mode.mode == MODE_REPORT_BYTE_TIMINGS ) {

    Wire.write( (byte*)&ircomm.metrics.byte_timings, sizeof( ircomm.metrics.byte_timings ) );

  } else if ( last_mode.mode == MODE_REPORT_FRAME_ERRS ) {
    
    Wire.write( (byte*)&ircomm.metrics.frame_errors, sizeof( ircomm.metrics.frame_errors ) );

  }  else if ( last_mode.mode == MODE_SIZE_MSG0 ) {

    ir_mode_t msg_status;
    msg_status.mode = ircomm.msg_len[0];
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG1 ) {
    ir_mode_t msg_status;
    msg_status.mode = ircomm.msg_len[1];
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG2 ) {
    ir_mode_t msg_status;
    msg_status.mode = ircomm.msg_len[2];
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );


  } else if ( last_mode.mode == MODE_SIZE_MSG3 ) {
    ir_mode_t msg_status;
    msg_status.mode = ircomm.msg_len[3];
    Wire.write( (byte*)&msg_status, sizeof( msg_status ) );

  } else if ( last_mode.mode == MODE_REPORT_MSG0 ) {
    if ( ircomm.msg_len[0] == 0 ) {
      Wire.write("!");  // Error token
    } else {
      Wire.write( ircomm.ir_msg[0], ircomm.msg_len[0] );
      // Delete message
      ircomm.clearRxMsg( 0 );
    }

  } else if ( last_mode.mode == MODE_REPORT_MSG1 ) {
    if ( ircomm.msg_len[1] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.ir_msg[1], ircomm.msg_len[1] );
      // Delete message
      ircomm.clearRxMsg( 1 );
    }

  } else if ( last_mode.mode == MODE_REPORT_MSG2 ) {
    if ( ircomm.msg_len[2] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.ir_msg[2], ircomm.msg_len[2] );

      // Delete message
      ircomm.clearRxMsg( 2 );
    }


  } else if ( last_mode.mode == MODE_REPORT_MSG3 ) {
    if ( ircomm.msg_len[3] == 0 ) {
      Wire.write("!");
    } else {
      Wire.write( ircomm.ir_msg[3], ircomm.msg_len[3] );

      // Delete message
      ircomm.clearRxMsg( 3 );
    }

  } else if ( last_mode.mode == MODE_REPORT_CYCLES ) {

    // Transmit
    Wire.write( (byte*)&ircomm.metrics.cycles, sizeof( ircomm.metrics.cycles ) );

  } else if ( last_mode.mode == MODE_REPORT_ERRORS ) {

    // Transmit
    Wire.write( (byte*)&ircomm.metrics.errors, sizeof( ircomm.metrics.errors ) );

  } else if ( last_mode.mode == MODE_REPORT_RX_VECTORS ) {

    // Transmit
    Wire.write( (byte*)&ircomm.metrics.vectors, sizeof( ircomm.metrics.vectors) );

  } else if (  last_mode.mode == MODE_REPORT_RX_BEARING ) {

    // Transmit
    Wire.write( (byte*)&ircomm.metrics.bearing, sizeof( ircomm.metrics.bearing ) );

  } else if ( last_mode.mode == MODE_REPORT_SENSORS ) {

    // Collect sensor readings - should be very quick
    ircomm.metrics.sensors.ldr[0] = (int16_t)analogRead( LDR0_PIN );
    ircomm.metrics.sensors.ldr[1] = (int16_t)analogRead( LDR1_PIN );
    ircomm.metrics.sensors.ldr[2] = (int16_t)analogRead( LDR2_PIN );
    ircomm.metrics.sensors.prox[0] = (int16_t)analogRead( PROX0_PIN );
    ircomm.metrics.sensors.prox[1] = (int16_t)analogRead( PROX1_PIN );

    // Transmit
    Wire.write( (byte*)&ircomm.metrics.sensors, sizeof( ircomm.metrics.sensors ) );

  } else if ( last_mode.mode == MODE_REPORT_HIST ) {

    Wire.write( (byte*)&ircomm.metrics.hist, sizeof( ircomm.metrics.hist ) );

  } else if ( last_mode.mode == MODE_GET_TX ) {

    Wire.write( (byte*)&ircomm.config.tx, sizeof( ircomm.config.tx ) );


  } else if ( last_mode.mode == MODE_GET_RX ) {

    Wire.write( (byte*)&ircomm.config.rx, sizeof( ircomm.config.rx ) );

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

  NeoSerial.println("Nano");

  last_mode.mode = MODE_NOT_SET;

  full_reset = false;
  // Paul: I was using this to test
//  setRandomMsg(8);
}




// Construct a message of length len out
// of random ascii characters, avoiding
// * and @
int setRandomMsg(int len) {
  // Let's test variable message lengths
  int max_chars = len;
  byte buf[ MAX_MSG ];

  memset( buf, 0, sizeof( buf ) );

  //sprintf( buf, "%lu", millis() );
  //int ms_len = strlen( buf );
  //buf[ms_len] = ':';
  //ms_len++;

//  sprintf(buf, "123456789~123456789~123456789~^^");
  //  sprintf(buf, "123456789");
  for ( int i = 0; i < len; i++ ) {
    buf[i] = (byte)random( 0, 256 );
  }



  //  typedef struct msg {
  //    float v[2];
  //  } msg_t;
  //  msg_t msg;
  //  for ( int i = 0; i < 2; i++ ) msg.v[i] = random(0, 1000) - 500;
  //  ircomm.tx_len = ircomm.parser.formatIRMessage( ircomm.tx_buf, (byte*)&msg, sizeof( msg ) );


  //  Checking the format of what is being sent.
  ircomm.config.tx.len = ircomm.parser.formatIRMessage( ircomm.tx_buf, buf, len );
  //  while(1) {
  //    NeoSerial.print("tx buf: ");
  //    NeoSerial.println( (char*)ircomm.tx_buf );
  //    for( int i = 0; i < ircomm.tx_len; i++ ) {
  //      NeoSerial.print( (char)ircomm.tx_buf[i] );
  //      NeoSerial.print( " = " );
  //      NeoSerial.print( ircomm.tx_buf[i], DEC);
  //      NeoSerial.print(",");
  //    }
  //    NeoSerial.println();
  //    delay(1000);
  //
  //  }

  //  while (true) {
  //    ircomm.printMsgForDebugging();
  //
  //    delay(500);
  //  }

  return ircomm.config.tx.len;
}

void loop() {
//
//  ircomm.doTransmit();
//  return;
//  ////
//    if( millis() - test_ts > 250 ) {
//        test_ts = millis();
//        int e = setRandomMsg(8);
//      }
  //
  //
  //  sendTest();
  //  return;

  if ( full_reset ) {
    ircomm.fullReset();
    full_reset = false;
  }


  // This line must be called to process new
  // received messages and transmit new messages
  ircomm.update();
//  //
//  if ( millis() - test_ts > 1000 ) {
//    test_ts = millis();
//    ircomm.printRxMsgForDebugging();
//  }

  

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
    //NeoSerial.println(b, BIN);

    r |= (b << i);
    delayMicroseconds(10);
  }
  //NeoSerial.println(r, BIN);
  randomSeed( r );
}
