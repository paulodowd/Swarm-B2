/*
 * 
 * Currently getting 2.93ms per byte for IR transmission.
 * I2C limits the string to max 32 bytes.  However, using
 * 4 bytes as start, token, checkbyte and terminal.
 * Serial UART can buffer 64bytes.
 */


#include <avr/io.h>
#include <Wire.h>
#include "ircomm.h"
#include "ircomm_data.h"

#define I2C_ADDR  8

#define TEST_DISABLED 0
#define TEST_TX 1
#define TEST_RX 2
#define SELF_TEST_MODE TEST_DISABLED
//#define SELF_TEST_MODE TEST_TX
//#define SELF_TEST_MODE TEST_RX

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
#define CYCLE_MS 250    // 250ms per receiver?

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
    if ( new_mode.mode < MAX_MODE && new_mode.mode > 0 ) {
      last_mode.mode = new_mode.mode;
    }

    // Special command
    if ( new_mode.mode == MODE_STOP_TX ) {
      ircomm.clearTxBuf();
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

  }
}




void setup() {



  // Set random seed.
  initRandomSeed();

  // Pinmodes
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

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

  if ( SELF_TEST_MODE == TEST_TX ) {
    char buf[20];
    sprintf( buf, "Testing:%lu", millis() );
    ircomm.formatString(buf, strlen(buf) );
  }

}




void loop() {


  // Periodic update of other board status
  if ( millis() - status_ts > STATUS_MS ) {
    status_ts = millis();

    // Capture LDR light values
    status.ldr[0] = analogRead( A0 );
    status.ldr[1] = analogRead( A1 );
    status.ldr[2] = analogRead( A2 );

    // use mode variable to report back which
    // receiver is currently active.
    status.mode = (uint8_t)ircomm.rx_pwr_index;
    //Serial.println("Status Update");

    if ( SELF_TEST_MODE == TEST_TX ) {
      char buf[20];
      sprintf( buf, "Testing:%lu", millis() );
      ircomm.formatString(buf, strlen(buf) );

    } else if ( SELF_TEST_MODE == TEST_RX ) {

//      Looking at actual strings received.
//      for ( int i = 0; i < RX_PWR_MAX; i++ ) {
//        Serial.print( ircomm.rx_msg[i] );
//
//
//        Serial.print("\n");
//      }
//      Serial.print("\n");

//      Looking at the time between receiving messages
      Serial.print("Rx Times: ");
      for ( int i = 0; i < RX_PWR_MAX; i++ ) {
        Serial.print( ircomm.msg_dt[i] );
        Serial.print(", ");
      }
      Serial.print("\n");

      //      Looking at how many received without error (pass)
      //      //Serial.print("Pass count: ");
      //      for ( int i = 0; i < RX_PWR_MAX; i++ ) {
      //        Serial.print( ircomm.pass_count[i] );
      //        Serial.print(", ");
      //      }
      //      Serial.print("\n");
      //
      //      Looking at how many received with error (fail)
      //      Serial.print("Fail count: ");
      //      for ( int i = 0; i < RX_PWR_MAX; i++ ) {
      //        Serial.print( ircomm.fail_count[i] );
      //        Serial.print(", ");
      //      }
      //      Serial.print("\n");
      //
      //
      //      Serial.print("Rx Ratios: ");
      //      for ( int i = 0; i < RX_PWR_MAX; i++ ) {
      //        float pass = (float)ircomm.pass_count[i];
      //        float fail = (float)ircomm.fail_count[i];
      //        if ( fail > 0 && pass > 0 ) {
      //          Serial.print( (fail / pass) );
      //        } else {
      //          Serial.print( "0.0" );
      //        }
      //        Serial.print(", ");
      //      }
      //      Serial.print("\n");
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

  for ( int i = 0; i < 4; i++ ) status.msg_count[ i ] = ircomm.pass_count[i];

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
