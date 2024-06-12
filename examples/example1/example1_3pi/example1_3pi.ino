#include <Wire.h>           // i2c to connect to IR communication board.
#define IRCOMM_I2C_ADDR  8

#pragma pack(1)

// Data structure used to get general
// status from the IR communication
// board (the arduino Nano)
// Currently using 23 bytes (max 32)
typedef struct ircomm_status {
  uint8_t mode;                       // 1  bytes
  uint8_t ldr[3];                     // 6  bytes
  unsigned long msg_count[4];         // 16 bytes
} ircomm_status_t;

typedef struct i2c_mode {
  uint8_t mode;
} i2c_mode_t;


// These "modes" are sent to the communication
// board over i2c and determine what data is
// then read back.  You need to make sure these
// modes are defined the same on both devices.
#define MODE_REPORT_STATUS  0
#define MODE_STOP_TX        1
#define MODE_REPORT_LDR0    2
#define MODE_REPORT_LDR1    3
#define MODE_REPORT_LDR2    4
#define MODE_STATUS_MSG0    5
#define MODE_STATUS_MSG0    6
#define MODE_STATUS_MSG1    7
#define MODE_STATUS_MSG2    8
#define MODE_STATUS_MSG3    9
#define MODE_REPORT_MSG0    10
#define MODE_REPORT_MSG1    11
#define MODE_REPORT_MSG2    12
#define MODE_REPORT_MSG3    13
#define MAX_MODE            14

// A data structure to commmunicate
// the mode.
// Keep consistent between devices.
i2c_mode_t ircomm_mode;

// A data structure to receive back
// the status of the board.
// Keep consistent between devices.
ircomm_status_t ircomm_status;


// Timestamps to periodically
// check for either the status of
// the communication board, to
// set a new message to send, or
// to get the latest messages
// received.
unsigned long status_ts;
unsigned long new_msg_ts;
unsigned long recv_msg_ts;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  status_ts = millis();
  new_msg_ts = millis();
  recv_msg_ts = millis();
}


void loop() {

  // Periodically check status
  //  if ( millis() - status_ts > 1000 ) {
  //    status_ts = millis();
  //
  //    // Set mode to status request
  //    ircomm_mode.mode = MODE_REPORT_STATUS;
  //    Wire.beginTransmission( IRCOMM_I2C_ADDR );
  //    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  //    Wire.endTransmission();
  //
  //    Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ircomm_status ));
  //    Wire.readBytes( (uint8_t*)&ircomm_status, sizeof( ircomm_status ));
  //
  //
  //    // Report how many messages have been received on each
  //    // receiver
  //    for ( int i = 0; i < 4; i++ ) {
  //      Serial.print( ircomm_status.msg_count[i] );
  //      Serial.print(",");
  //    }
  //    Serial.println();
  //
  //  }

  // Send a message every 5000ms (5s)
  //  if ( millis() - new_msg_ts > 5000) {
  //
  //    new_msg_ts = millis();
  //
  //
  //    // Let's just send the current time from millis()
  //    // as an example.
  //
  //    char buf[32];
  //    float f_to_send = (float)millis();
  //    f_to_send /= 1000.0;
  //
  //    // Convert float to a string, store in the
  //    // message buffer.
  //    // I had a lot of trouble finding a solution for this.
  //    // This is an odd, non-standard function I think.
  //    // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
  //    // https://www.programmingelectronics.com/dtostrf/
  //    //  - a minimum of 6 character (e.g. 000.00)
  //    //  - 2 digits after decimal
  //    //  - store in buf
  //    dtostrf(f_to_send, 6, 2, buf);
  //
  //    // This function call tells the communication board
  //    // (the nano) to start ending the requested message.
  //    // It will keep doing this until a new call to this
  //    // function is made.
  //    setIRMessage(buf, strlen(buf));
  //  }

  if ( millis() - recv_msg_ts > 500 ) {
    recv_msg_ts = millis();

    // Any messages arrived on receiver 0?
    getIRMessage( 0 );

  }



  delay(100);

}



// Use this function to set a message to transmit to
// other robots.
void setIRMessage(char* str_to_send, int len) {

  // Message must be shorter than 32 bytes
  // and at least 1 byte.
  if ( len <= 32 && len >= 1 ) {


    // The communication board will always default
    // to waiting to receive a message to transmit
    // so we don't need to change the mode.
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)str_to_send, len);
    Wire.endTransmission();
  }
}

// Get the latest message from the communication board
// from receiver "which_rx" (0,1,2,3).
void getIRMessage(int which_rx ) {
  if ( which_rx < 0 || which_rx >= 4 ) {
    // Invalid
    return;
  }

  if ( which_rx == 0 ) {

    // First, ask how many bytes are available.
    // 0 = no message.
    i2c_mode msg_status;

    ircomm_mode.mode = MODE_STATUS_MSG0;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

    Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_status ));
    Wire.readBytes( (uint8_t*)&msg_status, sizeof( msg_status ));

    // If data is available, we change mode to read it
    // across.
    if ( msg_status.mode > 0 ) {

      // Set mode.
      ircomm_mode.mode = MODE_REPORT_MSG0;
      Wire.beginTransmission( IRCOMM_I2C_ADDR );
      Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
      Wire.endTransmission();

      char buf[32];
      int count = 0;
      Wire.requestFrom( IRCOMM_I2C_ADDR, msg_status.mode );
      while ( Wire.available() && count < 32 ) {
        char c = Wire.read();
        buf[count] = c;
        count++;
      }

      if ( count > 0 ) {
        Serial.print("Received " );
        Serial.print( count );
        Serial.print(" bytes on ");
        Serial.print( which_rx );
        Serial.print(": ");
        for ( int i = 0; i < count; i++ ) {
          Serial.print( buf[i]  );
        }
        Serial.println();
        //Serial.println( buf );
      }
    }




    // Depending on which receiver we want to get a
    // message from, we need to first write the
    // mode change to the board.
    // Set correct mode:
    //  if ( which_rx == 0 ) {
    //
    //    // Set the IRComm board to the right mode
    //    ircomm_mode.mode = MODE_REPORT_MSG0;
    //    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    //    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    //    Wire.endTransmission();
    //
    //  } else if ( which_rx == 1 ) {
    //
    //    // Set the IRComm board to the right mode
    //    ircomm_mode.mode = MODE_REPORT_MSG1;
    //    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    //    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    //    Wire.endTransmission();
    //  } else if ( which_rx == 2 ) {
    //
    //    // Set the IRComm board to the right mode
    //    ircomm_mode.mode = MODE_REPORT_MSG2;
    //    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    //    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    //    Wire.endTransmission();
    //  } else if ( which_rx == 3 ) {
    //
    //    // Set the IRComm board to the right mode
    //    ircomm_mode.mode = MODE_REPORT_MSG3;
    //    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    //    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    //    Wire.endTransmission();
    //  }
    //
    //  // Prepare to receive.
    //  // Note: we assume we will get back 32 bytes, even if the
    //  // actual message will be shorter.  The communication board
    //  // will terminate communication if less than 32.
    //  char buf[32];
    //  int count = 0;
    //  int recv = Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( buf ));
    //  Serial.print("Reported " );
    //  Serial.println( recv );
    //  if ( recv > 0 ) {
    //    while ( Wire.available() && count < 32 ) {
    //      char c = Wire.read();
    //      if( count == 0 && c == 0 ) break;
    //      if ( c == '!' ) break; // end of message
    //      buf[count] = c;
    //      count++;
    //    }
    //  } else {
    //    Serial.println("Nothing to receive");
    //  }
    //
    //
    //  // What should we do with the message just received?
    //  // Did we get a valid number of bytes?
    //  if ( count > 0 ) {
    //    Serial.print("Received " );
    //    Serial.print( count );
    //    Serial.print(" bytes on ");
    //    Serial.print( which_rx );
    //    Serial.print(": ");
    //    for ( int i = 0; i < count; i++ ) {
    //      Serial.print( buf[i], BIN );
    //      Serial.print(" ");
    //    }
    //    Serial.println();
    //  }

  }
}
