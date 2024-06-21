#include <Wire.h>           // i2c to connect to IR communication board.
#define IRCOMM_I2C_ADDR  8

#include "ircomm_data.h"

// A data structure to commmunicate
// the mode.
// Keep consistent between devices.
i2c_mode_t ircomm_mode;

// A data structure to receive back
// the status of the board.
// Keep consistent between devices.
i2c_status_t ircomm_status;


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
  if ( millis() - status_ts > 500 ) {
    status_ts = millis();

    unsigned long start = micros();

    ircomm_mode.mode = MODE_REPORT_DIRECTION;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

    i2c_bearing_t bearing;
    Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( bearing ));
    Wire.readBytes( (uint8_t*)&bearing, sizeof( bearing ));

    unsigned long end = micros();
//    Serial.print( (end-start) );
//    Serial.print(",");

    Serial.print(bearing.theta, 4);
    Serial.print(",");
    Serial.print( bearing.mag, 4 );
    Serial.println();

    //        ircomm_mode.mode = MODE_REPORT_ACTIVITY;
    //        Wire.beginTransmission( IRCOMM_I2C_ADDR );
    //        Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    //        Wire.endTransmission();
    //
    //        i2c_activity_t activity;
    //        Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( activity ));
    //        Wire.readBytes( (uint8_t*)&activity, sizeof( activity ));
    //
    //        for ( int i = 0; i < 4; i++ ) {
    //          Serial.print( activity.rx[i] * 10, 7 );
    //          Serial.print(",");
    //
    //        }
    //        float x = activity.rx[0] - activity.rx[2];
    //        float y = activity.rx[1] - activity.rx[3];
    //
    //        Serial.println( atan2( y , x ), 4);

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
    ////    Serial.print("Msg Pass:\t");
    //    for ( int i = 0; i < 4; i++ ) {
    //      Serial.print( ircomm_status.pass_count[i] );
    //      Serial.print("\t");
    //    }
    ////    Serial.println();
    ////    Serial.print("Msg Fail:\t");
    //    for ( int i = 0; i < 4; i++ ) {
    //      float m = (float)ircomm_status.fail_count[i];
    //
    //      Serial.print( -m );
    //      Serial.print("\t");
    //    }
    //    //Serial.println();
    //
    //    Serial.println( ircomm_status.msg_dir );

    // Test for getting data from extra sensors.
    //getSensors();

  }

  // Update the message being transmitted every 5000ms (5s)
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
  //
  //
  //  // Check a receiver every 500ms
  //  if ( millis() - recv_msg_ts > 500 ) {
  //    recv_msg_ts = millis();
  //
  //    for ( int i = 0; i < 4; i++ ) {
  //      // Any messages arrived ?
  //      getIRMessage( i );
  //    }
  //  }



  delay(10);

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

void getSensors() {
  // Set mode to read fetch sensor data
  ircomm_mode.mode = MODE_REPORT_SENSORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  // struct to store sensor data
  i2c_sensors_t sensors;

  // Read across bytes
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( sensors ));
  Wire.readBytes( (uint8_t*)&sensors, sizeof( sensors ));

  Serial.println("Sensors:");
  Serial.print(" - LDR: ");
  for (int i = 0; i < 3; i++ ) {
    Serial.print( sensors.ldr[i] );
    Serial.print(",");
  }
  Serial.print("\n - Prox:");
  for (int i = 0; i < 2; i++ ) {
    Serial.print( sensors.prox[i] );
    Serial.print(",");
  }
  Serial.println();

}

// Get the latest message from the communication board
// from receiver "which_rx" (0,1,2,3).
// This is a little bit more complicated because we don't
// know how long a message will be. So we have to first
// ask how many bytes are available (present).
void getIRMessage(int which_rx ) {
  if ( which_rx < 0 || which_rx >= 4 ) {
    // Invalid
    return;
  }


  // Select right mode to ask how many bytes are
  // available.
  if ( which_rx < 0 || which_rx > 3 ) {
    // catch error first.
    return;

  } else if ( which_rx == 0 ) {
    ircomm_mode.mode = MODE_STATUS_MSG0;
  } else if ( which_rx == 1 ) {
    ircomm_mode.mode = MODE_STATUS_MSG1;
  } else if ( which_rx == 2 ) {
    ircomm_mode.mode = MODE_STATUS_MSG2;
  } else if ( which_rx == 3 ) {
    ircomm_mode.mode = MODE_STATUS_MSG3;
  }

  // Set mode to read back how many bytes are
  // available.
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  // We'll use this struct to check how many
  // bytes are available of a message.
  // 0 bytes means no message.
  i2c_mode msg_status;

  // Request the message size to be sent across into
  // msg_status
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_status ));
  Wire.readBytes( (uint8_t*)&msg_status, sizeof( msg_status ));

  // If data is available, we change mode to read it
  // across.
  if ( msg_status.mode <= 0 || msg_status.mode > 32 ) {
    // catch error first
    // message size shouldn't be -ve, 0 or more than
    // 32 bytes.
    return;

  } else { //bytes available > 0

    // Format mode request for which receiver
    if ( which_rx == 0 ) {
      ircomm_mode.mode = MODE_REPORT_MSG0;
    } else if ( which_rx == 1 ) {
      ircomm_mode.mode = MODE_REPORT_MSG1;
    }  else if ( which_rx == 2 ) {
      ircomm_mode.mode = MODE_REPORT_MSG2;
    }  else if ( which_rx == 3 ) {
      ircomm_mode.mode = MODE_REPORT_MSG3;
    } else {
      // error should have been caught much earlier.
    }

    // Set mode to send across a full message
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

    // char array to store message into
    char buf[32];
    int count = 0;
    Wire.requestFrom( IRCOMM_I2C_ADDR, msg_status.mode );
    while ( Wire.available() && count < 32 ) {
      char c = Wire.read();
      buf[count] = c;
      count++;
    }

    // Mainly debugging here.
    // Need to decide what to do with the char array
    // once a message has been sent across.
    if ( count > 0 ) {
      Serial.print("Rx" );
      Serial.print( which_rx );
      Serial.print(":\t");
      for ( int i = 0; i < count; i++ ) {
        Serial.print( buf[i]  );
      }
      Serial.println();
      //Serial.println( buf );
    }
  }
}
