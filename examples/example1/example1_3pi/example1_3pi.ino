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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  status_ts = millis();
  new_msg_ts = millis();

  pinMode(6, OUTPUT);


  

}


void loop() {

  // Periodically check status
  if ( millis() - status_ts > 50 ) {
    status_ts = millis();

    bool received = false;

    // Check all 4 receivers
    for ( int i = 0; i < 4; i++ ) {

      int n = checkRxMsgReady( i );

      if ( n > 0 ) {
        valid = true;
    
        getIRMessage( i, n );

      } 
    }

    // We received at least one message, make a beep
    if ( received ) {

      analogWrite(6, 120);
      delay(5);
      analogWrite(6, 0 );


    }

    // So we can plot the count of messages
    reportStatusCSV();
  }






  delay(10);

}


void getMsgTimings() {
  ircomm_mode.mode = MODE_REPORT_TIMINGS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  i2c_msg_timings_t msg_timings;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_timings ));
  Wire.readBytes( (uint8_t*)&msg_timings, sizeof( msg_timings ));

  Serial.println("ms between messages:");
  for ( int i = 0; i < 4; i++ ) {
    Serial.print("- Rx ");
    Serial.print( i );
    Serial.print(": ");
    Serial.print( msg_timings.msg_dt[i] );
    Serial.println();
  }
  Serial.println("Last rx time in ms:");
  for ( int i = 0; i < 4; i++ ) {
    Serial.print("- Rx ");
    Serial.print( i );
    Serial.print(": ");
    Serial.print( msg_timings.msg_t[i] );
    Serial.println();
  }

  Serial.print("Tx Delay: "); Serial.println( msg_timings.tx_delay );
  Serial.print("Rx Delay: "); Serial.println( msg_timings.rx_delay );

}

// Reports a range and bearing estimate
// of messsages received.  Note that sometimes
// this might not be sensible, such as if there
// is a robot on opposite sides of this robot,
// giving a large magnitude but odd angle.
// Also note that a lack of messages could give
// a sensible angle but a very small magnitude.
void getRxDirection() {

  ircomm_mode.mode = MODE_REPORT_RX_DIRECTION;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  i2c_bearing_t bearing;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( bearing ));
  Wire.readBytes( (uint8_t*)&bearing, sizeof( bearing ));

  Serial.print(bearing.theta, 4);
  Serial.print(",");
  Serial.print( bearing.mag, 4 );
  Serial.println();
}

// Reports the activity level of each receiver, which
// will vary continously over time between 0 and 1
void getRxActivity() {
  ircomm_mode.mode = MODE_REPORT_RX_ACTIVITY;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  i2c_activity_t activity;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( activity ));
  Wire.readBytes( (uint8_t*)&activity, sizeof( activity ));

  for ( int i = 0; i < 4; i++ ) {
    Serial.print( activity.rx[i], 4 );
    Serial.print(",");

  }
  Serial.println();
}


// Use to see if there is a message available on
// receivers 0 to 3 (which_rx).
// If returns 0, none available.
int checkRxMsgReady(int which_rx) {



  if ( which_rx == 0 ) {
    ircomm_mode.mode = MODE_SIZE_MSG0;
  } else if ( which_rx == 1 ) {
    ircomm_mode.mode = MODE_SIZE_MSG1;
  } else if ( which_rx == 2 ) {
    ircomm_mode.mode = MODE_SIZE_MSG2;
  } else if ( which_rx == 3 ) {
    ircomm_mode.mode = MODE_SIZE_MSG3;
  }

  // Set mode to read back how many bytes are
  // available.
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  // We'll use this struct to check how many
  // bytes are available of a message.
  // 0 bytes means no message.
  i2c_msg_status_t msg_status;

  // Request the message size to be sent across into
  // msg_status
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_status ));
  Wire.readBytes( (uint8_t*)&msg_status, sizeof( msg_status ));

  return msg_status.n_bytes;// 0 : 29

}


void testTransmit() {
  //   Update the message being transmitted every 5000ms (5s)
  if ( millis() - new_msg_ts > 5000) {

    new_msg_ts = millis();


    // Let's just send the current time from millis()
    // as an example.

    char buf[32];
    float f_to_send = (float)millis();
    f_to_send /= 1000.0;

    // Convert float to a string, store in the
    // message buffer.
    // I had a lot of trouble finding a solution for this.
    // This is an odd, non-standard function I think.
    // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
    // https://www.programmingelectronics.com/dtostrf/
    //  - a minimum of 6 character (e.g. 000.00)
    //  - 2 digits after decimal
    //  - store in buf
    dtostrf(f_to_send, 6, 2, buf);

    // This function call tells the communication board
    // (the nano) to start ending the requested message.
    // It will keep doing this until a new call to this
    // function is made.
    setIRMessage(buf, strlen(buf));
  }

}

void reportStatusCSV() {
  //         Set mode to status request
  ircomm_mode.mode = MODE_REPORT_STATUS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ircomm_status ));
  Wire.readBytes( (uint8_t*)&ircomm_status, sizeof( ircomm_status ));


  // Report how many messages have been received on each
  // receiver
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.pass_count[i] );
    Serial.print(",");
  }
  for ( int i = 0; i < 4; i++ ) {
    float m = (float)ircomm_status.fail_count[i];

    Serial.print( -m );
    Serial.print(",");
  }
  Serial.println();

}

void reportStatus() {
  //         Set mode to status request
  ircomm_mode.mode = MODE_REPORT_STATUS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ircomm_status ));
  Wire.readBytes( (uint8_t*)&ircomm_status, sizeof( ircomm_status ));


  // Report how many messages have been received on each
  // receiver
  Serial.print("Msg Pass:\t");
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.pass_count[i] );
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("Msg Fail:\t");
  for ( int i = 0; i < 4; i++ ) {
    float m = (float)ircomm_status.fail_count[i];

    Serial.print( -m );
    Serial.print("\t");
  }
  Serial.println();

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
void getIRMessage(int which_rx, int n_bytes ) {
  if ( which_rx < 0 || which_rx >= 4 ) {
    // Invalid
    return;
  }

  // Invalid number of message bytes
  if ( n_bytes < 1 || n_bytes > 29 ) {
    return;
  }




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
  Wire.requestFrom( IRCOMM_I2C_ADDR, n_bytes );
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
