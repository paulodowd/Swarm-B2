#include <Wire.h>           // i2c to connect to IR communication board.

#include "ircomm_i2c.h"

#define BUZZER_PIN 6

// A data structure to commmunicate
// the mode.
// Keep consistent between devices.
i2c_mode_t ircomm_mode;

// A data structure to receive back
// the status of the board.
// Keep consistent between devices.
i2c_status_t ircomm_status;


// There is 2560 bytes available for global variables
// Our data takes up 32 bytes per trial.  Let's use
// 2000 bytes.
// 2000 / 32 = 62.5
#define MAX_TRIALS 10
#define MAX_POSITIONS 6
i2c_status_t results[MAX_TRIALS][MAX_POSITIONS];


// struct to store sensor data
i2c_sensors_t sensors;

// Timestamps to periodically
// - check if there are new messages
// - update the message we are sending
unsigned long check_message_ts;
unsigned long update_message_ts;
unsigned long status_ts;

// What time interval in ms should we
// check / send new messages?
unsigned long check_message_ms = 100;    // every 100ms
unsigned long update_message_ms = 1000; // every 1000ms
unsigned long status_update = 4000; // every 4s

void setup() {
  Serial.begin(115200);

  Wire.begin();


  for( int trial = 0; trial < MAX_TRIALS; trial++ ) {
    for( int pos = 0; pos < MAX_POSITIONS; pos++ ) {
      for( int rx = 0; rx < 4; rx++ ) {
        results[trial][pos].pass_count[rx] = 0;
        results[trial][pos].fail_count[rx] = 0;
        results[trial][pos].error_type[rx] = 0;
        results[trial][pos].activity[rx] = 0;
      }
    }
  
  }


  // Let's use the buzzer pin so we can
  // hear when a robot is receiving a
  // message
  pinMode(BUZZER_PIN, OUTPUT);

  check_message_ts = millis();
  update_message_ts = millis();



  status_ts = millis();

}


void loop() {


  // Get the latest light dependent resistor values
  // I think because this was not within a millis()
  // operation to control when it was executed, it was
  // very quickly interupting the communication board
  // so that it could not do it's work. (a bad thing)
  //getSensors();
  // Decide what to do
  //if( sensors.ldr[0] < 500 ) {
  // ???
  //}
  if( millis() - status_ts > status_update ) {

    reportStatusCSV();
    analogWrite(6, 120 );
    delay(50);
    analogWrite(6, 0);
    doResetStatus();
    

    status_ts = millis();
  }

  return;

  // Periodically check to see if there are new messages
  // waiting to be read from the communication board.
  if ( millis() - check_message_ts > check_message_ms ) {
    check_message_ts = millis();



    // Let's use a bool to understand if we got
    // a message or not.  Using a bool avoids a
    // beep for every receiver, we get just 1 beep.
    bool got_message = false;

    // Check all 4 receivers
    for ( int i = 0; i < 4; i++ ) {

      // If this returns more than 0, it means there
      // is a message waiting to be read.
      int n = checkRxMsgReady( i );

      // Old debugging
      //    Serial.print("Rx " );
      //    Serial.print(i);
      //    Serial.print(": ");
      //    Serial.print( n );
      //    Serial.println(" bytes ready");


      // If there is a message ready, we use 'n' as
      // the number of bytes to read down from the
      // communication board through receiver 'i'
      // (0,1,2 or 3).
      //
      if ( n > 0 ) {

        got_message = true;

        // If we don't care what the message says
        // because we just want the statistics
        // (e.g., the count of messages or the
        // timing), we can just tell the board to
        // delete the message.  You don't have to
        // do this.  In fact, using getIRMessage()
        // will do it automatically.
        //deleteMessage(i);

        // This function gets the message on receiver
        // 'i', and currently prints the message over
        // Serial.print().  You can decide what new
        // thing to do with this message.
        // Note, calling this function gets the message
        // from the IR communication board and deletes
        // it from the communication board.
        getIRMessage( i, n );


      } else {

        // n = 0, which means there were no bytes
        // available, no message.
      }
    }

    // Beep if we got a message
    if ( got_message ) {
      analogWrite( BUZZER_PIN, 120 );
      delay(5);
      analogWrite( BUZZER_PIN, 0);

    }

    // This functions is useful to see the count of
    // how many messages were received correctly and
    // how many messages were received incorrectly,
    // in a format that can be viewed on the Serial
    // plotter.
    //reportStatusCSV();
  }


  // Note that, the communication board will automatically
  // keep sending the same message. Therefore, once you use
  // the function setIRMessage() the board will periodically
  // transmit again and again.  This is actually more desirable
  // then sending a message just once, because the messaging
  // is quite unreliable (there is no guarantee that a robot
  // will receive a message sent only once).
  // Therefore, this section of code is only updating the
  // content of the message that is being transmitted.
  if ( millis() - update_message_ts > update_message_ms ) {
    update_message_ts = millis();

    // Just as an example, let's send the current count
    // in millis() so that we can see it changing over
    // time on another robot that receives it.

    char buf[32]; // Important! The max we can send is 32 bytes

    // Let's get millis() as a float. I think you will want
    // to send a value like 0.55 in the future.
    float f_to_send = (float)millis();

    // Let's divide by 1000 just to make it more interesting
    // as a float.
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

    // We used this to test/show that we can send
    // a string to the other robot.
    //memset(buf, 0, sizeof(buf)); // clear the buffer
    //sprintf( buf, "testing" );   // set some text instead


    // Let's print what we are going to send to make sure
    // it is sensible.
    //    Serial.print("Going to send: ");
    //    Serial.println( buf );

    // This function call tells the communication board
    // (the nano) to start ending the requested message.
    // It will keep doing this until a new call to this
    // function is made.
    setIRMessage(buf, strlen(buf));


    // It is possible to check how frequently
    // the communication board is sending and
    // receiving messages with the following
    // function:
    //getMsgTimings();

  }






  delay(10);

}

/*

    Below are example functions to use the communication
    board.  Not all of them will be useful to you.

*/


void doResetStatus() {
  ircomm_mode.mode = MODE_RESET_STATUS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();
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

  Serial.print("Tx Delay: "); Serial.println( msg_timings.tx_period );
  Serial.print("Rx Delay: "); Serial.println( msg_timings.rx_timeout );

}

// If you use getIRMessage(), the corresponding message
// on the communication board is deleted.  However, in
// some cases you may not use getIRMessage(), but you
// want to delete the message anyway.  It is not
// necessary to always call this function.
void deleteMessage( int which_rx ) {
  if ( which_rx == 0 ) {
    ircomm_mode.mode = MODE_CLEAR_MSG0;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

  } else if ( which_rx == 1 ) {
    ircomm_mode.mode = MODE_CLEAR_MSG1;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

  } else if ( which_rx == 2 ) {
    ircomm_mode.mode = MODE_CLEAR_MSG2;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

  } else if ( which_rx == 3 ) {
    ircomm_mode.mode = MODE_CLEAR_MSG3;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

  }



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
  } else {

    return 0;
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

  // Let's show the message failures as negative numbers
  // so that we can view both at the same time on the plotter
  // to compare pass versus fail.
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.fail_count[i] );
    Serial.print(",");
  }

  // The type of failures
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.error_type[i] );
    Serial.print(",");
  }

  // Quantity of bytes received
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.activity[i] );
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

  // Message must be shorter than 29 bytes
  // and at least 1 byte.
  if ( len <= 29 && len >= 1 ) {


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
    Serial.print("Received on Rx" );
    Serial.print( which_rx );
    Serial.print(":\t");
    for ( int i = 0; i < count; i++ ) {
      Serial.print( buf[i]  );
    }
    Serial.println();
    //Serial.println( buf );
  }

}
