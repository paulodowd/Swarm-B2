// for i2c to connect to IR communication board.
#include <Wire.h>

// definition of data structs to operate IR
// communication board
#include "ircomm_i2c.h"
#include "motors.h"

#define BUZZER_PIN 6
/*


   An example that sends a message via
   the IR Communication board and beeps if
   it receives any messages.

   Other helper functions are available to
   test.
*/

#define TRIAL_MS  250000 // 250ms
//#define TRIAL_MS  2000000 // 2s

float bearing_lpf = 0.0;

// A data structure to commmunicate
// the mode.
// Keep consistent between devices.
ir_mode_t ircomm_mode;

// A data structure to receive back
// the status of the board.
// Keep consistent between devices.
ir_status_t ircomm_status;

// struct to store sensor data
ir_sensors_t sensors;

// for getting and setting the parameters
// of the IR communication board
ir_rx_params_t rx_settings;
ir_tx_params_t tx_settings;

// Timestamps to periodically
// - check if there are new messages
// - update the message we are sending
// - get the general messaging metrics
unsigned long check_message_ts;
unsigned long update_message_ts;
unsigned long status_ts;
unsigned long settings_ts;

// What time interval in ms should we
// check / send new messages?
unsigned long check_message_ms = 100;    // every 100ms
unsigned long update_message_ms = 1000; // every 1000ms
unsigned long status_update_ms = 1000; // every 1000ms
unsigned long settings_update_ms = 500;

bool rx[4];

int random_pitch;

unsigned long start_ts;
int pos;
int trial;

Motors_c motors;

void setup() {
  Serial.begin(115200);

  delay(2000);

  // Enable i2c on the 3Pi
  Wire.begin();


  // Let's use the buzzer pin so we can
  // hear when a robot is receiving a
  // message
  pinMode(BUZZER_PIN, OUTPUT);

  initRandomSeed();
  random_pitch = random( 220, 880);


  // Initialise timestamps
  status_ts = millis();
  check_message_ts = millis();
  update_message_ts = millis();

  motors.initialise();

  updateSettings();

  // Make sure the IR Communication board
  // is reset and ready.
  pos = 0;
  trial = 0;
  start_ts = micros();
  doResetStatus();
  //  motors.setMotorsPWM( 20, 20 );

  delay(100);
  while ( true ) {
    reportStatusErrorsCSV();
    delay(1);
//    getByteTimings();
    delay(100);
  }


}
void initRandomSeed() {
  pinMode(A1, INPUT);
  byte r = 0x00;
  for ( int i = 0; i < 8; i++ ) {
    byte b = (byte)analogRead(A1);
    b = b & 0x01;
    //Serial.println(b, BIN);

    r |= (b << i);
    delayMicroseconds(10);
  }
  //Serial.println(r, BIN);
  randomSeed( r );
}

void loop() {


  //getMsgTimings();


  //  reportErrorsCSV();
  //getRxDirection();
  //  getRxActivity();

  //  for( int i = 0; i < 4; i++ ) {
  //    Serial.print( rx[i] == true ? "1," : "0,");
  //  }
  //  Serial.println();
  //getSensors();
  unsigned long dt = micros() - start_ts;
  if ( dt >= TRIAL_MS ) {
    Serial.print( pos );
    Serial.print(",");
    Serial.print( trial );
    Serial.print(",");
    reportStatusErrorsCSV();
    trial++;
    if ( trial >= 10 ) {
      pos++;
      trial = 0;

      for ( int i = 0; i < 8; i++ ) {
        tone(BUZZER_PIN, random_pitch + 100, 10);
        delay(500);
      }
      //getRxSettings();
    }
    start_ts = micros();
    doResetStatus();
    tone(BUZZER_PIN, random_pitch, 10);

  }

}

void updateMessageToSend() {
  // Note that, the communication board will automatically
  // keep sending the same message. Once you have set a
  // message to send, you don't need to do it again.
  // Therefore, once you use
  // the function setIRMessage() the board will periodically
  // transmit again and again.  This is actually more desirable
  // then sending a message just once, because the messaging
  // is quite unreliable (there is no guarantee that a robot
  // will receive a message sent only once).
  // Therefore, this section of code is only updating the
  // content of the message that is being transmitted, and
  // this is helpful because we can see on the receiving
  // robot if it is getting updates messages.
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
    //    // it is sensible.
    //                Serial.print("Going to send: ");
    //                Serial.println( buf );

    // This function call tells the communication board
    // (the nano) to start ending the requested message.
    // It will keep doing this until a new call to this
    // function is made.

    //setIRMessage(buf, strlen(buf));

  }
}



void updateSettings() {

  // Uncomment below for the example
  // that changes the board configuration

  if ( millis() - settings_ts > settings_update_ms ) {
    settings_ts = millis();

    // To be safe, lets first get the current settings
    // from the board.  These update the structs declared
    // in the global scope.
    getRxSettings();
    delay(10);
    //getTxSettings();
    //delay(10);

    // Let's now modify the structs and send it back.
    // We should see the change on the next iteration
    // of loop()
    // Lets test by just togggling some binary flags
    rx_settings.flags.bits.cycle = false;
    rx_settings.flags.bits.cycle_on_rx = true;
    rx_settings.flags.bits.desync = false;          // don't randomise
    rx_settings.flags.bits.overrun = true;
    rx_settings.index = 0;
    rx_settings.flags.bits.predict_period = true; // don't optimise
    rx_settings.period_max = 2000;  // use 2000ms
    rx_settings.flags.bits.rx0 = 1;
    rx_settings.flags.bits.rx1 = 1;
    rx_settings.flags.bits.rx2 = 1;
    rx_settings.flags.bits.rx3 = 1;
    //    tx_settings.tx_desync = 0;         // don't randomise
    //    tx_settings.tx_period_max = 10000; // transmit every 2 seconds
    //
    setRxSettings();
    //    delay(5);
    //    setTxSettings();

    getRxSettings();
  }

}

void checkForMessages() {
  // Periodically check to see if there are new messages
  // waiting to be read from the communication board.
  if ( millis() - check_message_ts > check_message_ms ) {
    check_message_ts = millis();


    // Let's use a bool to understand if we got a message
    // on any receiver. We'll make a beep if any receiver
    // got an IR message.
    bool got_message = false;

    // Check all 4 receivers
    for ( int i = 0; i < 4; i++ ) {

      rx[i] = false;

      // If this returns more than 0, it means there
      // is a message waiting to be read.
      // The value of n is the number of bytes we need
      // to get from the IR Communication board
      int n = checkRxMsgReady( i );

      //       Old debugging
      //                Serial.print("Rx " );
      //                Serial.print(i);
      //                Serial.print(": ");
      //                Serial.print( n );
      //                Serial.println(" bytes ready");


      // If there is a message ready, we use 'n' as
      // the number of bytes to read down from the
      // communication board through receiver 'i'
      // (0,1,2 or 3).
      if ( n > 0 ) {
        rx[i] = true;
        got_message = true;

        // If we don't care what the message says
        // because we just want the statistics
        // (e.g., the count of messages or the
        // timing), we can just tell the board to
        // delete the message.  You don't have to
        // do this.  If you use getIRMessage()
        // it will be deleted automatically.
        deleteMessage(i);

        // This function gets the message on receiver
        // 'i', and currently prints the message over
        // Serial.print().  You can decide what new
        // thing to do with this message.
        // Note, calling this function gets the message
        // from the IR communication board and deletes
        // it from the communication board.
        //getIRMessage( i, n );


      } else {

        // n = 0, which means there were no bytes
        // available, no message.
      }
    }

    // Beep if we got a message
    if ( got_message ) {
      tone(BUZZER_PIN, random_pitch, 10);
      //      analogWrite( BUZZER_PIN, 120 );
      //      delay(5);
      //      analogWrite( BUZZER_PIN, 0);

    }

  }
}


/*

    Below are example functions to use the communication
    board.  Not all of them will be useful to you.

*/

void setTxSettings() {

  // First, set the mode
  ircomm_mode.mode = MODE_SET_TX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();
  delay(5);

  // Now send the struct
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&tx_settings, sizeof( tx_settings ));
  Wire.endTransmission();

}

void setRxSettings() {

  // First, set the mode
  ircomm_mode.mode = MODE_SET_RX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();
  delay(5);

  // Now send the struct
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&rx_settings, sizeof( rx_settings ));
  Wire.endTransmission();

}

// ask the ir communication board what it's current
// rx settings are.
void getRxSettings() {

  // Set correct mode.
  ircomm_mode.mode = MODE_GET_RX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( rx_settings ));
  Wire.readBytes( (uint8_t*)&rx_settings, sizeof( rx_settings ));

  // Show data for debugging
  Serial.println("Rx settings:");
  Serial.print(" - cycle: ");       Serial.println(rx_settings.flags.bits.cycle > 0 ? "true" : "false");
  Serial.print(" - cycle on rx: ");  Serial.println(rx_settings.flags.bits.cycle_on_rx > 0 ? "true" : "false");
  Serial.print(" - desync rx: ");  Serial.println(rx_settings.flags.bits.desync > 0 ? "true" : "false");
  Serial.print(" - predict timeout: "); Serial.println(rx_settings.flags.bits.predict_period > 0 ? "true" : "false");
  Serial.print(" - overrun: ");    Serial.println(rx_settings.flags.bits.overrun > 0 ? "true" : "false");
  Serial.print(" - current timeout: ");    Serial.println(rx_settings.period);
  Serial.print(" - timeout max: ");    Serial.println(rx_settings.period_max);
  Serial.print(" - timeout multiplier: ");    Serial.println(rx_settings.predict_multi);
  Serial.print(" - power index: ");    Serial.println(rx_settings.index);
  Serial.print(" - byte timeout: ");    Serial.println(rx_settings.byte_timeout);
  Serial.print(" - Rx0 available: ");    Serial.println(rx_settings.flags.bits.rx0 > 0 ? "true" : "false");
  Serial.print(" - Rx1 available: ");    Serial.println(rx_settings.flags.bits.rx1 > 0 ? "true" : "false");
  Serial.print(" - Rx2 available: ");    Serial.println(rx_settings.flags.bits.rx2 > 0 ? "true" : "false");
  Serial.print(" - Rx3 available: ");    Serial.println(rx_settings.flags.bits.rx3 > 0 ? "true" : "false");
  Serial.println();
}

void getTxSettings() {

  // Set correct mode.
  ircomm_mode.mode = MODE_GET_TX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( rx_settings ));
  Wire.readBytes( (uint8_t*)&tx_settings, sizeof( tx_settings ));

  // Show data for debugging
  Serial.println("Tx settings:");
  Serial.print(" - mode: ");       Serial.println(tx_settings.flags.bits.mode > 0 ? "interleaved" : "periodic");
  Serial.print(" - desync: ");       Serial.println(tx_settings.flags.bits.desync > 0 ? "true" : "false");
  Serial.print(" - repeat: ");  Serial.println(tx_settings.repeat );
  Serial.print(" - current period: "); Serial.println(tx_settings.period);
  Serial.print(" - max period: "); Serial.println(tx_settings.period_max);
  Serial.println();

}


// Completely resets the IR Communication board.
void doResetStatus() {
  ircomm_mode.mode = MODE_RESET_STATUS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();
}


// This will get the measurements of the timings
// of receiving messages.  See msg_timings struct.
void getMsgTimings() {
  ircomm_mode.mode = MODE_REPORT_MSG_TIMINGS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  ir_msg_timings_t msg_timings;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_timings ));
  Wire.readBytes( (uint8_t*)&msg_timings, sizeof( msg_timings ));

  //  Serial.println("ms between messages:");
  for ( int i = 0; i < 4; i++ ) {

    Serial.print( msg_timings.dt_ms[i] );
    Serial.print(",");
  }
  Serial.println();
  //  //Serial.println("Last rx time in ms:");
  //  for ( int i = 0; i < 4; i++ ) {
  //    Serial.print("- Rx ");
  //    Serial.print( i );
  //    Serial.print(": ");
  //    Serial.print( msg_timings.msg_t[i] );
  //    Serial.println();
  //  }


}
void getByteTimings() {
  ircomm_mode.mode = MODE_REPORT_BYTE_TIMINGS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  ir_byte_timings_t byte_timings;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( byte_timings ));
  Wire.readBytes( (uint8_t*)&byte_timings, sizeof( byte_timings ));

  //  Serial.println("ms between messages:");
  for ( int i = 0; i < 4; i++ ) {

    Serial.print( byte_timings.dt_us[i] );
    Serial.print(",");
  }
  //Serial.println();
  //  //Serial.println("Last rx time in ms:");
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( byte_timings.ts_us[i] );
    Serial.print(",");
  }
  Serial.println();


}

// Quickly delete a message on a receiver.
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
// is a robot on opposite sides of this robot.
// If there are robots on either side, we would
// expect a low magnitude (vectors cancelling)
// and an unreliable angle.  Therefore, if we
// have a high magnitude (vectors adding) we can
// be more confident of the bearing estimate.
void getRxDirection() {

  ircomm_mode.mode = MODE_REPORT_RX_BEARING;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  ir_bearing_t bearing;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( bearing ));
  Wire.readBytes( (uint8_t*)&bearing, sizeof( bearing ));

  bearing_lpf = (bearing_lpf * 0.7) + (bearing.theta * 0.3);
  Serial.print( bearing_lpf, 4);
  Serial.print(",");
  Serial.print(bearing.theta, 4);
  Serial.print(",");
  Serial.print( bearing.mag, 4 );
  Serial.print(",");
  Serial.println( bearing.sum, 4);
}

// Reports the activity level of each receiver, which
// will vary continously over time between 0 and 1
// This activity level is what is being used to
// calculate the bearing information above.
void getRxActivity() {
  ircomm_mode.mode = MODE_REPORT_RX_VECTORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  ir_vectors_t vectors;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( vectors));
  Wire.readBytes( (uint8_t*)&vectors, sizeof( vectors ));

  for ( int i = 0; i < 4; i++ ) {
    Serial.print( vectors.rx[i], 4 );
    Serial.print(",");

  }
  Serial.println();
}


// Use to see if there is a message available on
// receivers 0 to 3 (which_rx).
// If returns 0, no messages available.
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
  ir_msg_status_t msg_status;

  // Request the message size to be sent across into
  // msg_status
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_status ));
  Wire.readBytes( (uint8_t*)&msg_status, sizeof( msg_status ));

  return msg_status.n_bytes;// 0 : 29

}

void reportStatusErrorsCSV() {


  ircomm_mode.mode = MODE_REPORT_STATUS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ircomm_status ));
  Wire.readBytes( (uint8_t*)&ircomm_status, sizeof( ircomm_status ));


  // Report how many messages have been received on each
  // receiver
    Serial.print("P,");
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.pass_count[i] );
    Serial.print(",");
  }

  // Let's show the message failures as negative numbers
  // so that we can view both at the same time on the plotter
  // to compare pass versus fail.

    Serial.print("F,");
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.fail_count[i] );
    Serial.print(",");
  }

    Serial.print("A,");
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.activity[i] );
    Serial.print(",");
  }
  Serial.print("S,");
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.saturation[i] );
    Serial.print(",");
  }


  ir_errors_t errors;
  ircomm_mode.mode = MODE_REPORT_ERRORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( errors ));
  Wire.readBytes( (uint8_t*)&errors, sizeof( errors ));

  // for each recevier
  for ( int i = 0; i < 4; i++ ) {

    // error type

    //  Serial.print("E");Serial.print(i);Serial.print(",");
    for ( int j = 0; j < 4; j++ ) {
      Serial.print( errors.type[i][j] );
      Serial.print(",");
    }
  }


  // Histogram
  ir_hist_t hist;
  ircomm_mode.mode = MODE_REPORT_HIST;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( hist ));
  Wire.readBytes( (uint8_t*)&hist, sizeof( hist ));

  // for each recevier
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( hist.id[i] );
    Serial.print(",");
  }


  Serial.print("\n");

}

void reportErrorsCSV() {
  ir_errors_t errors;
  ircomm_mode.mode = MODE_REPORT_ERRORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( errors ));
  Wire.readBytes( (uint8_t*)&errors, sizeof( errors ));

  // for each recevier
  for ( int i = 0; i < 4; i++ ) { //
    Serial.print("Rx "); Serial.print( i ); Serial.print(": ");
    // for each error type
    for ( int j = 0; j < 4; j++ ) {  //
      Serial.print( errors.type[i][j] );
      Serial.print(",");
    }
    Serial.print("\n");

  }
}

// Gets the status metrics of communication and
// prints them as comma separated values.
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

  //  // The type of failures
  //  for ( int i = 0; i < 4; i++ ) {
  //    Serial.print( ircomm_status.error_type[i] );
  //    Serial.print(",");
  //  }

  // Quantity of bytes received
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.activity[i] );
    Serial.print(",");
  }

  Serial.println();

}

// Gets the status metrics of communication and
// prints them in a more readable format.
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
// other robots.  Once set, the robot will keep
// repeating the transmission.  The tranmission will
// occur periodically (between 150-300ms).
void setIRMessage(char* str_to_send, int len) {

  // Message must be maximum 32 bytes
  if ( len <= 32 ) {


    // Set mode to set a new IR Message
    ircomm_mode.mode = MODE_SET_MSG;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();
    delayMicroseconds(250);


    // The communication board will always default
    // to waiting to receive a message to transmit
    // so we don't need to change the mode.
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)str_to_send, len);
    Wire.endTransmission();
  }
}

// Used to get readings from the sensors on
// the IR Communication board. These are
// potentially 3 LDR and 2 IR Proximity sensors.
// Note, these need to be on the board!
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
  if ( n_bytes < 1 || n_bytes > 32 ) {
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
    Serial.print("Received on Rx " );

    Serial.print( which_rx );
    Serial.print(" at ");
    Serial.print( millis() );
    Serial.print(":\t");
    for ( int i = 0; i < count; i++ ) {
      Serial.print( buf[i]  );
    }
    Serial.println();
    Serial.print("Bytes: ");
    for ( int i = 0; i < count; i++ ) {
      Serial.print( (byte)buf[i]  );
      Serial.print(",");
    }
    Serial.println();
    //    Serial.println( buf );
  }

}
