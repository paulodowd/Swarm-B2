
/*
 * 
 * Simplest code to check if new IR
 * messages are being received from
 * the M5.
 * 
 */


#include <M5Core2.h>
#include <Wire.h>
#include "ircomm_i2c.h"



unsigned long check_message_ts;
unsigned long settings_ts;

// for getting and setting the parameters
// of the IR communication board
ir_rx_params_t  rx_settings;
ir_tx_params_t  tx_settings;
ir_mode_t       ircomm_mode;

void setup() {

  M5.begin();
  Serial.begin(115200);
  
  Wire.begin();   // as master

  // Wait for other parts of system
  // to boot
  delay(500);

  // Use these to schedule activities
  check_message_ts = millis();
  settings_ts = millis();

  getRxSettings();
  delay(5);

  
  Serial.println("Receiving Unit Test");


}

void loop() {

  // Periodically check the tx settings
  // to see if the board is resetting
  if( millis() - check_message_ts > 250 ) {
    check_message_ts = millis();
    
    bool got_msg = false;
    for( int i = 0; i < 4; i++ ) {
      int n = checkRxMsgReady(i);

      if( n > 0 ) {
         got_msg = true;

         // Remove it from the IR comm board
         // buffer so that it doesn't look like
         // a new message on the next iteration
         deleteMessage( i );
        
      }
      
    }
    if( got_msg ) {
       Serial.print("Got message at: ");
       Serial.println( millis() );
      
    }
  }
}



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
  Serial.print(" - cycle: ");       Serial.println(rx_settings.rx_cycle > 0 ? "true" : "false");
  Serial.print(" - cycle on rx: ");  Serial.println(rx_settings.rx_cycle_on_rx > 0 ? "true" : "false");
  Serial.print(" - desync rx: ");  Serial.println(rx_settings.rx_desync > 0 ? "true" : "false");
  Serial.print(" - predict timeout: "); Serial.println(rx_settings.rx_predict_timeout > 0 ? "true" : "false");
  Serial.print(" - overrun: ");    Serial.println(rx_settings.rx_overrun > 0 ? "true" : "false");
  Serial.print(" - current timeout: ");    Serial.println(rx_settings.rx_timeout);
  Serial.print(" - timeout max: ");    Serial.println(rx_settings.rx_timeout_max);
  Serial.print(" - timeout multiplier: ");    Serial.println(rx_settings.rx_timeout_multi);
  Serial.print(" - power index: ");    Serial.println(rx_settings.rx_pwr_index);
  Serial.print(" - byte timeout: ");    Serial.println(rx_settings.rx_byte_timeout);
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
  Serial.print(" - mode: ");       Serial.println(tx_settings.tx_mode > 0 ? "interleaved" : "periodic");
  Serial.print(" - desync: ");       Serial.println(tx_settings.tx_desync > 0 ? "true" : "false");
  Serial.print(" - repeat: ");  Serial.println(tx_settings.tx_repeat );
  Serial.print(" - current period: "); Serial.println(tx_settings.tx_period);
  Serial.print(" - max period: "); Serial.println(tx_settings.tx_period_max);
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

  return msg_status.n_bytes;// 0 : 32

}

// Use this function to set a message to transmit to
// other robots.  Once set, the robot will keep
// repeating the transmission.  The tranmission will
// occur periodically (between 150-300ms).
void setIRMessage(char* str_to_send, int len) {

  // Message must be maximum 32 bytes
  if ( len <= 32 ) {


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
    Serial.println( buf );
  }

}
