#include "SwarmB2.h"
#include <Wire.h>

void SwarmB2_c::init() {
  // Populate the board config cache
  getRxSettings();
  getTxSettings();
}

bool SwarmB2_c::getIRMessage( uint8_t * buf, int which_rx ) {

  // Check receiver index is valid
  if ( which_rx >= 0 && which_rx < 4 ) {

    uint8_t len = getMsgStatus( which_rx );

    // valid message len is 1:32. 32 is the
    // maximum number of uint8_ts in an i2c
    // transaction on Arduino
    if ( len > 0 && len < 33 ) {
      
      ir_mode_t ircomm_mode;

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
        // error caught above.
      }

      // Set mode to send across the message
      Wire.beginTransmission( IRCOMM_I2C_ADDR );
      Wire.write( (uint8_t*)&ircomm_mode, sizeof( ircomm_mode));
      Wire.endTransmission();

      // Read across uint8_ts using the anticipated len.
      // Store into buffer provided as function argument
      Wire.requestFrom( IRCOMM_I2C_ADDR, len );
      Wire.readBytes( (uint8_t*)buf, len );

      // Flag that a message was copied across.
      return true;

    }

  } else {

    // Invalid receiver, signal no message
    return false;
  }
}

uint8_t SwarmB2_c::getMsgStatus( int which_rx ) {

  ir_mode_t ircomm_mode;

  if ( which_rx == 0 ) {
    ircomm_mode.mode = MODE_SIZE_MSG0;
  } else if ( which_rx == 1 ) {
    ircomm_mode.mode = MODE_SIZE_MSG1;
  } else if ( which_rx == 2 ) {
    ircomm_mode.mode = MODE_SIZE_MSG2;
  } else if ( which_rx == 3 ) {
    ircomm_mode.mode = MODE_SIZE_MSG3;
  } else {
    // Invalid receiver
    return 0;
  }

  // Set mode to read back how many uint8_ts are
  // available.
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  // We'll use this struct to check how many
  // uint8_ts are available of a message.
  // 0 uint8_ts means no message.
  ir_msg_status_t msg_status;

  // Request the message size to be sent across into
  // msg_status
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_status ));
  Wire.readBytes( (uint8_t*)&msg_status, sizeof( msg_status ));

  return msg_status.n_bytes;// 0 = error, else [1 : 32]
}

void SwarmB2_c::setIRMessage( uint8_t * payload, int len ) {
  // Message must be maximum 32 uint8_ts
  if ( len <= 32 ) {

    ir_mode_t ircomm_mode;

    // Set mode to set a new IR Message
    ircomm_mode.mode = MODE_SET_MSG;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (uint8_t*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

    // The communication board will always default
    // to waiting to receive a message to transmit
    // so we don't need to change the mode.
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (uint8_t*)payload, len);
    Wire.endTransmission();
  }
}

void SwarmB2_c::stopTransmitting() {
  ir_mode_t ircomm_mode;

  ircomm_mode.mode = MODE_STOP_TX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();
}

ir_bearing_t SwarmB2_c::getBearing() {
  ir_mode_t ircomm_mode;

  ircomm_mode.mode = MODE_REPORT_RX_BEARING;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  ir_bearing_t bearing;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( bearing ));
  Wire.readBytes( (uint8_t*)&bearing, sizeof( bearing ));

  return bearing;
}

void SwarmB2_c::resetMetrics() {
  ir_mode_t ircomm_mode;

  // Set i2c mode.
  ircomm_mode.mode = MODE_RESET_METRICS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();
}

void SwarmB2_c::getRxSettings() {
  ir_mode_t ircomm_mode;

  // Set i2c mode.
  ircomm_mode.mode = MODE_GET_RX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( rx_settings ));
  Wire.readBytes( (uint8_t*)&rx_settings, sizeof( rx_settings ));
}

void SwarmB2_c::getTxSettings() {
  ir_mode_t ircomm_mode;

  // Set correct mode.
  ircomm_mode.mode = MODE_GET_TX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( rx_settings ));
  Wire.readBytes( (uint8_t*)&tx_settings, sizeof( tx_settings ));
}

void SwarmB2_c::setRxSettings() {
}
void SwarmB2_c::setTxSettings() {
}
