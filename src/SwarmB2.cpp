#include "SwarmB2.h"
#include <Wire.h>
#include <stdint.h>
#include <limits.h>

void SwarmB2_c::init() {
  memset( (void*)&rx_settings, 0, sizeof( rx_settings ));
  memset( (void*)&tx_settings, 0, sizeof( tx_settings ));

  // Populate the board config cache ready for
  // any subsequent user changes
  getRxSettings();
  delay(5);
  getTxSettings();
  delay(5);

  // Ensure we have default settings
  configureDefault();
}


void SwarmB2_c::configureDefault() {

  // Settings for receiving
  rx_settings.flags.bits.cycle_on_rx    = true;
  rx_settings.flags.bits.desync         = true;
  rx_settings.flags.bits.overrun        = true;
  rx_settings.flags.bits.rand_rx        = false;
  rx_settings.flags.bits.rx0            = true;
  rx_settings.flags.bits.rx1            = true;
  rx_settings.flags.bits.rx2            = true;
  rx_settings.flags.bits.rx3            = true;
  rx_settings.skip_multi                = 0;
  rx_settings.predict_multi             = 0;
  rx_settings.index                     = 0;
  rx_settings.period_base_ms            = 60;
  rx_settings.timeout_multi             = 0;
  rx_settings.saturation_us             = 20000;
  setRxSettings();

  // Settings for tranmission
  tx_settings.flags.bits.desync = 1;
  tx_settings.repeat            = 3;
  tx_settings.predict_multi     = 4;
  tx_settings.defer_multi       = 1;
  tx_settings.preamble_repeat   = 4;
  tx_settings.period_base_ms    = 170;
  setTxSettings();

  resetMetrics();
}

void SwarmB2_c::updateSettings() {

  // Settings for receiving
  rx_settings.flags.bits.cycle_on_rx    = true;
  rx_settings.flags.bits.desync         = false;
  rx_settings.flags.bits.overrun        = true;
  rx_settings.flags.bits.rand_rx        = false;
  rx_settings.flags.bits.rx0            = true;
  rx_settings.flags.bits.rx1            = true;
  rx_settings.flags.bits.rx2            = true;
  rx_settings.flags.bits.rx3            = true;
  rx_settings.skip_multi                = 0;
  rx_settings.predict_multi             = 4;
  rx_settings.index                     = 0;
  rx_settings.period_base_ms            = 60;
  rx_settings.timeout_multi             = 6;
  rx_settings.saturation_us             = 20000;
  setRxSettings();

  // Settings for tranmission
  tx_settings.flags.bits.desync = 1;
  tx_settings.repeat            = 3;
  tx_settings.predict_multi     = 4;
  tx_settings.defer_multi       = 0;
  tx_settings.preamble_repeat   = 4;
  tx_settings.period_base_ms    = 170;
  setTxSettings();

  resetMetrics();
}


// This function is used to fetch a message
// from one of the four receivers on the
// IR communication board. This function will
// complete the check of whether or not there
// is a message available.
// Return:
//  -1    invalid request
//   0    no message available
//  [1:32]valid message length
int SwarmB2_c::getIRMessage( uint8_t * received, int which_rx ) {

  // Bad request
  if ( which_rx < 0 || which_rx > 3 ) return -1;

  // Get the message status from the board (1 byte)
  ir_status_t ir_status = getStatus();

  // If the status bit for this rx is set
  if ( ir_status.bits & (1 << which_rx) ) {

    // First, check if there is a message
    // available (+ve non-zero length)
    uint8_t len = getMsgLength( which_rx );

    // valid message len is 1:32. 32 is the
    // maximum number of uint8_ts in an i2c
    // transaction on Arduino
    if ( len > 0 && len < 33 ) {

      ir_mode_t ircomm;

      // Format mode request for which receiver
      if ( which_rx == 0 ) {
        ircomm.mode = MODE_REPORT_MSG0;
      } else if ( which_rx == 1 ) {
        ircomm.mode = MODE_REPORT_MSG1;
      }  else if ( which_rx == 2 ) {
        ircomm.mode = MODE_REPORT_MSG2;
      }  else if ( which_rx == 3 ) {
        ircomm.mode = MODE_REPORT_MSG3;
      } else {
        // error caught above.
      }

      // Set mode to send across the message
      Wire.beginTransmission( IRCOMM_I2C_ADDR );
      Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
      Wire.endTransmission();

      // Read across uint8_ts using the anticipated len.
      // Store into buffer provided as function argument
      Wire.requestFrom( IRCOMM_I2C_ADDR, len );
      Wire.readBytes( (uint8_t*)received, len );

      // Let the user know how many bytes were
      // received.
      return (int)len;

    } else {
      // Flag that no bytes were available
      return 0;
    }
  }
  return 0;

}

void SwarmB2_c::printStatus() {
  
  ir_status_t ir_status = getStatus();

  Serial.print("Msg[0:3], Activity[4:7]: ");

  for ( int i = 0; i < 8; i++ ) {
    if ( ir_status.bits & (1 << i) ) {
      Serial.print("1 ");
    } else {
      Serial.print("0 ");
    }
  }
  Serial.println();
}


void SwarmB2_c::printAnyMessage() {

  char buf[32];

  // First, check if any messages are available
  ir_status_t ir_status = getStatus();

  // Message available is first 4 bits
  for( int i = 0; i < 4; i++ ) {
    if ( ir_status.bits & (1 << i) ) {
      memset( (void*)&buf, 0, sizeof(buf) );
      int len = getIRMessage( (uint8_t*)buf, i );
      Serial.print("Rx");
      Serial.print( i );
      Serial.print(" len=");
      Serial.print(len);
      Serial.print(":");
      Serial.print( buf );
      Serial.print(" t=");
      Serial.println( millis ());
    }
  }

  
}

// Queries the IR communication board to see if
// a message has been stored for a receiver.
// There are 4 receivers, [0:3]
// Return:
//  -1    Invalid receiver request
//   0    No message available
//  [0:32]Available message length
uint8_t SwarmB2_c::getMsgLength( int which_rx ) {

  ir_mode_t ircomm;

  if ( which_rx == 0 ) {
    ircomm.mode = MODE_REPORT_LEN_MSG0;
  } else if ( which_rx == 1 ) {
    ircomm.mode = MODE_REPORT_LEN_MSG1;
  } else if ( which_rx == 2 ) {
    ircomm.mode = MODE_REPORT_LEN_MSG2;
  } else if ( which_rx == 3 ) {
    ircomm.mode = MODE_REPORT_LEN_MSG3;
  } else {
    // Invalid receiver
    return -1;
  }

  // Set mode to read back how many uint8_ts are
  // available.
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();

  // We'll use this struct to check how many
  // uint8_ts are available of a message.
  // 0 uint8_ts means no message.
  ir_msg_len_t msg_len;

  // Request the message size to be sent across into
  // msg_status
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_len ));
  Wire.readBytes( (uint8_t*)&msg_len, sizeof( msg_len ));

  return msg_len.n_bytes;// 0 = error, else [1 : 32]
}

//
void SwarmB2_c::setIRMessage( uint8_t * payload, int len ) {
  // Message must be length [1:32]
  if ( len <= 32 && len > 0) {

    ir_mode_t ircomm;

    // Set mode to set a new IR Message
    ircomm.mode = MODE_SET_MSG;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
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
  ir_mode_t ircomm;

  ircomm.mode = MODE_STOP_TX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();
}

ir_bearing_t SwarmB2_c::getBearing() {
  ir_mode_t ircomm;

  ircomm.mode = MODE_REPORT_RX_BEARING;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();

  ir_bearing_t bearing;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( bearing ));
  Wire.readBytes( (uint8_t*)&bearing, sizeof( bearing ));

  return bearing;
}

void SwarmB2_c::resetMetrics() {
  ir_mode_t ircomm;

  // Set i2c mode.
  ircomm.mode = MODE_RESET_METRICS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();
}


void SwarmB2_c::getRxSettings() {
  ir_mode_t ircomm;

  // Set i2c mode.
  ircomm.mode = MODE_GET_RX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( rx_settings ));
  Wire.readBytes( (uint8_t*)&rx_settings, sizeof( rx_settings ));
}


void SwarmB2_c::getTxSettings() {
  ir_mode_t ircomm;

  // Set correct mode.
  ircomm.mode = MODE_GET_TX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( rx_settings ));
  Wire.readBytes( (uint8_t*)&tx_settings, sizeof( tx_settings ));
}


void SwarmB2_c::setRxSettings() {

  ir_mode_t ircomm;

  // First, set the mode
  ircomm.mode = MODE_SET_RX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();

  // Now send the struct
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&rx_settings, sizeof( rx_settings ));
  Wire.endTransmission();
}


void SwarmB2_c::setTxSettings() {

  ir_mode_t ircomm;

  // First, set the mode
  ircomm.mode = MODE_SET_TX;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();

  // Now send the struct
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&tx_settings, sizeof( tx_settings ));
  Wire.endTransmission();
}

void SwarmB2_c::printTxSettings() {

  Serial.println("SwarmB2 Tx Settings:");
  //Serial.print(" Mode: \t\t\t"); Serial.println( tx_settings.flags.bits.mode > 0 ? "interleaved" : "periodic" );
  //Serial.print(" Predict Period: \t"); Serial.println( tx_settings.flags.bits.predict_period > 0 ? "true" : "false" );
  Serial.print(" Desync: \t\t"); Serial.println( tx_settings.flags.bits.desync > 0 ? "true" : "false" );
  Serial.print(" Preamble repeat: \t"); Serial.println( tx_settings.preamble_repeat );
  Serial.print(" Defer multi: \t"); Serial.println( tx_settings.defer_multi );
  Serial.print(" Repeats: \t\t"); Serial.println( tx_settings.repeat );
  Serial.print(" Predict Multi: \t"); Serial.println( tx_settings.predict_multi);
  Serial.print(" Period: \t\t"); Serial.println( tx_settings.period_ms );
  Serial.print(" Period Norm: \t\t"); Serial.println( tx_settings.period_base_ms );
  Serial.print(" Len: \t\t\t"); Serial.println( tx_settings.len );
  Serial.println("\n");
}
void SwarmB2_c::printRxSettings() {
  Serial.println("SwarmB2 Rx Settings:");
  //Serial.print(" Cycle: \t\t");       Serial.println(rx_settings.flags.bits.cycle > 0 ? "true" : "false");
  Serial.print(" Cycle on Rx: \t\t");  Serial.println(rx_settings.flags.bits.cycle_on_rx > 0 ? "true" : "false");
  //Serial.print(" Predict period: \t"); Serial.println(rx_settings.flags.bits.predict_period > 0 ? "true" : "false");
  Serial.print(" Overrun: \t\t");    Serial.println(rx_settings.flags.bits.overrun > 0 ? "true" : "false");
  Serial.print(" Desync: \t\t");  Serial.println(rx_settings.flags.bits.desync > 0 ? "true" : "false");
  Serial.print(" Rx0 available: \t");    Serial.println(rx_settings.flags.bits.rx0 > 0 ? "true" : "false");
  Serial.print(" Rx1 available: \t");    Serial.println(rx_settings.flags.bits.rx1 > 0 ? "true" : "false");
  Serial.print(" Rx2 available: \t");    Serial.println(rx_settings.flags.bits.rx2 > 0 ? "true" : "false");
  Serial.print(" Rx3 available: \t");    Serial.println(rx_settings.flags.bits.rx3 > 0 ? "true" : "false");
  //Serial.print(" Desaturate: \t\t");    Serial.println(rx_settings.flags.bits.desaturate > 0 ? "true" : "false");
  Serial.print(" Rand Rx: \t\t");    Serial.println(rx_settings.flags.bits.rand_rx > 0 ? "true" : "false");
  Serial.print(" Predict multi: \t");    Serial.println(rx_settings.predict_multi);
  Serial.print(" Period: \t\t");    Serial.println(rx_settings.period_ms);
  Serial.print(" Period norm: \t\t");    Serial.println(rx_settings.period_base_ms);
  Serial.print(" Index: \t\t");    Serial.println(rx_settings.index);
  Serial.print(" Skip multi: \t\t");    Serial.println(rx_settings.skip_multi);
  Serial.print(" Byte timeout: \t\t");    Serial.println(rx_settings.timeout_multi);
  Serial.print(" Saturation timeout: \t");    Serial.println(rx_settings.saturation_us);
  Serial.print(" Last len: \t\t");    Serial.println(rx_settings.len );
  Serial.println("\n");
}



ir_status_t   SwarmB2_c::getStatus() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_STATUS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_status_t ir_status;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ir_status));
  Wire.readBytes( (uint8_t*)&ir_status, sizeof( ir_status));

  // Return result
  return ir_status;

}

ir_vectors_t      SwarmB2_c::getRxVectors() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_RX_VECTORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_vectors_t vectors;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( vectors));
  Wire.readBytes( (uint8_t*)&vectors, sizeof( vectors ));

  // Return result
  return vectors;
}

ir_activity_t     SwarmB2_c::getRxActivity() {
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_ACTIVITY;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();

  ir_activity_t activity;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( activity ));
  Wire.readBytes( (uint8_t*)&activity, sizeof( activity ));

  return activity;
}

ir_saturation_t   SwarmB2_c::getRxSaturation() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_SATURATION;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_saturation_t saturation;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( saturation ));
  Wire.readBytes( (uint8_t*)&saturation, sizeof( saturation ));

  // Return result
  return saturation;
}

ir_msg_timings_t  SwarmB2_c::getMsgTimings() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_MSG_TIMINGS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_msg_timings_t msg_timings;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_timings ));
  Wire.readBytes( (uint8_t*)&msg_timings, sizeof( msg_timings ));

  // Return result
  return msg_timings;
}
ir_byte_timings_t SwarmB2_c::getByteTimings() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_BYTE_TIMINGS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_byte_timings_t byte_timings;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( byte_timings ));
  Wire.readBytes( (uint8_t*)&byte_timings, sizeof( byte_timings ));

  // Return result
  return byte_timings;

}

ir_skips_t        SwarmB2_c::getRxSkips() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_SKIPS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_skips_t skips;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( skips ));
  Wire.readBytes( (uint8_t*)&skips, sizeof( skips ));

  // Return result
  return skips;

}
ir_errors_t       SwarmB2_c::getRxErrors() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_ERRORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_errors_t errors;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( errors ));
  Wire.readBytes( (uint8_t*)&errors, sizeof( errors ));

  // Return result
  return errors;
}


ir_crc_t          SwarmB2_c::getRxCRC() {

  ir_mode_t ircomm;

  ircomm.mode = MODE_REPORT_CRC;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (uint8_t*)&ircomm, sizeof( ircomm));
  Wire.endTransmission();

  ir_crc_t crc;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( crc ));
  Wire.readBytes( (uint8_t*)&crc, sizeof( crc ));
  return crc;
}

ir_cycles_t       SwarmB2_c::getCycles() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_CYCLES;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_cycles_t cycles;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( cycles ));
  Wire.readBytes( (uint8_t*)&cycles, sizeof( cycles ));

  // Return result
  return cycles;
}
ir_sensors_t      SwarmB2_c::getSensors() {
  // Set correct more
  ir_mode_t ircomm;
  ircomm.mode = MODE_REPORT_SENSORS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm.mode, sizeof( ircomm.mode));
  Wire.endTransmission();

  // Get data
  ir_sensors_t sensors;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( sensors ));
  Wire.readBytes( (uint8_t*)&sensors, sizeof( sensors  ));

  // Return result
  return sensors;

}
