#include <NeoHWSerial.h>
#include <NeoHWSerial_private.h>
#include "ircomm.h"


#define DEBUG_LED 13

IRComm_c::IRComm_c() {
}

void IRComm_c::init() {

  disabled = false;

  // Let's set everything to 0 to start with
  memset( &config, 0, sizeof( config ));

  // Start by setting up the default config
  // set by the #define in ircomm.h, which
  // can then be over-rided by i2c.
  if ( TX_MODE == TX_MODE_PERIODIC ) {
    config.tx.flags.bits.mode           = TX_MODE_PERIODIC;

  } else if ( TX_MODE == TX_MODE_INTERLEAVED ) {
    config.tx.flags.bits.mode           = TX_MODE_INTERLEAVED;

  }
  config.tx.flags.bits.predict_period   = TX_PREDICT_PERIOD;
  config.tx.predict_multi               = TX_PREDICT_MULTI;
  config.tx.repeat                      = TX_DEFAULT_REPEAT;
  config.tx.flags.bits.desync           = TX_DESYNC;
  config.tx.flags.bits.defer            = TX_DEFER;
  config.tx.period_norm                 = TX_DEFAULT_PERIOD;
  config.tx.flags.bits.preamble         = TX_PREAMBLE;
  config.tx.len = 0;

  // Receiving Configuration
  config.rx.flags.bits.cycle            = RX_CYCLE;
  config.rx.flags.bits.cycle_on_rx      = RX_CYCLE_ON_RX;
  config.rx.flags.bits.predict_period   = RX_PREDICT_PERIOD;
  config.rx.flags.bits.overrun          = RX_OVERRUN;
  config.rx.flags.bits.desync           = RX_DESYNC;
  config.rx.flags.bits.desaturate       = RX_DESATURATE;
  config.rx.flags.bits.rx0              = 1;
  config.rx.flags.bits.rx1              = 1;
  config.rx.flags.bits.rx2              = 1;
  config.rx.flags.bits.rx3              = 1;
  config.rx.flags.bits.rand_rx          = RX_RAND_RX;
  config.rx.flags.bits.skip_inactive    = RX_SKIP_INACTIVE;
  config.rx.len                         = RX_DEFAULT_MSG_LEN;
  config.rx.skip_multi                  = RX_SKIP_MULTI;
  config.rx.period                      = 0; // updated below
  config.rx.period_norm                 = RX_DEFAULT_PERIOD;
  config.rx.predict_multi               = RX_PREDICT_MULTIPLIER;
  config.rx.index                       = 3;
  config.rx.byte_timeout                = RX_BYTE_TIMEOUT_MS;
  config.rx.sat_timeout                 = RX_SAT_TIMEOUT_US;

#ifdef IR_FREQ_38
  NeoSerial.begin( 4800 );
#endif

#ifdef IR_FREQ_56
  NeoSerial.begin( 9600 );
#endif

  // Debug led
  pinMode(13, OUTPUT);

  // RX Demodulator power pins
  pinMode(RX_PWR_0, OUTPUT);
  pinMode(RX_PWR_1, OUTPUT);
  pinMode(RX_PWR_2, OUTPUT);
  pinMode(RX_PWR_3, OUTPUT);
  powerOffAllRx();

  // Make sure we start with one receiver on
  // just in case we are configured not to
  // do cycleRx(), which otherwise toggles
  // rx state.
  powerOnRx( config.rx.index );


  // Set pin for 38khz/58khz carrier as output
  pinMode(TX_CLK_OUT, OUTPUT);


  // Set message buffers to invalid (empty) state
  memset(tx_buf, 0, sizeof(tx_buf));
  memset(ir_msg, 0, sizeof(ir_msg));



  config.tx.len = 0;             // start with no message to send.

  led_ts = millis();      // debug led
  rx_ts = millis();       // rx timeout
  tx_ts = millis();       // tx period
  bearing_ts = millis(); // bearing update

  resetUART();

  parser.reset();

  setRxPeriod();
  setTxPeriod();

  resetBearingActivity();
  resetMetrics();
}

void IRComm_c::resetMetrics() {
  memset( &metrics, 0, sizeof( metrics ));
  NeoSerial.resetFrameErrorCount();
  for ( int i = 0; i < MAX_RX; i++ ) {
    metrics.msg_timings.ts_ms[i] = millis();
    metrics.byte_timings.ts_us[i] = micros();
  }
}



// Used to activate the next IR receiver.
// Will either change sequentially or randomly
// depending on how the board is configured.
// Returns false if no cycling occured.
bool IRComm_c::cyclePowerRx() {

  // Pick a random receiver.  Using the rand() function
  // and a while loop, with a watchdog of 10 attempts.
  if ( config.rx.flags.bits.rand_rx == true ) {

    int watchdog = 0;
    int rx;
    do {
      rx = (int)random(0, 4);
      watchdog++;

    } while ( !isRxAvailable( rx ) && watchdog < 10 );

    // failed to find an available receiver, do nothing
    if ( watchdog == 10 ) {
      return false;
    }

    config.rx.index = rx;

  } else {

    // Try to do a rotation, checking if the rx
    // receiver is available/enabled.  We should
    // only need to do this 3 times before we get
    // back to the same receiver again (watchdog)
    int watchdog = 0;
    int rx = config.rx.index;
    do {
      rx++;
      if ( rx >= MAX_RX ) rx = 0;
      watchdog++;
    } while ( !isRxAvailable( rx ) && watchdog < 4 );

    // If count is 4, we returned to the same receiver
    // and so didn't really cycle
    if ( watchdog == 4 ) {
      return false;
    }

    config.rx.index = rx;

  }


  metrics.cycles.rx++;
  powerOnRx( config.rx.index );

  // Since we've cycled receiver,
  // set the new timeout period
  setRxPeriod();

  return true;
}

// To allow a user to disable specific rx 
// modules but retain the general power 
// cycling/polling behaviour.
bool IRComm_c::isRxAvailable( int which ) {
  switch ( which ) {
    case 0: return config.rx.flags.bits.rx0;
    case 1: return config.rx.flags.bits.rx1;
    case 2: return config.rx.flags.bits.rx2;
    case 3: return config.rx.flags.bits.rx3;
    default: return false;
  }
}


// Small wrapper for how to update the recorded activity
// level by some decay rate.
void IRComm_c::updateBearingActivity() {

  float sum = 0;
  for ( int i = 0; i < MAX_RX; i++ ) {
    sum += bearing_activity[i];
  }

  if ( sum > 0 ) {

    // Normalising and filtering
    for ( int i = 0; i < MAX_RX; i++ ) {
      metrics.vectors.rx[i] = (metrics.vectors.rx[i] * 0.3 ) + ((bearing_activity[i] / sum) * 0.7);
    }

  } else {

    // No activity? Just decay
    for ( int i = 0; i < MAX_RX; i++ ) {
      metrics.vectors.rx[i] = (metrics.vectors.rx[i] * 0.3 ); 
    }
  }

  // Update bearing estimate.
  metrics.bearing.sum  = sum;
  float x = (metrics.vectors.rx[0] - metrics.vectors.rx[2]);
  float y = (metrics.vectors.rx[1] - metrics.vectors.rx[3]);
  metrics.bearing.theta = atan2( y, x );
  metrics.bearing.mag = sqrt( pow(x, 2) + pow(y, 2));

  resetBearingActivity();
}
void IRComm_c::resetBearingActivity() {
  // Reset to allow to accumulate over next ACTIVITY_MS period
  for ( int i = 0; i < MAX_RX; i++ ) bearing_activity[i] = 0;
}

void IRComm_c::powerOnRx( byte index ) {


  if ( index == 0 && isRxAvailable(index) ) {// fwd
    digitalWrite( RX_PWR_0, HIGH ); 
    digitalWrite( RX_PWR_1, LOW );
    digitalWrite( RX_PWR_2, LOW );
    digitalWrite( RX_PWR_3, LOW );

  } else if ( index == 1 && isRxAvailable(index) ) {        // left
    digitalWrite( RX_PWR_0, LOW );
    digitalWrite( RX_PWR_1, HIGH );
    digitalWrite( RX_PWR_2, LOW );
    digitalWrite( RX_PWR_3, LOW );


  } else if ( index == 2 && isRxAvailable(index) ) {        // back
    digitalWrite( RX_PWR_0, LOW );
    digitalWrite( RX_PWR_1, LOW );
    digitalWrite( RX_PWR_2, HIGH );
    digitalWrite( RX_PWR_3, LOW );

  } else if ( index == 3 && isRxAvailable(index) ) {        // right
    digitalWrite( RX_PWR_0, LOW );
    digitalWrite( RX_PWR_1, LOW );
    digitalWrite( RX_PWR_2, LOW );
    digitalWrite( RX_PWR_3, HIGH );

  }
  
  // too quick?
  //delayMicroseconds(250);

  // Vishay stipulate 1-2ms for functional operation
  // but up to 20ms for stable filtering.
  delay(20);

  // After changing which receiver is active,
  // the NeoSerial buffer is full of old data.
  // We clear it now.
  resetUART();
  parser.reset();
}

// The arduino nano has a parallel NeoSerial
// interface,meaning with IR it can receive
// it's own tranmission. There, we access the
// UART register to disable RX functionality.
// When we re-enable it, this has the beneficial
// side-effect of clearing the input buffer.
void IRComm_c::disableRx() {
  cli();
  UCSR0B &= ~(1 << RXEN0);
  UCSR0B &= ~(1 << RXCIE0);
  sei();
}

// Re-enable the NeoSerial port RX hardware.
void IRComm_c::enableRx() {
  cli();
  UCSR0A &= ~(1 << FE0);
  UCSR0A &= ~(1 << DOR0);
  UCSR0A &= ~(1 << UPE0);
  UCSR0B |= (1 << RXEN0);
  UCSR0B |= (1 << RXCIE0);
  sei();
}


// Clears our received buffer and resets
// the variables used to read in bytes.
// calling disableRx and enableRx
// flushes the hardware buffer.
void IRComm_c::resetUART() {

  disableRx();
  delayMicroseconds(10);
  enableRx();


}

// TODO: watch out!  We set 4ms here, this might
//       need to scale depending on the IR demod
//       chip.  It takes 1.2ms to receive a byte
//       at 58khz, 2.2ms at 38khz?
void IRComm_c::toggleRxPower() {

  if ( config.rx.index == 0 ) {
    digitalWrite( RX_PWR_0, LOW );
    delayMicroseconds(1200); // 1.2ms
    digitalWrite( RX_PWR_0, HIGH );

  } else if ( config.rx.index == 1 ) {
    digitalWrite( RX_PWR_1, LOW );
    delayMicroseconds(1200); // 1.2ms
    digitalWrite( RX_PWR_1, HIGH );

  } else if ( config.rx.index == 2 ) {
    digitalWrite( RX_PWR_2, LOW );
    delayMicroseconds(1200); // 1.2ms
    digitalWrite( RX_PWR_2, HIGH );

  } else if ( config.rx.index == 3 ) {
    digitalWrite( RX_PWR_3, LOW );
    delayMicroseconds(1200); // 1.2ms
    digitalWrite( RX_PWR_3, HIGH );

  }
  // Wait to stabilise when on again
  //delayMicroseconds(50);
  resetUART();

}

void IRComm_c::powerOffAllRx() {
  digitalWrite( RX_PWR_0, LOW );
  digitalWrite( RX_PWR_1, LOW );
  digitalWrite( RX_PWR_2, LOW );
  digitalWrite( RX_PWR_3, LOW );
}


// I think it is very unadvised to do
// this because of the power draw from
// the arduino nano.
//void IRComm_c::powerOnAllRx() {
//  digitalWrite( RX_PWR_0, HIGH );
//  digitalWrite( RX_PWR_1, HIGH );
//  digitalWrite( RX_PWR_2, HIGH);
//  digitalWrite( RX_PWR_3, HIGH);
//}


// https://lucidar.me/en/NeoSerialib/most-used-baud-rates-table/
void IRComm_c::setRxPeriod() {

  // Assume max message len to start
  float t = (float)(config.rx.period_norm);

  // Is the board configured to dynamically adjust the
  // period value depending on the length of the
  // messages it is receiving?
  if ( config.rx.flags.bits.predict_period && config.rx.len > 0 && config.rx.predict_multi > 0 ) {

    // What is the length of the messages we are
    // receiving?
    t = (float)config.rx.len;

    // How many full message-lengths to listen for?
    t *= config.rx.predict_multi;

    // Scale for milliseconds
#ifdef IR_FREQ_56
    t *= MS_PER_BYTE_58KHZ; // How many ms per byte to transmit?
#endif

#ifdef IR_FREQ_38
    t *= MS_PER_BYTE_38KHZ; // How many ms per byte to transmit?
#endif



  }

    
  if ( config.rx.flags.bits.desync == true ) {
    float mod = t;
    mod *= 0.25; // take 25% of the total time
    // add modifier between 0: mod
    mod = (float)random(0, (long)mod);
    if ( mod > 0 ) mod = mod - (mod / 2.0); // center
    t += mod;
  }


  config.rx.period = (unsigned long)t;

  // If we've adjsuted the delay period,
  // we move the timestamp forwards.
  rx_ts = millis();
}

void IRComm_c::setTxPeriod() {

  // If configured to 0, do nothing
  if ( config.tx.period_norm == 0 ) return;

  // Set tx period as default
  float t = config.tx.period_norm;

  if ( config.tx.flags.bits.predict_period == true && config.tx.len > 0 && config.tx.predict_multi > 0 ) {
    t = (float)config.tx.len;
    t *= config.tx.predict_multi;

    // Scale for milliseconds per byte
#ifdef IR_FREQ_56
    t *= MS_PER_BYTE_58KHZ; // How many ms per byte to transmit?
#endif

#ifdef IR_FREQ_38
    t *= MS_PER_BYTE_38KHZ; // How many ms per byte to transmit?
#endif
  }

  if ( config.tx.flags.bits.desync == true ) {
    float mod = t;
    mod *= 0.25; // take 25% of the total time
    // add modifier between 0: mod
    mod = (float)random(0, (long)mod);
    if ( mod > 0 ) mod = mod - (mod / 2.0); // center
    t += mod;

  }


  config.tx.period = (unsigned long)t;

  // If we've adjsuted the delay period,
  // we move the timestamp forwards.
  tx_ts = millis();
}


// To stop NeoSerial transmission on the IR LEDs
// we stop timer2 by setting the clock source
// to 0.  We also set pin 4 to HIGH? LOW? to
// keep the IR LEDs off.
void IRComm_c::disableTx() {

  // Termporarily stop interupts
  cli();

  // disable t1 interrupts
  TIMSK1 = 0;

  // stop t1 clock
  TCCR1B = 0;

  // reset counter
  TCNT1 = 0;

  // Should we hold d4 high or low
  // here? 
  
  // enable interrupts
  sei();
}

void IRComm_c::enableTx() {
  setupTimer1();
}


// We use Timer2 to generate a 38khz clock
// which is electronically OR'd with the
// NeoSerial TX.
void IRComm_c::setupTimer1() {

  // Termporarily stop interupts
  cli();

  // Clear config
  TCCR1A  = 0;
  TCCR1B  = 0;
  TCNT1   = 0;

  // CTC Mode, Top = 0CR1A
  TCCR1B |= _BV(WGM12);

  // no prescale (16mhz)
  TCCR1B |= _BV(CS10);

  // Setup for 37khz, 4800 baud
#ifdef IR_FREQ_38
  // match value
  OCR1A = 209; 
#endif


  // Setup for 58khz, 9600 baud
#ifdef IR_FREQ_56
  // match value
  // TSDP34156 datasheet says 56Khz
  OCR1A = 142;
#endif

  // Clear any pending match
  TIFR1 |= _BV(OCF1A);

  // enable compare match interrupt
  TIMSK1 |= _BV(OCIE1A);
  
  // enable interrupts
  sei();
}

void IRComm_c::clearRxMsg(int which) {
  if ( which >= 0 && which < MAX_RX) {
    memset(ir_msg[which], 0, sizeof(ir_msg[which]));
    msg_len[which] = 0;
  }
}

// This clears out the message we are broadcasting
void IRComm_c::clearTxBuf() {
  memset(tx_buf, 0, sizeof(tx_buf));
  config.tx.len = 0;
}


/* main update.

   This function calls the ir parser to get the
   next byte, and if a message has arrived, to
   copy it into the correct i2c variables.

   This function is responsible for tracking
   the metrics of communication and for determining
   which actions to take (i.e. when to send, when
   to receive) based on the configuration of the
   board.

   This function is intended to be called iteratively
   and as fast as possible.

*/
bool IRComm_c::update() {


  // We periodically track update the activity
  // of each receiver to help estimate a bearing
  // of neighbouring boards.
  // Activity is simply bytes received
  if ( millis() - bearing_ts > UPDATE_BEARING_MS ) {
    bearing_ts = millis();
    updateBearingActivity();
  }

  // toggles off the LED if it was put on
  // somewhere else
  if ( millis() - led_ts > 100 ) {
    led_ts = millis();
    digitalWrite(DEBUG_LED, LOW );
  }

  // Assume we will not transmit or cycle
  // the receiver on this update().  Deciding
  // which to do and when is a bit complicated,
  // depending on how the board has been
  // configured (the main job of this update).
  // The precedence of transmit or receive is
  // determined by "drop through" the following
  // conditional statements against configuration
  bool transmit = false;
  bool cycle = false;
  bool skip = false; // to register if the cycle 
                      // is the results of a skip

  // Assume there is no byte activity
  bool activity = false;


  // We assume we always want to get the latest
  // byte from the UART buffer
  int status = parser.getNextByte( config.rx.byte_timeout );

  // Frame errors can happen even if a byte is not
  // received.  We update the counts here.  
  // I'm not sure if doing this very frequently will
  // interfere with the UART process (it suspends 
  // interrupts momentarily).
  metrics.frame_errors.rx[ config.rx.index ] += NeoSerial.getFrameErrorCount();
  NeoSerial.resetFrameErrorCount();


  // Use the value of status to update metrics and/or
  // to transfer a message into the i2c variables.
  // status  < 0 : error type defined in ir_parser.h
  // status == 0 : no bytes were received
  // status == 1 : 1 byte was received, not a full message
  // status  > 1 : a full message was received correctly
  if ( status < 0 ) { // log the error.

    // Note, if status equals byte timeout, it
    // will have reset itself, meaning that later
    // overrun will not occur (start byte no longer
    // registered as having been received).
    if ( status != -ERR_BYTE_TIMEOUT ) {

      // Register that there was some byte activity
      activity = true;
      
    } 

    // Increase count of fails for this rx
    metrics.crc.fail[ config.rx.index ]++;


    // Use status to index the log of errors.
    // Errors are returned here as negative sign.
    // We flip it to use it as the index for the
    // error type arrays
    status *= -1;

    // Our status is 1 to 4, but the
    // index is 0:3
    status -= 1;

    // Increase count of error type
    metrics.errors.type[ config.rx.index ][ status ]++;



  } else if ( status == 1 ) { // just got a byte

    // Register that there was some byte activity
    activity = true;

  } else if ( status > 1 ) { // got a message

    // Paul: I move this around a lot for debugging
    digitalWrite(13, HIGH);

    // Register that there was some byte activity
    activity = true;

    // Increase count of success for this rx
    metrics.crc.pass[ config.rx.index ]++;

    // The return value in status is the total number
    // of decoded bytes, including escaped characters.
    // This is what we need to know to attempt to
    // predict or optimise the time to wait when
    // listening for IR messages
    config.rx.len = status;

    // Record message length for i2c transfers
    msg_len[ config.rx.index ] = parser.msg_len;

    // Copy message into i2c buffer
    parser.copyMsg( ir_msg[ config.rx.index ] );

    // Paul: TO REMOVE
    // Try to read out an ID
    //    int id = atoi( ir_msg[ config.rx.index ] );
    //    if ( id > 0 && id < 4 ) {
    //      metrics.hist.id[id]++;
    //    }

    // Record timing statistics for messaging
    updateMsgTimestamp();

    // If the board is configured for interleaved transmission
    // that means the board will send after a message receipt.
    // If so, flag for transmission to happen at end of update().
    if ( config.tx.flags.bits.mode == TX_MODE_INTERLEAVED ) {
      transmit = true;

      // User may have configured the board to do interleaved
      // transmission but no cycling of receiver
      if ( config.rx.flags.bits.cycle == true ) {
        cycle = true;
      }
    }

    // User may have configured the board to cycle the receiver
    // only once a message has been received, but not
    // periodically (i.e. .cycle == false.
    if ( config.rx.flags.bits.cycle_on_rx ) {
      cycle = true;
    }

  }

  if ( activity ) { // We had some byte activity, log

    // Record timing statistics for byte activity
    updateByteTimestamp();

    //    digitalWrite(13, HIGH);
    bearing_activity[ config.rx.index ] += 1;
    metrics.activity.rx[ config.rx.index ]++;


  }

  
  // Track time between receiving messages and 
  // receiving bytes
  updateMsgElapsedTime();
  updateByteElapsedTime();
  
  if( !activity ) { // no activity

    // The user has configured the board to skip over any 
    // receivers which have no activity for a period of 
    // time.  Whilst we set cycle = true here, it is still
    // possible for this to be over-ruled later by the 
    // overrun check (if a message start has been received).
    if ( config.rx.flags.bits.skip_inactive == true ) {

#ifdef IR_FREQ_56
      if ( metrics.byte_timings.dt_us[config.rx.index] > (US_PER_BYTE_58KHZ * config.rx.skip_multi) ) {
#endif

#ifdef IR_FREQ_38
        if ( metrics.byte_timings.dt_us[config.rx.index] > (US_PER_BYTE_38KHZ * config.rx.skip_multi) ) {
#endif

          cycle = true;
          skip = true;
          
          
        }
      }





      // This is an attempt at a fix to a problem I haven't
      // fully diagnosed or solved yet.  It seems like the
      // AGC in the receiver can saturate or lower the gain
      // to 0, causing no bytes to be received.  If we find
      // that there is no activity for a long time, setting
      // desaturate = 1 will mean we attempt to power cycle
      // the receiver.
      if ( config.rx.flags.bits.desaturate == 1 ) {

        if ( metrics.byte_timings.dt_us[config.rx.index] > (unsigned long)config.rx.sat_timeout ) {

          toggleRxPower();

          // Advance this byte time stamp so
          // that we don't immediately trigger
          // again.
          metrics.byte_timings.ts_us[config.rx.index] = micros();

          // Add to our count of saturation
          // occurences.
          metrics.saturation.rx[config.rx.index]++;
        }
      }
    }

    //
    //  if ( disabled == true ) {
    //    return activity;
    //  }



    // overrun == true: The board is configured to finish receiving
    // once the start byte has been received
    if ( config.rx.flags.bits.overrun && parser.rx_state != RX_WAIT_START ) {

      // Prevent cycling or transmission
      transmit = false;
      cycle = false;

      // If in TX_MODE_PERIODIC, TX takes priority
    } else if ( config.tx.flags.bits.mode == TX_MODE_PERIODIC ) {

      //
      if ( config.tx.period == 0 || config.tx.period_norm == 0) {
        // No transmission.
      } else if ( millis() - tx_ts > config.tx.period ) {
        transmit = true;
      }
    }

    // We are in either TX_MODE_INTERLEAVED or
    // PERIODIC, and we need to check if it is time
    // to rotate the RX receiver.
    if ( config.rx.flags.bits.cycle == true ) {


      // Has the rx period passed?
      if ( millis() - rx_ts > config.rx.period) {

        if ( config.tx.flags.bits.mode == TX_MODE_INTERLEAVED ) {
          transmit = true;
        }

        if ( config.rx.period_norm > 0) {
          cycle = true;
        }

      }
    }


    // defer == true: recent byte activity will mean
    // that the transmission is deferred (cancelled)
    if ( config.tx.flags.bits.defer ) {

      // Was the last byte activity within the time
      // expected?
#ifdef IR_FREQ_56
      if ( micros() - metrics.byte_timings.ts_us[ config.rx.index ] < US_PER_BYTE_58KHZ * 2 ) {
#endif
#ifdef IR_FREQ_38
        if ( micros() - metrics.byte_timings.ts_us[ config.rx.index ] < US_PER_BYTE_38KHZ * 2 ) {
#endif

          transmit = false;
        }
      }


      if ( transmit ) {

        // Interupts any rx in process
        if ( doTransmit() ) {
          // It takes times to transmit and
          // the UART rx buffer is reset.
          // So we should also reset the
          // parser.
          parser.reset();

          // Update timestamp for the next
          // transmission occurence
          setTxPeriod();

          // Transmitting will have taken 
          // time out of our receiving period
          // too, so update those timestamps
          setRxPeriod();

          // If we did a transmission, our
          // saturation/inactivty timeout may
          // trigger because of the time
          // spent transmitting. So we
          // update all the byte timestamps
          // here.
          advanceByteTimestamps();
        }

      }


      
      

      if ( cycle ) {

        // This handles rotation of the
        // receiver with respect to the
        // config settings.
        // This sets the new rx timestamp
        if( skip ) metrics.skips.rx[config.rx.index]++;  
        
        if ( cyclePowerRx() ) {
          
          parser.reset();
          advanceByteTimestamps();
        }

      }



    }

    // Sometimes a blocking process will have
    // prevented our timestamps progressing.
    // For messages, we leave this alone as we
    // consider blocking processes and their
    // effect on the messaging performance a
    // matter of interest.
    // But for byte activity, we are using this
    // to detect receiver saturation.
    void IRComm_c::advanceByteTimestamps() {
      for ( int i = 0; i < 4; i++ ) metrics.byte_timings.ts_us[i] = micros();
    }

    // Note: in milliseconds.  At 58khz, it
    // takes about 1.2ms to receive 1 byte,
    // and the minimum message is 5 bytes
    void IRComm_c::updateMsgTimestamp() {
      metrics.msg_timings.ts_ms[ config.rx.index ] = millis();
    }
    void IRComm_c::updateMsgElapsedTime() {
      unsigned long dt = millis() - metrics.msg_timings.ts_ms[ config.rx.index ];
      metrics.msg_timings.dt_ms[ config.rx.index ] = dt;
    }
    void IRComm_c::advanceMsgTimestamps() {
      for ( int i = 0; i < 4; i++ ) {
        metrics.msg_timings.ts_ms[ i ] = millis();
      }
    }

    // Note: in microseconds. It is possible
    // to receive a byte in 1.2ms, so millis
    // isn't quite precise enough.
    void IRComm_c::updateByteTimestamp() {
      metrics.byte_timings.ts_us[ config.rx.index ] = micros();
    }

    void IRComm_c::updateByteElapsedTime() {
      unsigned long dt = micros() - metrics.byte_timings.ts_us[ config.rx.index ];
      metrics.byte_timings.dt_us[ config.rx.index ] = dt;
    }


    boolean IRComm_c::doTransmit() {

      // Don't do anything if there is no message
      // to transmit.
      if ( config.tx.len == 0 ) {

        // Schedule next transmission
        // Redundant if set to INTERLEAVED
        setTxPeriod();
        return false;
      } else {



        // Stop receiving
        disableRx();
        enableTx();


        // Transmission might be improved with some message
        // pre-amble that allows the receiving UART to
        // determine the clock of the source.
        if ( config.tx.flags.bits.preamble == 1 ) {

          // Add some preamble bytes. 0x55 = 0b01010101
          for ( int i = 0; i < TX_PREAMBLE_REPEAT; i++ ) {
            NeoSerial.write( TX_PREAMBLE_BYTE );
          }

        }

        // Using NeoSerial.print transmits over
        // IR.  NeoSerial TX is modulated with
        // the 38Khz or 58khz carrier in hardware.
        //unsigned long start_t = micros();
        for ( int i = 0; i < config.tx.repeat; i++ ) {

          // Checking HardwareNeoSerial.cpp, .write() is a blocking
          // function.  Therefore we don't need .flush()
          //NeoSerial.availableForWrite();
          for ( int j = 0; j < config.tx.len; j++ ) {
            NeoSerial.write( tx_buf[j] );
          }

          //NeoSerial.print(tx_buf);
          NeoSerial.flush();  // wait for send to complete
          metrics.cycles.tx++;
        }

        // Since we used disableRx(), we need to
        // re-enable the UART and so clear
        // the rx flags and rx_buf
        
        disableTx();
        
        resetUART();

        // Schedule next transmission
        // Redundant if tx_mode INTERLEAVED
        setTxPeriod();
        return true;

      }

      return false;

    }


    void IRComm_c::fullReset() {


      for ( int i = 0; i < 4; i++ ) {
        bearing_activity[i] = 0;
        clearRxMsg(i);
      }
      resetMetrics();
      clearTxBuf();
      resetUART();
      parser.reset();
    }

    // The smallest ISR I've written :)
    // Toggles the state of
    // pin D4 to generate a 38khz or 56khz
    // carrier signal.
    ISR(TIMER1_COMPA_vect) {
      PORTD ^= (1 << PD4);
    }
