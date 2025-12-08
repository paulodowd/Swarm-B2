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

  config.tx.repeat                      = DEFAULT_TX_REPEAT;
  config.tx.flags.bits.desync           = TX_DESYNC;
  config.tx.flags.bits.defer            = TX_DEFER;
  config.tx.period_max                  = DEFAULT_TX_PERIOD;
  config.tx.flags.bits.preamble         = TX_PREAMBLE;
  config.tx.len = 0;

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
  config.rx.period                      = 0; // updated below
  config.rx.period_max                  = RX_PERIOD_MAX;
  config.rx.predict_multi               = RX_PREDICT_MULTIPLIER;
  config.rx.index                       = 3;
  config.rx.byte_timeout                = RX_BYTE_TIMEOUT_MS;
  config.rx.sat_timeout                 = RX_SAT_TIMEOUT_US;



#ifdef IR_FREQ_38
  Serial.begin( 4800 );
#endif

#ifdef IR_FREQ_58
  Serial.begin( 9600 );
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


  //enableTx();
  resetUART();

  parser.reset();

  setRxPeriod();
  setTxPeriod();

  resetBearingActivity();
  resetMetrics();


}

void IRComm_c::resetMetrics() {
  memset( &metrics, 0, sizeof( metrics ));
  memset( &metrics.status, 0, sizeof( metrics.status ));

  for ( int i = 0; i < MAX_RX; i++ ) {
    metrics.msg_timings.ts_ms[i] = millis();
    metrics.byte_timings.ts_us[i] = micros();
  }
}

void IRComm_c::printTxMsgForDebugging() {
  for ( int i = 0; i < config.tx.len; i++ ) {
    if ( i == 1 ) {

      Serial.print( tx_buf[i], DEC);
      Serial.print("(L) " );

    } else {
      Serial.print( tx_buf[i], HEX);
      Serial.print("(");
      Serial.print( (char)tx_buf[i] );
      Serial.print( ") " );
    }
  }
  Serial.println();

}
void IRComm_c::printMetricsForDebugging() {
  disableRx();
  for ( int i = 0; i < 4; i++ ) { // rx
    Serial.print("Rx " ); Serial.print( i ); Serial.println();
    for ( int j = 0; j < 4; j++ ) {
      Serial.print("Type "); Serial.print(j); Serial.print(": ");
      Serial.println( metrics.errors.type[i][j] );
    }

  }
  enableRx();
}

void IRComm_c::printRxMsgForDebugging() {
  for ( int j = 0; j < 4; j++ ) {
    if ( msg_len[j] > 0 ) {
      disableRx();
      Serial.print("Rx");
      Serial.print( j );
      Serial.print(": ");
      for ( int i = 0; i < msg_len[j]; i++ ) {

        Serial.print( (char)ir_msg[j][i]);

      }
      Serial.println();
      Serial.flush();
      clearRxMsg(j);
      enableRx();
    }
  }

}


bool IRComm_c::cyclePowerRx() {

  // If the board is configured not to cycle
  // the receiver, we simply update the
  // continue functioning, without changing
  // to a new receiver.
  //  if ( config.rx.flags.bits.cycle == false || config.rx.period_max == 0 ) {
  if ( config.rx.period_max == 0 ) {

    // If we are not cycling the RX receiver
    // there seems to be an issue where the demod
    // chip saturates (or something?).  There isn't
    // a way to detect when this has happened.
    // When this happens, no bytes are received.
    // The odd thing is that waving a hand in front of
    // the demod chip gets it working again.
    // I thought it was an issue with the UART
    // losing it's synchronisation, but simply
    // resetting UART doesn't solve the issue.
    // If the board is cycling the rx it fixes
    // the issue.
    // Although this isn't ideal, it is better
    // than receiving no bytes at all.
    //toggleRxPower(); // Paul 5/7/25 this is causing trouble
    setRxPeriod();
    return false;
  }

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
    // receiver is available/enabled
    int watchdog = 0;
    int rx = config.rx.index;
    do {
      rx++;
      if ( rx >= MAX_RX ) rx = 0;
      watchdog++;
    } while ( !isRxAvailable( rx ) && watchdog < 4 );

    // If count is 4, we returned to the same receiver
    // and so, didn't really cycle
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

// I was tempted to do some bitwise things here,
// but if the union definition changes it would
// screw up this routine.
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
      metrics.vectors.rx[i] = (metrics.vectors.rx[i] * 0.3 ); // + ( 0 * 0.7);
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


  if ( index == 0 && isRxAvailable(index) ) {
    digitalWrite( RX_PWR_0, HIGH ); // fwd
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
  delayMicroseconds(250);

  // After changing which receiver is active,
  // the serial buffer is full of old data.
  // We clear it now.
  resetUART();
  parser.reset();
}

// The arduino nano has a parallel serial
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

// Re-enable the serial port RX hardware.
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

  //  // Flush?
  //  unsigned char dummy;
  //  while (UCSR0A & (1 << RXC0)) dummy = UDR0;

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

// Attempting an desynchronous send
// and listen procedure because we
// can't do both at the same time.
// https://lucidar.me/en/serialib/most-used-baud-rates-table/
// We are using 4800 baud, which is
// 4800 bits per second.
// 1.042ms per byte.
void IRComm_c::setRxPeriod() {

  // Assume max message len to start
  float t = (float)(config.rx.period_max);

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
#ifdef IR_FREQ_58
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
  if ( config.tx.period_max == 0 ) return;

  // Set tx period as default
  float t = config.tx.period_max;

  if ( config.tx.flags.bits.predict_period == true && config.tx.len > 0 && config.tx.predict_multi > 0 ) {
    t = (float)config.tx.len;
    t *= config.tx.predict_multi;

    // Scale for milliseconds per byte
#ifdef IR_FREQ_58
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


// To stop serial transmission on the IR LEDs
// we stop timer2 by setting the clock source
// to 0.  We also set pin 4 to HIGH? LOW? to
// keep the IR LEDs off.
void IRComm_c::disableTx() {

  // Termporarily stop interupts
  cli();
  // clear prescale on clock source
  TCCR2B = 0;

  // enable interrupts
  sei();
}

void IRComm_c::enableTx() {
  setupTimer2();
}


// We use Timer2 to generate a 38khz clock
// which is electronically OR'd with the
// serial TX.
void IRComm_c::setupTimer2() {

  // Termporarily stop interupts
  cli();

  // Setup Timer 2 to fire ISR every 38khz
  // Enable CTC mode
  TCCR2A = 0;
  TCCR2A |= (1 << WGM21);

  // Setup ctc prescaler
  TCCR2B = 0;

  // No prescale - therefore "counting" at the
  // system clock of 16Mhz (16,000,000)
  TCCR2B |= (1 << CS20);

  // Setup for 37khz, 4800 baud
#ifdef IR_FREQ_38
  // match value
  // TSDP34138 datasheet says 38.4Khz
  // Therefore, 16000000/208 = 76923.07
  // We need to toggle on then off.
  // 76923.07/2 = 38461.53hz (38.4khz)
  OCR2A = 208; // was 210?
#endif


  // Setup for 58khz, 9600 baud
#ifdef IR_FREQ_58
  // match value
  // TSDP34156 datasheet says 57.6Khz
  // Therefore, 16000000/139 = 115107.9
  // We need to toggle on then off.
  // 115107.9/2 = 57553.96hz (57.6khz)
  OCR2A = 139;
#endif

  // Set up which interupt flag is triggered
  TIMSK2 = 0;
  TIMSK2 |= (1 << OCIE2A);

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

   This function is written mainly as a drop-
   through if statement.  I thought about a
   state machine, but this drop through seemed
   like the easiest to understand and to have
   the fewest repetitive function calls.

   The drop-through is ordered with the
   precedence of the board configuration.
   By the end of the drop-through, the flags
   "cycle" and "transmit" will be appropriately
   set, and so cycleRxPower() and/or
   doTransmit() are triggered.

   After asking the parser class to read in
   the next byte, this update function will
   log any error metrics and update timings.

*/
bool IRComm_c::update() {


  bool activity = false;

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
  bool transmit = false;
  bool cycle = false;

  // We assume we always want to get the latest
  // byte from the UART buffer
  int status = parser.getNextByte( config.rx.byte_timeout );

  if ( status < 0 ) { // something went wrong.

    // Increase count of fails for this rx
    metrics.status.fail_count[ config.rx.index ]++;


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

    // Register that there was some byte activity
    activity = true;

  } else if ( status == 1 ) { // just got a byte

    // Register that there was some byte activity
    activity = true;


  } else if ( status > 1 ) { // got a message

    // Register that there was some byte activity
    activity = true;

    // Increase count of success for this rx
    metrics.status.pass_count[ config.rx.index ]++;

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

    // Paul: REMOVE LATER
    // Try to read out an ID
    //    int id = atoi( ir_msg[ config.rx.index ] );
    //    if ( id > 0 && id < 4 ) {
    //      metrics.hist.id[id]++;
    //    }

    // Record timing statistics for messaging
    updateMsgTimings();

    if ( config.tx.flags.bits.mode == TX_MODE_INTERLEAVED ) {
      transmit = true;
      if ( config.rx.flags.bits.cycle == true ) {
        cycle = true;
      }
    }

    if ( config.rx.flags.bits.cycle_on_rx && config.rx.flags.bits.cycle) {
      cycle = true;
    }

  }

  if ( activity ) {

    // Record timing statistics for byte activity
    updateByteTimings();


    //    digitalWrite(13, HIGH);
    bearing_activity[ config.rx.index ] += 1;
    metrics.status.activity[ config.rx.index ]++;


  } else { // no activity

    // Although we track a dt for the bytes
    // of each receiver, the dt is updated
    // when a byte is received. Here we are
    // checking specifically when a byte
    // hasn't been received.
    if ( config.rx.flags.bits.desaturate == 1 ) {
      unsigned long dt = micros();
      dt -= (unsigned long)metrics.byte_timings.ts_us[config.rx.index];
      if ( dt > (unsigned long)config.rx.sat_timeout ) {m
        toggleRxPower();

        // Advance this byte time stamp so
        // that we don't immediately trigger
        // again.
        metrics.byte_timings.ts_us[config.rx.index] = micros();

        // Add to our count of saturation
        // occurences.
        metrics.status.saturation[config.rx.index]++;
      }
    }
  }


  if ( disabled == true ) {
    return activity;
  }



  // overrun == true: The board is configured to finish receiving
  // once the start byte has been received
  if ( config.rx.flags.bits.overrun && parser.rx_state != RX_WAIT_START ) {

    // Prevent cycling or transmission
    transmit = false;
    cycle = false;

    // If in TX_MODE_PERIODIC, TX takes priority
  } else if ( config.tx.flags.bits.mode == TX_MODE_PERIODIC ) {

    //
    if ( config.tx.period == 0 || config.tx.period_max == 0) {
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

      if ( config.rx.period_max > 0) {
        cycle = true;
      }

    }
  }


  // defer == true: recent byte activity will mean
  // that the transmission is deferred (cancelled)
  if ( config.tx.flags.bits.defer ) {

    // Was the last byte activity within the time
    // expected?
#ifdef IR_FREQ_58
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

        // If we did a transmission, our
        // saturation timeout will likely
        // trigger because of the time
        // spent transmitting. So we
        // update the saturation timestamp
        // here.
        advanceTimings();
      }

    }

    if ( cycle ) {

      // This handles rotation of the
      // receiver with respect to the
      // config settings.
      // This sets the new rx timestamp
      if ( cyclePowerRx() ) {
        parser.reset();
        advanceTimings();
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
  void IRComm_c::advanceTimings() {
    for ( int i = 0; i < 4; i++ ) metrics.byte_timings.ts_us[i] = micros();
  }

  // Note: in milliseconds.  At 58khz, it
  // takes about 1.2ms to receive 1 byte,
  // and the minimum message is 5 bytes
  void IRComm_c::updateMsgTimings() {
    unsigned long dt = millis() - metrics.msg_timings.ts_ms[ config.rx.index ];
    metrics.msg_timings.dt_ms[ config.rx.index ] = dt;
    metrics.msg_timings.ts_ms[ config.rx.index ] = millis();
  }

  // Note: in microseconds. It is possible
  // to receive a byte in 1.2ms, so millis
  // isn't quite precise enough.
  void IRComm_c::updateByteTimings() {
    unsigned long dt = micros() - metrics.byte_timings.ts_us[ config.rx.index ];
    metrics.byte_timings.dt_us[ config.rx.index ] = dt;
    metrics.byte_timings.ts_us[ config.rx.index ] = micros();
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
          Serial.write( TX_PREAMBLE_BYTE );
        }

      }

      // Using Serial.print transmits over
      // IR.  Serial TX is modulated with
      // the 38Khz or 58khz carrier in hardware.
      //unsigned long start_t = micros();
      for ( int i = 0; i < config.tx.repeat; i++ ) {

        // Checking HardwareSerial.cpp, .write() is a blocking
        // function.  Therefore we don't need .flush()
        //Serial.availableForWrite();
        for ( int j = 0; j < config.tx.len; j++ ) {
          Serial.write( tx_buf[j] );
        }

        //Serial.print(tx_buf);
        Serial.flush();  // wait for send to complete
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
  // This ISR simply toggles the state of
  // pin D4 to generate a 38khz or 58khz
  // carrier signal.
  ISR(TIMER2_COMPA_vect) {
    PORTD ^= (1 << PD4);
  }
