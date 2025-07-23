#include "ircomm.h"


#define DEBUG_LED 13

IRComm_c::IRComm_c() {
}

void IRComm_c::init() {

  disabled = false;

  // Start by setting up the default config
  // set by the #define in ircomm.h, which
  // can then be over-rided by i2c.
  if ( TX_MODE == TX_MODE_PERIODIC ) {
    config.tx.mode       = TX_MODE_PERIODIC;
  } else if ( TX_MODE == TX_MODE_INTERLEAVED ) {
    config.tx.mode       = TX_MODE_INTERLEAVED;
  }

  config.tx.repeat     = DEFAULT_TX_REPEAT;
  config.tx.desync     = TX_DESYNC;
  config.tx.period_max = DEFAULT_TX_PERIOD;

  config.rx.cycle           = RX_CYCLE;
  config.rx.cycle_on_rx     = RX_CYCLE_ON_RX;
  config.rx.predict_timeout = RX_PREDICT_TIMEOUT;
  config.rx.overrun         = RX_OVERRUN;
  config.rx.len          = RX_DEFAULT_MSG_LEN;
  config.rx.timeout         = 0; // updated below
  config.rx.timeout_max     = RX_TIMEOUT_MAX;
  config.rx.timeout_multi   = RX_PREDICT_MULTIPLIER;
  config.rx.index           = 0;
  config.rx.byte_timeout    = MS_BYTE_TIMEOUT;
  config.rx.desync          = RX_DESYNC;




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


  resetBearingActivity();
  resetMetrics();

  tx_len = 0;             // start with no message to send.

  led_ts = millis();      // debug led
  rx_ts = millis();       // rx timeout
  tx_ts = millis();       // tx period
  bearing_ts = millis(); // bearing update


  //enableTx();
  resetUART();

  parser.reset();

  setRxTimeout();
  setTxPeriod();

}

void IRComm_c::resetMetrics() {
  memset( &metrics, 0, sizeof( metrics ));

  for ( int i = 0; i < MAX_RX; i++ ) metrics.timings.msg_t[i] = millis();
}


bool IRComm_c::cyclePowerRx() {

  // If the board is configured not to cycle
  // the receiver, we simply update the
  // continue functioning, without changing
  // to a new receiver.
  if ( config.rx.cycle == false || config.rx.timeout_max == 0 ) {

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
    setRxTimeout();
    return false;
  }


  config.rx.index++;
  metrics.cycles.rx++;

  // A full rotation has occured, process the
  // activity level, update the lpf
  if ( config.rx.index >= MAX_RX ) {
    config.rx.index = 0;
  }


  powerOnRx( config.rx.index );

  // Since we've cycled receiver,
  // set the new timeout period
  setRxTimeout();

  return true;
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


  if ( index == 0 ) {
    digitalWrite( RX_PWR_0, HIGH ); // fwd
    digitalWrite( RX_PWR_1, LOW );
    digitalWrite( RX_PWR_2, LOW );
    digitalWrite( RX_PWR_3, LOW );

  } else if ( index == 1 ) {        // left
    digitalWrite( RX_PWR_0, LOW );
    digitalWrite( RX_PWR_1, HIGH );
    digitalWrite( RX_PWR_2, LOW );
    digitalWrite( RX_PWR_3, LOW );


  } else if ( index == 2 ) {        // back
    digitalWrite( RX_PWR_0, LOW );
    digitalWrite( RX_PWR_1, LOW );
    digitalWrite( RX_PWR_2, HIGH );
    digitalWrite( RX_PWR_3, LOW );

  } else if ( index == 3 ) {        // right
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
    delayMicroseconds(250); // 0.25ms
    digitalWrite( RX_PWR_0, HIGH );

  } else if ( config.rx.index == 1 ) {
    digitalWrite( RX_PWR_1, LOW );
    delayMicroseconds(250); // 0.25ms
    digitalWrite( RX_PWR_1, HIGH );
  } else if ( config.rx.index == 2 ) {
    digitalWrite( RX_PWR_2, LOW );
    delayMicroseconds(250); // 0.25ms
    digitalWrite( RX_PWR_2, HIGH );
  } else if ( config.rx.index == 3 ) {
    digitalWrite( RX_PWR_3, LOW );
    delayMicroseconds(250); // 0.25ms
    digitalWrite( RX_PWR_3, HIGH );
  }
  // Wait to stabilise when on again
  delayMicroseconds(250);

  // Potentially here, we need to reset
  // the UART
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
void IRComm_c::setRxTimeout() {

  float t;

  // Is the board configured to dynamically adjust the
  // rx_timeout value depending on the length of the
  // messages it is receiving?
  if ( config.rx.predict_timeout && config.rx.len > 0 ) {

    // What is the length of the messages we are
    // receiving?
    t = (float)config.rx.len;

    // How many full message-lengths to listen for?
    t *= (float)config.rx.timeout_multi;

    // Scale for milliseconds
#ifdef IR_FREQ_58
    t *= MS_PER_BYTE_58KHZ; // How many ms per byte to transmit?
#endif

#ifdef IR_FREQ_38
    t *= MS_PER_BYTE_38KHZ; // How many ms per byte to transmit?
#endif



  } else { // Not using predict timeout

    // Use global and fiRxed parameters
    // to calculate a value
    t = (float)(config.rx.timeout_max);

  }

  if ( config.rx.desync == true ) {
    float mod = t;
    mod *= 0.25; // take 25% of the total time
    // add modifier between 0: mod
    mod = (float)random(0, (long)mod);
    if ( mod > 0 ) mod = mod - (mod / 2.0); // center
    t += mod;
  }


  config.rx.timeout = (unsigned long)t;

  // If we've adjsuted the delay period,
  // we move the timestamp forwards.
  rx_ts = millis();
}

void IRComm_c::setTxPeriod() {

  // If configured to 0, do nothing
  if ( config.tx.period_max == 0 ) return;

  // Set tx period as default
  float t = config.tx.period_max;

  if ( config.tx.desync == true ) {


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
// and adds a ! to the 0th character
void IRComm_c::clearTxBuf() {
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_len = 0;
}


int IRComm_c::update() {


  int error;

  // We periodically track update the activity
  // of each receiver to help estimate a bearing
  // of neighbouring boards.
  // Activity is simply bytes received
  if ( millis() - bearing_ts > UPDATE_BEARING_MS ) {
    bearing_ts = millis();
    updateBearingActivity();
  }

  if ( millis() - led_ts > 100 ) {
    led_ts = millis();
    digitalWrite(DEBUG_LED, LOW );

  }



  bool transmit = false;
  bool cycle = false;

  // We assume we always want to get the latest
  // byte from the UART buffer

  int status = parser.getNextByte( config.rx.byte_timeout );




  if ( status == 0 ) {
    // nothing happened
    error = 0;

  } else if ( status < 0 ) { // something went wrong.

    // Increase count of success for this rx
    metrics.status.fail_count[ config.rx.index ]++;


    // Use status to index the log of errors
    status *= -1;

    metrics.errors.type[ config.rx.index][ status ]++;

    error = -1;

  } else if ( status == 1 ) { // just got a byte
    // counts are incremented outside this if
    // statement
    error = 1;

  } else if ( status > 1 ) { // got a message

    error = status;

    // Increase count of success for this rx
    metrics.status.pass_count[ config.rx.index ]++;

    // Record how long the message was for
    // rx_predict_timeout to use.  This includes
    // the start, len and CRCx2 bytes (+4)
    config.rx.len = status;

    // Record message length for i2c transfers
    msg_len[ config.rx.index ] = parser.msg_len;

    // Copy message into i2c buffer
    parser.copyMsg( ir_msg[ config.rx.index ] );

    // Record timing statistics for messaging
    updateMsgTimings();

    // Reset the parser ready for the next message
    parser.reset();

    if ( config.tx.mode == TX_MODE_INTERLEAVED ) {
      transmit = true;
      if ( config.rx.cycle == true ) {
        cycle = true;
      }
    }

    if ( config.rx.cycle_on_rx && config.rx.cycle) {
      cycle = true;
    }

  }

  if ( status > 0 ) {
    digitalWrite(13, HIGH);
    bearing_activity[ config.rx.index ] += 1;
    metrics.status.activity[ config.rx.index ]++;
  }



  if ( disabled == true ) {
    return error;
  }



  if ( config.rx.overrun && parser.GOT_START_TOKEN ) {

    // If we're configured to allow RX to overrun and
    // we're in the process of receiving a message, we
    // just return this function.  That means that no
    // transmission or RX rotation will occur on this
    // iteration.
    // If we are receiving in error, eventually the
    // parser will flag an error and that will be the
    // end of this condition, allowing other operations

    // to resume.
    error = -3;
    return error;

    // If in TX_MODE_PERIODIC, TX takes priority
  } else if ( config.tx.mode == TX_MODE_PERIODIC ) {

    if ( config.tx.period == 0 || config.tx.period_max == 0) {
      // No transmission.
    } else if ( millis() - tx_ts > config.tx.period ) {
      transmit = true;
    }
  }

  // We are in either TX_MODE_INTERLEAVED or
  // PERIODIC, and we need to check if it is time
  // to rotate the RX receiver.
  if ( config.rx.cycle == true ) {
    if ( millis() - rx_ts > config.rx.timeout) {

      if ( config.tx.mode == TX_MODE_INTERLEAVED ) {
        transmit = true;
      }

      if ( config.rx.timeout_max > 0) {
        cycle = true;
      }

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
    }

  }

  if ( cycle ) {

    // This handles rotation of the
    // receiver with respect to the
    // config settings.
    // This sets the new rx timestamp
    if ( cyclePowerRx() ) {
      parser.reset();
    }

  }

}

void IRComm_c::updateMsgTimings() {
  unsigned long dt = millis() - metrics.timings.msg_t[ config.rx.index ];
  metrics.timings.msg_dt[ config.rx.index ] = dt;
  metrics.timings.msg_t[ config.rx.index ] = millis();
}

boolean IRComm_c::doTransmit() {

  // Don't do anything if there is no message
  // to transmit.
  if ( tx_len == 0 ) {

    // Schedule next transmission
    // Redundant if set to INTERLEAVED
    setTxPeriod();
    return false;
  } else {



    // Stop receiving
    disableRx();
    enableTx();
    // Using Serial.print transmits over
    // IR.  Serial TX is modulated with
    // the 38Khz carrier in hardware.
    //unsigned long start_t = micros();
    for ( int i = 0; i < config.tx.repeat; i++ ) {

      // Checking HardwareSerial.cpp, .write() is a blocking
      // function.  Therefore we don't need .flush()
      //Serial.availableForWrite();
      for ( int j = 0; j < tx_len; j++ ) {
        Serial.write( tx_buf[j] );
      }

      //Serial.print(tx_buf);
      Serial.flush();  // wait for send to complete
      metrics.cycles.tx++;
    }
    //Serial.println( (micros() - s ) );
    //unsigned long end_t = micros();
    //Serial.println( (end_t - start_t ) );

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

  parser.reset();
}

// This ISR simply toggles the state of
// pin D4 to generate a 38khz clock signal.
ISR(TIMER2_COMPA_vect) {
  PORTD ^= (1 << PD4);
}
