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
    ir_config.tx_mode   = TX_MODE_PERIODIC;
    ir_config.tx_repeat = DEFAULT_TX_REPEAT;
    ir_config.tx_period = DEFAULT_TX_PERIOD;

  } else if ( TX_MODE == TX_MODE_INTERLEAVED ) {
    ir_config.tx_mode   = TX_MODE_INTERLEAVED;
    ir_config.tx_repeat = DEFAULT_TX_REPEAT;
    ir_config.tx_period = 0; // updated below

  } else {
    // You need to set TX_MODE in ircomm.h
  }

  ir_config.rx_cycle            = RX_CYCLE;
  ir_config.rx_cycle_on_rx      = RX_CYCLE_ON_RX;
  ir_config.rx_predict_timeout  = RX_PREDICT_TIMEOUT;
  ir_config.rx_overrun          = RX_OVERRUN;
  ir_config.rx_length           = RX_DEFAULT_MSG_LEN;
  ir_config.rx_timeout          = 0; // updated below
  ir_config.rx_timeout_multi    = RX_PREDICT_MULTIPLIER;
  ir_config.rx_pwr_index        = 0;
  ir_config.rx_byte_timeout     = MS_BYTE_TIMEOUT;


  setRxTimeout();
  setTxPeriod();


#ifdef IR_FREQ_38
  Serial.begin( 4800 );
#endif

#ifdef IR_FREQ_58
  Serial.begin( 9600 );
#endif

  //reportConfiguration();

  // Debug led
  pinMode(13, OUTPUT);

  for ( int i = 0; i < 4; i++ ) {
    for ( int j = 0; j < 4; j++ ) {
      error_type[i][j] = 0;
    }
  }



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
  powerOnRx( ir_config.rx_pwr_index );


  // Set pin for 38khz/58khz carrier as output
  pinMode(TX_CLK_OUT, OUTPUT);


  // Set message buffers to invalid (empty) state
  memset(tx_buf, 0, sizeof(tx_buf));
  memset(rx_buf, 0, sizeof(rx_buf));
  memset(i2c_msg, 0, sizeof(i2c_msg));

  // sets up Timer2 to create 38khz/58khz carrier
  enableTx();
  //disableTx();


  // Set initial counts to 0
  for ( int i = 0; i < RX_PWR_MAX; i++ ) {
    pass_count[i] = 0;
    fail_count[i] = 0;

    activity[i] = 0;
    rx_activity[i] = 0;

    msg_dt[i] = 0;
    msg_t[i] = millis();

    msg_len[i] = 0;

  }

  msg_dir = 0.0;
  tx_count = 0;
  rx_cycles = 0;
  tx_len = 0;             // start with no message to send.
  led_ts = millis();      // debug led
  rx_ts = millis();       // rx timeout
  tx_ts = millis();       // tx period
  activity_ts = millis(); // bearing update
  byte_ts = millis();     // byte timeout

  resetRxProcess();
}

void IRComm_c::reportConfiguration() {


  Serial.print("Configured to:\n" );
  Serial.print("- tx mode: \t" ); Serial.println( ir_config.tx_mode );
  Serial.print("- tx repeat: \t" ); Serial.println( ir_config.tx_repeat );
  Serial.print("- tx period: \t" ); Serial.println( ir_config.tx_period );
  Serial.print("- rx cycle: \t" ); Serial.println( ir_config.rx_cycle );
  Serial.print("- rx predict: \t" ); Serial.println( ir_config.rx_predict_timeout );
  Serial.print("- rx overrun: \t" ); Serial.println( ir_config.rx_overrun );
  Serial.print("- rx length: \t" ); Serial.println( ir_config.rx_length );
  Serial.print("- rx timeout: \t" ); Serial.println( ir_config.rx_timeout );
  Serial.print("- rx to-multi: \t" ); Serial.println( ir_config.rx_timeout_multi );
  Serial.print("- rx pwr index:\t" ); Serial.println( ir_config.rx_pwr_index );
  Serial.print("- rx byte to: \t"); Serial.println( ir_config.rx_byte_timeout );
}

void IRComm_c::cyclePowerRx() {

  // If the board is configured not to cycle
  // the receiver, we simply update the
  // continue functioning, without changing
  // to a new receiver.
  if ( ir_config.rx_cycle == false ) {

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
    toggleRxPower();
    setRxTimeout();
    return;
  }

  ir_config.rx_pwr_index++;
  rx_cycles++;
  // A full rotation has occured, process the
  // activity level, update the lpf
  if ( ir_config.rx_pwr_index >= RX_PWR_MAX ) {
    ir_config.rx_pwr_index = 0;
  }


  powerOnRx( ir_config.rx_pwr_index );

  // Since we've cycled receiver,
  // set the new timeout period
  setRxTimeout();
}


// Small wrapper for how to update the recorded activity
// level by some decay rate.
void IRComm_c::updateActivity() {

  float sum = rx_activity[0] + rx_activity[1] + rx_activity[2] + rx_activity[3];
  if ( sum > 0 ) {
    for ( int i = 0; i < 4; i++ ) {
      rx_vectors[i] = (rx_vectors[i] * 0.3 ) + ((rx_activity[i] / sum) * 0.7);
    }

  } else {

    // No activity?
    for ( int i = 0; i < 4; i++ ) {
      rx_vectors[i] = (rx_vectors[i] * 0.3 ); // + ( 0 * 0.7);
    }
  }
  rx_activity[0] = rx_activity[1] = rx_activity[2] = rx_activity[3] = 0;


}

int IRComm_c::getActiveRx() {
  return ir_config.rx_pwr_index;
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

  // After changing which receiver is active,
  // the serial buffer is full of old data.
  // We clear it now.
  resetRxProcess();
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
void IRComm_c::resetRxProcess() {

  rx_index        = 0;
  GOT_START_TOKEN = false;
  crc_index       = 0;

  disableRx();

  // Flush?
  unsigned char dummy;
  while (UCSR0A & (1 << RXC0)) dummy = UDR0;

  memset(rx_buf, 0, sizeof(rx_buf));
  enableRx();

  if ( IR_DEBUG_OUTPUT ) {
    Serial.print("Buffer reset to: ");
    Serial.println( (char*)rx_buf );
  }
}

// Clears flags to allow next message to
// be processed, but without clearing the
// hardware uart buffer.
void IRComm_c::resetRxFlags() {
  rx_index        = 0;
  GOT_START_TOKEN = false;
  crc_index       = 0;
  memset(rx_buf, 0, sizeof(rx_buf));
}


// TODO: watch out!  We set 4ms here, this might
//       need to scale depending on the IR demod
//       chip.  It takes 1.2ms to receive a byte
//       at 58khz, 2.2ms at 38khz?
void IRComm_c::toggleRxPower() {

  if ( ir_config.rx_pwr_index == 0 ) {
    digitalWrite( RX_PWR_0, LOW );
    delayMicroseconds(250); // 0.25ms
    digitalWrite( RX_PWR_0, HIGH );

  } else if ( ir_config.rx_pwr_index == 1 ) {
    digitalWrite( RX_PWR_1, LOW );
    delayMicroseconds(250); // 0.25ms
    digitalWrite( RX_PWR_1, HIGH );
  } else if ( ir_config.rx_pwr_index == 2 ) {
    digitalWrite( RX_PWR_2, LOW );
    delayMicroseconds(250); // 0.25ms
    digitalWrite( RX_PWR_2, HIGH );
  } else if ( ir_config.rx_pwr_index == 3 ) {
    digitalWrite( RX_PWR_3, LOW );
    delayMicroseconds(250); // 0.25ms
    digitalWrite( RX_PWR_3, HIGH );
  }
}

void IRComm_c::powerOffAllRx() {
  digitalWrite( RX_PWR_0, LOW );
  digitalWrite( RX_PWR_1, LOW );
  digitalWrite( RX_PWR_2, LOW );
  digitalWrite( RX_PWR_3, LOW );
}

void IRComm_c::powerOnAllRx() {
  digitalWrite( RX_PWR_0, HIGH );
  digitalWrite( RX_PWR_1, HIGH );
  digitalWrite( RX_PWR_2, HIGH);
  digitalWrite( RX_PWR_3, HIGH);
}



// This function is used to format our message payload
// with the start token, CRC token, and 2 bytes of CRC.
void IRComm_c::formatString(char* str_to_send, byte len) {
  byte buf[MAX_BUF];
  int count;

  // Message payload should be 32 bytes
  // maximum, set with MAX_MSG
  // String to big? Drop the tail
  if (len > MAX_MSG) len = MAX_MSG;

  // Clear buffer
  memset(buf, 0, sizeof(buf));

  // Start indexing at 0
  count = 0;

  // set first character as our message
  // start byte
  buf[count++] = (byte)START_TOKEN;

  // Copy string in
  for (int i = 0; i < len; i++) {
    buf[count++] = (byte)str_to_send[i];
  }

  // Add checksum token
  buf[count++] = CRC_TOKEN;

  // Add checksum
  uint16_t crc = CRC16(buf, count );
  byte lb, ub;
  splitCRC16( &ub, &lb, crc );
  buf[count++] = ub;
  buf[count++] = lb;



  memset(tx_buf, 0, sizeof(tx_buf)); // clear first

  // Copy to tx buffer
  for (int i = 0; i < count; i++) {
    tx_buf[i] = buf[i]; // copy
  }

  tx_len = count;
  if ( IR_DEBUG_OUTPUT ) {
    //  if ( millis() - test_ts > 100 ) {
    //    test_ts = millis();
    //    setRandomMsg(8);
    //  }
    Serial.println("Format string created >>");
    Serial.print( (char*)buf );
    Serial.println("<< END");
  }

}


// This takes a number, converts it to a float, then
// it is sent to formatString()
void IRComm_c::formatFloat(float f_to_send) {

  char buf[MAX_MSG];

  // Convert float to a string, store in the
  // message buffer.
  // I had a lot of trouble finding a solution for this.
  // This is an odd, non-standard function I think.
  // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
  // https://www.programmingelectronics.com/dtostrf/
  //  - Convert "heading"
  //  - a minimum of 6 character (e.g. 000.00)
  //  - 2 digits after decimal
  //  - store in buf
  dtostrf(f_to_send, 6, 2, buf);
  formatString(buf, strlen(buf));
}


/*
   9/12/24
   Working from an arduino forum post:
   https://forum.arduino.cc/t/crc-8-i2c-cyclic-redundancy-check/644812/3

   Which references a microchip appnote:
   https://ww1.microchip.com/downloads/en/AppNotes/00730a.pdf

   For the arduino nano, I observed approximately 188us
   to compute for randomised 31 byte messages.

   Polynomial x^8 + x^5 + x^3 + x^2 + x + 1 is CRC8-AUTOSAR
   according to this wiki article:
   https://en.wikipedia.org/wiki/Cyclic_redundancy_check

   I think it would be better to use a larger CRC sum if we
   really want to transmit 32 bytes.  Since the CRC is computed
   and sent outside of the i2c transaction, we can extend the
   message by more bytes (up to 64 for UART I think).  For the
   time being, I will use this CRC8 and do most of my evaluations
   with messages less than 32bytes.

   There is some more interesting information here:
   https://www.sunshine2k.de/articles/coding/crc/understanding_crc.html

*/

char IRComm_c::CRC8(byte * bytes, byte len) {
  // polynomial = x^8 + x^5 + x^3 + x^2 + x + 1 (ignore MSB which is always 1)
  const byte generator = B00101111;
  byte crc = 0;
  byte index = 0;
  while (len--) { // while len > 0

    crc ^= bytes[index++]; /* XOR-in the next input byte */

    for (int i = 0; i < 8; i++) {
      if ((crc & 0x80) != 0) {
        crc = (uint8_t)((crc << 1) ^ generator);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void IRComm_c::splitCRC16( byte * u_byte, byte * l_byte, uint16_t crc ) {
  *l_byte = crc & 0xFF;
  *u_byte = crc >> 8;
}
uint16_t IRComm_c::mergeCRC16( byte u_byte, byte l_byte ) {
  return ((u_byte << 8 ) | l_byte );
}


uint16_t IRComm_c::CRC16( byte * bytes, byte len ) {
  const uint16_t generator = 0x1021; /* divisor is 16bit */
  uint16_t crc = 0; /* CRC value is 16bit */
  byte index = 0;
  while ( len-- ) {
    crc ^= (uint16_t)(bytes[index++] << 8); /* move byte into MSB of 16bit CRC */

    for (int i = 0; i < 8; i++) {
      if ((crc & 0x8000) != 0) { /* test for MSB = bit 15 */
        crc = (uint16_t)( (crc << 1) ^ generator);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

// Attempting an asynchronous send
// and listen procedure because we
// can't do both at the same time.
// https://lucidar.me/en/serialib/most-used-baud-rates-table/
// We are using 4800 baud, which is
// 4800 bits per second.
// 1.042ms per byte.
void IRComm_c::setRxTimeout() {

  // Is the board configured to dynamically adjust the
  // rx_timeout value depending on the length of the
  // messages it is receiving?
  if ( ir_config.rx_predict_timeout && ir_config.rx_length > 0 ) {

    // What is the length of the messages we are
    // receiving?
    float t = (float)ir_config.rx_length;

    // How many full message-lengths to listen for?
    t *= (float)ir_config.rx_timeout_multi;

    // Scale for milliseconds
#ifdef IR_FREQ_58
    t *= MS_PER_BYTE_58KHZ; // How many ms per byte to transmit?
#endif

#ifdef IR_FREQ_38
    t *= MS_PER_BYTE_38KHZ; // How many ms per byte to transmit?
#endif

    //float mod = t;
    //mod *= 0.25; // take 25% of the total time, and we'll add
    // this on again to desync robots
    //mod = (float)random(0, (long)mod);
    //t += mod;


    ir_config.rx_timeout = (unsigned long)t;

  } else {

    // Use global and fiRxed parameters
    // to calculate a value
#ifdef IR_FREQ_58
    float t = (float)(MS_PER_BYTE_58KHZ);
#endif
#ifdef IR_FREQ_38
    float t = (float)(MS_PER_BYTE_38KHZ);
#endif

    t *= (float)RX_DEFAULT_MSG_LEN;
    t *= (float)RX_PREDICT_MULTIPLIER;

    // Insert random delay to help
    // break up synchronous tranmission
    // between robots.
    //float mod = t;
    //mod *= 0.25; // take 25% of the total time, and we'll add
    // this on again to desync robots
    //mod = (float)random(0, (long)mod);
    //t += mod;


    ir_config.rx_timeout = (unsigned long)t;


  }

  // If we've adjsuted the delay period,
  // we move the timestamp forwards.
  rx_ts = millis();
}

void IRComm_c::setTxPeriod() {

  // If configured to 0, do nothing
  if ( DEFAULT_TX_PERIOD == 0 ) return;

  float t_mod = (float)random(0, DEFAULT_TX_PERIOD);
  t_mod -= (DEFAULT_TX_PERIOD / 2.0); // centre over 0
  t_mod *= 0.25; // downscale effect

  // Set tx period as default
  float t = DEFAULT_TX_PERIOD;

  // break up synchronous tranmission
  // between robots..
  t += t_mod;

  ir_config.tx_period = (unsigned long)t;


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
  if ( which >= 0 && which < RX_PWR_MAX) {
    memset(i2c_msg[which], 0, sizeof(i2c_msg[which]));
    msg_len[which] = 0;
  }
}

// This clears out the message we are broadcasting
// and adds a ! to the 0th character
void IRComm_c::clearTxBuf() {
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_len = 0;
}


// Quick helper function, assuming a float as
// string was received.
float IRComm_c::getFloatValue(int which) {
  if ( which >= 0 && which < RX_PWR_MAX) {
    // check for 0 length?
    return atof( i2c_msg[which] );
  } else {
    return -1;
  }
}


/*
   This is the main update routine.  It needs to switch
   between:
     listening: providing enough time to receive a message
     sending:   Serial.print and flush a message out.

   How long it will take to send a message will depend
   on the message length (number of bytes).  This also
   means there will be an unknown optimal time to wait for
   incoming messages.

   Because there is no hardware synchronisation/handshake
   between our IR devices, we use tokens to recognise the
   start and end of a data structure. These are:

     = start of the message
   @ = end message content, checksum byte after

   So for example *2839.23@E

   This update() function is called iteratively and so it
   will read in bytes from the Serial device in a
   non-blocking way. update() will trigger processMsg()
   once it has received both the start token '*' and the
   checkbyte token '@', and two more bytes for the CRC.

   Arduino TWI/I2C can only send/receive 32 bytes, so
   that defines the maximum possible message length. When
   transmitting over IR, we add *, @ and 2 bytes of CRC.
   MAX_BUF is therefore 32+4=36
   If update() has received the start token '*' but the
   buffer index exceeds MAX_BUF, it will abort the message
   receiving process and reset.

   update() will keep reseting the message index to 0
   until it receives the start token '*', which means
   it should gracefully handle trailing messages and/or
   the checkbyte token '@' without the start token '*'

   There is an order of precedence to these operations.
   Currently the rationale is:
   1) Rx Over-run: if this is enabled, and the system
      has captured the start token, it means we will
      attempt to capture more bytes over IR even if the
      timeout period for Rx has expired.  This might be
      quite useful to capture messages of varying
      lengths (e.g. if the rx timeout is too short. We
      can then use this to dynamically adjust rx_timeout
      if rx_predict_timeout is enabled.
   2) rx_timeout: We check if the time has elapsed for
      the rx timeout period,and if so, cycles the
      receiver.
   3) tx mode PERIODIC: operating a periodic transmit
      means that it is not synchronised with the
      receiving process. It isn't known when within
      the rx_timeout period a transmission might happen
      and therefore whether any useful time is left to
      receive. Transmission time could be short or long
      Therefore, we give the rx_timeout precedence,
      which means that after tx mode periodic, it may
      return to receiving.  Any garbled data will simply
      be processed in the byte capture process.
*/
void IRComm_c::update() {

  // We periodically track update the activity
  // of each receiver to help estimate a bearing
  // of neighbouring boards.
  // Activity is simply bytes received
  if ( millis() - activity_ts > UPDATE_ACTIVITY_MS ) {
    activity_ts = millis();
    updateActivity();
  }

  if ( millis() - led_ts > 50 ) {
    led_ts = millis();
    digitalWrite(DEBUG_LED, LOW );

  }

  if ( disabled == true ) return;

  // Using if/else if here to create an order of
  // precedence for particular operations.
  if ( ir_config.rx_overrun && GOT_START_TOKEN ) {

    // If overrun is true, it means we will allow a partially
    // received message the chance to be completely received
    // before anything else.
    // This is recommended as it allows for a prediction on
    // message size to be used/updated.
    //
    // This top level if() means that it will override
    // any of the below behaviours, e.g., getNewIRBytes()
    // without changing the receiver.
    //
    // If we fail to receive a consecutive byte in good time
    // then getNewBytes() reset the flags and stop this overrun
    // process.
    // Without this, it is possible to get a start token, and
    // then receive no more bytes (at all!), and be stuck on
    // one receiver (no cycling).
    getNewIRBytes();


    if ( IR_DEBUG_OUTPUT ) {
      Serial.println("RX OVERRUN: Waiting for message RX to finish");
    }


  } else if ( millis() - rx_ts > ir_config.rx_timeout) {

    // This means we've exceeded the listening time
    // for this receiver. It means we will initate
    // a change of receiver, and so flush out the
    // buffers, reset flags.  We don't update the
    // rx_ts here, setRxDelay() handles this.

    if ( IR_DEBUG_OUTPUT ) {
      Serial.print("RX_TIMEOUT: expired ");
      Serial.println(ir_config.rx_timeout);
    }

    // If we are in TX_MODE_INTERLEAVED, it means
    // a tranmission occurs at each cycle of the
    // receiver.  Do the transmission now.
    if ( ir_config.tx_mode == TX_MODE_INTERLEAVED ) {

      if ( IR_DEBUG_OUTPUT ) {
        Serial.println("TX_MODE_INTERLEAVED: do Tx");
      }

      doTransmit();

    }

    // Switch receiver.  This also does a full
    // disable/enable cycle, resets all rx flags
    // and calls setRxDelay()
    cyclePowerRx();

    if ( IR_DEBUG_OUTPUT ) {
      Serial.println(" - cycle Rx power");
    }


  } else if ( ir_config.tx_mode == TX_MODE_PERIODIC ) {
    // If we are in PERIODIC mode for transmission,
    // we check if it now time to send a message.
    // If so, it will abort any receiving process and
    // flush the buffer, regardless of any timeout
    // process on message receiving (rx_timeout)
    // We don't need to update tx_ts here, it is
    // updated by setTxPeriod() within doTransmit()

    if ( ir_config.tx_period == 0 ) {
      doTransmit();
      return; // done.
    } else if ( millis() - tx_ts > ir_config.tx_period ) {


      if ( IR_DEBUG_OUTPUT ) {
        Serial.println("TX_MODE_PERIODIC: expired, do Tx");
      }


      // The tx period could interrupt the rx timeout
      // period, and at the moment I'm not sure if this
      // means we should by default cycle the receiver,
      // or let it complete. For now, let's not cycle.
      // doTransmit() calls setTxPeriod()
      doTransmit();
      return; //done

    } else {

      // If we're not doing a periodic cycle of the
      // recevier, we still need to check for new
      // bytes
      if ( IR_DEBUG_OUTPUT ) {
        Serial.println("TX_MODE_PERIODIC: checking for new IR bytes");
      }
      getNewIRBytes();
    }

  } else {

    if ( IR_DEBUG_OUTPUT ) {
      Serial.println("update(): checking for new IR bytes");
    }
    getNewIRBytes();

  }

}

// Check for new message in serial buffer
// Reads in currently available char's, should
// not take long.
// Importantly, we use the PROCESS_MSG flag
// to trigger when we should attempt to process
// the message.  Therefore, we're not expecting
// to get a complete message within one cycle of
// this while loop - it just reads out the
// currently available bytes.
void IRComm_c::getNewIRBytes() {

  while ( Serial.available() ) {

    rx_buf[rx_index] = Serial.read();
    byte_ts = millis(); // register when we got this byte

    // Register activity on this receiver, whether pass or fail
    rx_activity[ ir_config.rx_pwr_index ] += 1;
    activity[ ir_config.rx_pwr_index ]++;

    // Start token? If yes, we can start to fill the
    // receiver buffer (rx_buf), and after setting
    // GOT_START_TOKEN=true rx_index will start to
    // be incremented.
    if ( rx_buf[ rx_index ] == START_TOKEN ) {
      // We could get * again, let's reset the
      // index back to zero.
      rx_index = 0;
      GOT_START_TOKEN = true;
    }

    // CRC token?  crc_index will be 0 when we haven't yet
    // found the token.  We set this to the index of where
    // the crc token has been found. We will want to process
    // the buffer once we have got the next byte after this.
    if ( rx_buf[ rx_index ] == CRC_TOKEN ) crc_index = rx_index;

    // Note that, the next line might seem like rx_index will
    // exceed the buffer.
    // This while(Serial.available() ) may not read in a whole
    // message, so this section of code is called iteratively.
    // Processing the message will reset rx_index
    // to 0.
    // By using GOT_START_TOKEN, I think that we should
    // always have a full message starting from index 0 in
    // rx_buf
    // If rx_index does exceed max_message, we break.
    if ( GOT_START_TOKEN ) rx_index++;

    // If we exceed the buffer size, we also
    // will try to process the message.
    // Because crc_index is assigned from rx_index, it
    // means crc_index should also never be more than
    // MAX_BUF, resetRxFlags() will always be called if so.
    // Therefore we avoid trying to call processMsg() with
    // unsafe index values.
    if (rx_index > MAX_BUF) {
      if ( IR_DEBUG_OUTPUT ) {
        Serial.println("Buffer full before CRC token");
        Serial.print("rx_buf: " );
        Serial.println( (char*)rx_buf );
      }

      error_type[ir_config.rx_pwr_index][TOO_LONG]++;
      fail_count[ir_config.rx_pwr_index]++;

      // Reset the flags but don't rotate the receiver.
      // This allows the message capture process to start
      // again.
      resetRxFlags();
    }

    // If crc_index is non-zero (found), and rx_index is
    // now crc_index +3 it means found the @ token and then
    // read another 2 bytes,  we can move to processing the
    // message.
    if ( crc_index != 0 && (rx_index >= (crc_index + 3)) ) {

      if ( IR_DEBUG_OUTPUT ) Serial.println("Got CRC token and +2 bytes");


      processRxBuf();
    }
  }

  // Did we receive a consecutive byte?
  // We only really care if we have started to collect bytes
  // for a message (start token = true).
  // If too much time has elapsed between bytes, we have a
  // broken message.  This could be because the source moved
  // out of range or became obstructed.
  if ( GOT_START_TOKEN) {
    if ( millis() - byte_ts > ir_config.rx_byte_timeout ) {
      error_type[ir_config.rx_pwr_index][INCOMPLETE]++;
      resetRxFlags();
    }
  }
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

    // Using Serial.print transmits over
    // IR.  Serial TX is modulated with
    // the 38Khz carrier in hardware.
    //unsigned long start_t = micros();
    for ( int i = 0; i < ir_config.tx_repeat; i++ ) {

      // Checking HardwareSerial.cpp, .write() is a blocking
      // function.  Therefore we don't need .flush()
      //Serial.availableForWrite();
      for ( int j = 0; j < tx_len; j++ ) {
        Serial.write( tx_buf[j] );
      }
      //Serial.print(tx_buf);
      Serial.flush();  // wait for send to complete
      tx_count++;
    }
    //Serial.println( (micros() - s ) );
    //unsigned long end_t = micros();
    //Serial.println( (end_t - start_t ) );

    // Since we used disableRx(), we need to
    // re-enable the UART and so clear
    // the rx flags and rx_buf
    resetRxFlags();
    enableRx();

    // Schedule next transmission
    // Redundant if tx_mode INTERLEAVED
    setTxPeriod();
    return true;
  }

}



// Needs to validate what is in the rx buffer by
// recalculating and comparing the checksum
// Assumes that the getNewIRBytes() function started
// storing after * and ended at @+1. Therefore, we
// don't do any sanity checking on the buffer.
// Instead, we'll rely on the result of the CRC
// to identify if the message has been received
// correctly or not.
int IRComm_c::processRxBuf( ) {

  if ( IR_DEBUG_OUTPUT ) {

    Serial.print("\n\nProcessing rx ");
    Serial.print( ir_config.rx_pwr_index );
    Serial.print(": ");
    Serial.print( (char*)rx_buf );
    Serial.print(" rx_index = " );
    Serial.println( rx_index );
  }


  // The smallest possible message we can get is 4
  // bytes, e.g. *a@F, where 'a' is the actual data.
  if ( rx_index <= 3 ) {
    if ( IR_DEBUG_OUTPUT ) Serial.println("message too short :(");
    error_type[ir_config.rx_pwr_index][TOO_SHORT]++;
    fail_count[ir_config.rx_pwr_index]++;

  } else { // string has valid length

    if ( IR_DEBUG_OUTPUT ) {
      Serial.print(" checksum after ");
      Serial.println( crc_index );
    }


    byte buf[MAX_BUF];
    memset(buf, 0, sizeof( buf ));
    int b = 0;  // count how many bytes

    // we copy upto and including the @
    for (int i = 0; i <= crc_index; i++) {
      buf[b] = rx_buf[i];
      b++;
    }



    if ( IR_DEBUG_OUTPUT ) {
      Serial.print("copied: " );
      Serial.print( (char*)buf );
      Serial.print( " b = ");
      Serial.println(b);

    }

    // Reconstruct checksum
    uint16_t crc;
    crc = CRC16(buf, (crc_index + 1));


    // Do the checksums match?
    // We will only have called processMsg() if we
    // had read 2 more bytes after the @ token
    byte ub = rx_buf[crc_index + 1];
    byte lb = rx_buf[crc_index + 2];
    uint16_t rx_crc = mergeCRC16( ub, lb );


    if ( IR_DEBUG_OUTPUT ) {
      Serial.print( crc );
      Serial.print( " vs ");
      Serial.println( rx_crc );
    }
    if (crc == rx_crc) {


      if ( IR_DEBUG_OUTPUT ) Serial.println("CRC GOOD!");

      // Flash so we can see when robots are
      // receiving messages correctly
      digitalWrite(DEBUG_LED, HIGH);


      // Since we are successful, we record the length
      // of the message received.  We are including
      // all tokens.
      // rx_length is used to set the polling timings
      ir_config.rx_length = crc_index + 2;// +2 crc bytes




      unsigned long dt = millis() - msg_t[ir_config.rx_pwr_index ];
      msg_dt[ ir_config.rx_pwr_index ] = dt;
      msg_t[ ir_config.rx_pwr_index ] = millis();


      // Make sure where we will store this
      // received message is clear.
      memset(i2c_msg[ir_config.rx_pwr_index], 0, sizeof(i2c_msg[ir_config.rx_pwr_index]));

      // Copy message across, stopping at (not including) @
      byte count = 0;
      for (int i = 1; i < crc_index; i++) {
        i2c_msg[ir_config.rx_pwr_index][i - 1] = rx_buf[i];
        count++;
      }

      // Removing the start token, crc token and CRC bytes
      // msg_len is used to transfer just the message content
      // over i2c
      msg_len[ ir_config.rx_pwr_index ] = count;

      float id = atof( i2c_msg[ir_config.rx_pwr_index] );
      if ( id == 1.00 ) {
        hist[0]++;
      } else if ( id == 2.00 ) {
        hist[1]++;
      } else if ( id == 3.00 ) {
        hist[2]++;
      } else {
        hist[3]++;
      }

      // Since we are successful, we increase the message
      // count for this receiver
      pass_count[ ir_config.rx_pwr_index ]++;

      // If this message was received correctly, and the
      // board is configured for cycle_on_rx, it means we
      // need to trigger an rx_cycle. Rather than call
      // cyclePowerRx() directly, we will set the rx_ts
      // to 0 to force update() to handle the cycling of
      // rx with other functionality, such as
      // TX_MODE_INTERLEAVED
      if ( ir_config.rx_cycle_on_rx ) {
        rx_ts = 0;
      }


      if ( IR_DEBUG_OUTPUT ) {
        Serial.print("Saved: \n" ) ;
        Serial.println( (char*)rx_buf );
        Serial.println( i2c_msg[ir_config.rx_pwr_index] );
        Serial.println( msg_dt[ ir_config.rx_pwr_index ] );
        Serial.println( msg_t[ ir_config.rx_pwr_index ] );
        Serial.println( pass_count[ ir_config.rx_pwr_index ] );
        Serial.println( fail_count[ ir_config.rx_pwr_index ] );
      }

      //Serial.println( millis() );
      //          Serial.println(( micros() - s) );




    } else {

      fail_count[ir_config.rx_pwr_index]++;
      error_type[ir_config.rx_pwr_index][BAD_CS]++;
      // Checksums do not match, corrupt message.
      if ( IR_DEBUG_OUTPUT ) Serial.println("bad checksum");

    }

  }

  // Whether we processed successfully or not
  // reset all the flags.
  resetRxFlags();

}

void IRComm_c::fullReset() {

  for ( int i = 0; i < 4; i++ ) {
    pass_count[i] = 0;
    fail_count[i] = 0;
    rx_activity[i] = 0;
    activity[i] = 0;
    clearRxMsg(i);
    hist[i] = 0;
  }
  clearTxBuf();
  rx_cycles = 0;
  tx_count = 0;


  for ( int i = 0; i < 4; i++ ) {
    for ( int j = 0; j < 4; j++ ) {
      error_type[i][j] = 0;
    }
  }

  resetRxProcess();
}

// Not used.
int IRComm_c::findChar(char c, char* str, byte len) {
  for (int i = 0; i < len; i++) {
    if (str[i] == c) {
      return i;
    }
  }
  return -1;
}


// This ISR simply toggles the state of
// pin D4 to generate a 38khz clock signal.
ISR(TIMER2_COMPA_vect) {
  PORTD ^= (1 << PD4);
}
