#include "ircomm.h"



IRComm_c::IRComm_c() {
}



void IRComm_c::init() {

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
    ir_config.tx_period = 0; // not used

  } else {
    // You need to set TX_MODE in ircomm.h
  }

  ir_config.rx_predict_timeout = RX_PREDICT_TIMEOUT;
  ir_config.rx_overrun        = RX_OVERRUN;
  ir_config.rx_length         = RX_DEFAULT_MSG_LEN;
  ir_config.rx_timeout        = 0;
  ir_config.rx_timeout_multi  = RX_PREDICT_MULTIPLIER;


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

  error_type[0] = 0;
  error_type[1] = 0;
  error_type[2] = 0;
  error_type[3] = 0;


  // RX Demodulator power pins
  pinMode(RX_PWR_0, OUTPUT);
  pinMode(RX_PWR_1, OUTPUT);
  pinMode(RX_PWR_2, OUTPUT);
  pinMode(RX_PWR_3, OUTPUT);

  powerOffAllRx();


  // Start with demodulator 0.
  // Incremented if cyclePowerRX() is called.
  rx_pwr_index = 0;

  // Set pin for 38khz carrier as output
  pinMode(TX_CLK_OUT, OUTPUT);


  // Set message buffers to invalid (empty) state
  memset(tx_buf, 0, sizeof(tx_buf));
  memset(rx_buf, 0, sizeof(rx_buf));
  memset(i2c_msg, 0, sizeof(i2c_msg));

  // Set recorded message lengths to invalid
  // to begin with. Used in conjunction with
  // PREDICT_TX_RX_DELAY true
  ir_config.rx_length = -1;


  enableTx(); // sets up Timer2 to create 38khz carrier
  //disableTx();


  // Set initial counts to 0
  led_ts = millis();
  for ( int i = 0; i < RX_PWR_MAX; i++ ) {
    pass_count[i] = 0;
    fail_count[i] = 0;

    rx_activity[i] = 0;

    msg_dt[i] = 0;
    msg_t[i] = millis();
  }

  msg_dir = 0.0;

  hist[0] = 0;
  hist[1] = 0;

  rx_ts = millis();
  tx_ts = millis();


  resetRxProcess();
}

void IRComm_c::reportConfiguration() {


  Serial.print("Configured to:\n" );
  Serial.print("- tx mode: \t" ); Serial.println( ir_config.tx_mode );
  Serial.print("- tx repeat: \t" ); Serial.println( ir_config.tx_repeat );
  Serial.print("- tx period: \t" ); Serial.println( ir_config.tx_period );
  Serial.print("- rx predict: \t" ); Serial.println( ir_config.rx_predict_timeout );
  Serial.print("- rx overrun: \t" ); Serial.println( ir_config.rx_overrun );
  Serial.print("- rx length: \t" ); Serial.println( ir_config.rx_length );
  Serial.print("- rx timeout: \t" ); Serial.println( ir_config.rx_timeout );
  Serial.print("- rx to-multi: \t" ); Serial.println( ir_config.rx_timeout_multi );
}

void IRComm_c::cyclePowerRx() {
  rx_pwr_index++;

  // A full rotation has occured, process the
  // activity level, update the lpf
  if ( rx_pwr_index >= RX_PWR_MAX ) {
    rx_pwr_index = 0;

    updateActivity();

  }


  powerOnRx( rx_pwr_index );

  // Since we've cycled receiver,
  // set the new timeout period
  setRxTimeout();
}


// Small wrapper for how to update the recorded activity
// level by some decay rate.
void IRComm_c::updateActivity() {

  for ( int i = 0; i < 4; i++ ) {

    // I use 4 here because I think the most number of messages
    // we should get per cycle is 4 (unless CYCLE_ON_RX = false).
    // This will reduce recorded activity per iteration if no
    // messages are received
    rx_activity[i] /= 4.0;
  }
}

int IRComm_c::getActiveRx() {
  return rx_pwr_index;
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
  PROCESS_MSG     = false;
  GOT_START_TOKEN = false;
  CRC_INDEX       = 0;

  disableRx();
  memset(rx_buf, 0, sizeof(rx_buf));
  enableRx();

  if ( IR_DEBUG_OUTPUT ) {
    Serial.print("Buffer reset to: ");
    Serial.println( rx_buf );
  }
}

// Clears flags to allow next message to
// be processed, but without clearing the
// hardware uart buffer.
void IRComm_c::resetRxFlags() {
  rx_index        = 0;
  PROCESS_MSG     = false;
  GOT_START_TOKEN = false;
  CRC_INDEX       = 0;
  memset(rx_buf, 0, sizeof(rx_buf));
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

// Buffer size is 32
// We have to add:
// - start token, 1 byte '*'
// - checksum token, 1 byte '@'
// - checksum itself, 1 byte
// So 3 bytes subtracted from the buffer
// So the maxmimum length string we can send is 29 bytes
void IRComm_c::formatString(char* str_to_send, byte len) {
  char buf[MAX_MSG];
  int count;

  // String to big? Drop the tail
  if (len > 29) len = 29;

  // Clear buffer
  memset(buf, 0, sizeof(buf));

  count = 0;

  // set first character as our message
  // start byte
  buf[count++] = '*';

  // Copy string in
  for (int i = 0; i < len; i++) {
    buf[count++] = str_to_send[i];
  }

  // Add checksum token
  buf[count++] = '@';

  // Add checksum
  buf[count++] = CRC8(buf, strlen(buf));


  // Copy to tx buffer
  memset(tx_buf, 0, sizeof(tx_buf)); // clear first
  for (int i = 0; i < count; i++) {
    tx_buf[i] = buf[i]; // copy
  }

  if ( IR_DEBUG_OUTPUT ) {
    Serial.println("Format string created >>");
    Serial.print( buf );
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

*/
char IRComm_c::CRC8(char * bytes, byte len) {
  const char generator = B00101111;   // polynomial = x^8 + x^5 + x^3 + x^2 + x + 1 (ignore MSB which is always 1)
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

// Attempting an asynchronous send
// and listen procedure because we
// can't do both at the same time.
// https://lucidar.me/en/serialib/most-used-baud-rates-table/
// We are using 4800 baud, which is
// 4800 bits per second.
// 1.042ms per byte.
// We can transmit up to 32 bytes.
// I think here we could do something intelligent
// like look at how many bytes we are going to transmit
// and then double this.
void IRComm_c::setRxTimeout() {

  if ( ir_config.rx_predict_timeout && ir_config.rx_length > 0 ) {

    float t = (float)ir_config.rx_length;
    t *= (float)ir_config.rx_timeout_multi; // twice as long to listen

#ifdef IR_FREQ_58
    t *= MS_PER_BYTE_58KHZ; // How many ms per byte to transmit?
#endif

#ifdef IR_FREQ_38
    t *= MS_PER_BYTE_38KHZ; // How many ms per byte to transmit?
#endif

    float mod = t;
    mod *= 0.25; // take 25% of the total time, and we'll add
    // this on again to desync robots
    mod = (float)random(0, (long)mod);
    t += mod;
    ir_config.rx_timeout = (unsigned long)t;

  } else {

    // Use global and fixed parameters
    float t = (float)random(0, 40);
    t += (float)DEFAULT_TX_PERIOD;
    t *= (float)RX_PREDICT_MULTIPLIER;
    // Insert random delay to help
    // break up synchronous tranmission
    // between robots.
    ir_config.rx_timeout = (unsigned long)t;


  }

  // If we've adjsuted the delay period,
  // we move the timestamp forwards.
  rx_ts = millis();
}

void IRComm_c::setTxPeriod() {


  float t = (float)random(0, TX_DELAY_MOD);
  t += TX_DELAY_BIAS;
  // break up synchronous tranmission
  // between robots.
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
  }
}

// This clears out the message we are broadcasting
// and adds a ! to the 0th character
void IRComm_c::clearTxBuf() {
  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = '!';
}

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

   The checkbyte is 1 byte, so our maximum message can be
   29 bytes.

   This update() function is called iteratively and so it
   will read in bytes from the Serial device in a
   non-blocking way. update() will trigger processMsg()
   once it has received both the start token '*' and the
   checkbyte token '@'.

   If update() has received the start token '*' but the
   buffer index exceeds 29, it will abort the message
   receiving process and reset.

   update() will keep reseting the message index to 0
   until it receives the start token '*', which means
   it should gracefully handle trailing messages and/or
   the checkbyte token '@' without the start token '*'

*/
void IRComm_c::update() {

  //  reportConfiguration();
  //  Serial.print( millis() ); Serial.print(" : " ); Serial.println( rx_ts );
  //  Serial.print( (millis() - rx_ts ) );
  //  Serial.print( " vs " );
  //  Serial.println( ir_config.rx_timeout );

  if ( ir_config.rx_overrun && GOT_START_TOKEN ) {
    // If overrun is true, it means we will allow a partially
    // received message the chance to be completely received
    // before anything else.
    // This top level if() means that it will override
    // any of the below behaviours.
    getNewIRBytes();

    if ( IR_DEBUG_OUTPUT ) {
      Serial.println("Waiting for message RX to finish");
    }


  } else if ( ir_config.tx_mode == TX_MODE_PERIODIC ) {
    // If we are in PERIODIC mode for transmission,
    // we check if it now time to send a message.
    // If so, it will abort any receiving process and
    // flush the buffer, regardless of any timeout
    // process on message receiving (rx_timeout)

    if ( millis() - tx_ts > ir_config.tx_period ) {
      tx_ts = millis();

      if ( IR_DEBUG_OUTPUT ) {
        Serial.println("TX_MODE_PERIODIC: expired, do Tx");
      }

      // By default, this will cycle the receiver
      doTransmit();

    } else {
      if ( IR_DEBUG_OUTPUT ) {
        Serial.println("TX_MODE_PERIODIC: checking for new IR bytes");
      }
      getNewIRBytes();
    }

  } else if ( millis() - rx_ts > ir_config.rx_timeout) {
    // This means we've exceeded the listening time
    // for this receiver. It means we will initate
    // a change of receiver, and so flush out the
    // buffers, reset flags.

    //rx_ts = millis(); // setRxDelay() handles this

    if ( IR_DEBUG_OUTPUT ) {
      Serial.println("RX_TIMEOUT: expired, checking for new IR bytes");
    }

    // Last check for a valid message
    getNewIRBytes();

    if ( ir_config.tx_mode == TX_MODE_INTERLEAVED ) {

      if ( IR_DEBUG_OUTPUT ) {
        Serial.println("TX_MODE_INTERLEAVED: do Tx");
      }

      // Do a transmission, also cycles receiver
      doTransmit();

    } else {
      if ( IR_DEBUG_OUTPUT ) {
        Serial.println(" - cycling Rx");
      }

      // Switch receiver.  This also does a full
      // disable/enable cycle and resets all rx flags
      cyclePowerRx();
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

    // Start token? If yes, we can start to fill the
    // receiver buffer (rx_buf), and after setting
    // GOT_START_TOKEN=true rx_index will start to
    // be incremented.
    if ( rx_buf[ rx_index ] == '*' ) GOT_START_TOKEN = true;

    // CRC token?  CRC_INDEX will be 0 when we haven't yet
    // found the token.  We set this to the index of where
    // the crc token has been found. We will want to process
    // the buffer once we have got the next byte after this.
    if ( rx_buf[ rx_index ] == '@' ) CRC_INDEX = rx_index;

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
    if (rx_index >= MAX_MSG) {
      if ( IR_DEBUG_OUTPUT ) Serial.println("Buffer full before CRC token");

      error_type[TOO_LONG]++;
      fail_count[rx_pwr_index]++;

      // Reset the flags but don't rotate the receiver.
      // This allows the message capture process to start
      // again.
      resetRxFlags();
    }

    // If CRC_INDEX is non-zero (found), and rx_index is
    // now CRC_INDEX +2 (we read another byte and incremented
    // index again) it means we processed the checkbyte and
    // can move to processing the message.
    if ( CRC_INDEX != 0 && (rx_index >= (CRC_INDEX + 2)) ) {
      if ( IR_DEBUG_OUTPUT ) Serial.println("Got CRC token and +1 byte");
      processRxBuf();
    }
  }
}

void IRComm_c::doTransmit() {

  // Don't do anything if there is no message
  // to transmit.
  if ( strlen( tx_buf ) == 0 || tx_buf[0] == 0 ) {

    // This resets and re-enables RX
    // by virtue of changing the receiver
    cyclePowerRx();

    // Schedule next transmission
    // Redundant if set to INTERLEAVED
    setTxPeriod();

  } else {



    // Stop receiving
    disableRx();

    // Using Serial.print transmits over
    // IR.  Serial TX is modulated with
    // the 38Khz carrier in hardware.
    //unsigned long start_t = micros();
    for ( int i = 0; i < ir_config.tx_repeat; i++ ) {
      Serial.print(tx_buf);
      Serial.flush();  // wait for send to complete
    }
    //Serial.println( (micros() - s ) );
    //unsigned long end_t = micros();
    //Serial.println( (end_t - start_t ) );

    // This resets and re-enables RX
    // by virtue of changing the receiver
    cyclePowerRx();

    // Schedule next transmission
    // Redundant if set to INTERLEAVED
    setTxPeriod();
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
int IRComm_c::processRxBuf() {

  if ( IR_DEBUG_OUTPUT ) {

    Serial.print("\n\nProcessing rx ");
    Serial.print( rx_pwr_index );
    Serial.print(": ");
    Serial.print( rx_buf );
    Serial.print(" rx_index = " );
    Serial.println( rx_index );
  }


  // The smallest possible message we can get is 4
  // bytes, e.g. *a@F, where 'a' is the actual data.
  if ( strlen( rx_buf ) <= 3 ) {
    if ( IR_DEBUG_OUTPUT ) Serial.println("message too short :(");
    error_type[TOO_SHORT]++;
    fail_count[rx_pwr_index]++;

  } else { // string has valid length

    if ( IR_DEBUG_OUTPUT ) {
      Serial.print(" checksum after ");
      Serial.println( CRC_INDEX );
    }


    char buf[rx_index];
    memset(buf, 0, sizeof( buf ));
    int b = 0;  // count how many bytes

    // we copy upto and including the @
    for (int i = 0; i <= CRC_INDEX; i++) {
      buf[b] = rx_buf[i];
      b++;
    }



    if ( IR_DEBUG_OUTPUT ) {
      Serial.print("copied: " );
      Serial.print( buf );
      Serial.print( " b = ");
      Serial.println(b);
      Serial.print(" strlen = ");
      Serial.println( strlen(buf));
    }

    // Reconstruct checksum
    char cs;
    cs = CRC8(buf, strlen(buf));

    if ( IR_DEBUG_OUTPUT ) {
      Serial.print( cs );
      Serial.print( " vs ");
      Serial.println( (uint8_t)rx_buf[CRC_INDEX + 1]);
    }

    // Do the checksums match?
    if (cs == rx_buf[CRC_INDEX + 1]) {

      if ( IR_DEBUG_OUTPUT ) Serial.println("CRC GOOD!");

      // Flash so we can see when robots are
      // receiving messages correctly
      digitalWrite(13, HIGH);


      // Since we are successful, we record the length
      // of the message received.  We are including
      // all tokens.
      ir_config.rx_length = strlen( rx_buf );


      msg_dt[ rx_pwr_index ] = millis() - msg_t[ rx_pwr_index ];
      msg_t[ rx_pwr_index ] = millis();


      // Make sure where we will store this
      // received message is clear.
      memset(i2c_msg[rx_pwr_index], 0, sizeof(i2c_msg[rx_pwr_index]));

      // Copy message across, stopping at (not including) @
      for (int i = 1; i < CRC_INDEX; i++) {
        i2c_msg[rx_pwr_index][i - 1] = rx_buf[i];
      }

      // Add a ! to the message, which will be
      // used when later sending down I2C to
      // the Master to indicate the end of the
      // string.
      // Note: all characters were shifted back
      // by 1 because we removed the *
      i2c_msg[rx_pwr_index][CRC_INDEX - 1] = '!';

      float id = atof( i2c_msg[rx_pwr_index] );
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
      pass_count[ rx_pwr_index ]++;

      if ( IR_DEBUG_OUTPUT ) {
        Serial.print("Saved: \n" ) ;
        Serial.println( rx_buf );
        Serial.println( i2c_msg[rx_pwr_index] );
        Serial.println( msg_dt[ rx_pwr_index ] );
        Serial.println( msg_t[ rx_pwr_index ] );
        Serial.println( pass_count[ rx_pwr_index ] );
        Serial.println( fail_count[ rx_pwr_index ] );
      }

      //Serial.println( millis() );
      //          Serial.println(( micros() - s) );




    } else {

      fail_count[rx_pwr_index]++;
      error_type[BAD_CS]++;
      // Checksums do not match, corrupt message.
      if ( IR_DEBUG_OUTPUT ) Serial.println("bad checksum");

    }

  }

  // Register activity on this receiver, whether pass or fail
  rx_activity[ rx_pwr_index ] += 1;

  // Unlikely to receive another message within
  // this window, so skip cycle
  if ( CYCLE_ON_RX ) {

    cyclePowerRx(); // Rotate to next receiver

  } else {

    resetRxFlags();
  }
}

int IRComm_c::findChar(char c, char* str, byte len) {
  for (int i = 0; i < len; i++) {
    if (str[i] == c) {
      return i;
    }
  }
  return -1;
}

int IRComm_c::hasMsg(int which) {
  if ( which >= 0 && which < RX_PWR_MAX) { // valid request?
    if (i2c_msg[which][0] == '!') {      // no message?
      return -1;
    }
  } else {                              // invalid request
    return -1;
  }

  // Valid, has positive length
  return strlen(i2c_msg[rx_pwr_index]);
}

// This ISR simply toggles the state of
// pin D4 to generate a 38khz clock signal.
ISR(TIMER2_COMPA_vect) {
  PORTD ^= (1 << PD4);
}