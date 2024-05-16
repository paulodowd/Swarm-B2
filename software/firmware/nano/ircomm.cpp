#include "ircomm.h"

IRComm_c::IRComm_c() {
}

void IRComm_c::init() {

  // Debug led
  pinMode(13, OUTPUT);


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

  state = STATE_IR_TX_ON;


  // Set message buffers to invalid (empty) state
  memset(tx_buf, 0, sizeof(tx_buf));
  memset(rx_buf, 0, sizeof(rx_buf));
  memset(rx_msg, 0, sizeof(rx_msg));

  enableTx(); // sets up Timer2 to create 38khz carrier
  //disableTx();

  // Start assuming we are not transmitting
  // a message, and that we have not received
  // a message to process.
  BROADCAST = false;
  PROCESS_MSG = false;
  rx_count = 0;


  // Start Serial.  Note, here we are using
  // serial to transmit strings over IR by
  // modulating it with the 38khz carrier.
  // The IR demodulator chip can support
  // baud up to 4800, but other variants
  // might not.
  Serial.begin(4800);
  Serial.setTimeout(250);



  // Set initial counts to 0
  led_ts = millis();
  for ( int i = 0; i < RX_PWR_MAX; i++ ) {
    pass_count[i] = 0;
    fail_count[i] = 0;
    rx_ratio[i] = 0.0;
    msg_dt[i] = 0;
    msg_t[i] = millis();
  }

  tx_ts = millis();
  cycle_ts = millis();
  setTXDelay();
  resetRxBuf();
}

void IRComm_c::cyclePowerRx() {

  if ( rx_pwr_index >= 3 ) {
    rx_pwr_index = 0;
  } else {
    rx_pwr_index++;
  }
  powerOnRx( rx_pwr_index );
}

int IRComm_c::getActiveRx() {
  return rx_pwr_index;
}



void IRComm_c::powerOnRx( byte index ) {

  
  if ( index == 0 ) {
    digitalWrite( RX_PWR_0, HIGH );
    digitalWrite( RX_PWR_1, LOW );
    digitalWrite( RX_PWR_2, LOW );
    digitalWrite( RX_PWR_3, LOW );
  } else if ( index == 1 ) {
    digitalWrite( RX_PWR_0, LOW );
    digitalWrite( RX_PWR_1, HIGH );
    digitalWrite( RX_PWR_2, LOW );
    digitalWrite( RX_PWR_3, LOW );
  } else if ( index == 2 ) {
    digitalWrite( RX_PWR_0, LOW );
    digitalWrite( RX_PWR_1, LOW );
    digitalWrite( RX_PWR_2, HIGH );
    digitalWrite( RX_PWR_3, LOW );
  } else if ( index == 3 ) {
    digitalWrite( RX_PWR_0, LOW );
    digitalWrite( RX_PWR_1, LOW );
    digitalWrite( RX_PWR_2, LOW );
    digitalWrite( RX_PWR_3, HIGH );
  }

  // After changing which receiver is active,
  // the serial buffer is full of old data.
  // We clear it now.
  resetRxBuf();
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
void IRComm_c::resetRxBuf() {
  rx_count = 0;
  PROCESS_MSG = false;
  GOT_START_TOKEN = false;
  disableRx();
  memset(rx_buf, 0, sizeof(rx_buf));
  enableRx();
}

// Clears flags to allow next message to
// be processed, but without clearing the
// hardware uart buffer.
void IRComm_c::resetRxFlags() {
  rx_count = 0;
  PROCESS_MSG = false;
  GOT_START_TOKEN = false;
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
// - terminal character, 1 byte '!'
// So 4 bytes subtracted from the buffer
// So the maxmimum length string we can send is 28 bytes
void IRComm_c::formatString(char* str_to_send, int len) {
  char buf[MAX_MSG];
  int count;

  // String to big? Drop the tail
  if (len > 28) len = 28;

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
  buf[count++] = CRC(buf, strlen(buf));

  // Stop transmission whilst we update
  // the buffer.
  state = STATE_IR_TX_OFF;

  // Copy to tx buffer
  memset(tx_buf, 0, sizeof(tx_buf));
  for (int i = 0; i < count; i++) {
    tx_buf[i] = buf[i];
  }

  // Add terminal token
  tx_buf[count] = '!';

  //Serial.println("Created:");
  //Serial.print( buf );
  //Serial.println("END");

  // Restart IR transmission
  state = STATE_IR_TX_ON;
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


uint8_t IRComm_c::CRC(char* buf, int len) {
  uint8_t cs;
  int i;

  cs = 0;
  for (i = 0; i < len; i++) {
    cs = cs ^ buf[i];
  }
  cs &= 0xff;

  return cs;
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
void IRComm_c::setTXDelay() {
  //
  //  if( tx_buf[0] == '!' ) {            // no message
  //    t = 1;
  //  } else if( strlen(tx_buf) == 0 ) {  // no message
  //    t = 1;
  //  } else {
  //
  //  }

  float t = (float)random(0, TX_DELAY_MOD);
  t += TX_DELAY_BIAS;
  // Insert random delay to help
  // break up synchronous tranmission
  // between robots.
  tx_delay = (unsigned long)t;
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

  // No prescale?
  //TCCR2B |= (1<<CS21);
  TCCR2B |= (1 << CS20);

  // match value
  OCR2A = 210;  // counts to 53, every 38,000hz
  //OCR2A = 53; // counts to 53, every 38,000hz


  // Set up which interupt flag is triggered
  TIMSK2 = 0;
  TIMSK2 |= (1 << OCIE2A);

  // enable interrupts
  sei();
}

void IRComm_c::stopTx() {
  state = STATE_IR_TX_OFF;
}

void IRComm_c::startTx() {

  state = STATE_IR_TX_ON;

}

void IRComm_c::clearRxMsg(int which) {
  if ( which >= 0 && which < RX_PWR_MAX) {
    memset(rx_msg[which], 0, sizeof(rx_msg[which]));
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
    return atof( rx_msg[which] );
  } else {
    return -1;
  }
}



/*
   This update routine needs to switch betweeen
   listening: providing enough time to receive a message
     sending: just Serial.print/flush a message out.

   How long it will take to send a message will depend
   on the message length (number of bytes).

   The more important aspect is probably how long it takes
   to receive a message, and this is made more complicated
   because the sender will not be synchronised with
   the receiver.  So, if we allowed only a fixed period to
   receive, the sender needs to start and finish within this
   period, but may not start sending at the beginning.

   The message format implemented has some tokens to help
   make sense of whether a message has been transmitted
   successfully:

 * * = start of the message
   @ = end message content, checksum byte after
   ! = end of whole message

   So for example *2839.23@E!

*/
void IRComm_c::update() {

  // Toggle whether we are broadcasting
  // or listening for IR messages
  if (millis() - tx_ts > tx_delay) {
    BROADCAST = !BROADCAST;
    tx_ts = millis();

    // Set a different delay for next
    // iteration
    setTXDelay();
  }

  // Clear status LED
  if ( millis() - led_ts > 100 ) {
    led_ts = millis();
    digitalWrite( 13, LOW );
  }


  // If we are in broadcast mode and the LEDS
  // are active.
  if (state == STATE_IR_TX_ON && BROADCAST) {


    if (strlen(tx_buf) == 0 || tx_buf[0] == '!' ) {
      // don't attempt send, empty buffer.

    } else {  // Message from Master to send.

      // We have a problem where we pick up
      // our own reflected transmission through
      // a parallel implementation in hardware.
      disableRx();


      // Using Serial.print transmits over
      // IR.  Serial TX is modulated with
      // the 38Khz carrier in hardware.
      Serial.println(tx_buf);
      Serial.flush();  // wait for send to complete
      //delay(2);

      // With TX finished, we can receive again
      // and not get our own transmission.
      enableRx();
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
  while (Serial.available() && !PROCESS_MSG) {
    // If we find a newline, we flag we have
    // a message to attempt to process.
    rx_buf[rx_count] = Serial.read();

    // Start token? If yes, we can start to fill the
    // receiver buffer (rx_buf), and after setting
    // GOT_START_TOKEN=true rx_count will start to
    // be incremented.
    if ( rx_buf[ rx_count ] == '*' ) GOT_START_TOKEN = true;

    if (rx_buf[rx_count] == '\n' || rx_buf[rx_count] == '!') {
      if ( IR_DEBUG_OUTPUT ) Serial.println("Processing because token");
      PROCESS_MSG = true;
    }


    // Note that, the next line might seem like rx_count will
    // exceed the buffer.
    // This while(Serial.available() ) may not read in a whole
    // message, so this section of code is called iteratively.
    // Processing the message will reset rx_count
    // to 0.
    // By using GOT_START_TOKEN, I think that we should
    // always have a full message starting from index 0 in
    // rx_buf
    // If rx_count does exceed max_message, we break.
    if ( GOT_START_TOKEN ) rx_count++;

    // If we exceed the buffer size, we also
    // will try to process the message.
    if (rx_count >= MAX_MSG) {
      if ( IR_DEBUG_OUTPUT ) Serial.println("Processing because buffer full");
      PROCESS_MSG = true;
    }
  }


  // Do we think we have a message to process?
  if (PROCESS_MSG) {

    // Debug
    //Serial.print("got "); Serial.print(rx_buf); Serial.print(":"); Serial.println(rx_count);

    // Invalid conditions:
    if (rx_count <= 0 || rx_buf[0] == 0) {
      // bad read.
      if ( IR_DEBUG_OUTPUT )Serial.println("bad serial");
      
      // I was doing a hardware reset of UART.
      // However, the UART buffer is 64 bytes, and so
      // it could be full of other potentially valid 
      // messages. Therefore, we just reset the flags.
      //resetRxBuf();
      resetRxFlags();
      
    } else {
      
      // From testing, this processRxBuf() takes less than
      // 1ms to execute.
      processRxBuf();
      
    }
  }

  // Time to rotate the receiver?
  // Note, this is done automatically if
  // a message is successfully received.
  if ( millis() - cycle_ts > CYCLE_TIME_MS ) {
    cycle_ts = millis();
    cyclePowerRx();
  }
}

// Needs to validate what is in the rx buffer
// by recalculating and comparing the checksum
int IRComm_c::processRxBuf() {


  if ( IR_DEBUG_OUTPUT ) {

    Serial.print("\n\nProcessing: ");
    Serial.print( rx_buf );
    Serial.print(" rx_count = " );
    Serial.println( rx_count );
  }
  // Check for start byte
  int start = findChar('*', rx_buf, rx_count);  // -1 = default error state

  if ( IR_DEBUG_OUTPUT ) {
    Serial.print("Start at: " );
    Serial.println( start );
  }

  // Message too short or bad
  // e.g. a good message would be:
  //      *abcdef@a  (start,message,CRC token, CRC value)
  //      a useless message would be:
  //      minimum (absurd) message would be *@_
  //      min message length = 3
  // if start < 0, then no * symbol was found
  if ( start < 0 ) {
    // No start token.
    // But a newline or buffer full must have brought
    // us here.
    if ( IR_DEBUG_OUTPUT ) Serial.println("No start token");

    fail_count[rx_pwr_index]++;


  } else if ((rx_count - start) <= 3 ) {
    if ( IR_DEBUG_OUTPUT ) Serial.println("message too short :(");

    fail_count[rx_pwr_index]++;


  } else {  // message valid, * index found

    // Check CRC, find @ token to separate
    // out CRC value.
    int last = findChar('@', rx_buf, rx_count);

    if ( IR_DEBUG_OUTPUT ) {
      Serial.print(" checksum after ");
      Serial.println( last );
    }

    // Start should be 0 or more
    // Note, we include the start byte *
    // in the checksum
    if (last > start && last < (rx_count - 1)) {
      char buf[rx_count];
      memset(buf, 0, sizeof( buf ));
      int b = 0;  // count how many bytes

      // we copy.
      for (int i = start; i <= last; i++) {
        buf[b] = rx_buf[i];
        b++;
      }

      // We need at least 1 byte of data
      // between start and checksum
      if (b >= 1) {

        if ( IR_DEBUG_OUTPUT ) {
          Serial.print("copied: " );
          Serial.print( buf );
          Serial.print( " b = ");
          Serial.println(b);
          Serial.print(" strlen = ");
          Serial.println( strlen(buf));
        }

        // Reconstruct checksum
        uint8_t cs;
        cs = CRC(buf, strlen(buf));

        if ( IR_DEBUG_OUTPUT ) {
          Serial.print( cs );
          Serial.print( " vs ");
          Serial.println( (uint8_t)rx_buf[last + 1]);
        }

        // Do the checksums match?
        if (cs == rx_buf[last + 1]) {

          if ( IR_DEBUG_OUTPUT ) Serial.println("CRC GOOD!");

          // Flash so we can see when robots are
          // receiving messages correctly
          digitalWrite(13, HIGH);

          // Since we are successful, we increase the message
          // count for this receiver
          pass_count[ rx_pwr_index ]++;

          msg_dt[ rx_pwr_index ] = millis() - msg_t[ rx_pwr_index ];
          msg_t[ rx_pwr_index ] = millis();


          // Make sure where we will store this
          // received message is clear.
          memset(rx_msg[rx_pwr_index], 0, sizeof(rx_msg[rx_pwr_index]));

          // Copy message across, ignoring the *
          for (int i = 0; i < b - 2; i++) {
            rx_msg[rx_pwr_index][i] = buf[i + 1];
          }

          // Add a ! to the message, which will be
          // used when later sending down I2C to
          // the Master to indicate the end of the
          // string.
          rx_msg[rx_pwr_index][b - 1] = '!';


          if ( IR_DEBUG_OUTPUT ) {
            Serial.print("Saved: " ) ;
            Serial.println( rx_msg[rx_pwr_index] );
          }



        } else {

          fail_count[rx_pwr_index]++;

          // Checksums do not match, corrupt message.
          if ( IR_DEBUG_OUTPUT ) Serial.println("bad checksum");

        }
      }
    } else {

      // checksum token missing or invalid
      if ( IR_DEBUG_OUTPUT ) Serial.println("invalid message");

      fail_count[rx_pwr_index]++;
    }
  }

  // Unlikely to receive another message within
  // this window, so skip cycle
  if ( CYCLE_ON_SUCCESS ) {
    cycle_ts = millis();
    cyclePowerRx();
  } else {
    //resetRxBuf();
    resetRxFlags();
  }
}

int IRComm_c::findChar(char c, char* str, int len) {
  for (int i = 0; i < len; i++) {
    if (str[i] == c) {
      return i;
    }
  }
  return -1;
}

int IRComm_c::hasMsg(int which) {
  if ( which >= 0 && which < RX_PWR_MAX) { // valid request?
    if (rx_msg[which][0] == '!') {      // no message?
      return -1;
    }
  } else {                              // invalid request
    return -1;
  }

  // Valid, has positive length
  return strlen(rx_msg[rx_pwr_index]);
}

// This ISR simply toggles the state of
// pin D4 to generate a 38khz clock signal.
ISR(TIMER2_COMPA_vect) {
  PORTD ^= (1 << PD4);
}
