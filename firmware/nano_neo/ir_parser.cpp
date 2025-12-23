#include "ir_parser.h"
#include <NeoHWSerial.h>
#include <NeoHWSerial_private.h>


IRParser_c::IRParser_c() {
  reset();
}

void IRParser_c::reset() {
  rx_state    = RX_WAIT_START;

  dec_pos     = 0;
  enc_remain  = 0;
  esc_count   = 0;

  timeout_ts  = millis();

}

void IRParser_c:: copyMsg( uint8_t * dest ) {

  // Make sure the underlying memory is clear. I've
  // found that copying in a shorter amount of memory
  // can leave some persistent garbage in the rest
  // and cause trouble later.
  memset( dest, 0, MAX_MSG );

  if ( msg_len <= 0 || msg_len > MAX_MSG ) {
    return;
  } else {
    memcpy( dest, msg, msg_len );
  }

}

static unsigned long last_t;
int IRParser_c::getNextByte( unsigned long byte_timeout ) {


  // Note: not using while.  We don't want to
  // block the code.  Instead, we'll call this
  // function iteratively and fast.
  if ( NeoSerial.available() ) {

    // move the timeout timestamp forwards
    timeout_ts = millis();

    uint8_t b = (uint8_t)NeoSerial.read();

    // We're either in WAIT_START or WAIT LEN and
    // get the start byte
    if (rx_state != RX_READ_ENC && b == START_BYTE) {
      reset();

      // If in WAIT_START, no error, indicate 1 byte
      // received.
      if ( rx_state == RX_WAIT_START ) {
        rx_state = RX_WAIT_LEN;
        return REPORT_ONE_BYTES;
      }


      rx_state = RX_WAIT_LEN;

      // Must be in WAIT_LEN, shouldn't have got the
      // start byte.
      return -ERR_RESYNC;
    }

    // Read length (RAW, never escaped)
    if (rx_state == RX_WAIT_LEN) {

      if ( b == 0 || b > MAX_MSG ) {

        reset();
        return -ERR_BAD_LENGTH;
      }

      // Add 2 because we also need to read in
      // the two CRC bytes after the message
      // payload.
      enc_remain = b + NUM_CRC_BYTES;

      //NeoSerial.print("set encRemain to "); NeoSerial.println( encRemain );
      rx_state = RX_READ_ENC;

      // No error, just indicate 1 byte received
      return REPORT_ONE_BYTES;
    }

    // Only handle escaping INSIDE encoded region
    if (rx_state == RX_READ_ENC) {

      // Flag to escape next byte on next iteration.
      // Note, we don't increment our decoded byte
      // count (decPos) because that will need to
      // match the unencoded payload length
      if (b == ESC_BYTE && !escape_next ) {
        escape_next = true;
        esc_count++;
        // No error, just indicate 1 byte received
        return REPORT_ONE_BYTES;
      }


      if (escape_next) {
        escape_next = false;
        b ^= XOR_MASK;

      } else {

        // A non-escaped START_BYTE means that we've
        // received the start byte again.
        if ( b == START_BYTE ) {

          //          NeoSerial.println("Got ~ inside decoding");
          //          for( int i = 0; i < decPos; i++ ) {
          //            NeoSerial.println( (char)decBuf[i]);
          //          }
          //          NeoSerial.println( encRemain );
          reset();
          rx_state = RX_WAIT_LEN;
          return -ERR_RESYNC;
        }

      }

      dec_buf[dec_pos++] = b;
      enc_remain--;

      // Frame complete?
      if (enc_remain == 0) {


        uint8_t payload_len = dec_pos - NUM_CRC_BYTES;

        uint16_t recv_CRC = mergeCRC16( dec_buf[payload_len], dec_buf[payload_len + 1] );
        uint16_t calc_CRC = CRC16( dec_buf, payload_len);

        if (recv_CRC == calc_CRC) {
          memset( msg, 0, sizeof( msg ));
          memcpy( msg, dec_buf, payload_len);
          msg_len = payload_len;
          reset();

          //          digitalWrite( 13, HIGH );

          uint8_t total_decoded;
          total_decoded = NUM_HEADER_BYTES + msg_len + NUM_CRC_BYTES + esc_count;
          return total_decoded;

        } else {
          //NeoSerial.println("Bad CRC");
          reset();
          return -ERR_BAD_CRC;
        }
      }
    }
  }

  // If we started to receive a message but we
  // didn't get any more bytes, indicate the
  // timeout error.  We also reset the parser
  // because we need to have consecutive bytes 
  // to get a correct message (CRC).
  if ( rx_state != RX_WAIT_START ) {
    if ( byte_timeout > 0 ) {
      if ( millis() - timeout_ts > byte_timeout ) {
        reset();
        return -ERR_BYTE_TIMEOUT;
      }
    }
  }

  // Nothing happened
  return REPORT_ZERO_BYTES;
}







int IRParser_c::formatIRMessage( uint8_t * tx_buf, uint8_t * msg, byte len ) {
  if ( len  > MAX_MSG ) {

    // ERROR, formatted messsage would be too long
    return -1;
  }

  if ( tx_buf == NULL || msg == NULL || len == 0 ) {

    // ERROR, improper arguments
    return -1;
  }

  // Clear out tx_buf
  memset( tx_buf, 0, MAX_TX_BUF );


  // Get the CRC based on just our message
  // payload
  uint16_t crc = CRC16(msg, len );
  byte lb, ub;
  splitCRC16( &ub, &lb, crc );


  // Set the first bytes of tx_buf as the
  // start token
  tx_buf[0] = START_BYTE;

  // save the original size of the payload
  tx_buf[1] = len;

  // copy the message into tx_buf
  // do this byte-by-byte to add in escape
  // characters where necessay
  // Start from index 2 (0=start,1=len)
  uint8_t encoded_len = 2;
  for ( uint8_t i = 0; i < len; i++ ) {
    encodeEscape( msg[i], tx_buf, encoded_len );
  }


  // Now append the two CRC bytes
  tx_buf[ encoded_len ] = ub;
  tx_buf[ encoded_len + 1 ] = lb;

  // Report final length
  return (encoded_len + 2);
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

char IRParser_c::CRC8(byte * bytes, byte len) {
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

void IRParser_c::splitCRC16( byte * u_byte, byte * l_byte, uint16_t crc ) {
  *l_byte = crc & 0xFF;
  *u_byte = crc >> 8;
}
uint16_t IRParser_c::mergeCRC16( byte u_byte, byte l_byte ) {
  return ((u_byte << 8 ) | l_byte );
}


uint16_t IRParser_c::CRC16( byte * bytes, byte len ) {
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

// Escape only '~' and '^'
bool IRParser_c::mustEscape(uint8_t byte_in ) {
  return byte_in == START_BYTE || byte_in == ESC_BYTE;
}


void IRParser_c::encodeEscape(uint8_t byte_in, uint8_t* buf, uint8_t& pos) {

  if ( mustEscape(byte_in) ) {

    buf[pos++] = ESC_BYTE;
    buf[pos++] = (uint8_t)( byte_in ^ XOR_MASK );


  } else {

    buf[pos++] = byte_in;

  }
}
