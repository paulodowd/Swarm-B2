#include "ir_parser.h"


IRParser_c::IRParser_c() {
  reset();
}

void IRParser_c::reset() {
  buf_index = 0;
  header_len = 0;
  GOT_START_TOKEN = false;
  //memset( buf, 0, sizeof( buf ));
}

void IRParser_c:: copyMsg( byte * dest ) {
  memset( dest, 0, MAX_MSG );

  if ( msg_len <= 0 || msg_len > MAX_MSG ) {
    return;
  } else {
    memcpy( dest, msg, msg_len );
  }

}

int IRParser_c::getNextByte( ) {

  int status = 0;

  // Note: not using while.  We don't want to
  // block the code.  Instead, we'll call this
  // function iteratively and fast.
  if ( Serial.available() ) {

    byte b = Serial.read();
    timeout_ts = millis();

    // If we receive the start token, we will
    // simply assume that we're starting to
    // receive a message.  This will mean that
    // any message that was in progress will
    // be terminated and the receipt is started
    // again
    if ( b == START_TOKEN ) {

      // If we already had the start token...
      if ( GOT_START_TOKEN ) {
        // ERROR: message too short
        reset();
        buf[ buf_index ] = b;
        GOT_START_TOKEN = true;

        // Not a critical error, we can continue
        // and so increase the buf_index
        status = -1;
      } else {

        // Register that we've got the start
        // token.
        buf[ buf_index ] = b;
        GOT_START_TOKEN = true;
        status = 1;
      }
    } else {
      buf[buf_index] = b;
    }

    // The byte immediately after the start
    // token should be the length of the
    // incoming message
    if ( buf_index == 1 ) {

      // buf is byte, therefore unsigned 0:255
      // We check that the value of the length of
      // the message set in the header is within
      // the expected range.
      if ( buf[ buf_index ] > MAX_BUF || buf[buf_index] < 5) {
        // ERROR: bad incoming message format
        reset();
        status = -2;

        // For a critical error, we don't
        // allow buf_index to increment and
        // return here.
        return status;
      } else {
        header_len = buf[ buf_index ];
      }
    }


    // Prepare for the next byte
    if ( GOT_START_TOKEN ) buf_index++;



    // This means that we have received the
    // second byte, and we know how long the
    // message should be.
    // Note, it has to be more than 4, because
    // 4 is the start token, length, CRCx2
    // header_len will equal 0 whenever reset()
    // has been called, prior to buf_index == 1
    if ( header_len > 0 ) {

      if ( buf_index >= header_len ) {

        // Finished receiving, check the CRC

        // Calculate the CRC ignoring the received
        // CRC bytes
        uint16_t crc0 = CRC16( buf, header_len - 2 );

        // Merge two bytes of received CRC
        uint16_t crc1 = mergeCRC16( buf[ header_len - 2 ], buf[ header_len - 1 ] );

        if ( crc0 == crc1 ) {

          // Got message :)
          memset( msg, 0, sizeof(msg) );
          memcpy( msg, buf + 2, header_len - 4);
          msg_len = header_len - 4;
          reset();
          status = header_len;
          return status;

        } else {

          // ERROR, CRC mismatch
          reset();


          // For a critical error, we don't
          // allow buf_index to increment and
          // return here.
          status = -3;
          return status;
        }

      }
    }

    if ( buf_index >= MAX_BUF ) {
      // ERROR: We've exceeded the buffer
      reset();

      // For a critical error, we don't
      // allow buf_index to increment and
      // return here.
      status = -4;
      return status;

    }

    // We simply received a new byte, but not
    // a new message.
    status = 1;
    return status;

  } else {

    // No byte activity, return 0
    status = 0;
    return status;
  }

  return status;

}



int IRParser_c::formatIRMessage( byte * tx_buf, const byte * msg, byte len  ) {
  if ( len + 4 > MAX_BUF ) {

    // ERROR, formatted messsage would be too long
    return -1;
  }

  if ( tx_buf == NULL || msg == NULL || len == 0 ) {

    // ERROR, improper arguments
    return -1;
  }

  // Clear out tx_buf
  memset( tx_buf, 0, MAX_BUF );



  // Set the first two bytes of tx_buf as the
  // start token and the length
  // We +4 for start, len, CRCx2
  tx_buf[0] = START_TOKEN;
  tx_buf[1] = len + 4;


  // copy the message into tx_buf
  // We copy starting position 2, where
  // 0 is start token
  // 1 is length value
  // We've already checked that len+4
  // is less than our MAX_BUF.
  memcpy( tx_buf + 2, msg, len );

  // Get the 16 Bit CRC based on the current
  // tx_buf

  // Add checksum
  uint16_t crc = CRC16(tx_buf, len + 2 );
  byte lb, ub;
  splitCRC16( &ub, &lb, crc );


  // Now append the two CRC bytes
  tx_buf[ len + 2 ] = ub;
  tx_buf[ len + 3 ] = lb;

  // Final length
  return (len + 4);
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
