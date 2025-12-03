#include "ir_parser.h"



IRParser_c::IRParser_c() {
  reset();
}

void IRParser_c::reset() {
  rxState = RX_WAIT_START;
  ESCAPE_NEXT = false;
  decPos = 0;
  encRemain = 0;
  timeout_ts = millis();
  //memset( decBuf, 0, sizeof( decBuf ));
}

void IRParser_c:: copyMsg( uint8_t * dest ) {
  memset( dest, 0, MAX_MSG );

  if ( msg_len <= 0 || msg_len > MAX_MSG ) {
    return;
  } else {
    memcpy( dest, msg, msg_len );
  }

}

int IRParser_c::getNextByte( unsigned long byte_timeout ) {

  // Note: not using while.  We don't want to
  // block the code.  Instead, we'll call this
  // function iteratively and fast.
  if ( Serial.available() ) {


    uint8_t b = (uint8_t)Serial.read();

    // Raw '~' always means start of new frame
    if (rxState != RX_READ_ENC && b == START_BYTE) {
      reset();
      rxState = RX_WAIT_LEN;
      return 0;
    }

    // Read length (RAW, never escaped)
    if (rxState == RX_WAIT_LEN) {
      encRemain = b;
      //Serial.print("set encRemain to "); Serial.println( encRemain );
      encRemain -= 2; // subtract start and len bytes
      rxState = RX_READ_ENC;
      return 0;
    }

    // Only handle escaping INSIDE encoded region
    if (rxState == RX_READ_ENC) {

      if (b == ESC_BYTE && !escapeNext ) {
        escapeNext = true;
        encRemain--;
        return 0;
      }

      if (escapeNext) {
        escapeNext = false;
        b ^= 0x20;

      } else {


        // Pre escape:  test~me^~age
        // Post escape: ~Xtest^^me^~^^age⸮⸮
        //
        if ( b == START_BYTE ) {

          //          Serial.println("Got ~ inside decoding");
          //          for( int i = 0; i < decPos; i++ ) {
          //            Serial.println( (char)decBuf[i]);
          //          }
          //          Serial.println( encRemain );
          reset();
          rxState = RX_WAIT_LEN;
          return 0;
        }

      }



      // Now a decoded byte
      //      if (encRemain == 0) {
      //        // Shouldn't happen, but guard
      //        reset();
      //        return 0;
      //      }

      decBuf[decPos++] = b;
      encRemain--;

      // Frame complete?
      if (encRemain == 0) {

        if (decPos < 3) {
          // Must be payload >=1 + CRC2
          rxState = RX_WAIT_START;
          decPos = 0;
          //          Serial.println("too small");
          return 0;
        }

        uint8_t payloadLen = decPos - 2;
        //        uint16_t rxCRC = ((uint16_t)decBuf[payloadLen] << 8) | decBuf[payloadLen + 1];
        uint16_t rxCRC = mergeCRC16( decBuf[payloadLen], decBuf[payloadLen + 1] );
        uint16_t calc = CRC16( decBuf, payloadLen);

        if (rxCRC == calc) {
          memset( msg, 0, sizeof( msg ));
          memcpy( msg, decBuf, payloadLen);
          msg_len = payloadLen;
          reset();

          digitalWrite( 13, HIGH );
          return msg_len;
        } else {
          //          Serial.println("Bad CRC");
        }

        return 0;
      }
    }
  }

  return 0;
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

  // copy the message into tx_buf
  // do this byte-by-byte to add in escape
  // characters where necessay
  // Start from index 2 (0=start,1=len)
  uint8_t encoded_len = 2;
  for ( uint8_t i = 0; i < len; i++ ) {
    encodeEscape( msg[i], tx_buf, encoded_len );
  }

  // At this point, encoded_len is the next vacant
  // element of tx_buf, or in other words the total
  // number of bytes to transmit.  We will add 2
  // more bytes for the 16 bit CRC. Therefore, we
  // can now update tx_buf[1] with the total length
  // of our transmission (encoded_len +2)
  tx_buf[1] = encoded_len + 2;

  // Now append the two CRC bytes
  tx_buf[ encoded_len ] = ub;
  tx_buf[ encoded_len + 1 ] = lb;

  // Report final length
  return tx_buf[1];
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
