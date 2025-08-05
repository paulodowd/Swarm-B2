#ifndef IR_PARSER_H
#define IR_PARSER_H

#include <avr/io.h>
#include "Arduino.h"
#include "config.h"

#define ERR_TOO_SHORT     1
#define ERR_BAD_LENGTH    2
#define ERR_BAD_CRC       3
#define ERR_BYTE_TIMEOUT  4

class IRParser_c {

  public:

    byte buf_index;           // tracks the progress of receiving bytes
    byte header_len;          // message length declared as 2nd byte
    bool GOT_START_TOKEN;     // have we started to receive a message?
    byte buf[MAX_BUF];        // buffer to store the incoming message
    byte msg[MAX_MSG];
    byte msg_len;
    unsigned long timeout_ts; // timestamp to watch for a lapse in receiving bytes

    IRParser_c();

    void reset();
    int getNextByte( unsigned long byte_timout );
    void copyMsg( byte * dest );

    char CRC8(byte * bytes, byte len);
    uint16_t CRC16(byte * bytes, byte len);
    void splitCRC16( byte * u_byte, byte * l_byte, uint16_t crc );
    uint16_t mergeCRC16( byte u_byte, byte l_byte );

    // This function will take an input string/bytes
    // and prefix it with the start token, length
    // and suffix it with the 16 bit CRC.
    // The start token and length are included in
    // the CRC processing.
    int formatIRMessage( byte * tx_buf, const byte * msg, byte len  );

    
};


#endif
