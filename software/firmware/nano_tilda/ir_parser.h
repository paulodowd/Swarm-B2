#ifndef _IR_PARSER_H
#define _IR_PARSER_H

#include <avr/io.h>
#include "Arduino.h"
#include "config.h"

class IRParser_c {

  public:

    byte rx_index;             // tracks the progress of receiving bytes
    byte header_len;           // message length declared as 2nd byte
    bool GOT_START_TOKEN;     // have we started to receive a message?
    byte buf[MAX_BUF];        // buffer to store the incoming message
    byte msg[MAX_MSG];
    unsigned long timeout_ts; // timestamp to watch for a lapse in receiving bytes

    IRParser_c();

    void reset();
    int getNextByte();

    char CRC8(byte * bytes, byte len);
    uint16_t CRC16(byte * bytes, byte len);
    void splitCRC16( byte * u_byte, byte * l_byte, uint16_t crc );
    uint16_t mergeCRC16( byte u_byte, byte l_byte );


    // This function will take an input string
    // and prefix it with the start token, length
    // and suffix it with the 16 bit CRC.
    // The start token and length are included in
    // the CRC processing.
    int formatIRMessage( char * tx_buf, const char * msg, byte len  );

    
};


#endif
