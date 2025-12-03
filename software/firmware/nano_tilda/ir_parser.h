#ifndef IR_PARSER_H
#define IR_PARSER_H

#include <avr/io.h>
#include "Arduino.h"
#include "config.h"

#define ERR_TOO_SHORT     1
#define ERR_BAD_LENGTH    2
#define ERR_BAD_CRC       3
#define ERR_BYTE_TIMEOUT  4


#define START_BYTE  '~'
#define ESC_BYTE    '^'
#define XOR_MASK    0x20

#define RX_WAIT_START 0
#define RX_WAIT_LEN   1
#define RX_READ_ENC   2

class IRParser_c {

  public:
    
    uint8_t rxState;
    bool escapeNext = false;
    uint8_t encRemain = 0;
    uint8_t decBuf[MAX_MSG+2];
    uint8_t decPos = 0;
    
    //uint8_t buf_index;           // tracks the progress of receiving bytes
    //uint8_t header_len;          // message length declared as 2nd byte
    //uint8_t encoded_len;        // tracking total encoded bytes received
    //bool GOT_START_TOKEN;     // have we started to receive a message?
    bool ESCAPE_NEXT;
    //uint8_t buf[MAX_BUF];        // buffer to store the incoming message
    uint8_t msg[MAX_MSG];
    uint8_t msg_len;
    unsigned long timeout_ts; // timestamp to watch for a lapse in receiving bytes

    IRParser_c();

    void reset();
    int getNextByte( unsigned long byte_timout );
    void copyMsg( uint8_t * dest );

    char CRC8(uint8_t * bytes, uint8_t len);
    uint16_t CRC16(uint8_t * bytes, uint8_t len);
    void splitCRC16( uint8_t * u_byte, uint8_t * l_byte, uint16_t crc );
    uint16_t mergeCRC16( uint8_t u_byte, uint8_t l_byte );

    // This function will take an input string/bytes
    // and prefix it with the start token, length
    // and suffix it with the 16 bit CRC.
    // The start token and length are included in
    // the CRC processing.
    int formatIRMessage( uint8_t * tx_buf, uint8_t * msg, uint8_t len );
    bool mustEscape(uint8_t byte_in );
    void encodeEscape(uint8_t byte_in, uint8_t* buf, uint8_t& pos);



};


#endif
