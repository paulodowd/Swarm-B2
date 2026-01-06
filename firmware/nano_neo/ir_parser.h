#ifndef IR_PARSER_H
#define IR_PARSER_H

#include <avr/io.h>
#include "Arduino.h"
#include "config.h"

#define ERR_RESYNC        1   // getting start byte again
#define ERR_BAD_LENGTH    2   // len byte error
#define ERR_BAD_CRC       3   // crc error
#define ERR_BYTE_TIMEOUT  4   // too much time between bytes

#define NUM_CRC_BYTES       2 // using CRC16, so 2 bytes.
#define NUM_HEADER_BYTES    2 // start byte and message length byte

#define REPORT_ONE_BYTES    1 
#define REPORT_ZERO_BYTES   0

#define START_BYTE  '~'
#define ESC_BYTE    '^'
#define XOR_MASK    0x20

// Decoding FSM states
#define RX_WAIT_START 0
#define RX_WAIT_LEN   1
#define RX_READ_ENC   2

class IRParser_c {

  public:
    
    uint8_t rx_state;                           // Tracks receiving state (RX_...).
    bool    escape_next;                        // Flag to perform XOR to escape next byte
    uint8_t enc_remain;                         // Counts down the number of encoded bytes to read in
    uint8_t msg[MAX_MSG];                       // Persistent buffer of the last message received
    uint8_t msg_len;                            // Persistent length status for msg_len buf
    uint8_t dec_buf[MAX_MSG + NUM_CRC_BYTES ];  // Temporary buffer of incoming decoded bytes
    uint8_t dec_pos;                            // Tracks decoded byte buffer position
    uint8_t esc_count;                          // Counts up how many bytes that were escaped
    unsigned long timeout_ts;                   // timestamp to watch for a lapse in receiving bytes

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
