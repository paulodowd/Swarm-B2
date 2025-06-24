
#ifndef IRCOMM_C_H
#define IRCOMM_C_H



#include <avr/io.h>
#include "Arduino.h"
#include "config.h"

// I2C constrains the message payload to 32
// bytes.  
#define MAX_MSG 32

// We then add a start token, 
// CRC token, and 2 bytes for CRC16.
// Therefore, we need to store at maximum
// 32 + 4 = 36 bytes.
#define MAX_BUF (MAX_MSG + 4)

#define RX_PWR_0  3 // Forward
#define RX_PWR_1  2 // LEFT
#define RX_PWR_2  5 // BACK
#define RX_PWR_3  7 // RIGHT
#define RX_PWR_MAX  4 // How many?


// 38Khz signal generated on
// digital pin 4.
#define TX_CLK_OUT 4



// A struct to store the configuration, because in
// future work I anticipate optimising these parameters
// in real time.
typedef struct ircomm_config {
  byte          tx_mode;           // 0 = periodic, 1 = interleaved
  byte          tx_repeat;         // how many repeated IR transmissions?
  unsigned long tx_period;         // how frequently in ms to send messages?
  boolean       rx_cycle;           // for testing
  boolean       rx_cycle_on_rx;     // if message received ok, cycle rx?
  boolean       rx_predict_timeout; // true/false
  boolean       rx_overrun;        // if a start token received, wait to finish?
  byte          rx_length;         // current measured length of received messages
  unsigned long rx_timeout;        // how long to wait in ms before switching receiver?
  float         rx_timeout_multi;  // how many message-lengths for the timeout period?
  byte          rx_pwr_index;      // Which receiver is active?
  unsigned long rx_byte_timeout;   // If we haven't received a consecutive byte, timout
  
} ircomm_config_t;

class IRComm_c {

  public:
    ircomm_config_t ir_config;

    // Two operational states
    int state;
    unsigned long activity_ts;
    int INCOMPLETE  = 0;
    int BAD_CS      = 1;
    int TOO_SHORT   = 2;
    int TOO_LONG    = 3;
    uint16_t error_type[4][4];

    // Decoding Flags
    bool GOT_START_TOKEN;
    

    // Tx/Rx Message buffers for IR, copying from UART
    // Note: we can only receive from one receiver
    //       at a time, so we don't use 2d arrays
    //       here.
    volatile int  rx_index;           // tracks how full the rx buffer is.
    volatile int  crc_index;          // logs where the CRC token was found
    volatile byte tx_buf[MAX_BUF];  // buffer for IR out (serial)
    volatile byte rx_buf[MAX_BUF];  // buffer for IR in  (serial)

    // magnitudes to construct an angle of
    // message reception.  We simply sum the
    // message counts within a period of time
    float rx_activity[4];
    float rx_vectors[4];
    uint16_t activity[4];

    // To avoid using strlen, we will keep a record
    // of how long the message to transmit is. If 
    // set to 0, transmission will not happen.
    // Avoiding strlen because it searches for the
    // null character, which may occur within a 
    // datastruct.
    uint8_t tx_len;

    boolean disabled;

    float msg_dir;

    uint16_t tx_count;
    uint16_t rx_cycles;

    // I2C buffer
    // Note: we use a 2d array here because we will
    //       store a message per receiver to send back
    //       over i2c
    char ir_msg[RX_PWR_MAX][MAX_MSG];
    uint8_t msg_len[RX_PWR_MAX];

    // Message receiving stats
    unsigned long pass_count[RX_PWR_MAX];   // received correctly
    unsigned long fail_count[RX_PWR_MAX];   // received with error
    unsigned long msg_dt[RX_PWR_MAX];       // time between last 2 messages
    unsigned long msg_t[RX_PWR_MAX];        // last message time in millis
    unsigned long hist[RX_PWR_MAX];


    unsigned long rx_ts;     // receiver rotation time-stamp
    unsigned long tx_ts;     // periodic transmit timestamp
    unsigned long led_ts;    // general time stamp
    unsigned long byte_ts;   // per byte timeout




    IRComm_c();
    void init();
    void update();
    void setupTimer2();
    void setRxTimeout();  // how long to listen for?
    void setTxPeriod();   // how often to transmit in periodic mode?

    void powerOffAllRx();
    void powerOnAllRx();
    void powerOnRx( byte index );
    void toggleRxPower();

    void cyclePowerRx();
    void reportConfiguration();
    int getActiveRx();
    void enableRx();
    void disableRx();

    void formatString( char * str_to_send, byte len );
    void formatFloat( float f_to_send );

    void getNewIRBytes(); 
    

    int findChar( char c, char * str, byte len);
    void resetRxProcess();
    void resetRxFlags();
    int processRxBuf( );
    char CRC8(byte * bytes, byte len);
    uint16_t CRC16(byte * bytes, byte len);
    void splitCRC16( byte * u_byte, byte * l_byte, uint16_t crc );
    uint16_t mergeCRC16( byte u_byte, byte l_byte );


    void fullReset();

    boolean doTransmit(); // false, nothing sent.

    void updateActivity();

    void enableTx();
    void disableTx();

    void stopTx();
    void startTx();

    void clearTxBuf();
    void clearRxMsg(int which);
    float getFloatValue(int which);

};

#endif
