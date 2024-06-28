
#ifndef IRCOMM_C_H
#define IRCOMM_C_H



#include <avr/io.h>
#include "Arduino.h"

#define MAX_MSG 32

// It seems to be about 2.5ms per byte
// during serial tranmission at 4800
// baud.  Therefore, we'd need 80ms to
// send/receive a full 32 bytes.
// If we need at least double the time
// to receive a full message, that gives
// 160ms.  How much can this vary?
// Let's say a message length (80ms)
// but as +/-40ms.
#define RX_DELAY_BIAS 160
#define RX_DELAY_MOD  40

// Used only for periodic tranmission mode.
// How often do we transmit in ms?
#define TX_DELAY_BIAS 600  
#define TX_DELAY_MOD  400





// I think we might get better performance if we 
// repeat the message transmission.  We are expecting
// a listening window of at least x2 the size of the 
// message, and robots will listen across 4 receivers.
// So 8?
// With 6 bytes of data, sending x8 takes ~200ms
// with 29 bytes of data, sending x8 takes ~500ms
#define TX_REPEAT 8

#define RX_PWR_0  3 // Forward
#define RX_PWR_1  2 // LEFT
#define RX_PWR_2  5 // BACK
#define RX_PWR_3  7 // RIGHT
#define RX_PWR_MAX  4 // How many?

// If the robot attempts to process a received 
// message (pass or fail), should it rotate to
// the next receiver?
// It takes time to receive a full message, so 
// there is unlikely to be enough time remaining
// to get another message (e.g, set to true).
#define CYCLE_ON_RX true


// If we know we are going to send and receive 
// messages of a certain length then we could 
// configure the device for a more optimal
// rx_delay and tx_delay.  To try and have the
// board do this for itself, set the below to 
// true.  Set true, the board will look at the 
// length of the message to transmit and the
// length of messages received, and setup the
// tx_delay and rx_delay to be appropriate for
// which ever of the two is longest.
// If set to false, the board will use the 
// #defines set above for tx/rx_delay _bias _mod.
#define PREDICT_TX_RX_DELAY true
#define PREDICT_RX_MULTIPLIER 4



// 38Khz signal generated on
// digital pin 4.
#define TX_CLK_OUT 4


// What system should we use for the tranmission?
// Periodic: Decouples receiving from transmission, meaning
//          that tranmission happens every (x)ms.
// Interleaved: Means that after a receiver is rotated, we
//          also transmit.  This means that if a robot is
//          receiving messages well, the tranmission rate 
//          would also increase.  Seems complicated.
#define TX_MODE_PERIODIC    0   // set a period for when to do a tx
#define TX_MODE_INTERLEAVED 1   // tx after every receiver rotation (not working)

#define TX_MODE TX_MODE_PERIODIC
//#define TX_MODE TX_MODE_INTERLEAVED


// Uncomment to see debug output.  Note that, we
// are going to use the serial port for debugging,
// which means we will effectively be transmitting
// debug output to other boards, and without disabling
// the rx component we'll also receive our own
// transmission.  Complicated!
//#define IR_DEBUG_OUTPUT true
#define IR_DEBUG_OUTPUT false


class IRComm_c {

  public:
    // Two operational states
    int state;

    // Decoding Flags
    bool PROCESS_MSG;
    bool GOT_START_TOKEN;

    // IR Tx/Rx Message buffers
    int rx_count;           // tracks how full the rx buffer is.
    char tx_buf[MAX_MSG];  // buffer for IR out (serial)
    char rx_buf[MAX_MSG];  // buffer for IR in  (serial)

    // magnitudes to construct an angle of
    // message reception.  We simply sum the
    // message counts within a period of time
    
    float rx_activity[4];
    
    float msg_dir;

    // I2C buffer
    char i2c_msg[RX_PWR_MAX][MAX_MSG];

    // Message receiving stats
    // uint16_t = 65,535 max
    uint16_t pass_count[RX_PWR_MAX];   // received correctly
    uint16_t fail_count[RX_PWR_MAX];   // received with error
    uint16_t msg_dt[RX_PWR_MAX];       // time between last 2 messages
    uint16_t msg_t[RX_PWR_MAX];        // last message time in millis

    
    int rx_len; // to monitor the length of messages received
    
    unsigned long rx_ts;     // receiver rotation time-stamp
    unsigned long rx_delay;  // delay between receiver rotations

    unsigned long tx_ts;    // periodic transmit timestamp
    unsigned long tx_delay; // delay between transmission
    
    unsigned long led_ts;        // general time stamp

    

    // to keep track of which of the 5
    // IR Demodulators is currently active.
    byte rx_pwr_index;

    IRComm_c();
    void init();
    void update();
    void setupTimer2();
    void setRxDelay();
    void setTxDelay();

    void powerOffAllRx();
    void powerOnAllRx();
    void powerOnRx( byte index );
    void cyclePowerRx();
    int getActiveRx();
    void enableRx();
    void disableRx();

    void formatString( char * str_to_send, int len );
    void formatFloat( float f_to_send );


    int hasMsg(int which);

    int findChar( char c, char * str, int len);
    void resetRxBuf();
    void resetRxFlags();
    int processRxBuf();
    uint8_t CRC( char * buf, int len);

    void doTransmit();

    void enableTx();
    void disableTx();

    void stopTx();
    void startTx();

    void clearTxBuf();
    void clearRxMsg(int which);
    float getFloatValue(int which);

};

#endif
