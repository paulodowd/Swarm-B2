
#ifndef IRCOMM_C_H
#define IRCOMM_C_H



#include <avr/io.h>
#include "Arduino.h"


// Uncomment depending on your IR
// Receiver module.  This will determine
// the correct Serial Baud rate and the
// configuration of timer2 for the carrier
// signal
// Other parts of the code depend on this
// being set correctly.
//#define IR_FREQ_38  // For chip TSDP34138
#define IR_FREQ_58    // For chip TSDP34156


// I2C constrains the message payload to 32
// bytes.  
#define MAX_MSG 32

// We then add a start token (*), 
// CRC token (@), and 2 bytes for CRC16.
// Therefore, we need to store at maximum
// 32 + 4 = 36 bytes.
#define MAX_BUF (MAX_MSG + 4)



// I think we might get better performance if we
// repeat the message transmission.  We are expecting
// a listening window of at least x2 the size of the
// message, and robots will listen across 4 receivers.
// So 8?
// With 6 bytes of data, sending x8 takes ~200ms
// with 29 bytes of data, sending x8 takes ~500ms


#define RX_PWR_0  3 // Forward
#define RX_PWR_1  2 // LEFT
#define RX_PWR_2  5 // BACK
#define RX_PWR_3  7 // RIGHT
#define RX_PWR_MAX  4 // How many?

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
#define RX_CYCLE              true  // should the board poll receivers?
#define RX_CYCLE_ON_RX        true  // is a message is received, cycle?
#define RX_PREDICT_TIMEOUT    true  // try to optimse performance?
#define RX_PREDICT_MULTIPLIER 1.0   // how many message-size to wait?
#define RX_OVERRUN            true  // allow for rx message to complete? 
#define RX_DEFAULT_MSG_LEN    MAX_BUF // 36 is worst case
#define MS_PER_BYTE_58KHZ     1.1   // 58khz
#define MS_PER_BYTE_38KHZ     2.5   // 38khz

#ifdef IR_FREQ_58
#define MS_BYTE_TIMEOUT       (MS_PER_BYTE_58KHZ*4.0)     
#endif
#ifdef IR_FREQ_38
#define MS_BYTE_TIMEOUT       (MS_PER_BYTE_38KHZ*4.0)     
#endif

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
#define TX_MODE_PERIODIC     0 // set a period for when to do a tx
#define TX_MODE_INTERLEAVED  1 // tx after every receiver rotation (not working)
//#define TX_MODE (TX_MODE_PERIODIC)
#define TX_MODE (TX_MODE_INTERLEAVED)

#ifdef IR_FREQ_38
#define DEFAULT_TX_PERIOD (000)
#endif

#ifdef IR_FREQ_58
#define DEFAULT_TX_PERIOD (000)
#endif

// How many times should we repeat the transmission
// of a message? This should be set as a positive
// no zero value (1+)
#define DEFAULT_TX_REPEAT 8


// Uncomment to see debug output.  Note that, we
// are going to use the serial port for debugging,
// which means we will effectively be transmitting
// debug output to other boards, and without disabling
// the rx component we'll also receive our own
// transmission.  Complicated!
//#define IR_DEBUG_OUTPUT true
#define IR_DEBUG_OUTPUT false


#define UPDATE_ACTIVITY_MS  50



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
    unsigned long error_type[4];

    // Decoding Flags
    bool GOT_START_TOKEN;
    

    // Tx/Rx Message buffers for IR, copying from UART
    // Note: we can only receive from one receiver
    //       at a time, so we don't use 2d arrays
    //       here.
    int  rx_index;           // tracks how full the rx buffer is.
    int  crc_index;          // logs where the CRC token was found
    char tx_buf[MAX_BUF];  // buffer for IR out (serial)
    char rx_buf[MAX_BUF];  // buffer for IR in  (serial)

    // magnitudes to construct an angle of
    // message reception.  We simply sum the
    // message counts within a period of time
    float rx_activity[4];
    float rx_vectors[4];
    unsigned long activity[4];

    float msg_dir;

    // I2C buffer
    // Note: we use a 2d array here because we will
    //       store a message per receiver to send back
    //       over i2c
    char i2c_msg[RX_PWR_MAX][MAX_MSG];

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
    void cyclePowerRx();
    void reportConfiguration();
    int getActiveRx();
    void enableRx();
    void disableRx();

    void formatString( char * str_to_send, byte len );
    void formatFloat( float f_to_send );

    void getNewIRBytes();  // false = byte timeout occured
    int hasMsg(int which);

    int findChar( char c, char * str, byte len);
    void resetRxProcess();
    void resetRxFlags();
    int processRxBuf();
    char CRC8(char * bytes, byte len);
    uint16_t CRC16(char * bytes, byte len);
    void splitCRC16( byte * u_byte, byte * l_byte, uint16_t crc );
    uint16_t mergeCRC16( byte u_byte, byte l_byte );


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
