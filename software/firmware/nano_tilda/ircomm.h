
#ifndef IRCOMM_C_H
#define IRCOMM_C_H



#include <avr/io.h>
#include "Arduino.h"
#include "config.h"
#include "ircomm_i2c.h"
#include "ir_parser.h"

// A struct to store the configuration, because in
// future work I anticipate optimising these parameters
// in real time.
typedef struct ircomm_config {// 31 bytes
  ir_tx_params_t tx;          // 11 bytes
  ir_rx_params_t rx;          // 20 bytes
} ircomm_config_t;

// Structs drawn from ircomm_i2c.h
typedef struct ircomm_metrics {
  ir_status_t       status;
  ir_errors_t       errors;
  ir_cycles_t       cycles;
  ir_msg_timings_t  timings;
  ir_vectors_t      vectors;
  ir_bearing_t      bearing;
  ir_sensors_t      sensors;
} ircomm_metrics_t;

class IRComm_c {

  public:

    // How the board should operate
    ircomm_config_t config;

    // Metrics and status of the 
    // board operation
    ircomm_metrics_t  metrics;

    // Instance of the parser
    IRParser_c parser;

    // Tx/Rx Message buffers for IR, copying from UART
    // Note: we can only receive from one receiver
    //       at a time, so we don't use 2d arrays
    //       here.
    volatile byte tx_buf[MAX_BUF];  // buffer for IR out (serial)
    
    // To avoid using strlen, we will keep a record
    // of how long the message to transmit is. If 
    // set to 0, transmission will not happen.
    // Avoiding strlen because it searches for the
    // null character, which may occur within a 
    // datastruct.
    uint8_t tx_len;

    boolean disabled;

    // I2C buffer
    // Note: we use a 2d array here because we will
    //       store a message per receiver to send back
    //       over i2c
    uint8_t ir_msg[MAX_RX][MAX_MSG];
    uint8_t msg_len[MAX_RX];

    float bearing_activity[MAX_RX];

    unsigned long rx_ts;     // receiver rotation time-stamp
    unsigned long tx_ts;     // periodic transmit timestamp
    unsigned long led_ts;    // general time stamp
    unsigned long bearing_ts;   // per byte timeout
    

    IRComm_c();
    void init();
    int update();
    void setupTimer2();
    void setRxTimeout();  // how long to listen for?
    void setTxPeriod();   // how often to transmit in periodic mode?

    void powerOffAllRx();
    void powerOnAllRx();
    void powerOnRx( byte index );
    void toggleRxPower();

    void cyclePowerRx();
    void enableRx();
    void disableRx();

    void resetMetrics();
    

    int findChar( char c, char * str, byte len);
    void resetRxProcess();
    void resetUART();

    void fullReset();

    boolean doTransmit(); // false, nothing sent.

    void updateMsgTimings();
    
    void resetBearingActivity();
    void updateBearingActivity();

    void enableTx();
    void disableTx();

    void stopTx();
    void startTx();

    void clearTxBuf();
    void clearRxMsg(int which);
    float getFloatValue(int which);

};



#endif
