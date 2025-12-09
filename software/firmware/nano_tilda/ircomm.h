
#ifndef IRCOMM_C_H
#define IRCOMM_C_H



#include <avr/io.h>
#include "Arduino.h"
#include "config.h"
#include "ircomm_i2c.h"
#include "ir_parser.h"

// Board parameters. These are initially
// set from config.h, but can be set by
// the user via i2c
typedef struct ircomm_config { // 31 bytes
  ir_tx_params_t  tx;          // 11 bytes
  ir_rx_params_t  rx;          // 20 bytes
} ircomm_config_t;

// Structs drawn together from ircomm_i2c.h
// Individually sent over i2c on request.
// All relating to board functions and
// performance.
typedef struct ircomm_metrics {
  ir_status_t       status;
  ir_errors_t       errors;
  ir_cycles_t       cycles;
  ir_msg_timings_t  msg_timings;
  ir_byte_timings_t  byte_timings;
  ir_vectors_t      vectors;
  ir_bearing_t      bearing;
  ir_sensors_t      sensors;
  ir_hist_t         hist;
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

    // A buffer to store the message
    // to transmit.
    volatile byte tx_buf[MAX_TX_BUF];

    // Allows the IR board to be
    // entirely disabled/enabled
    // via i2c.
    volatile bool disabled;

    // A buffer containing the last
    // message received on each of
    // the 4 receivers. There is a
    // hard limit of 32 bytes
    // determined by the arduino i2c
    // implementation.
    uint8_t ir_msg[MAX_RX][MAX_MSG];

    // Length of the message stored
    // for each receiver.  We can
    // send/receive binary data,
    // which means strlen() is not
    // reliable.
    uint8_t msg_len[MAX_RX];

    // A record of byte activity per
    // receiver which is periodically
    // reset to 0.  Allows for the
    // estimation of bearing to other
    // transmitting boards/robots.
    float bearing_activity[MAX_RX];

    // Timestamps to loosely schedule
    // activities.
    unsigned long rx_ts;     // receiver rotation
    unsigned long tx_ts;     // periodic transmit
    unsigned long led_ts;    // LED time stamp
    unsigned long bearing_ts;// bearing estimation


    IRComm_c();           // blank.
    void init();          // configures UART etc.
    bool update();         // main function.
    void setupTimer2();   // 38khz or 58khz
    void setRxPeriod();  // set how long to listen for.
    void setTxPeriod();   // set how often to transmit.

    void powerOffAllRx(); // disables all receivers.
    void powerOnRx( byte index ); // power up 1 receiver
    void toggleRxPower(); // experimental, not used.
    bool isRxAvailable(int which);
    bool cyclePowerRx();  // rotates which receiver is on.
    
    void enableRx();      // enables UART RX function
    void disableRx();     // disables UART RX function
    void resetUART();     // does disable & enable
    
    void resetMetrics();  // zero's all metrics

    void fullReset();     // returns board to default


    // disables Rx, does Tx, enables Rx again
    bool doTransmit();    

    void updateMsgTimestamp(); 
    void updateMsgElapsedTime();
    void advanceMsgTimestamps(); 
    void updateByteTimestamp();
    void updateByteElapsedTime();
    void advanceByteTimestamps();

    // Used to update the bearing estimates
    void resetBearingActivity();
    void updateBearingActivity();

    // Enables and disables Timer2
    void enableTx();
    void disableTx();

    // Clearing tx_buf disables tranmission
    void clearTxBuf();
    
    // Used to clear the currently stored message
    // in rx_buf[].  Can be called manually via 
    // i2c.  Is called automatically when a 
    // message is downloaded via i2c.
    void clearRxMsg(int which);

    void printTxMsgForDebugging();
    void printRxMsgForDebugging();
    void printMetricsForDebugging();

};



#endif
