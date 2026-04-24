#ifndef SWARMB2_H
#define SWARMB2_H

#include <Arduino.h>
#include "ircomm_i2c.h"

class SwarmB2_c {

  public:

    // Persistent cache of board settings
    // Because the settings are numerous
    // and would require some informed 
    // decision making, I think it is best
    // if the user edits these directly
    // and simply called get/set below.
    volatile ir_rx_params_t rx_settings;
    volatile ir_tx_params_t tx_settings;

    SwarmB2_c() {

    }

    // Fetches current config from board
    // storing into cache
    void init();

    // Basic commands
    void getRxSettings();
    void getTxSettings();
    void setRxSettings();
    void setTxSettings();
    void printTxSettings();
    void printRxSettings();
    void resetMetrics();
    void stopTransmitting();
    void configureDefault();
    void updateSettings();

    // Messaging operations
    uint8_t getMsgLength( int which_rx );
    void    setIRMessage( uint8_t * payload, int len );
    int     getIRMessage( uint8_t * msg_buf, int rx );

    void printStatus();
    void printAnyMessage();

    // Functions for board metrics
    // Refer to ircomm_i2c.h for datatypes
    ir_status_t       getStatus();
    ir_bearing_t      getBearing();
    ir_vectors_t      getRxVectors();
    ir_activity_t     getRxActivity();
    ir_saturation_t   getRxSaturation();
    ir_msg_timings_t  getMsgTimings();
    ir_byte_timings_t getByteTimings();
    ir_skips_t        getRxSkips();
    ir_errors_t       getRxErrors();
    ir_crc_t          getRxCRC();
    ir_cycles_t       getCycles();
    ir_sensors_t      getSensors();
};

#endif
