#ifndef SWARMB2_H
#define SWARMB2_H

#include <Arduino.h>
#include "ircomm_i2c.h"

class SwarmB2_c {

  public:
    // Empty constructor intended.
    SwarmB2_c() {}
    
    void init();

    // Basic commands
    void getRxSettings();
    void getTxSettings();
    void setRxSettings();
    void setTxSettings();
    void resetMetrics();
    void stopTransmitting();

    // Messaging operations
    uint8_t getMsgStatus( int which_rx );
    void    setIRMessage( uint8_t * payload, int len );
    bool    getIRMessage( uint8_t * message, int rx );

    // Functions for board metrics
    // Refer to ircomm_i2c.h for datatypes
    ir_bearing_t      getBearing();
    ir_vectors_t      getRxVectors();
    ir_activity_t     getRxActivity();
    ir_saturation_t   getRxSaturation();
    ir_msg_timings_t  getMsgTimings();
    ir_byte_timings_t getByteTimings();
    ir_skips_t        getRxSkips();
    ir_errors_t       getRxErrors();
    ir_crc_t          getRxCRC();
    ir_cycles_t       getRxCycles();
    ir_sensors_t      getSensors();

  protected:
    // Persistent cache of board settings
    // to reduce the number of total i2c
    // transactions
    ir_rx_params_t rx_settings;
    ir_tx_params_t tx_settings;

};

#endif
