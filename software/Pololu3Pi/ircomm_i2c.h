#ifndef IRCOMM_I2C_H
#define IRCOMM_I2C_H

#pragma pack 1

#define IRCOMM_I2C_ADDR  0xB2

// A general status structure to discover
// what mode the board is in, and the number
// of messages received correctly or with error
// The max possible size of an i2c struct is
// 32 bytes.
typedef struct i2c_status { 
  uint16_t fail_count[4];         // 2x4 = 8 bytes
  uint16_t pass_count[4];         // 2x4  = 8 bytes
  uint16_t error_type[4];         // 2x4  = 8 bytes 
  uint16_t activity[4];           // 2x4  = 8 bytes
} i2c_status_t;

// Small struct used to change mode.
// It's also 1 byte, so convenient to
// pass around 1 byte of data.
typedef struct i2c_mode {
  uint8_t mode;
} i2c_mode_t;

// To find out if a message is ready
// to collect
typedef struct i2c_msg_status {
  uint8_t n_bytes;
} i2c_msg_status_t;

// To find out the relative timing of
// message activity
typedef struct i2c_msg_timings {
  uint16_t msg_dt[4];         // 8 bytes
  uint16_t msg_t[4];          // 8 bytes
  uint16_t rx_timeout;          // 2 bytes
  uint16_t tx_period;          // 2 bytes
} i2c_msg_timings_t;

// Struct to track the activity levels
// of the receivers
typedef struct i2c_activity {
   float rx[4];         // 4x4 = 16bytes
} i2c_activity_t;

// Struct to report just the estimated
// direction of neighbours
typedef struct i2c_bearing {
  float theta; 
  float mag;
} i2c_bearing_t;

typedef struct i2c_id_hist {
  uint8_t id[4];
} i2c_id_hist_t;

typedef struct i2c_sensors {
  int16_t ldr[3];     // 6 bytes
  int16_t prox[2];    // 4 bytes
} i2c_sensors_t;

#define MODE_REPORT_STATUS  0
#define MODE_STOP_TX        1
#define MODE_REPORT_LDR0    2
#define MODE_REPORT_LDR1    3
#define MODE_REPORT_LDR2    4
#define MODE_SIZE_MSG0    5
#define MODE_SIZE_MSG1    6
#define MODE_SIZE_MSG2    7
#define MODE_SIZE_MSG3    8
#define MODE_REPORT_MSG0    9
#define MODE_REPORT_MSG1    10
#define MODE_REPORT_MSG2    11
#define MODE_REPORT_MSG3    12
#define MODE_CLEAR_MSG0     13
#define MODE_CLEAR_MSG1     14
#define MODE_CLEAR_MSG2     15
#define MODE_CLEAR_MSG3     16
#define MODE_REPORT_SENSORS 17
#define MODE_RESET_STATUS   18
#define MODE_REPORT_RX_ACTIVITY 19
#define MODE_REPORT_RX_DIRECTION 20
#define MODE_REPORT_TIMINGS 21
#define MODE_REPORT_HIST    22
#define MODE_CLEAR_HIST     23
#define MODE_FULL_RESET     24
#define MAX_MODE            25

#endif
