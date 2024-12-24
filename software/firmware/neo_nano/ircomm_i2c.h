
/*
 * This file contains the data structures and flags
 * used to transfer data over i2c. 
 * 
 * The most important thing is that the maximum 
 * amount that can be transferred in a single 
 * transaction is 32bytes.
 * 
 * i2c interaction with this board requires that first
 * the correct mode is set by transferring the 
 * i2c_mode struct, then making the corresponding 
 * request with the correct receiving struct.
 */

#ifndef IRCOMM_I2C_H
#define IRCOMM_I2C_H

#pragma pack 1

#define IRCOMM_I2C_ADDR  0xB2


// This 1 byte struct is used to change the i2c
// operation.  For example, to reset counts on
// the board, or to transfer a message to or from
// the board, etc.
typedef struct i2c_mode {
  uint8_t mode;
} i2c_mode_t;

// These flags are used to request specific data
// from this board, or cause specific functions.
// i2c_mode.mode should be set with one of these
// flags.
#define MODE_REPORT_STATUS  0
#define MODE_STOP_TX        1
#define MODE_REPORT_LDR0    2
#define MODE_REPORT_LDR1    3
#define MODE_REPORT_LDR2    4
#define MODE_SIZE_MSG0      5
#define MODE_SIZE_MSG1      6
#define MODE_SIZE_MSG2      7
#define MODE_SIZE_MSG3      8
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
#define MODE_REPORT_CYCLES  25
#define MODE_REPORT_ERRORS  26
#define MAX_MODE            27


// A general status structure to discover
// what mode the board is in, and the number
// of messages received correctly or with error
// The max possible size of an i2c struct is
// 32 bytes.
typedef struct i2c_status { 
  uint16_t fail_count[4];         
  uint16_t pass_count[4];         
  uint16_t activity[4];           
} i2c_status_t;

typedef struct i2c_errors {
  uint16_t error_type[4][4];// 4*4 = 16 (*2bytes)
} i2c_errors_t;

typedef struct i2c_cycles {
  uint16_t rx_cycles;
  uint16_t long tx_count;
} i2c_cycles_t;

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



#endif
