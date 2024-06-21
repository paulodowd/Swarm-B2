#ifndef IRCOMM_DATA_H
#define IRCOMM_DATA_H



#pragma pack 1

// A general status structure to discover
// what mode the board is in, and the number
// of messages received correctly or with error
// The max possible size of an i2c struct is
// 32 bytes.
typedef struct i2c_status { 
  uint8_t mode;                   // 1  bytes
  uint16_t fail_count[4];          // 8 bytes
  uint16_t pass_count[4];         // 8 bytes 
} i2c_status_t;

// Small struct used to change mode.
// It's also 1 byte, so convenient to
// pass around 1 byte of data.
typedef struct i2c_mode {
  uint8_t mode;
} i2c_mode_t;

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

typedef struct i2c_sensors {
  int16_t ldr[3];     // 6 bytes
  int16_t prox[2];    // 4 bytes
} i2c_sensors_t;

#define MODE_REPORT_STATUS  0
#define MODE_STOP_TX        1
#define MODE_REPORT_LDR0    2
#define MODE_REPORT_LDR1    3
#define MODE_REPORT_LDR2    4
#define MODE_STATUS_MSG0    5
#define MODE_STATUS_MSG0    6
#define MODE_STATUS_MSG1    7
#define MODE_STATUS_MSG2    8
#define MODE_STATUS_MSG3    9
#define MODE_REPORT_MSG0    10
#define MODE_REPORT_MSG1    11
#define MODE_REPORT_MSG2    12
#define MODE_REPORT_MSG3    13
#define MODE_REPORT_SENSORS 14
#define MODE_RESET_COUNTS   15
#define MODE_REPORT_ACTIVITY 16
#define MODE_REPORT_DIRECTION 17
#define MAX_MODE            18



#endif
