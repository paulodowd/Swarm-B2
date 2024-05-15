#ifndef IRCOMM_DATA_H
#define IRCOMM_DATA_H

#pragma pack(1)

// Currently 23 bytes (max 32)
typedef struct i2c_status { 
  uint8_t mode;                       // 1  bytes
  uint8_t ldr[3];                     // 3  bytes
  unsigned long msg_count[4];         // 16 bytes
} i2c_status_t;

#define MODE_REPORT_STATUS  0
#define MODE_STOP_TX        1
#define MODE_REPORT_LDR0    2
#define MODE_REPORT_LDR1    3
#define MODE_REPORT_LDR2    4
#define MODE_REPORT_MSG0    5
#define MODE_REPORT_MSG1    6
#define MODE_REPORT_MSG2    7
#define MODE_REPORT_MSG3    8
#define MAX_MODE            9

typedef struct i2c_mode {
  uint8_t mode;
} i2c_mode_t;

#endif
