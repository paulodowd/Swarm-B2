
/*
   This file contains the data structures and flags
   used to transfer data over i2c.

   The most important thing is that the maximum
   amount that can be transferred in a single
   transaction is 32bytes.

   i2c interaction with this board requires that first
   the correct mode is set by transferring the
   ir_mode struct, then making the corresponding
   request with the correct receiving struct.
*/

#ifndef IRCOMM_IR_H
#define IRCOMM_IR_H

#pragma pack(1)

#define IRCOMM_I2C_ADDR  0x11


// This 1 byte struct is used to change the i2c
// operation.  For example, to reset counts on
// the board, or to transfer a message to or from
// the board, etc.
typedef struct ir_mode {
  uint8_t mode;
} ir_mode_t;

// These flags are used to request specific data
// from this board, or cause specific functions.
// ir_mode.mode should be set with one of these
// flags.
#define MODE_NOT_SET        0
#define MODE_REPORT_STATUS  1
#define MODE_STOP_TX        2
#define MODE_REPORT_LDR0    3
#define MODE_REPORT_LDR1    4
#define MODE_REPORT_LDR2    5
#define MODE_SIZE_MSG0      6
#define MODE_SIZE_MSG1      7
#define MODE_SIZE_MSG2      8
#define MODE_SIZE_MSG3      9
#define MODE_REPORT_MSG0    10
#define MODE_REPORT_MSG1    11
#define MODE_REPORT_MSG2    12
#define MODE_REPORT_MSG3    13
#define MODE_CLEAR_MSG0     14
#define MODE_CLEAR_MSG1     15
#define MODE_CLEAR_MSG2     16
#define MODE_CLEAR_MSG3     17
#define MODE_REPORT_SENSORS 18
#define MODE_RESET_STATUS   19
#define MODE_REPORT_RX_VECTORS 20
#define MODE_REPORT_RX_BEARING 21
#define MODE_REPORT_MSG_TIMINGS 22
#define MODE_REPORT_BYTE_TIMINGS 23
#define MODE_REPORT_HIST    24
#define MODE_CLEAR_HIST     25
#define MODE_FULL_RESET     26
#define MODE_REPORT_CYCLES  27
#define MODE_REPORT_ERRORS  28
#define MODE_STOP_RX        29
#define MODE_START_RX       30
#define MODE_SET_RX         31
#define MODE_SET_TX         32
#define MODE_GET_RX         33
#define MODE_GET_TX         34
#define MODE_SET_MSG        35
#define MAX_MODE            36


// A general status structure to discover
// what mode the board is in, and the number
// of messages received correctly or with error
// The max possible size of an i2c struct is
// 32 bytes.
typedef struct ir_status {  // 32 bytes
  uint16_t fail_count[4];   // 2*4 = 8
  uint16_t pass_count[4];   // 2*4 = 8
  uint16_t activity[4];     // 2*4 = 8
  uint16_t saturation[4];   // 2*4 = 8
} ir_status_t;

typedef struct ir_hist {
  uint16_t id[4];
} ir_hist_t;

typedef struct ir_errors {  // 32 bytes
  uint16_t type[4][4];// 4*4 = 16*2bytes
} ir_errors_t;

typedef struct ir_cycles {  // 4 bytes
  uint16_t rx;              // 2
  uint16_t tx;              // 2
} ir_cycles_t;  

// To find out if a message is ready
// to collect
typedef struct ir_msg_status {  // 1 byte
  uint8_t n_bytes;
} ir_msg_status_t;

// To find out the relative timing of
// message activity
typedef struct ir_msg_timings { // 32 bytes
  uint32_t dt_ms[4];           // 16 bytes
  uint32_t ts_ms[4];            // 16 bytes
} ir_msg_timings_t;

typedef struct ir_byte_timings { // 32 bytes
  uint32_t dt_us[4];           // 16 bytes
  uint32_t ts_us[4];            // 16 bytes
} ir_byte_timings_t;


// Struct to track the activity levels
// of the receivers
typedef struct ir_vectors {
  float rx[4];         // 4x4 = 16bytes
} ir_vectors_t;

// Struct to report just the estimated
// direction of neighbours
typedef struct ir_bearing {
  float theta;
  float mag;
  float sum;
} ir_bearing_t;

typedef struct ir_sensors {
  int16_t ldr[3];     // 6 bytes
  int16_t prox[2];    // 4 bytes
} ir_sensors_t;



typedef struct ir_tx_params {   // total = 18 bytes
  union {                       // 2 bytes
    uint8_t all_flags;  // access all flags at once
    struct {
      uint8_t mode           : 1; // 0=periodic, 1=interleaved
      uint8_t predict_period     : 1; // predict repeat period on len?
      uint8_t defer         : 1; // if received a byte, defer tx?
      uint8_t desync          : 1; // randomise period?
      uint8_t preamble        : 1;
      uint8_t reserved             : 3; // 3 more bools available
    } bits;
  } flags;
  uint8_t        repeat;         // 1: how many repeated IR transmissions?
  float          predict_multi;   // 4: how many multiples of tx_len to use with predict?
  unsigned long period;         // 4: periodic:  current ms period to send messages
  unsigned long period_max;     // 4: maximum tx period allowable
  uint8_t          len;            // 1: how long is the message to transmit?
} ir_tx_params_t;



typedef struct ir_rx_params {   // total = 24 bytes.
  union {                       // 2 bytes
    uint16_t all_flags;  // access all flags at once
    struct {
      uint16_t cycle           : 1; // should the board cycle receivers?
      uint16_t cycle_on_rx     : 1; // cycle when message received?
      uint16_t predict_period  : 1; // predict period on msg len?
      uint16_t overrun         : 1; // complete recieve outside period?
      uint16_t desync          : 1; // randomise period?
      uint16_t rx0             : 1; // receiver available to use?
      uint16_t rx1             : 1; // receiver available to use?
      uint16_t rx2             : 1; // receiver available to use?
      uint16_t rx3             : 1; // recevier available to use?
      uint16_t desaturate      : 1; // toggle power to receiver if zero activity
      uint16_t rand_rx         : 1; // randomise rx cycling
      uint16_t skip_inactive   : 1; // skip a receiver with no activity?
      uint16_t reserved;       : 4; // 4 more bools available
    } bits;
  } flags;
  float predict_multi;         //  4: multiplier when predicting how long to listen for.
  unsigned long period;        //  4: current ms used to wait before switching receiver
  unsigned long period_max;    //  4: maximum rx_period allowable
  byte          index;         //  1: Which receiver is active? if cycle is false, sets Rx
  uint8_t       skip_multi;    //  1: how many multiples of a byte inactivity before skip?
  unsigned long byte_timeout;  //  4: If we haven't received a consecutive byte, timeout
  unsigned long sat_timeout;   //  4: Rx seems to saturate, watch for 0 byte activity.
  byte          len;           //  1: how long was the last received message?
} ir_rx_params_t;



#endif
