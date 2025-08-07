
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
#define MODE_REPORT_TIMINGS 22
#define MODE_REPORT_HIST    23
#define MODE_CLEAR_HIST     24
#define MODE_FULL_RESET     25
#define MODE_REPORT_CYCLES  26
#define MODE_REPORT_ERRORS  27
#define MODE_STOP_RX        28
#define MODE_START_RX       29
#define MODE_SET_RX         30
#define MODE_SET_TX         31
#define MODE_GET_RX         32
#define MODE_GET_TX         33
#define MODE_SET_MSG        34
#define MAX_MODE            35


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

typedef struct ir_errors {  // 32 bytes
  uint16_t type[4][4];// 4*4 = 16*2bytes
} ir_errors_t;

typedef struct ir_cycles {  // 4 bytes
  uint16_t rx;
  uint16_t tx;
} ir_cycles_t;

// To find out if a message is ready
// to collect
typedef struct ir_msg_status {  // 1 byte
  uint8_t n_bytes;
} ir_msg_status_t;

// To find out the relative timing of
// message activity
typedef struct ir_msg_timings { // 16 bytes
  uint16_t msg_dt[4];           // 8 bytes
  uint16_t msg_t[4];            // 8 bytes
} ir_msg_timings_t;

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



typedef struct ir_tx_params {   // 18 bytes
  byte          mode;           // 1: 0 = periodic, 1 = interleaved
  byte          repeat;         // 1: how many repeated IR transmissions?
  bool          predict_period; // 1: show we try to predict a good tx period?
  float          predict_multi;   // 4: how many multiples of tx_len to use with predict?
  unsigned long period;         // 4: periodic:  current ms period to send messages
  unsigned long period_max;     // 4: maximum tx period allowable
  bool          desync;         // 1: should tx_period receive small randomisation?
  byte          len;            // 1: how long is the message to transmit?
} ir_tx_params_t;

typedef struct ir_rx_params {   // 28 bytes.
  //byte          rx_enable;     // Use a value 1:15 to encode which rx can be used.
  bool          cycle;          //  1: total count of rx polling rotations
  bool          cycle_on_rx;    //  1: if message received ok, immediately cycle rx?
  bool          predict_period;//  1: set rx_period based on last message length?
  bool          overrun;        //  1: if a start token received, wait to finish receiving?
  unsigned long period;        //  4: current ms used to wait before switching receiver
  unsigned long period_max;    //  4: maximum rx_period allowable
  float         predict_multi;  //  4: with prediction, how many message-lengths to wait?
  bool          desync;         //  1: should rx_period receive small randomisation?
  byte          index;          //  1: Which receiver is active? if cycle is false, sets Rx
  unsigned long byte_timeout;   //  4: If we haven't received a consecutive byte, timeout
  unsigned long sat_timeout;    //  4: Rx seems to saturate, watch for 0 byte activity.
  byte          len;            //  1: how long was the last received message?
} ir_rx_params_t;



#endif
