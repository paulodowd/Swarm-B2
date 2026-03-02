
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

#include <Arduino.h>
#include <stdint.h>

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
#define MODE_NOT_SET            0
#define MODE_REPORT_CRC         1
#define MODE_REPORT_ACTIVITY    2
#define MODE_REPORT_SATURATION  3
#define MODE_REPORT_SKIPS       4
#define MODE_STOP_TX            5
#define MODE_SIZE_MSG0          6
#define MODE_SIZE_MSG1          7
#define MODE_SIZE_MSG2          8
#define MODE_SIZE_MSG3          9
#define MODE_REPORT_MSG0        10
#define MODE_REPORT_MSG1        11
#define MODE_REPORT_MSG2        12
#define MODE_REPORT_MSG3        13
#define MODE_CLEAR_MSG0         14
#define MODE_CLEAR_MSG1         15
#define MODE_CLEAR_MSG2         16
#define MODE_CLEAR_MSG3         17
#define MODE_REPORT_SENSORS     18
#define MODE_RESET_METRICS      19
#define MODE_REPORT_RX_VECTORS  20
#define MODE_REPORT_RX_BEARING  21
#define MODE_REPORT_MSG_TIMINGS 22
#define MODE_REPORT_BYTE_TIMINGS 23
#define MODE_REPORT_HIST        24
#define MODE_REPORT_FRAME_ERRS  25
#define MODE_CLEAR_HIST         26
#define MODE_FULL_RESET         27
#define MODE_REPORT_CYCLES      28
#define MODE_REPORT_ERRORS      29
#define MODE_STOP_RX            30
#define MODE_START_RX           31
#define MODE_SET_RX             32
#define MODE_SET_TX             33
#define MODE_GET_RX             34
#define MODE_GET_TX             35
#define MODE_SET_MSG            36
#define MAX_MODE                37

// Contains pass/fail count for the
// crc decoded at the end of each message.
typedef struct ir_crc {
  uint32_t fail[4];   // 4 * 4 = 16bytes
  uint32_t pass[4];   // 4 * 4 = 16bytes
} ir_crc_t;

// Contains a simple count of byte activity
// per receiver.  Used to estimate bearing
// elsewhere.
typedef struct ir_activity {
  uint32_t rx[4];
} ir_activity_t;

// Used to periodically create component
// vectors for a bearing estimation, 
// drawn from the activity struct.
typedef struct ir_vectors {
  float rx[4];         // 4x4 = 16bytes
} ir_vectors_t;

// Contains the latest bearing estimation
// components.
// Theta: angle estimate.
// Mag: resultant magnitude. If 1, theta
//      is very confident. If 0, counts for
//      each receiver have cancelled out.
// Sum: Pre-normalised sum of rx counts used.
typedef struct ir_bearing {
  float theta;
  float mag;
  float sum;
} ir_bearing_t;

// Used to store a count of frame errors at
// the UART hardware level.
typedef struct ir_frame_errors {
  uint32_t rx[4];       // 4x4 = 16 bytes
} ir_frame_errors_t;

// Used to count how often the receivers 
// are power cycled due to prolonged period
// of inactivity.
typedef struct ir_saturation {
  uint32_t rx[4];
} ir_saturation_t;

// Used to count how often the receiver is
// cycled because of a byte timeout error 
// or no activity in a short period.
typedef struct ir_skips {
  uint32_t rx[4];
} ir_skips_t;

// Used by Paul for some tests
// To be removed!
typedef struct ir_hist {
  uint16_t id[4];
} ir_hist_t;

// Counts for each type of error
// per receiver.
// [ rx ][ error ]
typedef struct ir_errors {  // 32 bytes
  uint16_t type[4][4];// 4*4 = 16*2bytes
} ir_errors_t;

// Used to count how many cycles of the
// receivers have taken place.
typedef struct ir_cycles {  // 4 bytes
  uint32_t rx;              // 2
  uint32_t tx;              // 2
} ir_cycles_t;

// To find out if a message is ready
// to collect.
// 0: no message.
// <33: message length. 
typedef struct ir_msg_status {  // 1 byte
  uint8_t n_bytes;
} ir_msg_status_t;

// To find out the relative timing of
// message activity
typedef struct ir_msg_timings { // 32 bytes
  uint32_t dt_ms[4];           // 16 bytes
  uint32_t ts_ms[4];            // 16 bytes
} ir_msg_timings_t;

// To find out the relative timing of
// byte activity (not full messages 
// correctly received)
typedef struct ir_byte_timings { // 32 bytes
  uint32_t dt_us[4];           // 16 bytes
  uint32_t ts_us[4];            // 16 bytes
} ir_byte_timings_t;

// Used to report back readings from the
// extra sensors that can be mounted on
// the communication board
typedef struct ir_sensors {
  int16_t ldr[3];     // 6 bytes
  int16_t prox[2];    // 4 bytes
} ir_sensors_t;


// Struct to contain the configuration
// for transmission.
typedef struct ir_tx_params {      // total = 17 bytes
  union {                          // 2 bytes
    uint8_t all_flags;             // to access all flags at once
    struct {
      uint8_t defer           : 1; // if received a byte, defer tx?
      uint8_t desync          : 1; // randomise period?
      uint8_t reserved        : 6; // 3 more bools available
    } bits;
  } flags;
  uint8_t  repeat;            // 1: how many repeated IR transmissions?
  float    predict_multi;     // 4: how many multiples of tx_len to use with predict?
  uint8_t  defer_multi;       // 1: how many multiples of ms since rx to cancel a tx?
  uint8_t  preamble_repeat;   // 1: how many repeated preamble bytes before transmission?
  uint32_t period_ms;         // 4: periodic:  current ms period to send messages
  uint32_t period_base_ms;    // 4: maximum tx period allowable
  uint8_t  len;               // 1: how long is the message to transmit?
} ir_tx_params_t;


// Struct to contain the configuration
// for reception.
typedef struct ir_rx_params {       // total = ?? bytes.
  union {                           // 2 bytes
    uint16_t all_flags;             // to access all flags at once
    struct {
      uint16_t cycle_on_rx     : 1; // cycle when message received?
      uint16_t overrun         : 1; // complete recieve outside period?
      uint16_t desync          : 1; // randomise period?
      uint16_t rx0             : 1; // receiver available to use?
      uint16_t rx1             : 1; // receiver available to use?
      uint16_t rx2             : 1; // receiver available to use?
      uint16_t rx3             : 1; // recevier available to use?
      uint16_t rand_rx         : 1; // randomise rx cycling
      uint16_t skip_inactive   : 1; // skip a receiver with no activity?
      uint16_t reserved;       : 7; // 4 more bools available
    } bits;
  } flags;
  float     predict_multi; //  4: multiplier when predicting how long to listen for.
  uint32_t  period_ms;        //  4: current ms used to wait before switching receiver
  uint32_t  period_base_ms;   //  4: normal rx_period to use (can be modified)
  uint8_t   index;         //  1: Which receiver is active? if cycle is false, sets Rx
  uint8_t   skip_multi;    //  1: how many multiples of a byte inactivity before skip?
  uint8_t   byte_timeout_ms;  //  1: If we haven't received a consecutive byte, timeout
  uint32_t  sat_timeout_us;   //  1: Rx seems to saturate, watch for 0 byte activity.
  uint8_t   len;           //  1: how long was the last received message?
} ir_rx_params_t;



#endif
