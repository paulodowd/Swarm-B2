
#ifndef IRCOMM_C_H
#define IRCOMM_C_H

#include <avr/io.h>
#include "Arduino.h"

#define STATE_IR_TX_ON 1
#define STATE_IR_TX_OFF 2
#define MAX_MSG 32

// It seems to be about 2.5ms per byte
// during serial tranmission at 4800 
// baud.  Therefore, we'd need 80ms to
// send/receive a full 32 bytes. 
// If we need at least double the time
// to receive a full message, that gives 
// 160ms.  How much can this vary?
// Let's say a message length (80ms) 
// but as +/-40ms.  
#define RX_DELAY_BIAS 160
#define RX_DELAY_MOD  40

#define RX_PWR_0  3 // Forward
#define RX_PWR_1  2 // LEFT
#define RX_PWR_2  5 // BACK
#define RX_PWR_3  7 // RIGHT
#define RX_PWR_MAX  4 // How many?

#define CYCLE_ON_SUCCESS true

#define RX_MAG_MS 500

// 38Khz signal generated on
// digital pin 4.
#define TX_CLK_OUT 4

// Uncomment to see debug output.  Note that, we
// are going to use the serial port for debugging, 
// which means we will effectively be transmitting
// debug output to other boards, and without disabling
// the rx component we'll also receive our own 
// transmission.  Complicated!
//#define IR_DEBUG_OUTPUT true
#define IR_DEBUG_OUTPUT false


class IRComm_c {

public:
// Two operational states
  int state;

  // Flags
    bool PROCESS_MSG;
  bool GOT_START_TOKEN;

  // IR Tx/Rx Message buffers
  int rx_count;           // tracks how full the rx buffer is.
  char tx_buf[MAX_MSG];  // buffer for IR out (serial)
  char rx_buf[MAX_MSG];  // buffer for IR in  (serial)

  // magnitudes to construct an angle of
  // message reception.  We simply sum the
  // message counts within a period of time
  unsigned long rx_mag_ts;
  float rx_mag[4];
  float msg_dir;

  // I2C buffer
  char rx_msg[RX_PWR_MAX][MAX_MSG];   

  // Message receiving stats
  // uint16_t = 65,535 max
  uint16_t pass_count[RX_PWR_MAX];   // received correctly
  uint16_t fail_count[RX_PWR_MAX];   // received with error
  uint16_t msg_dt[RX_PWR_MAX];       // time between last 2 messages
  uint16_t msg_t[RX_PWR_MAX];        // last message time in millis

  float rx_ratio[RX_PWR_MAX];
  
  unsigned long rx_ts;     // transmit time-stamp
  unsigned long rx_delay;  // delay between tx
  unsigned long led_ts;        // general time stamp


  // To manage how quickly we cycle across
  // all receivers.
  unsigned long CYCLE_TIME_MS = 200;
  unsigned long cycle_ts;

  // to keep track of which of the 5 
  // IR Demodulators is currently active.
  byte rx_pwr_index;

  IRComm_c();
  void init();
  void update();
  void setupTimer2();
  void setRXDelay();
  
  void powerOffAllRx();
  void powerOnAllRx();
  void powerOnRx( byte index );
  void cyclePowerRx();
  int getActiveRx();    
  void enableRx();
  void disableRx();
  
  void formatString( char * str_to_send, int len );
  void formatFloat( float f_to_send );
  
  
  int hasMsg(int which);

  int findChar( char c, char * str, int len);
  void resetRxBuf();
  void resetRxFlags();
  int processRxBuf();
  uint8_t CRC( char * buf, int len);

  void doTransmit();
  
  void enableTx();
  void disableTx(); 
  
  void stopTx();
  void startTx();

  void clearTxBuf();
  void clearRxMsg(int which);
  float getFloatValue(int which);
  
};

#endif
