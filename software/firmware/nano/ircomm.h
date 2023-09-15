
#ifndef IRCOMM_C_H
#define IRCOMM_C_H

#include <avr/io.h>
#include "Arduino.h"

#define STATE_IR_TX_ON 1
#define STATE_IR_TX_OFF 2
#define MAX_MSG 32

#define TTL 100          // keep a message for 0.1 second 
                          // then delete.



#define RX_PWR_0  2 // Forward
#define RX_PWR_1  3 // LEFT
#define RX_PWR_2  5 // BACK
#define RX_PWR_3  7 // RIGHT
#define RX_PWR_N  4



// 38Khz signal generated on
// digital pin 4.
#define TX_CLK_OUT 4

//#define IR_DEBUG_OUTPUT true
#define IR_DEBUG_OUTPUT false


class IRComm_c {

public:
// Two operational states
  int state;

  // Flags
  bool BROADCAST;
  bool PROCESS_MSG;

// Message buffers
  char tx_buf[MAX_MSG];  // buffer for IR out (serial)
  char rx_buf[MAX_MSG];  // buffer for IR in  (serial)
  char rx_msg[RX_PWR_N][MAX_MSG];  // holding buffer 

  int rx_count;           // tracks how full the rx buffer is.

  unsigned long msg_ttl[RX_PWR_N];  // msg time to live

  unsigned long tx_ts;     // transmit time-stamp
  unsigned long tx_delay;  // delay between tx
  unsigned long ts;        // general time stamp

  // to keep track of which of the 5 
  // IR Demodulators is currently active.
  byte rx_pwr_index;

  IRComm_c();
  void init();
  void update();
  void setupTimer2();
  void setTXDelay();
  
  void powerOffAllRx();
  void powerOnAllRx();
  void powerOnRx( byte index );
  void cyclePowerRx();
  int getActiveRx();    
  void enableRx();
  void disableRx();
  
  void transmitString( char * str_to_send, int len );
  void transmitFloat( float f_to_send );
  
  
  int hasMsg(int which);

  int findChar( char c, char * str, int len);
  void resetRxBuf();
  int processRxBuf();
  uint8_t CRC( char * buf, int len);
  
  void enableTx();
  void disableTx(); 
  
  void stopTx();
  void startTx();

  void clearTxBuf();
  void clearRxMsg(int which);
  float getFloatValue(int which);
  
};

#endif
