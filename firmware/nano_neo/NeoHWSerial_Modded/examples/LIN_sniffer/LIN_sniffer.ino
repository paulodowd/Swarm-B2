////////////////
// INCLUDE LIBRARIES
////////////////

#include <NeoHWSerial.h>


////////////////
// GLOBAL MACROS / DEFINES
////////////////

// LIN versions (required for checksum calculation)
#define LIN_V1    1
#define LIN_V2    2


////////////////
// GLOBAL STRUCTS / TYPEDEFS
////////////////

// frame type
typedef struct
{
  int           numData;      // number of data bytes

  union
  {
    uint8_t     bufRx[12];    // raw frame: BRK, SYNC, ID, DATA0..8, CHK

    // access to individual bytes by name
    struct
    {
      uint8_t   BRK;        // sync break (always 0x00)
      uint8_t   SYNC;       // sync field (always 0x55)
      uint8_t   ID;         // frame ID
      uint8_t   DATA[8];    // data bytes (max. 8)
      uint8_t   CHK;        // frame checksum
    };
    
  };
  
} frame_t;


////////////////
// GLOBAL VARIABLES
////////////////

// global frame buffer
frame_t   frame;

// flag to indicate that frame was received. Set in Serial Rx-ISR on reception of next BRK
bool      flagFrame = false;


////////////////
// GLOBAL FUNCTIONS
////////////////

/////////////////////
// calculate the LIN frame checksum
// note: checksum is different for v1.x and v2.x; exceptions are diagnostic frames which use v1.x checksum
/////////////////////
uint8_t LIN_checksum(uint8_t version, uint8_t id, uint8_t numData, uint8_t *data)
{
  uint16_t chk=0x00;

  // LIN2.x uses extended checksum which includes protected ID, i.e. including parity bits
  // LIN1.x uses classical checksum only over data bytes
  // Diagnostic frames with ID 0x3C and 0x3D/0x7D always use classical checksum (see LIN spec "2.3.1.5 Checkum")
  if (!((version == LIN_V1) || (id == 0x3C) || (id == 0x7D)))    // if version 2  & no diagnostic frames (0x3C=60 (PID=0x3C) or 0x3D=61 (PID=0x7D))
    chk = (uint16_t) id;

  // loop over data bytes
  for (uint8_t i = 0; i < numData; i++)
  {
    chk += (uint16_t) (data[i]);
    if (chk>255)
      chk -= 255;
  }
  chk = (uint8_t)(0xFF - ((uint8_t) chk));   // bitwise invert

  // return frame checksum
  return chk;

} // LIN_checksum()



/////////////////////
// custom Serial receive interrupt function
/////////////////////
static bool ISR_Rx( uint8_t rx, uint8_t UCSRA )
{
  static uint8_t idx = 0;     // byte index in frame
  static uint8_t Rx[12];
  
  // check for LIN sync break (=0x00 with framing error)
  if ((rx == 0x00) && (UCSRA & (1<<FE0)))
  {
    // previous frame complete -> set flag. Skip for first frame
    if (frame.BRK != 0xFF)
      flagFrame = true;

    // copy previous frame to global variables to avoid conflict
    frame.numData  = (int) idx - 3;
    memcpy(frame.bufRx, Rx, 12);
    frame.CHK = Rx[idx];
    
    // reset byte index in frame
    idx=0;

  } // sync break received

  
  // normal frame bytes -> just increase byte index in frame
  else
  {
    // avoid buffer overrun
    if (idx<11)
      idx++;

  } // normal frame bytes
  

  // copy received byte to local buffer
  Rx[idx] = rx;

  // do not store received data to Serial ring buffer
  return false;
  
} // ISR_Rx()



/////////////////////
// initialization
/////////////////////
void setup()
{
  // initialize BRK to 0xFF to indicate 1st frame
  frame.BRK = 0xFF;

  // initilize PC connection
  NeoSerial.begin(115200);
  
  // initialize LIN interface
  NeoSerial2.begin(19200);

  // attach protocol handler to used Serial
  NeoSerial2.attachInterrupt( ISR_Rx );

} // setup()



/////////////////////
// main loop
/////////////////////
void loop()
{  
  // a LIN frame was received --> handle it
  if (flagFrame)
  {
    // handle each frame only once
    flagFrame = false;

    // check sync break (must be 0x00 w/ framing error)
    if (frame.BRK != 0x00)
    {
      NeoSerial.print("error: wrong BRK, expect 0x00, read 0x");
      NeoSerial.println((int) (frame.BRK), HEX);
    }
    
    // check sync field (must be 0x55)
    else if (frame.SYNC != 0x55)
    {
      NeoSerial.print("error: wrong SYNC, expect 0x55, read 0x");
      NeoSerial.println((int) (frame.SYNC), HEX);
    }
    
    // check frame checksum (different for LIN v1.x and v2.x!)
    else if (frame.CHK != LIN_checksum(LIN_V2, frame.ID, frame.numData, frame.DATA))
    {
      NeoSerial.print("error: checksum error, expect 0x");
      NeoSerial.print((int) (LIN_checksum(LIN_V2, frame.ID, frame.numData, frame.DATA)), HEX);
      NeoSerial.print(", read 0x");
      NeoSerial.println((int) (frame.CHK), HEX);
    }

    // no error -> handle frame (here only print)
    else {

      // print ID
      NeoSerial.print("ID:");
      NeoSerial.print((int) frame.ID, HEX);
      NeoSerial.print(" ");

      // print data bytes
      NeoSerial.print("DATA:");
      for (int i=0; i<frame.numData; i++)
      {
        NeoSerial.print((int) (frame.DATA[i]), HEX);
        NeoSerial.print(" ");
      }
      NeoSerial.println();

    } // no error
  
  } // handle previous LIN frame
    
} // loop()
