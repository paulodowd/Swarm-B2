//////
// LIN communication error codes
//////
#define SUCCESS                 0                           // no error
#define ERROR_TIMOUT            1                           // receive timeout
#define ERROR_CHK               2                           // checksum error
#define ERROR_MISC              255                         // misc error


//////
// global macros
//////
#define LIN_SERIAL              Serial1                     // used Serial interface
#define LIN_BAUDRATE            19200                       // LIN baudrate


//////
// global variables
//////
uint8_t       LIN_err       = SUCCESS;
 

//////
// setup routine (called once)
//////
void setup() 
{
  // communication to PC
  Serial.begin(115200);
  while (!Serial);

  // initialize LIN interface
  LIN_SERIAL.begin(LIN_BAUDRATE);
  while (!LIN_SERIAL);

} // setup


//////
// loop routine (called periodically)
//////
void loop() 
{
  static uint8_t  Tx[8] = {0,1,2,3,4,5,6,7};
  uint8_t         Rx[8];

  // send master request
  LIN_sendMasterRequest(2, 0x12, 8, Tx);
  Tx[0]++;
  delay(100);

  // send/ receive slave response
  LIN_receiveSlaveResponse(2, 0x23, 8, Rx);
  delay(100);
  
} // loop
