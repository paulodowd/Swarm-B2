#include <Wire.h>           // i2c to connect to IR communication board.
#define IRCOMM_I2C_ADDR  8

#pragma pack(1)

// Data structure used to get general 
// status from the IR communication 
// board (the arduino Nano)
// Currently using 23 bytes (max 32)
typedef struct ircomm_status {
  uint8_t mode;                       // 1  bytes
  uint8_t ldr[3];                     // 6  bytes
  unsigned long msg_count[4];         // 16 bytes
} ircomm_status_t;

typedef struct i2c_mode {
  uint8_t mode;
} i2c_mode_t;

#define MODE_REPORT_STATUS  0
#define MODE_STOP_TX        1
#define MODE_REPORT_LDR0    2
#define MODE_REPORT_LDR1    3
#define MODE_REPORT_LDR2    4
#define MODE_REPORT_MSG0    5
#define MODE_REPORT_MSG1    6
#define MODE_REPORT_MSG2    7
#define MODE_REPORT_MSG3    8

i2c_mode_t ircomm_mode;
ircomm_status_t ircomm_status;

unsigned long last_msg_count[4];

unsigned long new_msg_ts;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  last_msg_count[0] = 0;
  last_msg_count[1] = 0;
  last_msg_count[2] = 0;
  last_msg_count[3] = 0;

  new_msg_ts = millis();
}

void setIRMessage(char* str_to_send, int len) {

  if ( len < 32 && len > 1 ) {

    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)str_to_send, len);
    Wire.endTransmission();
  }
}

void getIRMessage(int which_rx ) {
  if ( which_rx < 0 || which_rx >= 4 ) {
    return;
  }

  if ( which_rx == 0 ) {

    // Set the IRComm board to the right mode
    ircomm_mode.mode = MODE_REPORT_MSG0;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();

  } else if ( which_rx == 1 ) {

    // Set the IRComm board to the right mode
    ircomm_mode.mode = MODE_REPORT_MSG1;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();
  } else if ( which_rx == 2 ) {

    // Set the IRComm board to the right mode
    ircomm_mode.mode = MODE_REPORT_MSG2;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();
  } else if ( which_rx == 3 ) {

    // Set the IRComm board to the right mode
    ircomm_mode.mode = MODE_REPORT_MSG3;
    Wire.beginTransmission( IRCOMM_I2C_ADDR );
    Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
    Wire.endTransmission();
  }

  // Prepare to receive.
  char buf[32];
  int count = 0;
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( buf ));
  while ( Wire.available() && count < 32 ) {
    char c = Wire.read();
    if ( c == '!' ) break; // end of message
    buf[count] = c;
    count++;
  }


  // What should we do with the message just received?
  if ( count > 0 ) {
    Serial.print("Received on " );
    Serial.print( which_rx );
    Serial.print(": ");
    //Serial.println( buf );
    Serial.println( buf );

  }

}

void loop() {
  // put your main code here, to run repeatedly:

  // Read values back from the robot.

  //getIRMessage(0);
  //setIRMessage("123.4,56.78,910.1", 17);


  // Send a message every 5000ms (5s)
  if ( millis() - new_msg_ts > 5000) {
    
    new_msg_ts = millis();


    // Let's just send the current time from millis()
    // as an example.

    char buf[32];
    float f_to_send = (float)millis();
    f_to_send /= 1000.0;
    
    // Convert float to a string, store in the
    // message buffer.
    // I had a lot of trouble finding a solution for this.
    // This is an odd, non-standard function I think.
    // dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
    // https://www.programmingelectronics.com/dtostrf/
    //  - a minimum of 6 character (e.g. 000.00)
    //  - 2 digits after decimal
    //  - store in buf
    dtostrf(f_to_send, 6, 2, buf);

    // This function call tells the communication board
    // (the nano) to start ending the requested message.
    // It will keep doing this until a new call to this
    // function is made.
    setIRMessage(buf, strlen(buf));
  }


  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ircomm_status ));
  Wire.readBytes( (uint8_t*)&ircomm_status, sizeof( ircomm_status ));
  

  // Report how many messages have been received on each
  // receiver
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( ircomm_status.msg_count[i] );
    Serial.print(",");
  }
  Serial.println();

  delay(100);

}
