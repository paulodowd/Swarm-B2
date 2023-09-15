#include <M5Core2.h>
#include <Wire.h>           // i2c to connect to IR communication board.
#define IRCOMM_I2C_ADDR  8

#pragma pack(1)

// Currently 23 bytes (max 32)
typedef struct ircomm_status {
  uint8_t mode;                       // 1  bytes
  uint8_t ldr[3];                     // 6  bytes
  unsigned long msg_count[4];         // 16 bytes
} ircomm_status_t;

typedef struct i2c_mode {
  uint8_t mode;
} i2c_mode_t;
#define MODE_REPORT_STATUS  0
#define MODE_NEW_MSG        1
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

void setup() {
  M5.begin();
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.begin();

  last_msg_count[0] = 0;
  last_msg_count[1] = 0;
  last_msg_count[2] = 0;
  last_msg_count[3] = 0;


  M5.Lcd.setTextColor(WHITE);
  M5.lcd.println("\nSetting up I2C");
}

void setIRMessage(char* str_to_send, int len){

  if( len < 32 && len > 1 ) {
  
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

  if ( count > 0 ) {
    //Serial.print("Received on 0:" );
    //Serial.println( buf );
    M5.Lcd.clear( BLACK );
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);
    M5.lcd.println( buf );

  }

}

void loop() {
  // put your main code here, to run repeatedly:

  // Read values back from the robot.

  //getIRMessage(0);
  //setIRMessage("123.4,56.78,910.1", 17);

  

    Serial.println("Read: ");
    Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( ircomm_status ));
    Wire.readBytes( (uint8_t*)&ircomm_status, sizeof( ircomm_status ));
    M5.Lcd.clear( BLACK );
    M5.Lcd.setCursor(0, 0);

    for ( int i = 0; i < 3; i++ ) {
    float percent = ircomm_status.ldr[i];
    percent /= 1024.0;
    percent *= 50.0; // screen can display 50 characters?
    for ( int j = 0; j < (int)percent; j++ ) M5.lcd.print("-");
    M5.lcd.println("");
    }
    M5.lcd.setTextSize(1);
    M5.lcd.print( ircomm_status.ldr[0] ); M5.lcd.print(",");
    M5.lcd.print( ircomm_status.ldr[1] ); M5.lcd.print(",");
    M5.lcd.print( ircomm_status.ldr[2] ); M5.lcd.println("");

    M5.lcd.print( ircomm_status.msg_count[0] ); M5.lcd.print(",");
    M5.lcd.print( ircomm_status.msg_count[1] ); M5.lcd.print(",");
    M5.lcd.print( ircomm_status.msg_count[2] ); M5.lcd.print(",");
    M5.lcd.print( ircomm_status.msg_count[3] ); M5.lcd.println("");

    M5.lcd.println(ircomm_status.mode);

    long msg_change[4];
    for ( int i = 0; i < 4; i++ ) {
    msg_change[i] = ircomm_status.msg_count[i] - last_msg_count[i];
    last_msg_count[i] = ircomm_status.msg_count[i];

    }

    if ( msg_change[0] > 0 ) {  // Front of robot, base of screen
    M5.Lcd.fillCircle(160, 240, 25, GREEN);
    }

    if ( msg_change[1] > 0 ) {  // Left of robot, right of screen
    M5.Lcd.fillCircle(320, 120, 25, GREEN);
    }


    if ( msg_change[2] > 0 ) {  // Back of robot, top of screen
    M5.Lcd.fillCircle(160, 0, 25, GREEN);
    }

    if ( msg_change[3] > 0 ) {  // Right of robot, left of screen
    M5.Lcd.fillCircle(0, 120, 25, GREEN);
    }


  



  delay(100);

}
