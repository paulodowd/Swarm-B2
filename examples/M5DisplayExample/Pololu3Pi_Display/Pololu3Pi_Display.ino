// for i2c to connect to the M5.
// the M5 is configured as a i2c slave device
// and will print any string it receives onto
// the display
#include <Wire.h>


#define M5_I2C_ADDR 0x55


void setup() {
  Serial.begin(115200);


  // Enable i2c on the 3Pi
  Wire.begin();

  delay(1000);



}


void loop() {

  // Let's send the current millis() count
  // just as an example
  // The maximum string length we can send
  // is 32 bytes
  char buf[32];

  // Ensure memory is clear
  memset(buf, 0, sizeof(buf));

  sprintf(buf, "%ul", millis() );
  
  Wire.beginTransmission( M5_I2C_ADDR );
  Wire.write( (byte*)buf, strlen( buf ));
  Wire.endTransmission();
  
  delay(500);

}
