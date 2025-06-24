#include <M5Core2.h>
#include "Wire.h"

#define M5_I2C_ADDR 0x55

unsigned long disp_update_ts;
#define DISP_UPDATE_MS  40

#define DISP_MAX_LINES  20 // I've not check if this is good
#define MAX_BYTES       32 // limited by i2c
char display_buf[ DISP_MAX_LINES ][ MAX_BYTES ];
int display_line_index = 0;

volatile boolean disp_update = false;

// Function to receive commands from the master device.
// This will typically be in the format of a mode change
// and then any appropriate data.
void i2c_receive( int len ) {

  // Read the incoming bytes from i2c
  // and store into our display buffer

  // Clear this line of the buffer
  memset(display_buf[display_line_index], 0, sizeof( display_buf[display_line_index] ));

  // We will make sure we only read in MAX_BYTES
  int count = 0;
  while ( Wire.available() && count < MAX_BYTES ) {
    char c = Wire.read();
    display_buf[display_line_index][count] = c;
    count++;
  }

  // Read the remaining bytes if any.
  while ( count < len ) Wire.read();

  // increment our display buffer line index
  display_line_index++;
  if ( display_line_index >= DISP_MAX_LINES ) display_line_index = 0;

  disp_update = true;

}

void setup() {

  // Setup the m5
  M5.begin();

  // Do we need to update the display?
  // initially, no.
  disp_update = false;

  // Clear out our display text buffer
  for ( int i = 0; i < DISP_MAX_LINES; i++ ) {
    memset( display_buf[i], 0, sizeof( display_buf[i] ) );
  }


  // Serial for debugging
  // Might be easier to debug by
  // printing to the LCD to avoid having
  // a cable plugged in.
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  // Start i2c as the master
  Wire.begin( M5_I2C_ADDR );

  // Acting as a display, we only
  // expect to receive data from
  // the Pololu 3Pi+ to then put
  // onto the screen.
  Wire.onReceive( i2c_receive );


  // Clear LCD Screen
  M5.Lcd.clear( BLACK );
  M5.Lcd.setCursor(0, 0);

  disp_update_ts = millis();

}


// Periodically update the display
// to show what is in our display
// text buffer.
void loop() {

  if ( disp_update ) {
    disp_update = false;

    // Clear LCD Screen
    M5.Lcd.clear( BLACK );
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(1);

    // Print each line of the buffer.
    // Note, if you send a string with a newline \n,
    // you'll probably get two newlines on the screen
    for ( int i = 0; i < DISP_MAX_LINES; i++ ) {
      M5.Lcd.println( display_buf[i] );
    }
  }
}
