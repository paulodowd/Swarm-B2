
/*
*/

// For M5 functionality
#include <M5Unified.h>
#include <M5GFX.h>

// For Swarm-B2 board
#include <Wire.h>
#include "SwarmB2.h"

SwarmB2_c SwarmB2;

// Frame buffer to remove screen flicker
M5Canvas canvas(&M5.Display);

unsigned long bearing_ts;


// Last messages received
char buf[4][32];

void setup() {

  // For debugging
  Serial.begin(115200);

  Wire.begin();     // Required for SwarmB2
  Wire.setClock(400000);

  // Wait for connection
  //  while (!Serial);
  Serial.println("Ready");


  SwarmB2.init();   // Handles IR communication

  // Show user the current config
  //  SwarmB2.printRxSettings();
  //  SwarmB2.printTxSettings();

  // Setup a message to send, for testing.
  //  char buf[32];
  //  memset(buf, 0, sizeof(buf));
  //  sprintf(buf, "test%u", millis());
  //  SwarmB2.setIRMessage(buf, strlen(buf));

  // Setup the GUI buffer
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  canvas.setColorDepth(16);
  canvas.createSprite( M5.Display.width(), M5.Display.height() );
  M5.Display.fillScreen(TFT_BLACK);

  for (int i = 0; i < 4; i++ ) {
    memset( (void*)buf[i], 0, sizeof( buf[i] ));
  }

  bearing_ts = millis();
}


// Assuming screen in landscape mode, therefore height is the
// shorted dimension.
void drawGUI() {

  const float max_radius = (float)M5.Display.height()  / 2.0;
  const float x_offset = (M5.Display.width() - M5.Display.height()) / 2.0;
  const float centre_x = (float)M5.Display.width() / 2.0;
  const float centre_y = (float)M5.Display.height() / 2.0;

  ir_status_t ir_status = SwarmB2.getStatus();
  //    SwarmB2.printStatus();

  canvas.clear(BLACK);

  canvas.drawCircle(centre_x, centre_y, max_radius, TFT_WHITE);


  uint8_t mask;
  float angle;

  //   Indicate any receiver activity
  angle = 0;
  mask = 0x10;  // starting at bit 0b00010000
  for ( int i = 0; i < 4; i++ ) {
    angle = ((TWO_PI / 4.0) * i);
    float x = centre_x + (max_radius * sin(angle));
    float y = centre_y + (max_radius * cos(angle));

    if ( ir_status.bits & (mask << i ) ) {
      canvas.fillCircle( x, y, 12, PINK );
    }
  }


  //   Indicate messages correctly received
  angle = 0;
  mask = 0x01; // starting at bit 0b00000001
  for ( int i = 0; i < 4; i++ ) {
    if ( ir_status.bits & (mask << i ) ) {


      angle = ((TWO_PI / 4.0) * i);
      float x = centre_x + (max_radius * sin(angle));
      float y = centre_y + (max_radius * cos(angle));

      canvas.fillCircle( x, y, 10, TFT_GREEN );

      // Retrieve the message to clear the status bit
      SwarmB2.getIRMessage( (uint8_t*)buf[i], i );

    }
  }


  ir_bearing_t bearing = SwarmB2.getBearing();

  // maximum radius to draw the bearing indicator
  const float bearing_radius = (max_radius*0.66);

  // draw general direction indicator
  canvas.drawCircle( centre_x, centre_y, bearing_radius, DARKCYAN);

  // draw dot indicator for direction
  float x = centre_x + (bearing_radius) * sin( -bearing.theta );
  float y = centre_y + (bearing_radius) * cos( -bearing.theta );
  canvas.fillCircle( x, y, 10, CYAN);

  // sum of all activity
  canvas.fillCircle( centre_x, centre_y, bearing.sum * 0.25 * bearing_radius, DARKGREEN);

  // draw line indicator scaled to magnitude
  float x1 = (bearing.mag * bearing_radius) * sin( -bearing.theta  );
  float y1 = (bearing.mag * bearing_radius) * cos( -bearing.theta  );
  x1 += centre_x;
  y1 += centre_y;
  canvas.drawLine( centre_x, centre_y, x1, y1, TFT_GREEN);

  // Display list of current messages received
  canvas.setTextColor(DARKGREY);
  canvas.setCursor( 0,10 );
  for( int i = 0; i < 4; i++ ) {
    canvas.printf("Rx%d: %s\n", i, (char*)buf[i]);
  }


  // Finally,draw to screen
  canvas.pushSprite(0, 0);

}

void loop() {

  M5.update();

  drawGUI();

  delay(100);
}
