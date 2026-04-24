
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
  while (!Serial);
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


void drawGUI() {
  const int central_w_h = M5.Display.height() - 1;

  const int x_offset = (M5.Display.width() - M5.Display.height()) / 2;
  //canvas.drawRect( x_offset, 0, central_w_h, central_w_h, TFT_DARKGREY);


  ir_status_t ir_status = SwarmB2.getStatus();

  //zzzzzzzzzzzzSwarmB2.printStatus();


  /*
   * Indicate messages correctly received
   */
  // Forward = bottom of m5
  if ( ir_status.bits & 0b00000001 ) {
    SwarmB2.getIRMessage( (uint8_t*)buf[0], 0 );
    canvas.drawLine( x_offset, central_w_h, x_offset + central_w_h, central_w_h, TFT_GREEN);
  } else {
    canvas.drawLine( x_offset, central_w_h, x_offset + central_w_h, central_w_h, TFT_BLACK);
  }

  // left = right of m5
  if ( ir_status.bits & 0b00000010 ) {
    SwarmB2.getIRMessage( (uint8_t*)buf[1], 1 );
    canvas.drawLine( x_offset + central_w_h, 0, x_offset + central_w_h, central_w_h, TFT_GREEN);
  } else {
    canvas.drawLine( x_offset + central_w_h, 0, x_offset + central_w_h, central_w_h, TFT_BLACK);
  }

  // back = top of m5
  if ( ir_status.bits & 0b00000100 ) {
    SwarmB2.getIRMessage( (uint8_t*)buf[2], 2 );
    canvas.drawLine( x_offset, 0, x_offset + central_w_h, 0, TFT_GREEN);
  } else {
    canvas.drawLine( x_offset, 0, x_offset + central_w_h, 0, TFT_BLACK);
  }

  // right = left of m5
  if ( ir_status.bits & 0b00001000 ) {
    SwarmB2.getIRMessage( (uint8_t*)buf[3], 3 );
    canvas.drawLine( x_offset, 0, x_offset, central_w_h, TFT_GREEN);
  } else {
    canvas.drawLine( x_offset, 0, x_offset, central_w_h, TFT_BLACK);
  }


  /*
   * Indicate activity
   */
  // Forward = bottom of m5
  if ( ir_status.bits & 0b00010000 ) {
    canvas.drawLine( x_offset+1, central_w_h-1, x_offset + central_w_h-1, central_w_h-1, TFT_BLUE);
  } else {
    canvas.drawLine( x_offset+1, central_w_h-1, x_offset + central_w_h-1, central_w_h-1, TFT_BLACK);
  }

  // left = right of m5
  if ( ir_status.bits & 0b00100000 ) {
    canvas.drawLine( x_offset + central_w_h-1, 1, x_offset + central_w_h-1, central_w_h-1, TFT_BLUE);
  } else {
    canvas.drawLine( x_offset + central_w_h-1, 1, x_offset + central_w_h-1, central_w_h-1, TFT_BLACK);
  }

  // back = top of m5
  if ( ir_status.bits & 0b0100000 ) {
    canvas.drawLine( x_offset+1, 1, x_offset + central_w_h-1, 1, TFT_BLUE);
  } else {
    canvas.drawLine( x_offset+1, 1, x_offset + central_w_h-1, 1, TFT_BLACK);
  }

  // right = left of m5
  if ( ir_status.bits & 0b10000000 ) {
    canvas.drawLine( x_offset+1, 1, x_offset+1, central_w_h-1, TFT_BLUE);
  } else {
    canvas.drawLine( x_offset+1, 1, x_offset+1, central_w_h-1, TFT_BLACK);
  }

  // Draw bearing information at a slower interval
  if ( millis() - bearing_ts > 100 ) {
    bearing_ts = millis();
    ir_bearing_t bearing = SwarmB2.getBearing();

    const float bearing_radius = 50;

    // "Clear" this area of the screen by placing a black circle
    canvas.fillCircle( M5.Display.width()/2, M5.Display.height()/2, bearing_radius, TFT_BLACK);

    canvas.fillCircle( M5.Display.width()/2, M5.Display.height()/2, bearing.sum, TFT_BLUE);

    // resolve bearing to x,y
    float x1 = (bearing.mag * bearing_radius) * sin( -bearing.theta );
    float y1 = (bearing.mag * bearing_radius) * cos( -bearing.theta );
    x1 += M5.Display.width()/2;
    y1 += M5.Display.height()/2;
    canvas.drawLine( M5.Display.width()/2, M5.Display.height()/2, x1,y1, TFT_GREEN);
    
  }


  // Finally,draw to screen
  canvas.pushSprite(0, 0);

}

void loop() {

  M5.update();

  drawGUI();

  delay(50);
}
