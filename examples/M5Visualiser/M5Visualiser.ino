
/*
*/

// For M5 functionality
#include <M5Unified.h>
#include <M5GFX.h>

// For Swarm-B2 board
#include <Wire.h>
#include "SwarmB2.h"

// Instance of the class to communicate
// with SwarmB2, the IR communication
// board.
SwarmB2_c SwarmB2;

// Frame buffer to remove screen flicker
M5Canvas canvas(&M5.Display);

// Persistent variable to track the
// bearing estimation from SwarmB2
ir_bearing_t bearing;

// State tracker of which GUI to
// draw to the screen.
enum GuiState {
  MSG_INDICATOR,
  MSG_LIST,
  RGB_DEMO,
  METRICS,
  MAX_STATES
};
GuiState gui_state = MSG_INDICATOR;


// A struct of data to send as an
// IR message.  The maximum size
// that can be sent is 32 bytes.
typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} ir_msg_t;

ir_msg_t received_msgs[4];
ir_msg_t my_msg;

void setup() {

  // For debugging
  Serial.begin(115200);

  Wire.begin();     // Required for SwarmB2
  Wire.setClock(400000);

  // Wait for connection
  //  while (!Serial);
  Serial.println("Ready");

  uint32_t seed = esp_random();
  randomSeed(seed);

  SwarmB2.init();   // Handles IR communication

  // Create a message of a random
  // red/green/blue combination to
  // send over IR to other boards
  memset(&my_msg, 0, sizeof( ir_msg_t ));
  my_msg.red    = random(0, 255);
  my_msg.green  = random(0, 255);
  my_msg.blue   = random(0, 255);
  SwarmB2.setIRMessage( (uint8_t*)&my_msg, sizeof( ir_msg_t ));


  // Setup the GUI buffer
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  canvas.setColorDepth(16);
  canvas.createSprite( M5.Display.width(), M5.Display.height() );
  M5.Display.fillScreen(TFT_BLACK);

  // Clear out any old data
  memset( (uint8_t*)received_msgs, 0, sizeof( received_msgs ));

}


void drawMessageList() {
  canvas.clear(BLACK);

  // Get any new messages
  for ( int i = 0; i < 4; i++ ) {
    SwarmB2.getIRMessage( (uint8_t*)&received_msgs[i], i );
  }

  canvas.setTextColor(WHITE);
  canvas.setCursor( 0, 10 );
  canvas.printf("t = %lu\n", millis());
  for ( int i = 0; i < 4; i++ ) {
    canvas.printf("Rx%d: %d, %d, %d\n", i, received_msgs[i].red, received_msgs[i].green, received_msgs[i].blue);
  }

  // Finally,draw to screen
  canvas.pushSprite(0, 0);
}

void drawRGBDemo() {
  canvas.clear(BLACK);

  const float max_radius = (float)M5.Display.height()  / 2.0;
  const float x_offset = (M5.Display.width() - M5.Display.height()) / 2.0;
  const float centre_x = (float)M5.Display.width() / 2.0;
  const float centre_y = (float)M5.Display.height() / 2.0;

  // Draw a centre circle repesenting
  // the colour being sent from this
  // board.
  uint16_t colour = M5.Lcd.color565( my_msg.red, my_msg.green, my_msg.blue);
  canvas.fillCircle( centre_x, centre_y, 30, colour );

  // Get any new messages
  for ( int i = 0; i < 4; i++ ) {
    SwarmB2.getIRMessage( (uint8_t*)&received_msgs[i], i );

    // Draw a circle representing the rgb
    // received on each receiver
    float angle = ((TWO_PI / 4.0) * i);
    float x = centre_x + (max_radius * sin(angle));
    float y = centre_y + (max_radius * cos(angle));
    colour = M5.Lcd.color565( received_msgs[i].red, received_msgs[i].green, received_msgs[i].blue);
    canvas.fillCircle(x, y, 30, colour);

  }

  // Finally,draw to screen
  canvas.pushSprite(0, 0);
}

void drawMetrics() {

  canvas.clear(BLACK);

  ir_vectors_t      vectors     = SwarmB2.getRxVectors();
  ir_activity_t     activity    = SwarmB2.getRxActivity();
  ir_saturation_t   saturation  = SwarmB2.getRxSaturation();
//  ir_msg_timings_t  msg_timings = SwarmB2.getMsgTimings();
//  ir_byte_timings_t byte_timings = SwarmB2.getByteTimings();
//  ir_skips_t        rx_skips    = SwarmB2.getRxSkips();
//  ir_errors_t       rx_errors   = SwarmB2.getRxErrors();
  ir_crc_t          crc         = SwarmB2.getRxCRC();
  
  canvas.setTextColor(WHITE);
  canvas.setCursor( 0, 10 );
  canvas.printf("t = %lu\n", millis());

  canvas.printf("Vectors:\t%.2f,\t%.2f,\t%.2f,\t%.2f\n", vectors.rx[0], vectors.rx[1],vectors.rx[2],vectors.rx[3]);  
  canvas.printf("Activity:\t%lu,\t%lu,\t%lu,\t%lu\n", activity.rx[0], activity.rx[1],activity.rx[2],activity.rx[3]);
  canvas.printf("Saturation:\t%lu,\t%lu,\t%lu,\t%lu\n", saturation.rx[0], saturation.rx[1],saturation.rx[2],saturation.rx[3]);
  canvas.printf("Msgs PASS:\t%lu,\t%lu,\t%lu,\t%lu\n", crc.pass[0],crc.pass[1],crc.pass[2],crc.pass[3]);
  canvas.printf("Msgs FAIL:\t%lu,\t%lu,\t%lu,\t%lu\n", crc.fail[0],crc.fail[1],crc.fail[2],crc.fail[3]);
  // Finally,draw to screen
  canvas.pushSprite(0, 0);

}

// Assuming screen in landscape mode, therefore height is the
// shorted dimension.
void drawMessageIndicators() {

  const float max_radius = (float)M5.Display.height()  / 2.0;
  const float x_offset = (M5.Display.width() - M5.Display.height()) / 2.0;
  const float centre_x = (float)M5.Display.width() / 2.0;
  const float centre_y = (float)M5.Display.height() / 2.0;

  ir_status_t ir_status = SwarmB2.getStatus();

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
      SwarmB2.getIRMessage( (uint8_t*)&received_msgs[i], i );

    }
  }


  bearing = SwarmB2.getBearing();

  // maximum radius to draw the bearing indicator
  const float bearing_radius = (max_radius * 0.66);

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

  // Finally,draw to screen
  canvas.pushSprite(0, 0);

}

void loop() {

  M5.update();

  // If the screen is touched, change the
  // gui displayed
  if ( M5.Touch.getCount() > 0 ) {
    auto touch = M5.Touch.getDetail(0);
    if ( touch.wasPressed() ) {
      gui_state = GuiState((gui_state + 1 ) % MAX_STATES);
    }
  }

  switch ( gui_state ) {
    case MSG_INDICATOR: drawMessageIndicators();
      break;
    case MSG_LIST: drawMessageList();
      break;
    case RGB_DEMO: drawRGBDemo();
      break;
    case METRICS: drawMetrics();
      break;
  }


  delay(100);
}
