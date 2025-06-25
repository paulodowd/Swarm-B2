/*
   In this example, the M5 is configured to connect over
   Wifi and TCP/IP to the tracking system.  The M5 will
   gain an IP address from the Wifi, and then needs to
   display the AruCo marker with the number (e.g. if
   the IP is 192.168.1.4, display marker 4).

   In this example, we use the connection to the tracking
   system to also trigger the start of the robot
   behaviour.  To do this, the related Pololu3Pi_tracker
   example requests the tracker_packet_t data and it
   will only leave it's setup() once
   tracker_packet_t.valid == true.  Therefore, we first
   initialise tracker_packet_t.valid to false, until
   it is fixed by a stable connection (and data) from
   the tracking system.

   Once the Pololu3Pi_tracker loop() begins, it will
   send "results data" to the M5, which will then
   upload it back to the tracking system.

   This M5 example therefore also needs to respond
   to i2c request/receive functions from the Pololu
   3Pi+.

*/

#include <M5Core2.h>

// For connecting to the tracking system
#include <WiFi.h>
#include <WiFiMulti.h>
#include "tracker_config.h" // data exchanged with tracking system
volatile tracker_packet_t tracker_data;

// For connecting the M5 to the Pololu 3Pi+ as
// a slave device.
// We're expecting the Pololu 3Pi+ to send some
// data about it's IR communication so we define
// the data struct below.  Remember that the i2c
// bus is limited to 32 bytes max.
#include <Wire.h>
#define M5_I2C_ADDR 0x55


#pragma pack(1)
typedef struct i2c_results {
  int16_t robotID;
  int16_t Gen;
  float Fitness;
  float Vector[6];
} i2c_results_t;

volatile i2c_results_t results_data;

// We need to know when the Pololu 3Pi+ has
// sent new data over i2c to send to the
// tracking system over wifi / tcp/ip.
// This is toggled by the i2c_receive()
// function.
volatile bool upload_new_results = false;

// These pairs of bytes define unique AruCo Codes.
// Taking up 512 bytes of memory total.
// We can probably chop this down to just the first
// 100 or so.
// I've extracted [0..255] to match TCP/IP addresses.
// Taken from the json file at:
// https://github.com/okalachev/arucogen
// Using the 4x4 dictionary.
const byte dictionary[255][2] = {{181, 50}, {15, 154}, {51, 45}, {153, 70}, {84, 158}, {121, 205}, {158, 46}, {196, 242}, {254, 218}, {207, 86}, {249, 145}, {17, 167}, {14, 183}, {42, 15}, {36, 177}, {38, 62}, {70, 101}, {102, 0}, {108, 94}, {118, 175}, {134, 139}, {176, 43}, {204, 213}, {221, 130}, {254, 71}, {148, 113}, {172, 228}, {165, 84}, {33, 35}, {52, 111}, {68, 21}, {87, 178}, {158, 207}, {240, 203}, {8, 174}, {9, 41}, {24, 117}, {4, 255}, {13, 246}, {28, 90}, {23, 24}, {42, 40}, {50, 140}, {56, 178}, {36, 232}, {46, 235}, {45, 63}, {75, 100}, {80, 46}, {80, 19}, {81, 148}, {85, 104}, {93, 65}, {95, 151}, {104, 1}, {104, 103}, {97, 36}, {97, 233}, {107, 18}, {111, 229}, {103, 223}, {126, 27}, {128, 160}, {131, 68}, {139, 162}, {147, 122}, {132, 108}, {133, 42}, {133, 156}, {156, 137}, {159, 161}, {187, 124}, {188, 4}, {182, 91}, {191, 200}, {183, 171}, {202, 31}, {201, 98}, {217, 88}, {211, 213}, {204, 152}, {199, 160}, {197, 55}, {233, 93}, {249, 37}, {251, 187}, {238, 42}, {247, 77}, {53, 117}, {138, 173}, {118, 23}, {10, 207}, {6, 75}, {45, 193}, {73, 216}, {67, 244}, {79, 54}, {79, 211}, {105, 228}, {112, 199}, {122, 110}, {180, 234}, {237, 79}, {252, 231}, {254, 166}, {0, 37}, {0, 67}, {10, 136}, {10, 134}, {2, 111}, {0, 28}, {0, 151}, {8, 55}, {10, 49}, {9, 198}, {11, 1}, {9, 251}, {11, 88}, {16, 130}, {24, 45}, {16, 120}, {16, 115}, {18, 116}, {18, 177}, {26, 249}, {19, 6}, {12, 14}, {12, 241}, {4, 51}, {12, 159}, {14, 242}, {14, 253}, {7, 76}, {15, 164}, {7, 47}, {5, 181}, {15, 145}, {7, 219}, {30, 228}, {20, 57}, {29, 128}, {21, 200}, {31, 139}, {21, 186}, {29, 177}, {32, 128}, {40, 233}, {34, 162}, {40, 83}, {42, 240}, {34, 247}, {41, 64}, {33, 70}, {41, 185}, {43, 156}, {43, 178}, {56, 202}, {56, 46}, {48, 7}, {56, 231}, {58, 73}, {58, 101}, {50, 93}, {59, 136}, {57, 29}, {59, 211}, {38, 71}, {39, 128}, {47, 170}, {45, 20}, {37, 222}, {37, 83}, {47, 119}, {52, 72}, {60, 168}, {60, 65}, {52, 13}, {52, 251}, {54, 154}, {61, 224}, {53, 106}, {61, 9}, {61, 237}, {63, 196}, {63, 108}, {55, 206}, {61, 92}, {61, 118}, {55, 176}, {63, 23}, {63, 255}, {72, 229}, {66, 104}, {74, 45}, {65, 96}, {73, 81}, {65, 221}, {75, 223}, {88, 79}, {90, 72}, {88, 22}, {80, 93}, {90, 250}, {90, 181}, {81, 35}, {91, 138}, {89, 25}, {81, 53}, {76, 105}, {70, 193}, {78, 11}, {68, 95}, {78, 89}, {77, 131}, {77, 125}, {71, 216}, {71, 115}, {92, 133}, {94, 68}, {86, 43}, {92, 187}, {85, 195}, {95, 110}, {95, 235}, {93, 18}, {85, 94}, {98, 112}, {98, 21}, {97, 194}, {107, 32}, {99, 69}, {107, 92}, {107, 91}, {120, 12}, {122, 207}, {120, 127}, {121, 128}, {113, 229}, {113, 116}, {121, 182}, {113, 211}, {123, 51}, {100, 106}, {102, 168}, {110, 167}, {110, 145}, {101, 34}, {109, 203}, {103, 141}, {109, 49}, {126, 128}, {126, 226}, {126, 141}, {116, 210}, {124, 50}};


WiFiMulti WiFiMulti;
WiFiClient client;
IPAddress local_ip;



// The Tracking system sends updates
// every 100ms.
unsigned long tracker_update_ts;
#define TRACKER_UPDATE_MS 100



void setup() {

  M5.begin();
  M5.Lcd.fillScreen(BLACK);

  // Initialise our tracker data which the Pololu3Pi+
  // may ask for.
  tracker_data.marker_id = 0;
  tracker_data.x = 0.0;
  tracker_data.y = 0.0;
  tracker_data.theta = 0.0;
  tracker_data.valid = 0;

  // Start i2c
  Wire.begin( M5_I2C_ADDR );
  Wire.onReceive( i2c_receive );
  Wire.onRequest( i2c_request );

  // Enable this if you want to debug the wifi
  // connection.
  //WiFi.onEvent(WiFiEvent);

  Serial.begin(115200);
  delay(10);

  // We start by connecting to a WiFi network
  WiFiMulti.addAP("SwarmArenaWifi2G", "swarm-b2");
  Serial.println();
  Serial.print("Waiting for WiFi... ");

  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  local_ip = WiFi.localIP();
  Serial.println( local_ip[3] );

  // If we are not using the screen for anything else,
  // we only need to call drawAruCo once.
  if ( local_ip[3] > 0 && local_ip[3] < 200 ) {
    drawAruCo( local_ip[3] );
  }

  // Try to connect.  If we fail here,
  // we try again periodically in the loop
  connectClient();

  // Register when we start.
  tracker_update_ts = millis();

}



void loop() {

  // Periodically check for new tracking data over wifi
  if ( millis() - tracker_update_ts > TRACKER_UPDATE_MS ) {
    tracker_update_ts = millis();

    if ( client.connected() ) {
      if ( client.available() >= sizeof( tracker_packet_t ) ) {

        client.readBytes( (uint8_t*)&tracker_data, sizeof( tracker_packet_t) );
        Serial.print( tracker_data.marker_id ); Serial.print(",");
        Serial.print( tracker_data.x ); Serial.print(",");
        Serial.print( tracker_data.y ); Serial.print(",");
        Serial.print( tracker_data.theta ); Serial.print(",");
        Serial.print( tracker_data.valid ); Serial.print(",");
        Serial.println();
      } else {
        // No data to receive
      }
    } else { // client not connected/disconnected
      client.stop();
      connectClient();
    }
  }


  // This flag is toggled by the i2c receive interupt.
  // This flag becomes true if the 3Pi+ has uploaded 
  // a new i2c_results_t struct.  So we don't need to
  // do much here, just communicate the struct over 
  // tcp/ip and toggle the flag back to false
  if ( upload_new_results ) {
    if ( client.connected() ) {

      Serial.print("Received from robot: ");
      Serial.println( results_data.robotID );

      int err = client.write( (uint8_t*)&results_data, sizeof( results_data ) );

      if ( err == sizeof( results_data ) ) {
        // Upload was successful, toggle flag.
        upload_new_results = false;
        Serial.println("Sent new results!");

      } else {
        // We will need to try this again on the
        // next iteration of loop().
        Serial.println("Error sending results");
      }


    } else { // client not connected/disconnected
      client.stop();
      connectClient();
    }

  }


  // Other code to run could go here
  // ...

}



// This function will receive a data packet
// from the Pololu 3Pi+.  For this example, it
// is expecting to receive i2c_results_t defined
// above and in the Pololu3Pi_tracker example
// (these structs must match!).
void i2c_receive( int len ) {

  // Receiving the expected data struct
  if ( len == sizeof( i2c_results_t ) ) {

    Wire.readBytes( (uint8_t *)&results_data, sizeof(results_data) );

    // Flag for upload
    upload_new_results = true;

  } else {
    // Wrong data, just read out the data and
    // discard it.
    while ( Wire.available() ) {
      Wire.read();
    }

  }
}


// This function will send the tracking data down to
// the Pololu 3Pi+ when requested over i2c.  Note that
// we're only expecting the Pololu 3Pi+ to ask for this
// data, so we're not handling a request for different
// data (different structs).
void i2c_request() {

  // This is assuming that the tracker data
  // has been populated, either by setup or
  // by the wifi client.  We're not currently
  // using any mutex, and this data is being
  // updated by two sources.
  Wire.write( (uint8_t*)&tracker_data, sizeof( tracker_data ) );

}



// Screen is 320x240px.
// AruCo grid square width is therefore 240/4 = 60
// To draw code:
// - Read each bit of a byte backwards (e.g. bit 7, 6, 5...)
// - Starting with first byte then second
// - Iterate across and down grid
// - if bit(n) == 1, white square.
// Our device has a black border, so let's not
// draw a black border, making the AruCo as big as possible.
void drawAruCo(int index ) {

  // To help with size and placement of
  // the AruCo code.
  int x_offset = (320 - 240) / 2; // AruCo is square, cut edges
  int grid_w = 240 / 4;


  // Clear the screen
  M5.Lcd.fillScreen(BLACK);

  // Placing grid squares
  // We'll count down from 15
  int count = 15;
  for ( int y = 0; y < 4; y++ ) {
    for ( int x = 0; x < 4; x++ ) {

      // Count 0:7 = Parse byte 1
      if ( count < 8 ) {
        if ( (dictionary[index][1] >> count ) & 0x01 ) {
          // White
          // draw at x,y, height, width
          M5.Lcd.fillRect(x_offset + (x * grid_w), y * grid_w, grid_w, grid_w, WHITE);
        } else {
          // black - do nothing.
        }

      } else { // Count 8:15 = parse byte 0
        if ( (dictionary[index][0] >> (count - 8) ) & 0x01 ) {
          // white
          // draw at x,y, height, width
          M5.Lcd.fillRect(x_offset + (x * grid_w), y * grid_w, grid_w, grid_w, WHITE);
        } else {
          // black - do nothing.
        }
      }

      // Increment which bit we are inspecting
      count--;
    }
  }
}




// Returns err=1 if successful.
// Else <= 0 if unsucessful
bool connectClient() {

  client.setTimeout(1);

  unsigned long start_time = millis();
  // Connect to the tracking system.
  Serial.println("Connecting to tracking system");
  int err;

  err = client.connect( server_ip, server_port );
  //Serial.print(err);
  delay(250);

  return err;
}

// This is useful for debugging the wifi.
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event) {
    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("WiFi interface ready");
      break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi client started");
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      Serial.println("WiFi clients stopped");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected to access point");
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
      Serial.println("Authentication mode of access point has changed");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("Lost IP address and IP address is reset to 0");
      break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:
      Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_FAILED:
      Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:
      Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
      break;
    case ARDUINO_EVENT_WPS_ER_PIN:
      Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
      break;
    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println("WiFi access point started");
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println("WiFi access point  stopped");
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println("Client connected");
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.println("Client disconnected");
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      Serial.println("Assigned IP address to client");
      break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
      Serial.println("Received probe request");
      break;
    case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
      Serial.println("AP IPv6 is preferred");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
      Serial.println("STA IPv6 is preferred");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP6:
      Serial.println("Ethernet IPv6 is preferred");
      break;
    case ARDUINO_EVENT_ETH_START:
      Serial.println("Ethernet started");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("Ethernet stopped");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet connected");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet disconnected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("Obtained IP address");
      break;
    default: break;
  }
}
