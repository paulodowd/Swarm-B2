// for i2c to connect to IR communication board.
#include <Wire.h>

// definition of data structs to operate IR
// communication board
#include "ircomm_i2c.h"

// The data struct we are going to request from
// the M5, which has received from the wifi
// tracking system.
#include "tracker_config.h"
volatile tracker_packet_t tracking_data;
int robot_id = 0;

// The i2c address for the M5, as defined in the
// example code M5_TrackerClient
#define M5_I2C_ADDR 0x55


// The data struct we are going to upload to the
// tracking system.

// robotID:
//  - this should be the id of the robot that sent the messaage, 1 : 199 possible IP addresses.
//  - if you set this to 0, the tracking system will just log the data as a result.


typedef struct tracker_upload {
int16_t robotID;
  int16_t Gen;
  float Fitness;
  float Vector[ 6 ];
} tracker_upload_t;
tracker_upload_t upload;

#define N_BYTES 6
typedef struct i2c_results {
  int16_t robotID;
  int16_t Gen;
  float Fitness;
  float Vector[ N_BYTES ];
} ir_results_t;

volatile ir_results_t results_data;
volatile ir_results_t robot_data;


#define BUZZER_PIN 6

int random_tone;


// A data structure to commmunicate
// the mode.
// Keep consistent between devices.
ir_mode_t ircomm_mode;

// A data structure to receive back
// the status of the board.
// Keep consistent between devices.
ir_status_t ircomm_status;


// Timestamps to periodically
unsigned long check_message_ts;


// What time interval in ms should we
// check / send new messages?
unsigned long check_message_ms = 100;    // every 100ms

void setup() {
  Serial.begin(115200);


  // Enable i2c on the 3Pi
  Wire.begin();

  // Wait for other systems to initialise
  delay(3000);

  Serial.println("RESET");

  // Let's use the buzzer pin so we can
  // hear when a robot is receiving a
  // message
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialise our results data to known
  // values.
  memset( (byte*)&results_data, 0, sizeof( results_data ));
  memset( (byte*)&upload, 0, sizeof( upload ));
  
  randomSeed( analogRead(A0 ));
  random_tone = random( 220, 880);

  // Make sure the IR Communication board
  // is reset and ready.
  doResetStatus();

  // Wait here until we receive a valid data
  // packet from the tracking system via
  // the M5.
  memset((byte*)&tracking_data, 0, sizeof( tracking_data ));
  do {
    Serial.println("Waiting for tracking data");
    robot_id = getTrackingDataFromM5();
    delay(50);
  } while ( tracking_data.valid == 0 || robot_id < 1 || robot_id > 199  );




  // Tell the IR Communication board to transmit
  // the ID number gained from the Wifi tracking
  // system as the message for this robot.
  //  char buf[32];
  //  memset( buf, 0, sizeof( buf ));
  //  //sprintf( buf, "%f", (float)tracking_data.marker_id );
  //  dtostrf((float)tracking_data.marker_id, 4, 2, buf);
  //  Serial.print("Setting IR message to: " );
  //  Serial.println(buf);
  //  setIRMessage(buf, strlen(buf));

  robot_data.robotID = tracking_data.marker_id;
  robot_data.Gen = 0;
  robot_data.Fitness = random(0, 100);
  for ( int i = 0; i < N_BYTES; i++ ) robot_data.Vector[i] = random(0, 100);
  setIRMessage( (byte*)&robot_data, sizeof( robot_data ) );


  // Initialise timestamp
  check_message_ts = millis();


}


// returns the id from the tracking system
int getTrackingDataFromM5() {
  Wire.requestFrom( M5_I2C_ADDR, sizeof( tracking_data ));
  Wire.readBytes( (uint8_t*)&tracking_data, sizeof( tracking_data ));


  Serial.println("Got from tracker:");
  Serial.print( tracking_data.marker_id );
  Serial.print(",");
  Serial.print( tracking_data.x );
  Serial.print(",");
  Serial.print( tracking_data.y );
  Serial.print(",");
  Serial.print( tracking_data.theta );
  Serial.print(",");
  Serial.print( tracking_data.valid );
  Serial.println("");

  return tracking_data.marker_id;
}


// For this example, we are assuming that getIRMessage()
// has modified our results_data struct, and then this
// function is called immediately afterwards.  See loop().
void sendResultsToM5() {
  Wire.beginTransmission( M5_I2C_ADDR );
  Wire.write( (byte*)&upload, sizeof( upload ));
  Wire.endTransmission();
}

void loop() {


  // Periodically check to see if there are new messages
  // waiting to be read from the communication board.
  if ( millis() - check_message_ts > check_message_ms ) {
    check_message_ts = millis();

    //    Serial.println("Loop");
    getTrackingDataFromM5();
    if ( tracking_data.valid ) {
      if ( tracking_data.marker_id > 0 && tracking_data.marker_id < 200 ) {
        robot_id = tracking_data.marker_id;
        robot_data.robotID = robot_id;
        robot_data.Gen = 0;
        robot_data.Fitness = random(0, 100);
        for ( int i = 0; i < N_BYTES; i++ ) robot_data.Vector[i] = random(0, 100);
        setIRMessage( (byte*)&robot_data, sizeof( robot_data ) );

      }

    }

    // Let's use a bool to understand if we got a message
    // on any receiver. We'll make a beep if any receiver
    // got an IR message.
    bool got_message = false;

    // Check all 4 receivers
    for ( int i = 0; i < 4; i++ ) {

      // If this returns more than 0, it means there
      // is a message waiting to be read.
      // The value of n is the number of bytes we need
      // to get from the IR Communication board
      int n = checkRxMsgReady( i );

      // If there is a message ready, we use 'n' as
      // the number of bytes to read down from the
      // communication board through receiver 'i'
      // (0,1,2 or 3).
      if ( n > 0 ) {

        got_message = true;


        // This function gets the message on receiver
        // 'i', and I've set it to populate our results
        // data struct for this example.
        getIRMessage( i, n );

        // Tell the M5 to upload our new results
        
        sendResultsToM5();


      } else {

        // We can send data to the tracking system even
        // if there has been no communication.
        //results_data.robotID = 0; // no valid, so the tracking system only writes this data to the csv file
        //results_data.Vector[0] = 99.9; // etc.
        // ...
        //sendResultsToM5();



        // n = 0, which means there were no bytes
        // available, no message.
      }
    }

    // Beep if we got a message
    if ( got_message ) {
      tone( BUZZER_PIN, random_tone, 10 );
    }

  }

}

/*

    Below are example functions to use the communication
    board.  Not all of them will be useful to you.

*/

// Completely resets the IR Communication board.
void doResetStatus() {
  ircomm_mode.mode = MODE_RESET_STATUS;
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();
}


// Use to see if there is a message available on
// receivers 0 to 3 (which_rx).
// If returns 0, no messages available.
int checkRxMsgReady(int which_rx) {

  if ( which_rx == 0 ) {
    ircomm_mode.mode = MODE_SIZE_MSG0;
  } else if ( which_rx == 1 ) {
    ircomm_mode.mode = MODE_SIZE_MSG1;
  } else if ( which_rx == 2 ) {
    ircomm_mode.mode = MODE_SIZE_MSG2;
  } else if ( which_rx == 3 ) {
    ircomm_mode.mode = MODE_SIZE_MSG3;
  } else {

    return 0;
  }

  // Set mode to read back how many bytes are
  // available.
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  // We'll use this struct to check how many
  // bytes are available of a message.
  // 0 bytes means no message.
  ir_msg_status_t msg_status;

  // Request the message size to be sent across into
  // msg_status
  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( msg_status ));
  Wire.readBytes( (uint8_t*)&msg_status, sizeof( msg_status ));

  return msg_status.n_bytes;// 0 : 32

}



// Use this function to set a message to transmit to
// other robots.  Once set, the robot will keep
// repeating the transmission.  The tranmission will
// occur periodically (between 150-300ms).
void setIRMessage(byte* data_to_send, int len) {

  // The communication board will always default
  // to waiting to receive a message to transmit
  // so we don't need to change the mode.
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)data_to_send, len);
  Wire.endTransmission();
}



// Get the latest message from the communication board
// from receiver "which_rx" (0,1,2,3).
// This is a little bit more complicated because we don't
// know how long a message will be. So we have to first
// ask how many bytes are available (present).
void getIRMessage(int which_rx, int n_bytes ) {
  if ( which_rx < 0 || which_rx >= 4 ) {
    // Invalid
    return;
  }


  // Format mode request for which receiver
  if ( which_rx == 0 ) {
    ircomm_mode.mode = MODE_REPORT_MSG0;
  } else if ( which_rx == 1 ) {
    ircomm_mode.mode = MODE_REPORT_MSG1;
  }  else if ( which_rx == 2 ) {
    ircomm_mode.mode = MODE_REPORT_MSG2;
  }  else if ( which_rx == 3 ) {
    ircomm_mode.mode = MODE_REPORT_MSG3;
  } else {
    // error should have been caught much earlier.
  }

  // Set mode to send across a full message
  Wire.beginTransmission( IRCOMM_I2C_ADDR );
  Wire.write( (byte*)&ircomm_mode, sizeof( ircomm_mode));
  Wire.endTransmission();

  ir_results_t rx_msg;
  memset( &rx_msg, 0, sizeof( rx_msg ) );

  Wire.requestFrom( IRCOMM_I2C_ADDR, sizeof( rx_msg ));
  Wire.readBytes( (byte*)&rx_msg, sizeof( rx_msg ));



  // Transmit the ID we received up to the tracking
  // system.
  upload.robotID = rx_msg.robotID;
}
