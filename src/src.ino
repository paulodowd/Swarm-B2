
/*
   A sketch to test the functionality of the
   SwarmB2_c class.

*/
#include <Wire.h>
#include "SwarmB2.h"

SwarmB2_c SwarmB2;



void setup() {
  Serial.begin(115200);
  Wire.begin();     // Required for SwarmB2
  Wire.setClock(400000);

  delay(100);
  SwarmB2.init();   // Handles IR communication

  delay(2000);
  SwarmB2.printRxSettings();
  
  SwarmB2.updateSettings();

  SwarmB2.getTxSettings();
  SwarmB2.printTxSettings();
  SwarmB2.getRxSettings();
  SwarmB2.printRxSettings();
  char buf[32];
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "test%u", millis());
  SwarmB2.setIRMessage(buf, strlen(buf));

}

void loop() {

//  SwarmB2.getRxActivity();

//    ir_bearing_t bearing = SwarmB2.getBearing();
//    Serial.print( bearing.theta, 3 );
//    Serial.print(",");
//    Serial.print( bearing.mag, 3 );
//    Serial.print(",");
//    Serial.print( bearing.sum, 3 );
//    Serial.print(",");
//

  //  Serial.print(",");
  //  Serial.print(",");
  //

  ir_skips_t skips = SwarmB2.getRxSkips();
  
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( skips.rx[i] );
    Serial.print(",");
  }
  
  ir_crc_t crc = SwarmB2.getRxCRC();
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( crc.pass[i] );
    Serial.print(",");
  }
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( crc.fail[i] );
    Serial.print(",");
  }

  ir_activity_t activity = SwarmB2.getRxActivity();
  for ( int i = 0; i < 4; i++ ) {
    Serial.print( activity.rx[i] );
    Serial.print(",");
  }

//  ir_cycles_t cycles = SwarmB2.getCycles();
//    Serial.print( cycles.rx );
//    Serial.print(",");
//    Serial.print( cycles.tx );
//    Serial.print("\n");
  
  //
    ir_frame_errors_t frame_errs = SwarmB2.getRxFrameErrors();
    Serial.print("Fe:");
    Serial.print( frame_errs.rx[0] );
    Serial.print(",");
    ir_errors_t errors = SwarmB2.getRxErrors();
    Serial.print("R:");
    Serial.print( errors.type[0][0] );
    Serial.print(",L:");
    Serial.print( errors.type[0][1] );
    Serial.print(",C:");
    Serial.print( errors.type[0][2] );
    Serial.print(",T:");
    Serial.print( errors.type[0][3] );




  Serial.println();

  char buf[32];
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "test%u", millis());
  SwarmB2.setIRMessage(buf, strlen(buf));
  //  Serial.print("Sending:  ");
  //  Serial.print( buf );
  //  Serial.print(" Len: " );
  //  Serial.print( strlen(buf));
  //  Serial.println();
  delay(250);
}
