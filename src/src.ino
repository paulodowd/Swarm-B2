
/*
 * A sketch to test the functionality of the 
 * SwarmB2_c class.
 * 
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
//  SwarmB2.updateSettings();
  SwarmB2.rx_settings.flags.bits.cycle = 1;
  SwarmB2.rx_settings.flags.bits.cycle_on_rx = 1;
  SwarmB2.setRxSettings();
  SwarmB2.resetMetrics();
  char buf[32];
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "test");
  SwarmB2.setIRMessage(buf, strlen(buf));

  
  SwarmB2.getTxSettings();
  SwarmB2.printTxSettings();
  unsigned long start = micros();
  SwarmB2.getRxSettings();
  unsigned long end = micros();
  Serial.println((end-start));
  SwarmB2.printRxSettings();
}

void loop() {

  //SwarmB2.getRxActivity();
  unsigned long start = micros();
  ir_bearing_t bearing = SwarmB2.getBearing();
  unsigned long end = micros();
  Serial.print( bearing.theta, 3 );
  Serial.print(","); 
  Serial.print( bearing.mag, 3 );
  Serial.print(","); 
  Serial.print( bearing.sum, 3 );
  Serial.print(",");
  Serial.println( (end-start));
  
//  Serial.print(",");
//  Serial.print(",");
//  
//  ir_crc_t crc = SwarmB2.getRxCRC();
//  for( int i = 0; i < 4; i++ ) {
//    Serial.print( crc.pass[i] );
//    Serial.print(",");
//  }
//  for( int i = 0; i < 4; i++ ) {
//    Serial.print( crc.fail[i] );
//    Serial.print(",");
//  }
//  ir_crc_t crc = SwarmB2.getRxCRC();
//  for( int i = 0; i < 4; i++ ) {
//    Serial.print( crc.pass[i] );
//    Serial.print(",");
//  }
//  for( int i = 0; i < 4; i++ ) {
//    Serial.print( crc.fail[i] );
//    Serial.print(",");
//  }
  Serial.println();
  
  delay(250);
}
