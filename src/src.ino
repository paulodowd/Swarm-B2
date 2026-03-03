
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
  //  SwarmB2.updateSettings();
  SwarmB2.rx_settings.flags.bits.cycle_on_rx    = false;
  SwarmB2.rx_settings.flags.bits.desync         = false;
  SwarmB2.rx_settings.flags.bits.overrun        = true;
  SwarmB2.rx_settings.flags.bits.rand_rx        = false;

  SwarmB2.rx_settings.flags.bits.rx0            = true;
  SwarmB2.rx_settings.flags.bits.rx1            = true;
  SwarmB2.rx_settings.flags.bits.rx2            = true;
  SwarmB2.rx_settings.flags.bits.rx3            = true;

  SwarmB2.rx_settings.skip_multi                = 0;
  SwarmB2.rx_settings.predict_multi             = 1.0;
  SwarmB2.rx_settings.index                     = 0;
  SwarmB2.rx_settings.period_base_ms            = 32;
  SwarmB2.rx_settings.byte_timeout_ms           = 12;
  SwarmB2.rx_settings.sat_timeout_us            = 0;
  SwarmB2.setRxSettings();
  SwarmB2.resetMetrics();

  SwarmB2.tx_settings.preamble_repeat = 0;
  SwarmB2.tx_settings.period_base_ms = 170;
  SwarmB2.tx_settings.period_ms = 0;
  SwarmB2.setTxSettings();


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

  //SwarmB2.getRxActivity();
  //  unsigned long start = micros();
  //  ir_bearing_t bearing = SwarmB2.getBearing();
  //  unsigned long end = micros();
  //  Serial.print( bearing.theta, 3 );
  //  Serial.print(",");
  //  Serial.print( bearing.mag, 3 );
  //  Serial.print(",");
  //  Serial.print( bearing.sum, 3 );
  //  Serial.print(",");
  //  Serial.println( (end-start));

  //  Serial.print(",");
  //  Serial.print(",");
  //
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

  ir_cycles_t cycles = SwarmB2.getCycles();
    Serial.print( cycles.rx );
    Serial.print(",");
    Serial.print( cycles.tx );
    Serial.print("\n");
  
  //
  //  ir_errors_t errors = SwarmB2.getRxErrors();
  //  Serial.print("Resync:");
  //  Serial.print( errors.type[0][0] );
  //  Serial.print(",Len:");
  //  Serial.print( errors.type[0][1] );
  //  Serial.print(",CRC:");
  //  Serial.print( errors.type[0][2] );
  //  Serial.print(",TO:");
  //  Serial.print( errors.type[0][3] );




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
