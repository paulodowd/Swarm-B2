
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

  // Wait for connection
  while(!Serial);
  Serial.println("Ready");
  
  SwarmB2.init();   // Handles IR communication

  // Show user the current config
  SwarmB2.printRxSettings();
  SwarmB2.printTxSettings();

  // Setup a message to send, for testing.
//  char buf[32];
//  memset(buf, 0, sizeof(buf));
//  sprintf(buf, "test%u", millis());
//  SwarmB2.setIRMessage(buf, strlen(buf));

}

void loop() {
  SwarmB2.printAnyMessage();
  delay(20);
}
