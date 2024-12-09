
/*
 * 9/12/24
 * Working from an arduino forum post:
 * https://forum.arduino.cc/t/crc-8-i2c-cyclic-redundancy-check/644812/3
 * 
 * Which references a microchip appnote:
 * https://ww1.microchip.com/downloads/en/AppNotes/00730a.pdf
 * 
 * 
 */
char CRC8(char *bytes, byte len) {
  const char generator = B00101111;   // polynomial = x^8 + x^5 + x^3 + x^2 + x + 1 (ignore MSB which is always 1)
  byte crc = 0;
  byte index = 0;
  while(len--) {  // while len > 0
    
    crc ^= bytes[index++]; /* XOR-in the next input byte */

    for (int i = 0; i < 8; i++) {
      if ((crc & 0x80) != 0) {
        crc = (uint8_t)((crc << 1) ^ generator);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void setup() {
  Serial.begin(9600);

}

// Assuming a message structure to have:
// - start token, 1 byte '*'    (byte  0)
// - checksum token, 1 byte '@' (byte 30)
// - checksum itself, 1 byte    (byte 31)
// Max message size is 32 bytes.
// Message content can be 29 bytes (32-3).
void loop() {

  char buf[32];
  memset(buf, 0, sizeof( buf ) );

  buf[0] = '*'; // start token
  for( int i = 1; i < 30; i++ ) { // up until the @ token
    // get random set of ascii values
    // to make up the test message
    buf[i] = 48 + random( 64 );
  }
  buf[30] = '@';
  
  Serial.print("Test message: ");
  Serial.println( buf ); 
  Serial.print("Test message len:");
  Serial.println( strlen( buf ) );

  unsigned long start_time = micros();
  uint8_t crc_byte = CRC8( buf, strlen( buf ) );
  unsigned long end_time = micros();

  Serial.print("CRC byte: ");
  Serial.println( crc_byte );
  Serial.print("CRC binary: ");
  Serial.println( crc_byte, BIN );
  Serial.print("time to compute (us): ");
  Serial.println( ( end_time - start_time ) );
  
}
