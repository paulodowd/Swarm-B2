
uint16_t CRC16( char * bytes, byte len ) {
  const uint16_t generator = 0x1021; /* divisor is 16bit */
  uint16_t crc = 0; /* CRC value is 16bit */
  byte index = 0;
  while( len-- ) {
    crc ^= (uint16_t)(bytes[index++] << 8); /* move byte into MSB of 16bit CRC */

            for (int i = 0; i < 8; i++) {
    if ((crc & 0x8000) != 0) { /* test for MSB = bit 15 */
        crc = (uint16_t)( (crc << 1) ^ generator);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

void splitCRC16( byte * l_byte, byte * u_byte, uint16_t crc ) {
  *l_byte = crc & 0xFF;
  *u_byte = crc >> 8;
}
uint16_t mergeCRC16( byte l_byte, byte u_byte ) {
  return ((u_byte << 8 ) | l_byte );
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
  uint16_t crc_byte = CRC16( buf, strlen( buf ) );
  byte lb,ub;
  splitCRC16( &lb, &ub, crc_byte);
  uint16_t merged = mergeCRC16( lb, ub );
  
  unsigned long end_time = micros();

  Serial.print("CRC byte: ");
  Serial.println( crc_byte );
  Serial.print("CRC binary: ");
  Serial.println( crc_byte, BIN );
  Serial.print( lb, BIN ); Serial.print(" ");
  Serial.println( ub, BIN );
  Serial.println( merged );
  
  Serial.print("time to compute (us): ");
  Serial.println( ( end_time - start_time ) );
  
}
