#include <Arduino.h>
#include <stdint.h>


static const uint8_t START_BYTE = '~';   // 0x7E
static const uint8_t ESC_BYTE   = '^';   // 0x5E

static const uint8_t MIN_PAYLOAD_LEN  = 1;
static const uint8_t MAX_PAYLOAD_LEN  = 32;

static const uint8_t MIN_DECODED_LEN  = MIN_PAYLOAD_LEN + 2;   // = 3
static const uint8_t MAX_DECODED_LEN  = MAX_PAYLOAD_LEN + 2;   // = 34


// CRC-16 ANSI
uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

inline bool mustEscape(uint8_t b) {
  return (b == START_BYTE) ||
         (b == ESC_BYTE)   ||
         (b == 0 ) ||
         (b >= MIN_DECODED_LEN && b <= MAX_DECODED_LEN);
}

// -----------------------------------------------------
// Emit a single encoded byte (1 or 2 output bytes)
// -----------------------------------------------------
inline void encodeByte(uint8_t b, uint8_t* out, uint16_t& pos) {
    if (mustEscape(b)) {
        out[pos++] = ESC_BYTE;
        out[pos++] = (b ^ 0x20);
    } else {
        out[pos++] = b;
    }
}


// -----------------------------------------------------
// FINAL ENCODER (simple, no shifting)
// -----------------------------------------------------
uint16_t encodeMessage(
    const uint8_t* payload,
    uint8_t payloadLen,
    uint8_t* outBuffer
) {
    if (payloadLen < MIN_PAYLOAD_LEN) return 0;
    if (payloadLen > MAX_PAYLOAD_LEN) return 0;

    uint16_t pos = 0;

    // 1. Write start marker
    outBuffer[pos++] = START_BYTE;

    // 2. Compute decoded length = payload + CRC2
    uint8_t decodedLen = payloadLen + 2;

    // 3. Encode decoded length (always 2 bytes)
    encodeByte(decodedLen, outBuffer, pos);

    // 4. Copy decoded payload + CRC into temp array
    uint8_t tmp[MAX_PAYLOAD_LEN + 2];
    memcpy(tmp, payload, payloadLen);

    uint16_t crc = crc16(tmp, payloadLen);
    tmp[payloadLen]     = (uint8_t)(crc >> 8);
    tmp[payloadLen + 1] = (uint8_t)(crc & 0xFF);

    // 5. Encode all decoded bytes (payload + CRC)
    for (uint8_t i = 0; i < decodedLen; ++i) {
        encodeByte(tmp[i], outBuffer, pos);
    }

    return pos;  // total encoded length
}



// ---------------- Decoder State ----------------
enum RxState { WAIT_START, WAIT_LEN, READ_PAYLOAD };
static RxState rxState = WAIT_START;

static bool escapeNext = false;
static uint8_t decodedLength = 0;             // total bytes expected (payload + CRC)
static uint8_t recvBuffer[MAX_DECODED_LEN];   // holds decoded bytes
static uint8_t recvPos = 0;


// ============================================================
// CRC-16 ANSI (polynomial 0xA001)
// ============================================================
static uint16_t crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}


// ============================================================
// Structured Statistics
// ============================================================
struct DecoderStats {
  uint32_t invalidStartResync = 0;
  uint32_t invalidLength      = 0;
  uint32_t overflow           = 0;
  uint32_t crcFail            = 0;
  uint32_t unexpectedByteInStart = 0;
  uint32_t ignoredByteInLen   = 0;
  uint32_t escapeDecoded      = 0;
  uint32_t escAtEnd           = 0;
};

static DecoderStats stats;


// ============================================================
// Process one decoded (non-escaped) byte
// Handles the decoder state machine
// Returns true when a full CRC-validated message is complete
// ============================================================
static bool processDecodedByte(uint8_t b, bool from_escape, uint8_t* out, uint8_t& outLen, uint8_t i) {

  switch (rxState) {

    // ---------------- WAIT_START ----------------
    case WAIT_START:
      if (b == START_BYTE) {
        rxState = WAIT_LEN;
        decodedLength = 0;
        recvPos = 0;
      } else {
        stats.unexpectedByteInStart++;
      }
      return false;


    // ---------------- WAIT_LEN ----------------
    case WAIT_LEN:
      if (b == START_BYTE && !from_escape) {
        // Another start — still searching for a valid length
        Serial.print("\n\nstart resync (wait len) at position ");Serial.print(i );Serial.print(" escaped: "); Serial.println( from_escape );
        stats.invalidStartResync++;
        decodedLength = 0;
        recvPos = 0;
        return false;
      }

      if (b >= MIN_DECODED_LEN && b <= MAX_DECODED_LEN) {
        decodedLength = b;
        recvPos = 0;
        rxState = READ_PAYLOAD;
      } else {
        stats.invalidLength++;
        rxState = WAIT_START;
      }
      return false;


    // ---------------- READ_PAYLOAD ----------------
    case READ_PAYLOAD:

      if (b == START_BYTE && !from_escape) {
        // A new start byte mid-frame → treat as resync
        Serial.print("\n\nstart resync (read payload) at position ");Serial.print(i );Serial.print(" escaped: "); Serial.println( from_escape );
        
        stats.invalidStartResync++;
        rxState = WAIT_LEN;
        decodedLength = 0;
        recvPos = 0;
        return false;
      }

      if (recvPos >= sizeof(recvBuffer)) {
        stats.overflow++;
        rxState = WAIT_START;
        decodedLength = 0;
        recvPos = 0;
        return false;
      }

      recvBuffer[recvPos++] = b;

      // Check if full message received (payload + CRC)
      if (recvPos >= decodedLength) {

        uint8_t payloadLen = decodedLength - 2;

        uint16_t rxCRC = ((uint16_t)recvBuffer[payloadLen] << 8) | recvBuffer[payloadLen + 1];

        uint16_t calcCRC = crc16(recvBuffer, payloadLen);

        // Reset for next frame
        rxState = WAIT_START;
        decodedLength = 0;
        recvPos = 0;

        if (rxCRC == calcCRC) {
          // Copy validated payload
          memcpy(out, recvBuffer, payloadLen);
          outLen = payloadLen;
          return true;
        } else {
          stats.crcFail++;
        }
      }

      return false;
  }

  return false;
}


// ============================================================
// MAIN NON-BLOCKING DECODER
// Call repeatedly from loop()
// ============================================================
bool readMessageNonBlocking(uint8_t* out, uint8_t& outLen) {

  while (Serial.available() > 0) {

    int raw = Serial.read();
    if (raw < 0) break;

    uint8_t b = (uint8_t)raw;

    // Existing ESC prefix awaiting next byte?
    if (escapeNext) {
      escapeNext = false;
      uint8_t decoded = b ^ 0x20;
      stats.escapeDecoded++;

      if (processDecodedByte(decoded, true, out, outLen,0))
        return true;

      continue;
    }

    // ESC byte → next byte must be decoded
    if (b == ESC_BYTE) {
      escapeNext = true;
      continue;
    }

    // Normal byte path
    if (processDecodedByte(b, false, out, outLen,0))
      return true;
  }

  // If ESC was last byte in buffer → dangling escape
  if (escapeNext && Serial.available() == 0)
    stats.escAtEnd++;

  return false;
}

bool decodeTest( const uint8_t* buf, uint8_t len, uint8_t* outPayload, uint8_t& outLen ) {
  for (uint8_t i = 0; i < len; i++) {

    uint8_t b = buf[i];

    // --- ESC-prefix pending: decode this byte ---
    if (escapeNext) {
      escapeNext = false;
      uint8_t decoded = b ^ 0x20;
      stats.escapeDecoded++;

      if ( processDecodedByte(decoded, true, outPayload, outLen,i) )
        return true;

      continue;
    }

    // --- New ESC encountered ---
    if (b == ESC_BYTE) {
      escapeNext = true;
      continue;
    }

    // --- Normal byte ---
    if (processDecodedByte(b, false, outPayload, outLen, i))
      return true;
  }

  // If buffer ends while ESC was pending → dangling escape
  if (escapeNext && len > 0) {
    stats.escAtEnd++;
    // Keep escapeNext = true because next call to decodeBuffer()
    // may be the continuation of the message.
  }

  return false; // no complete frame yet
}

// Experiment to see if reading the LSB
// of an analog read can generate a
// useful random seed.  Nothing connected
// to A3, so it is floating and picking up
// emf.
void initRandomSeed() {
  pinMode(A3, INPUT);
  byte r = 0x00;
  for ( int i = 0; i < 8; i++ ) {
    byte b = (byte)analogRead(A3);
    b = b & 0x01;
    //Serial.println(b, BIN);

    r |= (b << i);
    delayMicroseconds(10);
  }
  //Serial.println(r, BIN);
  randomSeed( r );
}


/*
    From: http://www.taygeta.com/random/gaussian.html

    This routine is a little troubling because it is
    non-deterministic (we don't know when it will solve)
    and computationally expensive.
    However, using gaussian distribution is useful for
    creating "controllable" random motion, Brownian Motion.
*/
float randGaussian( float mean, float sd ) {
  float x1, x2, w, y;

  do {
    // Adaptation here because arduino random() returns a long
    x1 = random(0, 20000) - 10000;
    x1 *= 0.001;
    x2 = random(0, 20000) - 10000;
    x2 *= 0.001;
    w = (x1 * x1) + (x2 * x2);

  } while ( w >= 1.0 );

  w = sqrt( (-2.0 * log( w ) ) / w );
  y = x1 * w;

  return mean + y * sd;

}

void setup() {
  Serial.begin(9600);
  initRandomSeed();
  Serial.println("RESET");
}

typedef struct msg {
  float v[8];
} msg_t;

void printStats() {
  Serial.println("Decoder stats:");
  Serial.print("  invalidStartResync: "); Serial.println(stats.invalidStartResync);
  Serial.print("  invalidLength:      "); Serial.println(stats.invalidLength);
  Serial.print("  overflow:           "); Serial.println(stats.overflow);
  Serial.print("  crcFail:            "); Serial.println(stats.crcFail);
  Serial.print("  unexpectedByteInStart: "); Serial.println(stats.unexpectedByteInStart);
  Serial.print("  ignoredByteInLen:   "); Serial.println(stats.ignoredByteInLen);
  Serial.print("  escapeDecoded:      "); Serial.println(stats.escapeDecoded);
  Serial.print("  escAtEnd:           "); Serial.println(stats.escAtEnd);
}

void resetDecoder() {
  rxState = WAIT_START;

  escapeNext = false;
  decodedLength = 0;             // total bytes expected (payload + CRC)
  memset(recvBuffer, 0, sizeof( recvBuffer ));
  recvPos = 0;

}

void loop() {

  // If we had a payload of 32 bytes with no escapes, we
  // would expect a total output message of 37 bytes
  // Bytes | value | notes
  //(1)    | ~     | start token, never escaped
  //(1)    | ^     | (escape)
  //(1)    |[3:34] | (3 = 1 byte payload + 2CRC, 34 = 32 byte payload + 2CRC)
  //(32)   |       | payload
  //(2)    |       | CRC
  //
  // It is possible that every 32 bytes of the payload and the CRC
  // are escaped.  In which case, the maximum output message would be
  // (1)+(1)+(1)+(32*2)+(2*2) = 71
  //
  // If I test this by setting the payload to a struct of 8 floats 
  // each with the value +0.0, then I get an output of 69 bytes, which
  // means the CRC calculates as non-escaped bytes (if escaped, +2).
  //
  // I think the Arduino default UART buffer is 64 bytes, so it would
  // be interesting to see what happens in this case.
  uint8_t output[72];
  
  uint8_t payload_len;
  uint8_t output_len;
  uint8_t decoded[32];

  msg_t my_msg;
  for ( int i = 0; i < 8; i++ ) {
    my_msg.v[i] = randGaussian(0, 1.5);
    my_msg.v[i] -= 1;
    my_msg.v[i] /= 100000;
    my_msg.v[i] = +0.0;
  }

  Serial.println("\nMessage:");
  for ( int i = 0; i < 8; i++ ) {
    Serial.print( my_msg.v[i], 10); Serial.print(",");
  }
  Serial.println();

  payload_len = sizeof( my_msg );

  output_len = encodeMessage( (uint8_t*)&my_msg, payload_len, output );
  Serial.print("Output length after encoding: "); Serial.println( output_len );

  //  Serial.println( "Original message: ");
  //  for( int i = 0; i < 32; i++ ) {
  //    Serial.print( (char)my_msg[i] ); Serial.print("(");Serial.print( payload[i] ); Serial.print(") ");
  //  }
  Serial.println();
  int total_escapes = 0;
  for ( int i = 0; i < output_len; i++ ) {
    Serial.print("[");Serial.print(i);Serial.print("]");Serial.print( (char)output[i] ); Serial.print("("); Serial.print( output[i] ); Serial.print(") ");
    if ( output[i] == '^' ) total_escapes++;
  }
  Serial.println();
  Serial.print("Total Escapes: "); Serial.println( total_escapes );

  resetDecoder();
  uint8_t decoded_len = 0;
  msg_t d_msg;
  memset( &d_msg, 0, sizeof( d_msg ));
  decodeTest( output, output_len, (uint8_t*)&d_msg, decoded_len );
  Serial.print("Decode test len:"); Serial.println( decoded_len );


  Serial.println("decoded:");
  for ( int i = 0; i < 8; i++ ) {
    Serial.print( d_msg.v[i], 10); Serial.print(",");
  }
  Serial.println();

  printStats();

  delay(4000);


}
