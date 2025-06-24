// Connection information for the tracking
// system.
const char * server_ip = "192.168.1.200";
const uint16_t server_port = 5000;


// The tcp/ip packet for the tracking
// system.
#pragma pack(push,1)
typedef struct tracker_packet {
  int8_t marker_id; // 1 byte
  float x;          // 4 bytes
  float y;          // 4 bytes
  float theta;      // 4 bytes
  int8_t valid;     // 1 byte
} tracker_packet_t; // = 14 bytes
#pragma pack(pop)
