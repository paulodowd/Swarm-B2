// Connection information for the tracking
// system.
const char * server_ip = "192.168.1.200";
const uint16_t server_port = 5000;


// The tcp/ip packet for the tracking
// system.
#pragma pack(push,1)
typedef struct tracker_packet {
  int16_t marker_id; // 2 byte
  float x;          // 4 bytes
  float y;          // 4 bytes
  float theta;      // 4 bytes
  int16_t valid;     // 2 byte
} tracker_packet_t; // = 16 bytes
#pragma pack(pop)
