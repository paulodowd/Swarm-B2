
#ifndef IRCOMM_CONFIG_H
#define IRCOMM_CONFIG_H

// The following provide the default configuration
// of the communication board.



// Uncomment depending on your IR
// Receiver module.  This will determine
// the correct Serial Baud rate and the
// configuration of timer2 for the carrier
// signal
// Other parts of the code depend on this
// being set correctly.
#define IR_FREQ_38  // For chip TSDP34138
//#define IR_FREQ_58    // For chip TSDP34156

// If we know we are going to send and receive
// messages of a certain length then we could
// configure the device for a more optimal
// rx_delay and tx_delay.  To try and have the
// board do this for itself, set the below to
// true.  Set true, the board will look at the
// length of the message to transmit and the
// length of messages received, and setup the
// tx_delay and rx_delay to be appropriate for
// which ever of the two is longest.
// If set to false, the board will use the
// #defines set above for tx/rx_delay _bias _mod.
#define RX_CYCLE              true  // should the board poll receivers?
#define RX_CYCLE_ON_RX        true  // if a message is received, cycle?
#define RX_PREDICT_TIMEOUT    true  // try to optimse polling performance?
#define RX_PREDICT_MULTIPLIER 1.0   // how many message-size to wait?
#define RX_OVERRUN            true  // allow for rx message to complete? 
#define RX_DEFAULT_MSG_LEN    MAX_BUF // 36 is worst case
#define MS_PER_BYTE_58KHZ     1.2   // 58khz
#define MS_PER_BYTE_38KHZ     2.5   // 38khz

// A rough estimate of how many ms per byte 
// during the transmit/receive process.
#ifdef IR_FREQ_58
#define MS_BYTE_TIMEOUT       (MS_PER_BYTE_58KHZ*4.0)     
#endif
#ifdef IR_FREQ_38
#define MS_BYTE_TIMEOUT       (MS_PER_BYTE_38KHZ*4.0)     
#endif

#ifdef IR_FREQ_58
#define RX_TIMEOUT_MAX       (MS_PER_BYTE_58KHZ*RX_DEFAULT_MSG_LEN)     
#endif
#ifdef IR_FREQ_38
#define RX_TIMEOUT_MAX       (MS_PER_BYTE_38KHZ*RX_DEFAULT_MSG_LEN)     
#endif



// What system should we use for the tranmission?
// Periodic: Decouples receiving from transmission, meaning
//          that tranmission happens every (x)ms.
// Interleaved: Means that after a receiver is rotated, we
//          also transmit.  This means that if a robot is
//          receiving messages well, the tranmission rate
//          would also increase.  Seems complicated.
#define TX_MODE_PERIODIC     0 // tx periodically, timing set below
#define TX_MODE_INTERLEAVED  1 // tx after every receiver rotation (not working)
#define TX_MODE (TX_MODE_PERIODIC)
//#define TX_MODE (TX_MODE_INTERLEAVED)

// When set in TX_MODE_PERIODIC
// How long should the robot wait before doing another
// transmission?  
// For 38khz, a 32byte message will take approximately 
// 80ms to transmit/receive.
// For 58khz, a 32byte message will take approximately 
// 39ms to transmit/receive
#ifdef IR_FREQ_38
#define DEFAULT_TX_PERIOD (320) // in ms, 0 disables tx
#endif

#ifdef IR_FREQ_58
#define DEFAULT_TX_PERIOD (156) // in ms, 0 disables tx
#endif

// Should we try to break synchrony between robots
// by randomising the tx period?
#define TX_desync  0 

// How many times should we repeat the transmission
// of a message? This should be set as a positive
// no zero value (1+)
#define DEFAULT_TX_REPEAT 3

// Uncomment to see debug output.  Note that, we
// are going to use the serial port for debugging,
// which means we will effectively be transmitting
// debug output to other boards, and without disabling
// the rx component we'll also receive our own
// transmission.  Complicated!
//#define IR_DEBUG_OUTPUT true
#define IR_DEBUG_OUTPUT false

// How often should the bearing estimate be updated?
#define UPDATE_ACTIVITY_MS  250


// I think that having the least number of logic 
// transitions in the byte will create the most 
// reliable transmission/reception.  
// Noise or interference can distort the 
// transitions. 
// Since we are looking out for the start token
// to begin receiving a message it makes sense to
// use one of these.  
// We can't use 0x00 because that is a string null
// character, which we're using with strlen()
#define START_TOKEN '*'     //  0x2A  0b00101010
#define CRC_TOKEN   '@'     // '@'  0b01000000



#endif
