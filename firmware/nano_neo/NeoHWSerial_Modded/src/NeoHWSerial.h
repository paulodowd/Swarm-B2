/*
  NeoHWSerial.h - Hardware serial library with attachInterrupt
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
  Modified 2 November 2015 by SlashDev
  Modified 31 October 2020 by Georg Icking-Konert
  Modified 26 January 2025 by Georg Icking-Konert
*/

// only compile for AVR architecture
#if defined(ARDUINO_ARCH_AVR)

#ifndef NeoHWSerial_h
#define NeoHWSerial_h

#include <inttypes.h>
#include <wiring_private.h>
#include <Stream.h>

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
// NOTE: a "power of 2" buffer size is reccomended to dramatically
//       optimize all the modulo operations for ring buffers.
// WARNING: When buffer sizes are increased to > 256, the buffer index
// variables are automatically increased in size, but the extra
// atomicity guards needed for that are not implemented. This will
// often work, but occasionally a race condition can occur that makes
// NeoHWSerial behave erratically. See https://github.com/arduino/Arduino/issues/2405
#if !defined(SERIAL_TX_BUFFER_SIZE)
  #if ((RAMEND - RAMSTART) < 1023)
    #define SERIAL_TX_BUFFER_SIZE 16
  #else
    #define SERIAL_TX_BUFFER_SIZE 64
  #endif
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
  #if ((RAMEND - RAMSTART) < 1023)
    #define SERIAL_RX_BUFFER_SIZE 16
  #else
    #define SERIAL_RX_BUFFER_SIZE 64
  #endif
#endif
#if (SERIAL_TX_BUFFER_SIZE>256)
  typedef uint16_t tx_buffer_index_t;
#else
  typedef uint8_t tx_buffer_index_t;
#endif
#if  (SERIAL_RX_BUFFER_SIZE>256)
  typedef uint16_t rx_buffer_index_t;
#else
  typedef uint8_t rx_buffer_index_t;
#endif

// Define config for Serial.begin(baud, config);
#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

class NeoHWSerial : public Stream
{
  protected:
    volatile uint8_t * const _ubrrh;
    volatile uint8_t * const _ubrrl;
    volatile uint8_t * const _ucsra;
    volatile uint8_t * const _ucsrb;
    volatile uint8_t * const _ucsrc;
    volatile uint8_t * const _udr;
    // Has any byte been written to the UART since begin()
    bool _written;
    
    // Paul adding here for Frame Error capture
    volatile uint32_t frame_error_count; // added by Paul.
   
    volatile rx_buffer_index_t _rx_buffer_head;
    volatile rx_buffer_index_t _rx_buffer_tail;
    volatile tx_buffer_index_t _tx_buffer_head;
    volatile tx_buffer_index_t _tx_buffer_tail;

    // Don't put any members after these buffers, since only the first
    // 32 bytes of this struct can be accessed quickly using the ldd
    // instruction.
    unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];

  public:
    inline NeoHWSerial(
      volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
      volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
      volatile uint8_t *ucsrc, volatile uint8_t *udr);
    void begin(unsigned long baud) { begin(baud, SERIAL_8N1); }
    void begin(unsigned long, uint8_t);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual int availableForWrite(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    uint32_t getFrameErrorCount(); // added by Paul.
    void resetFrameErrorCount();	// added by Paul.
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }
    inline void enable_rx(void) { sbi(*_ucsrb, RXEN0); }
    inline void disable_rx(void) { cbi(*_ucsrb, RXEN0); }
    
    // Receive interrupt handler - Not intended to be called externally
    inline void _rx_complete_irq(void)
    {
      volatile bool saveToBuffer;
      volatile unsigned char status, data;

      // get Rx status and data
      status = *_ucsra;
      data = *_udr;

      if( status & _BV(FE0) ) {  // Added by Paul
	frame_error_count++;  
      }

      // user receive function was attached -> call it with data and status byte
      if (_isr)
        saveToBuffer = _isr( data, status );
      
      // no user routine attached -> save data in ring buffer
      else
        saveToBuffer = true;

      // default: save data in ring buffer
      if (saveToBuffer) {


        // No Parity error, read byte and store it in the buffer if there is room
        if (bit_is_clear(status, UPE0)) {
          rx_buffer_index_t i = (unsigned int)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;

          // if we should be storing the received character into the location
          // just before the tail (meaning that the head would advance to the
          // current location of the tail), we're about to overflow the buffer
          // and so we don't write the character or advance the head.
          if (i != _rx_buffer_tail) {
            _rx_buffer[_rx_buffer_head] = data;
            _rx_buffer_head = i;
          }
        }

        // Parity or framing error, don't do anything (data is dropped)
        else { }

      } // if saveToBuffer
    } // _rx_complete_irq()

    // Transmit interrupt handler - Not intended to be called externally
    inline void _tx_udr_empty_irq(void)
    {
      // If interrupts are enabled, there must be more data in the output
      // buffer. Send the next byte
      unsigned char c = _tx_buffer[_tx_buffer_tail];
      _tx_buffer_tail = (_tx_buffer_tail + 1) % SERIAL_TX_BUFFER_SIZE;

      *_udr = c;

      // clear the TXC bit -- "can be cleared by writing a one to its bit
      // location". This makes sure flush() won't return until the bytes
      // actually got written. Other r/w bits are preserved, and zeroes
      // written to the rest.

    #ifdef MPCM0
      *_ucsra = ((*_ucsra) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
    #else
      *_ucsra = ((*_ucsra) & ((1 << U2X0) | (1 << TXC0)));
    #endif

      if (_tx_buffer_head == _tx_buffer_tail) {
        // Buffer empty, so disable interrupts
        cbi(*_ucsrb, UDRIE0);
      }
    } // _tx_udr_empty_irq()

    typedef bool (* isr_t)( uint8_t d, uint8_t s );
    void attachInterrupt( isr_t fn );
    void detachInterrupt() { attachInterrupt( (isr_t) NULL ); };

  private:
    isr_t  _isr;

    NeoHWSerial( const NeoHWSerial & );
    NeoHWSerial & operator =( const NeoHWSerial &);
};

#if defined(UBRRH) || defined(UBRR0H)
  extern NeoHWSerial NeoSerial;
  #define HAVE_HWSERIAL0
#endif
#if defined(UBRR1H)
  extern NeoHWSerial NeoSerial1;
  #define HAVE_HWSERIAL1
#endif
#if defined(UBRR2H)
  extern NeoHWSerial NeoSerial2;
  #define HAVE_HWSERIAL2
#endif
#if defined(UBRR3H)
  extern NeoHWSerial NeoSerial3;
  #define HAVE_HWSERIAL3
#endif

extern void serialEventRun(void) __attribute__((weak));            // called by main() -> name fixed

#endif // NeoHWSerial_h

#endif // ARDUINO_ARCH_AVR
