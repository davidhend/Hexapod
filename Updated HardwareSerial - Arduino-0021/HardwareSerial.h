/*
  HardwareSerial.h - Hardware serial library for Wiring
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
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>

#include "Stream.h"

// Experimental interrupt driven output
#define SERIAL_INTERRUPT_OUTPUT

// Find a generic name for the Uart data register empty interrupt vector
#if defined(SERIAL_INTERRUPT_OUTPUT)
# if defined(SIG_USART0_DATA)
#  define FIRST_UDRE_vect SIG_USART0_DATA
# endif

# if defined(USART0_UDRE_vect)
#  define FIRST_UDRE_vect USART0_UDRE_vect
# endif

# if defined(USART_UDRE_vect)
#  define FIRST_UDRE_vect USART_UDRE_vect
# endif

# if !defined(FIRST_UDRE_vect)
#  error Don't know what the Data Register Empty vector is called for the first UART
# endif
#endif



struct ring_buffer;

class HardwareSerial : public Stream
{
  private:
    ring_buffer *_rx_buffer;
#if defined (SERIAL_INTERRUPT_OUTPUT)  
    ring_buffer *_tx_buffer;
#endif
    volatile uint8_t *_ubrrh;
    volatile uint8_t *_ubrrl;
    volatile uint8_t *_ucsra;
    volatile uint8_t *_ucsrb;
    volatile uint8_t *_udr;
    uint8_t _rxen;
    uint8_t _txen;
    uint8_t _rxcie;
    uint8_t _udre;
    uint8_t _u2x;
#if defined (SERIAL_INTERRUPT_OUTPUT)  
	uint8_t _udrie;
#endif
  public:
#if defined (SERIAL_INTERRUPT_OUTPUT)  
    HardwareSerial(ring_buffer *rx_buffer,
	  ring_buffer *tx_buffer,
      volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
      volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
      volatile uint8_t *udr,
      uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udre, uint8_t u2x, uint8_t udrie);
#else
    HardwareSerial(ring_buffer *rx_buffer,
      volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
      volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
      volatile uint8_t *udr,
      uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udre, uint8_t u2x);
#endif
    void begin(long);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual void write(uint8_t);
    using Print::write; // pull in write(str) and write(buf, size) from Print
};

extern HardwareSerial Serial;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
#endif

#endif
