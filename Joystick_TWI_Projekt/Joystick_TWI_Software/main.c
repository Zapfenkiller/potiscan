/******************************************************************************\
*                                                                              *
* File        : main.c                                                         *
* Project     : Remote Control - Joystick_TWI                                  *
* Author      : R. Trapp                                                       *
* Initial date: 21 May  2012                                                   *
7.Nov.2013: Die muﬂ noch auf den neuen Schaltplan angepaﬂt werden!
* Release rev.:                                                                *
* Copyright   : (c) 2012 H.A.R.R.Y.                                            *
* Credits     : Peter Dannegger, danni@specs.de - key debouncing               *
*                <http://www.mikrocontroller.net/articles/Entprellung          *
* License     :                                                                *
* Target      : ATtiny2313                                                     *
* Description : Serve analog joystick (aka PC-joystick) and allow read out via *
*               I≤C bus. Device acts as I≤C slave. Output is scaled to 8..247. *
*               Joystick nominal values are 0..100kOhm. Tolerances apply to    *
*               this so each pot needs individual trim setting to calculate    *
*               final result. First experiments show readouts are stable so a  *
*               retrim should be necessary only under rare conditions.         *
*               Need provision to "teach in" some joystick positions from time *
*               to time (change of joystick).                                  *
*               Readout sequence of "read all" is:                             *
*                Joystick 1 - X (= Pot 1)                                      *
*                Joystick 1 - Y (= Pot 2)                                      *
*                Joystick 2 - X (= Pot 3)                                      *
*                Joystick 2 - Y (= Pot 4)                                      *
*                Pushbuttons: MSB                                    LSB       *
*                             V2Y, V2X, V1Y, V1X, J2B2, J2B1, J1B2, J1B1       *
*                                                                              *
*               The 'V' bits indicate INVALID pot reading (e.g. not connected) *
*               when set!                                                      *
*                                                                              *
*               Scan one pot every 2ms. Total turn around time thus is 8ms on  *
*               on 4 pots. Digitizing one pot takes approximately 1ms maximum  *
*               the remaining time is for discharging with low time constant   *
*               to perfectly meet the constraints for the next conversion.     *
*               If maximum pot resistance is exceeded - too high value or even *
*               no pot - the invalid idication shall be given.                 *
*               Timer 1 controls digitizing by 2 interrupt routines. One of    *
*               them checks for timeout (and thus maximum allowed resistance). *
*               If timeout did not occur the value is taken as a valid sample. *
*               Also the discharge cycle is started. The second interrupt      *
*               routine handles conversion start on the next channel.          *
*               After sampling a raw value the main loop schedules its conver- *
*               sion into the desired output value range.                      *
*               I≤C is handled if nothing else needs to be done. To speed up   *
*               TWI response its service routines should be changed to IRQ-    *
*               usage later on.                                                *
*                                                                              *
*               Debouncing the pushbuttons is done by a timer 0 interrupt      *
*               service.                                                       *
*                                                                              *
*               Intermediate experimental version will send also data via UART.*
*               Will also receive battery status message and pulldown PB6 if   *
*               "low bat" applies. PB6 is High-Z otherwise (connect LED and R).*
*               Thus one of the pushbuttons is not working here.               *
*               Expecting a battery status message after transmitting the joy- *
*               packet there are roughly 8 to 10 messages exchanged per second.*
*               If no battery status is received the transmission is repeated  *
*               after a certain timeout.                                       *
*                                                                              *
\******************************************************************************/

#define _ALSO_USE_UART_         /* define this for temporary UART support,
                                   will eat up 206 FLASH bytes, 1 RAM byte */

#include "../project.h"         /* contains all public definitions (also TWI) */
#include "joystick_twi.h"       /* contains private definitions */
#define __use_twi_slave__       /* select TWI support */
#include "i2c.h"                /* TWI service */
#include <avr/interrupt.h>      /* IRQ definitions */
#include <avr/eeprom.h>         /* EEPROM support */

#define TWI_BASE_address         TWI_JOYSTICK_ADDRESS
enum
{
  JOY1_X_INDEX = 0,
  JOY1_Y_INDEX,
  JOY2_X_INDEX,
  JOY2_Y_INDEX,
  JOYPBS_INDEX,
  /* ---- insert additional parameters above this line! ----*/
  RESULT_SIZE,
};


struct trim_data {
  int16_t min_resi; /* STICK_AT_MIN_RESI */
  int16_t max_resi; /* STICK_AT_MAX_RESI */
  int16_t factor;   /* RESCALING_FACTOR */
};


/* ########################################################################## */
// this data is put into EEPROM (or at least a preparatory EEPROM-hex-file is
// generated) by default
EEMEM struct trim_data joyTrim[RESULT_SIZE-1] =
{
  {STICK_AT_MIN_RESI, STICK_AT_MAX_RESI, RESCALING_FACTOR}, /* Pot 1 = Joy 1 X */
  {STICK_AT_MIN_RESI, STICK_AT_MAX_RESI, RESCALING_FACTOR}, /* Pot 2 = Joy 1 Y */
  {STICK_AT_MIN_RESI, STICK_AT_MAX_RESI, RESCALING_FACTOR}, /* Pot 3 = Joy 2 X */
  {STICK_AT_MIN_RESI, STICK_AT_MAX_RESI, RESCALING_FACTOR}, /* Pot 4 = Joy 2 Y */
};


/* ########################################################################## */
// global variables, interface between IRQ and normal mode routines
volatile  uint16_t  captured[RESULT_SIZE-1];
volatile  uint8_t   whoIsNext = JOY1_X_INDEX;
volatile  uint8_t   whoIsReady = JOY1_X_INDEX;
volatile  uint8_t   updated = 0;
volatile  uint8_t   key_state;
#ifdef _ALSO_USE_UART_
volatile  uint8_t   timeout = 3;
#endif // ifdef _ALSO_USE_UART_


#ifdef _ALSO_USE_UART_
/* ########################################################################## */
// for very first remote control of ROV 1 by joystick directly - shall be
// removed when the remote controller is completed!
#ifdef __AVR_ATtiny2313__
// the battery status LED
  #define LEDPORT       PORTB
  #define LEDDDR        DDRB
  #define LEDBIT        6
// UART
#define TXDATAREG       UDR     // transmitter data register
#define TXSTATREG       UCSRA   // transmitter status
#define TXCTRLREG       UCSRB   // transmitter control
#define RXDATAREG       UDR     // receiver data register
#define RXSTATREG       UCSRA   // receiver status
#define RXCTRLREG       UCSRB   // receiver control
// the bits
#define TXEMPTYFLAG     UDRE    // transmitter empty flag
#define RXFULLFLAG      RXC     // receiver full flag
#define TXENABLE        TXEN    // transmitter enable
#define RXENABLE        RXEN    // receiver enable
// the handshake signals
// /CTS controls the remote transmitter - not needed here!
// /RTS controls the local transmitter
#define RTSPORT         PIND
#define RTSBIT          6
// some macros for more readable coding
#define isTxFull        (!(TXSTATREG & (1 << TXEMPTYFLAG)))
#define isRxEmpty       (!(RXSTATREG & (1 << RXFULLFLAG)))
#define isRTSinactive   (RTSPORT & (1 << RTSBIT))
//#define clearTxFlag     UCSR0A = UCSR0A | (1 << TXC0)  // flag must be cleared manually!
#endif // __AVR_ATtiny2313__

#ifndef F_BAUD
#define F_BAUD          19200UL
#endif

#define FLAG_ACCU_IS_EMPTY    (1<<4) /* accumulator voltage too low */

void initCom (void)
// init UART and handshake IO
{
  LEDPORT &= ~(1 << LEDBIT); // LED is on per default
  LEDDDR |= (1<<LEDBIT);  // output
  // init handshake signals
  RTSPORT |= (1 << RTSBIT); // activate pullup (/RTS = inactive!)
  // init U(S)ART
  UBRRH = ((F_CPU/(16*F_BAUD))-1) >> 8;
  UBRRL = ((F_CPU/(16*F_BAUD))-1) & 0x00ff;
  TXCTRLREG = ((1 << TXENABLE) | (1 << RXENABLE));
}

void putChar (uint8_t data)
// transmit one byte
{
  while (isTxFull) {}       // wait until transmitter ready
  while (isRTSinactive) {}  // wait until transmission allowed
  TXDATAREG = data;          // send away
}

void sendSequence (void *ptr, uint8_t byteCount)
// transmit a certain count of bytes
{
  uint8_t *p = (uint8_t *) ptr;
  putChar('J');         // Joystick message header
  while (byteCount--)
    putChar(*p++);
  putChar(~'J');        // joystick message termination
}

uint8_t getChar (int8_t *ptr)
// check reception status, also report received byte
// returns '0' if nothing is in
{
  if (isRxEmpty)
    return(0);
  *ptr = RXDATAREG;
  return(~0);
}

void decodeReception (void)
// scans incoming stream for battery message and updates LED
{
  enum
  {
    await_header,
    await_flags,
    await_tail,
  };
  static uint8_t decoder_state = await_tail;
  static uint8_t flags = 0;
  int8_t byte;
  while (getChar(&byte))
  {
    switch(decoder_state)
    {
      case await_header:
        if (byte == 'B')
          decoder_state = await_flags;
        break;
      case await_flags:
        flags = byte & FLAG_ACCU_IS_EMPTY;
        decoder_state = await_tail;
        break;
      case await_tail:
        if (byte == ~'B')
        {
          if (flags)
            LEDPORT &= ~(1 << LEDBIT);
          else
            LEDPORT |= (1 << LEDBIT);
          cli();
          timeout = 0;
          sei();
        }
      default:
        decoder_state = await_header;
    }
  }
}

#endif // ifdef _ALSO_USE_UART_


/* ########################################################################## */
// read out actual pot value - also checks for timeout
// start discharge cycle
ISR(TIMER1_COMPA_vect)
{
  STOP_CHARGING;
  START_DISCHARGING;
  whoIsReady = whoIsNext;
  if (CAPTURE_OCCURED)
    // store time stamp
    captured[whoIsNext] = CAPTURE_RESULT_REG;
  else
    // indicate maximum
    captured[whoIsNext] = ~0;
  whoIsNext += 1;
  if (whoIsNext > JOY2_Y_INDEX)
    whoIsNext = JOY1_X_INDEX;
  CLEAR_CAPTURE_FLAG;
  updated = ~0;
}


/* ########################################################################## */
// end discharge cycle
// prepare next conversion
ISR(TIMER1_COMPB_vect)
{
  STOP_T1_OPERATION;
  CLEAR_T1_COUNT_REG;
  STOP_DISCHARGING;
  switch (whoIsNext) /* start next charging cycle */
  {
    case JOY1_X_INDEX:
      POT_DDR |= POT1_BIT;
      POT_PORT |= POT1_BIT;
      break;
    case JOY1_Y_INDEX:
      POT_DDR |= POT2_BIT;
      POT_PORT |= POT2_BIT;
      break;
    case JOY2_X_INDEX:
      POT_DDR |= POT3_BIT;
      POT_PORT |= POT3_BIT;
      break;
    case JOY2_Y_INDEX:
      POT_DDR |= POT4_BIT;
      POT_PORT |= POT4_BIT;
      break;
    default:
      ;
  }
  START_T1_OPERATION;
}


/* ########################################################################## */
// key scanning and debouncing - see credits
// key edge detection and repetition not necessary, thus removed
ISR(TIMER0_OVF_vect)
{
  static uint8_t ct0;
  static uint8_t ct1;
  uint8_t i = key_state ^ ~KEY_PIN;     // key changed?
  ct0 = ~(ct0 & i);                     // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);                // reset or count ct1
  i &= ct0 & ct1;                       // count until roll over?
  key_state ^= i;                       // then toggle debounced state

#ifdef _ALSO_USE_UART_
  if (timeout)
    timeout -= 1;
#endif // ifdef _ALSO_USE_UART_
}


/* ########################################################################## */
// EEPROM handling
// read out a byte
uint8_t EEPROM_read_byte(unsigned int address)
{
  while (EECR & (1 << EEPE));
  EEAR = address;
  EECR |= (1 << EERE);
  return (EEDR);
}


// EEPROM handling
// read out a word (2 bytes)
unsigned int EEPROM_read_word(unsigned int address)
{
  while (EECR & (1 << EEPE));
  EEAR = address++;
  EECR |= (1 << EERE);
  unsigned int data = EEDR;
  EEAR = address;
  EECR |= (1 << EERE);
  return (data | (EEDR << 8));
}


// EEPROM handling
// write a word (2 bytes)
void EEPROM_write_word(unsigned int address, unsigned int data)
{
  while (EECR & (1 << EEPE));
  EEAR = address++;
  EEDR = data;
  cli();
  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
  sei();
  while (EECR & (1 << EEPE));
  EEAR = address;
  EEDR = data >> 8;
  cli();
  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
  sei();
}


/* ########################################################################## */
// get bytes out of words
uint8_t lsb(void* word)
{
  uint8_t* p;
  p = (uint8_t*) word;
  return(*p); /* little endian model! */
}


uint8_t msb(void* word)
{
  uint8_t* p;
  p = (uint8_t*) word;
  return(*(p+1)); /* little endian model! */
}


/* ########################################################################## */
// calculate conversion factor from trim points (but only if points are valid)
// and store to EEPROM (but only if different from value already stored)
void calculate_trim_factor (int index)
{
  if ((EEPROM_read_word((unsigned int) &joyTrim[index].max_resi) < CAPTURE_LIMIT) \
    && (EEPROM_read_word((unsigned int) &joyTrim[index].min_resi) < CAPTURE_LIMIT))
  {
    int16_t trim_factor = (int16_t)EEPROM_read_word((unsigned int) &joyTrim[index].max_resi);
    trim_factor -= (int16_t)EEPROM_read_word((unsigned int) &joyTrim[index].min_resi);
    // allow for better precision multiplying by 6 (= 2 + 4)
    trim_factor = (trim_factor << 1) + (trim_factor << 2);
    trim_factor = trim_factor / (int16_t)(DESIRED_MAX_READING - DESIRED_MIN_READING + 1);
    if (trim_factor != (int16_t)EEPROM_read_word((unsigned int) &joyTrim[index].factor))
      // only do a write if EEPROM contents is different
      EEPROM_write_word((unsigned int) &joyTrim[index].factor, trim_factor);
  }
}


/* ########################################################################## */
// main program control:
// converts raw time stamps (resistance readings) to desired output range
// handles TWI traffic
int main(void)
{
  uint8_t result[RESULT_SIZE];
  result[JOYPBS_INDEX] = 0;
  /* set up IO ports */
  BUTTON_PORT |= (BUTTON4_BIT | BUTTON3_BIT | BUTTON2_BIT | BUTTON1_BIT);
  START_DISCHARGING;
  /* set up analog comparator */
  DIDR |= (1<<AIN1D) | (1<<AIN0D); // disable digital input on AIN1 and AIN0
  ACSR = 1 << ACIC; // enable comparator, use external reference, no IRQs, ICP
  /* set up TWI service */
  setupTwiBus(0);
#ifdef _ALSO_USE_UART_
  /* set up UART */
  initCom();
#else
  /* set unused IO to drive GND!!!
     ========================== */
  NC_PORT1 &= ~NC_BITS1;
  NC_DDR1 |= NC_BITS;
#endif // __AVR_ATtiny2313__
  /* set up timer 0 as desired (button debouncing) */
  INIT_T0;
  START_T0_OPERATION;
  /* set up timer 1 as desired (pot conversion) */
  INIT_T1;
  START_T1_OPERATION;
  /* finally start interrupt system */
  sei();
  /* now main loop takes over */
  uint8_t j;
  char x;
  char c=0;
  uint8_t twi_todo = 0;
  while (1)
  {
    /* ==== TWI handling ==== */
    x = TWI_BASE_address;
    if (twi_getaddressSlave(&x, 0b11111110) != (char) __twiFail__)
    {
      if ((x & __twiRead__) == __twiRead__)
        /* read access */
        switch (twi_todo)
        {
          case readJoyAll:
            j = JOY1_X_INDEX;
            while (!twi_sendByteSlave(result[j++]))
            {
              if (j >= RESULT_SIZE)
                j = JOY1_X_INDEX;
            }
            break;
          case readJoy1_X:
            while (!twi_sendByteSlave(result[JOY1_X_INDEX])) {}
            break;
          case readJoy1_Y:
            while (!twi_sendByteSlave(result[JOY1_Y_INDEX])) {}
            break;
          case readJoy2_X:
            while (!twi_sendByteSlave(result[JOY2_X_INDEX])) {}
            break;
          case readJoy2_Y:
            while (!twi_sendByteSlave(result[JOY2_Y_INDEX])) {}
            break;
          case readJoyPBs:
            while (!twi_sendByteSlave(result[JOYPBS_INDEX])) {}
            break;
          case readJoyAllRaw:
            j = JOY1_X_INDEX;
            while (1)
            {
              cli();
              uint8_t lsbByte = lsb((void*) &captured[j]);
              uint8_t msbByte = msb((void*) &captured[j]);
              sei();
              if (twi_sendByteSlave(lsbByte))
                break;
              if (twi_sendByteSlave(msbByte))
                break;
              if (++j >= (RESULT_SIZE - 1))
                j = JOY1_X_INDEX;
            }
            break;
          case readJoyTrimSetting:
            j = JOY1_X_INDEX;
            unsigned int base = (unsigned int) &joyTrim[JOY1_X_INDEX];
            while (!twi_sendByteSlave(EEPROM_read_byte(base + j)))
            {
              if (++j >= sizeof(joyTrim))
                j = JOY1_X_INDEX;
            }
            break;
          default:
            twi_todo = readJoyAll;
        }
      else
      {
        /* write access */
        while (!twi_receiveByteSlave(&c)) {}
        switch (c)
        {
          case setJoy1UpperLeftCorner:
            /* ATTENTION: stick needs to be in the upper left corner! */
            cli();
            uint16_t trim_x_min = captured[JOY1_X_INDEX];
            uint16_t trim_y_min = captured[JOY1_Y_INDEX];
            sei();
            EEPROM_write_word((unsigned int) &joyTrim[JOY1_X_INDEX].min_resi, trim_x_min);
            EEPROM_write_word((unsigned int) &joyTrim[JOY1_Y_INDEX].min_resi, trim_y_min);
            twi_todo = readJoyTrimSetting;
            break;
          case setJoy1LowerRightCorner:
            /* ATTENTION: stick needs to be in the lower right corner! */
            cli();
            uint16_t trim_x_max = captured[JOY1_X_INDEX];
            uint16_t trim_y_max = captured[JOY1_Y_INDEX];
            sei();
            EEPROM_write_word((unsigned int) &joyTrim[JOY1_X_INDEX].max_resi, trim_x_max);
            EEPROM_write_word((unsigned int) &joyTrim[JOY1_Y_INDEX].max_resi, trim_y_max);
            twi_todo = readJoyTrimSetting;
            break;
          case setJoy1ConversionFactor:
            /* ATTENTION: do adjustment of upper left / lower right first! */
            calculate_trim_factor(JOY1_X_INDEX);
            calculate_trim_factor(JOY1_Y_INDEX);
            twi_todo = readJoyTrimSetting;
            break;
          case setJoy2UpperLeftCorner:
            /* ATTENTION: stick needs to be in the upper left corner! */
            cli();
            trim_x_min = captured[JOY2_X_INDEX];
            trim_y_min = captured[JOY2_Y_INDEX];
            sei();
            EEPROM_write_word((unsigned int) &joyTrim[JOY2_X_INDEX].min_resi, trim_x_min);
            EEPROM_write_word((unsigned int) &joyTrim[JOY2_Y_INDEX].min_resi, trim_y_min);
            twi_todo = readJoyTrimSetting;
            break;
          case setJoy2LowerRightCorner:
            /* ATTENTION: stick needs to be in the lower right corner! */
            cli();
            trim_x_max = captured[JOY2_X_INDEX];
            trim_y_max = captured[JOY2_Y_INDEX];
            sei();
            EEPROM_write_word((unsigned int) &joyTrim[JOY2_X_INDEX].max_resi, trim_x_max);
            EEPROM_write_word((unsigned int) &joyTrim[JOY2_Y_INDEX].max_resi, trim_y_max);
            twi_todo = readJoyTrimSetting;
            break;
          case setJoy2ConversionFactor:
            /* ATTENTION: do adjustment of upper left / lower right first! */
            calculate_trim_factor(JOY2_X_INDEX);
            calculate_trim_factor(JOY2_Y_INDEX);
            twi_todo = readJoyTrimSetting;
            break;
          default:
            twi_todo = c;
        }
      }
    }
#ifdef _ALSO_USE_UART_
    /* ==== UART handling ==== */
    decodeReception();
    if (!timeout)
    {
      timeout = 230; // approx. 0.8s
      sendSequence((void*) &result[0], RESULT_SIZE);
    }
#endif // ifdef _ALSO_USE_UART_
    /* ==== debounced pushbuttons ==== */
    cli();
    uint8_t buttons = key_state & BUTTON_MASK;
    sei();
    if (buttons & BUTTON1_BIT)
      result[JOYPBS_INDEX] |= 0x01;
    else
      result[JOYPBS_INDEX] &= ~0x01;
    if (buttons & BUTTON2_BIT)
      result[JOYPBS_INDEX] |= 0x02;
    else
      result[JOYPBS_INDEX] &= ~0x02;
    if (buttons & BUTTON3_BIT)
      result[JOYPBS_INDEX] |= 0x04;
    else
      result[JOYPBS_INDEX] &= ~0x04;
    if (buttons & BUTTON4_BIT)
      result[JOYPBS_INDEX] |= 0x08;
    else
      result[JOYPBS_INDEX] &= ~0x08;
    /* ==== convert capture result to public output ==== */
    if (updated)
    {
      cli();
      updated = 0;
      uint8_t whoIsToRescale = whoIsReady;
      uint16_t rawValue = captured[whoIsToRescale];
      sei();
      if (rawValue <= CAPTURE_LIMIT)
      {
        int16_t rawResult = (int16_t)rawValue - (int16_t)EEPROM_read_word((unsigned int) &joyTrim[whoIsToRescale].min_resi);
        // allow for better precision multiplying by 6 (= 2 + 4)
        rawResult = (rawValue << 1) + (rawValue << 2);
        int16_t conversionResult = rawResult / (int16_t)EEPROM_read_word((unsigned int) &joyTrim[whoIsToRescale].factor);
        conversionResult = conversionResult + DESIRED_MIN_READING;
        if (conversionResult > (int16_t)ABSOLUTE_MAX_READING)
          result[whoIsToRescale] = ABSOLUTE_MAX_READING;
        else
          result[whoIsToRescale] = conversionResult;
        result[JOYPBS_INDEX] &= ~(1 << (whoIsToRescale + 4));
      }
      else
        result[JOYPBS_INDEX] |= (1 << (whoIsToRescale + 4));
    }
  }
  return(0);
}


/******************************************************************************\

 CVS history:

  $Id$

  $Log$

\******************************************************************************/
