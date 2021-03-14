/*****************************************************************************\
 *
 * Definitions for I²C usage. Attempt to uniquify different hardware schemes
 * (e.g. ATtiny26 - USI, ATmega8 - TWI) into easy to use SW routines. These
 * routines serve as low level driver with uniquified interface to higher
 * abstraction levels (e.g. communicating to or emulation of PCF8574, 2416).
 * Multi master mode is fully compliant to TWI multimaster arbitration schemes,
 * also it is addressable as slave.
 * 
 * Necessary parameter definitions in the makefile are:
 * MCU_TARGET, F_CPU, F_TWI
 * F_CPU and F_TWI are necessary for master modes only
 *
 * To avoid useless code overhead some more flags have to be specified some-
 * where in the project file(s):
 * __use_twi_slave__			for slave response only
 * __use_twi_single_master__	for single master mode
 * __use_twi_multi_master__		for multi master mode
 *
 * ATTENTION: THIS LOCAL COPY IS MODIFIED TO MONITOR ONLY nBITS OF THE 7BIT
 * TWI-address WHEN USI-SLAVE-MODE IS SELECTED! THIS IS NEEDED TO REACT ON TWO
 * SUBSEQUENT addressES FOR LCD COMMUNICATION!
 *
\*****************************************************************************/

#ifndef __i2c_h__
#define __i2c_h__

// ============================================================================
// some global definitions for TWI usage
// ============================================================================

#define __twiWrite__	0	/* R/W = '0' */
#define __twiRead__		1	/* R/W = '1' */
#define __twiAck__		0	/* ACK */
#define	__twiNoAck__	1	/* NACK */
#define __twiOk__		0	/* general no fail flag */
#define __twiFail__		-1	/* general fail flag */

// ==========================================================================
// all AVR devices with USI receive their necessary definitions here
// ==========================================================================

#if defined (__AVR_ATtiny2313__)
#include <avr/io.h>
#define __avrUsi__
#define TWIport 		PORTB
#define TWIread			PINB
#define TWIddr			DDRB
#define TWIsclBit		7
#define TWIsdaBit		5
#elif defined (__AVR_ATtiny26__)
#include <avr/io.h>
#define __avrUsi__
#define TWIport 		PORTB
#define TWIread			PINB
#define TWIddr			DDRB
#define TWIsclBit		2
#define TWIsdaBit		0
#elif defined (__AVR_ATmega169__)
#include <avr/io.h>
#define __avrUsi__
#define TWIport 		PORTE
#define TWIread			PINE
#define TWIddr			DDRE
#define TWIsclBit		4
#define TWIsdaBit		5

// ==========================================================================
// all AVR devices with TWI receive their definitions here
// ==========================================================================

#elif defined (__AVR_ATmega8__)
#include <avr/io.h>
#define __avrTwi__
#elif defined (__AVR_ATmega168__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega48__)
#include <avr/io.h>
#define __avrTwi__

// ==========================================================================
// all AVR devices without any USI/TWI hardware support, or unknown devices
// end up here
// ==========================================================================

#else
#define __noHwTwiSupport__
#error "MCU type not supported yet by i2c library (or no MCU type defined at all)"
#error "Please use 'softi2c.h' library for software emulation of single master"
#endif

// ============================================================================
// depending on intended mode the library routines are selected
// (this list only serves for overview since ALL the routines of this header
// file are visible to its includer)
// ============================================================================

#if defined __use_twi_slave__
// ------ slave mode ------
void setupTwiBus (char);			// set up ressources used
char twi_getaddressSlave (char*, char);// check for start condition and receive byte
char twi_receiveByteSlave (char*);	// receive byte, send ACK
char twi_sendByteSlave (char);		// send byte, check ACK
#elif defined __use_twi_single_master__
// ------ single master mode ------
void setupTwiBus (void);			// set up ressources used
void requestTwiBus (void);			// assert start condition
void restartTwiBus (void);			// repeat start condition
char twi_sendByteMaster (char);	// send byte, check ACK
char twi_receiveByteMaster (char);	// receive byte, send ACK or not
void releaseTwiBus (void);			// assert stop condition
#elif defined __use_twi_multi_master__
// ------ multi master mode ------
void setupTwiBus (char);			// set up ressources used
char requestTwiBus (void);			// assert start condition
char restartTwiBus (void);			// repeat start condition
char twi_sendByteMaster (char);	// send byte, check ACK
char twi_receiveByteMaster (char);	// receive byte, send ACK or not
char twi_getaddressSlave (char*);	// check for start condition and receive byte
char twi_receiveByteSlave (char*);	// receive byte, send ACK
char twi_sendByteSlave (char);		// send byte, check ACK
char releaseTwiBus (void);			// assert stop condition
#elif defined __use_soft_twi_single_master__
// ------ single master mode, TWI software emulation ------
#else
#error: can not identify intended TWI usage mode
#endif


// ============================================================================
// depending on intended mode the library routines are instantiated
// ============================================================================

#if defined __use_twi_slave__
#if defined __avrUsi__
// ----------------------------------------------------------------------------
// slave mode using USI
// ----------------------------------------------------------------------------
void twi_sendAckSlave (void)
{
	USIDR = 0x00;						// prepare for ACK
	TWIddr |= (1<<TWIsdaBit);			// enable SDA as output to the bus
	USISR = (1<<USIOIF) | 14;			// release SCL and preset for ACK sending
	while ((USISR & ((1<<USIOIF) | (1<<USISIF) | (1<<USIPF))) == 0) {}
	TWIddr &= ~(1<<TWIsdaBit);			// release SDA
}

void setupTwiBus (char dummy)
{
	// setup does not disrupt any I²C transfer!
	USICR = (0b11<<USIWM0) | (0b10<<USICS0) | (0b0<<USICLK);
/* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
/* typo in datasheet? 0b10<<USICS0 selects shift on falling edge */
/* and vice versa!                                  =======      */
	TWIddr &= ~((1<<TWIsdaBit) | (1<<TWIsclBit));
	TWIport |= (1<<TWIsdaBit) | (1<<TWIsclBit);
	USISR = (1<<USIOIF) | (1<<USIDC) | (1<<USISIF) | (1<<USIPF);
	TWIddr |= (1<<TWIsclBit);			// enable SCL drive by slave device
}

char twi_getaddressSlave (char *address, char mask)
{
	char adr = *address & mask;

	USISR = (1<<USIOIF) | (1<<USIPF);	// clear the counter (otherwise SCL gets blocked later!)
	if ((USISR & (1<<USISIF)) != 0)
	{
		while ((TWIread & (1<<TWIsclBit)) != 0) {}// wait for falling edge SCL
		USISR = (1<<USIOIF) | (1<<USISIF) | (1<<USIPF);// clear some flags and also the counter
		while ((USISR & ((1<<USIOIF) | (1<<USISIF) | (1<<USIPF))) == 0) {}// do until counter rolls over, start or stop condition happens
		if ((USISR & ((1<<USISIF) | (1<<USIPF))) == 0)
		{
			*address = USIDR;
			if (adr == (*address & mask))
			{
				twi_sendAckSlave();
				if ((USISR & ((1<<USISIF) | (1<<USIPF))) == 0)
					return (__twiOk__);// base address match and not aborted
			}
		}
	}
	return (__twiFail__);
}

char twi_receiveByteSlave (char *data)
{
	USISR = (1<<USIOIF);				// clear counter
	while ((USISR & ((1<<USIOIF) | (1<<USISIF) | (1<<USIPF))) == 0) {}
	if ((USISR & ((1<<USISIF) | (1<<USIPF))) == 0)
	{
		*data = USIDR;
		twi_sendAckSlave();
		return (__twiOk__);			// data received
	}
	return (__twiFail__);				// aborted by start/stop condition
}

char twi_sendByteSlave (char data)
{
	USIDR = data;						// put data to shifter
	while ((TWIread & (1<<TWIsclBit)) != 0) {}// wait for falling edge SCL
	TWIddr |= (1<<TWIsdaBit);			// enable SDA as output to the bus
	USISR = (1<<USIOIF);				// clear the counter
	while ((USISR & ((1<<USIOIF) | (1<<USISIF) | (1<<USIPF))) == 0) {}// do until counter rolls over, start or stop condition
	TWIddr &= ~(1<<TWIsdaBit);			// release SDA
	if ((USISR & ((1<<USISIF) | (1<<USIPF))) == 0)
	{
    USISR = (1<<USIOIF) | 14;			// release SCL and preset for ACK checking
    while ((USISR & ((1<<USIOIF) | (1<<USISIF) | (1<<USIPF))) == 0) {}
    if ((USISR & ((1<<USISIF) | (1<<USIPF))) == 0)
    {
      if ((USIDR & 0x01) == 0)		// check status
        return (__twiOk__);		// ACK received
    }
  }
	return (__twiFail__);				// no Ack or action aborted by start/stop condition
}
#elif defined __avrTwi__
// ----------------------------------------------------------------------------
// slave mode using TWI
// ----------------------------------------------------------------------------
void setupTwiBus (char address)
{
	TWAR = address;									// only needed on slave
	TWCR = (1<<TWEN) | (1<<TWEA);					// only needed on slave
}

char twi_getaddressSlave (char *address)
{
	switch (TWSR & (0b11111<<TWS3))				// check action status
	{
		case 0xa8:	/* address + R received */
			*address |= 0x01;
		case 0x60:	/* address + W received */
			return (0);
		case 0xf8:	/* TWI busy */
			return (-1);
		default:
			TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTO) | (1<<TWEA);
			return (-1);
	}
}

char twi_receiveByteSlave (char *data)
{
	TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);	// receive with ACK
	while ((TWCR & (1<<TWINT)) == 0) {}			// wait until HW finishes
	*data = TWDR;
	switch (TWSR & (0b11111<<TWS3))				// check action status
	{
		case 0x80:	/* data received, ACK sent */
			return (0);
		default:
			return (-1);
	}
}

char twi_sendByteSlave (char data)
{
	TWDR = data;									// put data to shifter
	TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);	// transmit data
	while ((TWCR & (1<<TWINT)) == 0) {}			// wait until HW finishes
	switch (TWSR & (0b11111<<TWS3))				// check action status
	{
		case 0xb8:	/* data sent, ACK received */
			TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);// release SCL
			return (0);
		case 0xc0:	/* data sent, no ACK received */
			TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);// release SCL
		default:
			return (-1);
	}
}
#endif
#elif defined __use_twi_single_master__
// ----------------------------------------------------------------------------
// single master prerequisites checking
// ----------------------------------------------------------------------------
#if !defined (F_CPU)
#error: MCU frequency ('F_CPU') must be defined in makefile for usage of i2c library!
#endif

#if !defined (F_TWI)
#error: TWI frequency ('F_TWI') must be defined in makefile for usage of i2c library!
#endif

#if defined __avrUsi__
// ----------------------------------------------------------------------------
// single master using USI
// ----------------------------------------------------------------------------
#include <util/delay.h>

void twiDelay (void)
{
	_delay_us (500000UL/F_TWI);
}

void setupTwiBus (void)
{
	// setup does not disrupt any I²C transfer!
	USICR = (0b11<<USIWM0) | (0b10<<USICS0) | (0b0<<USICLK);
	TWIddr &= ~((1<<TWIsdaBit) | (1<<TWIsclBit));
	TWIport |= (1<<TWIsdaBit) | (1<<TWIsclBit);
	TWIddr |= (1<<TWIsclBit);
	USISR = (1<<USIOIF) | (1<<USIDC) | (1<<USISIF) | (1<<USIPF);
}

void requestTwiBus (void)
{
	USIDR = 0x00;					// MSB controls SDA immediately
	TWIddr |= (1<<TWIsdaBit);		// make SDA output to bus
	twiDelay ();					// wait according to bus specification
	USICR |= (1<<USITC);			// falling edge on SCL
	twiDelay ();					// wait according to bus specification
	USISR |= (1<<USISIF);			// clear start condition detector
}

void restartTwiBus (void)
{
	TWIddr &= ~(1<<TWIsdaBit);		// release SDA
	USIDR = 0x00;					// prepare SDA to pull down
	twiDelay ();					// wait according to bus specification
	USICR |= (1<<USITC);			// toggle SCL (0 -> 1)
	twiDelay ();					// wait according to bus specification
	while ((TWIread & (1<<TWIsclBit)) == 0) {}// wait for slower slave!
	TWIddr |= (1<<TWIsdaBit);		// make SDA output to bus
	twiDelay ();					// wait according to bus specification
	USICR |= (1<<USITC);			// falling edge on SCL
	twiDelay ();					// wait according to bus specification
	USISR |= (1<<USISIF);			// clear start condition detector
}

char twi_sendByteMaster (char data)
{
	char result;

	USIDR = data;						// put data to shifter
	TWIddr |= (1<<TWIsdaBit);			// enable SDA as output to the bus
	twiDelay ();						// wait according to bus specification
	USISR = (1<<USIOIF) | (1<<USIDC);	// clear some flags and also the counter
	while ((USISR & (1<<USIOIF)) == 0)// do until counter rolls over
	{
		USICR |= (1<<USITC);			// toggle SCL (0 -> 1)
		twiDelay ();					// wait according to bus specification
		while ((TWIread & (1<<TWIsclBit)) == 0) {}// wait for slower slave!
		USICR |= (1<<USITC);			// toggle SCL (1 -> 0)
		twiDelay ();					// wait according to bus specification
	}
	TWIddr &= ~(1<<TWIsdaBit);			// release SDA
	USISR = (1<<USIOIF);				// clear edge counter overflow flag
	twiDelay ();						// wait according to bus specification
	USICR |= (1<<USITC);				// toggle SCL (0 -> 1)
	twiDelay ();						// wait according to bus specification
	while ((TWIread & (1<<TWIsclBit)) == 0) {}// wait for slower slave!
	if ((TWIread & (1<<TWIsdaBit)) == 0)// check device ACK status
		result = 0;						// ACK received
	else
		result = -1;					// no response from device
	USICR |= (1<<USITC);				// toggle SCL (1 -> 0)
	return (result);					// return device ACK status
}

char twi_receiveByteMaster (char ackOrNotAck)
{
	char received;

	TWIddr &= ~(1<<TWIsdaBit);			// release SDA
	twiDelay ();						// wait according to bus specification
	USISR = (1<<USIOIF) | (1<<USIDC);	// clear some flags and also the counter
	while ((USISR & (1<<USIOIF)) == 0)// do until counter rolls over
	{
		USICR |= (1<<USITC);			// toggle SCL (0 -> 1)
		twiDelay ();					// wait according to bus specification
		while ((TWIread & (1<<TWIsclBit)) == 0) {}// wait for slower slave!
		USICR |= (1<<USITC);			// toggle SCL (1 -> 0)
		twiDelay ();					// wait according to bus specification
	}
	received = USIDR;					// read data just received
	USISR = (1<<USIOIF);				// clear edge counter overflow flag
	USIDR = 0x00;						// prepare for ACK
	if (ackOrNotAck == __twiAck__)
		TWIddr |= (1<<TWIsdaBit);		// enable SDA as output to the bus
	twiDelay ();						// wait according to bus specification
	USICR |= (1<<USITC);				// toggle SCL (0 -> 1)
	twiDelay ();						// wait according to bus specification
	while ((TWIread & (1<<TWIsclBit)) == 0) {}// wait for slower slave!
	USICR |= (1<<USITC);				// toggle SCL (1 -> 0)
	return (received);					// return data received
}

void releaseTwiBus (void)
{
	USIDR = 0x00;					// force SDA low
	TWIddr |= (1<<TWIsdaBit);		// enable SDA as output to the bus
	twiDelay ();					// wait according to bus specification
	USICR |= (1<<USITC);			// toggle SCL (0 -> 1)
	twiDelay ();					// wait according to bus specification
	while ((TWIread & (1<<TWIsclBit)) == 0) {}// wait for slower slave!
	TWIddr &= ~(1<<TWIsdaBit);		// release SDA
	twiDelay ();					// wait according to bus specification
	USISR |= (1<<USIPF);			// clear stop condition detector
}
#elif defined __avrTwi__
// ----------------------------------------------------------------------------
// single master using TWI
// ----------------------------------------------------------------------------
#define twiDivisor	(F_CPU/F_TWI-16)/2
#if (twiDivisor < 256)
#if (twiDivisor < 10)
#warning: ratio F_CPU/F_TWI too small, bus speed decreased!
#define twiBitrate	10
#else
#define twiBitrate	twiDivisor
#endif
#define twiPrescale		0x00	/* :1 */
#elif (twiDivisor < 1021)
#define twiBitrate		twiDivisor/4
#define twiPrescale		0x01	/* :4 */
#elif (twiDivisor < 4081)
#define twiBitrate		twiDivisor/16
#define twiPrescale		0x02	/* :16 */
#elif (twiDivisor < 16321)
#define twiBitrate		twiDivisor/64
#define twiPrescale		0x03	/* :64 */
#else
#warning: ratio F_CPU/F_TWI too large, prescaling not possible
#define twiBitrate		0xff
#define twiPrescale		0x03
#endif

void setupTwiBus (void)
{
	TWSR = twiPrescale;								// only needed on master
	TWBR = twiBitrate;								// only needed on master
}

void requestTwiBus (void)
{
	TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTA);	// assert start condition
	while ((TWCR & (1<<TWINT)) == 0) {}			// wait until HW finishes
}

void restartTwiBus (void)
{
	TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTA);	// assert start condition
	while ((TWCR & (1<<TWINT)) == 0) {}			// wait until HW finishes
}

char twi_sendByteMaster (char data)
{
	TWDR = data;									// put data to shifter
	TWCR = (1<<TWEN) | (1<<TWINT);					// transmit data
	while ((TWCR & (1<<TWINT)) == 0) {}			// wait until HW finishes
	switch (TWSR & (0b11111<<TWS3))				// check action status
	{
		case 0x18:	/* address + W successfully sent */
		case 0x40:	/* address + R successfully sent */
		case 0x28:	/* data byte successfully sent */
			return (0);
			break;
		default:	/* something went wrong - most likely slave not answering */
			return (-1);
	}
}

char twi_receiveByteMaster (char ackOrNotAck)
{
	if (ackOrNotAck == __twiAck__)
		TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);// receive with ACK
	else
		TWCR = (1<<TWEN) | (1<<TWINT);				// receive w/o ACK
	while ((TWCR & (1<<TWINT)) == 0) {}			// wait until HW finishes
	return (TWDR);									// return data received
}

void releaseTwiBus (void)
{
	TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTO);	// assert stop condition
	while ((TWCR & (1<<TWSTO)) == 0) {}			// wait until HW finishes
}

#endif
#elif defined __use_twi_multi_master__
// ----------------------------------------------------------------------------
// multi master prerequisites checking
// ----------------------------------------------------------------------------
#if !defined (F_CPU)
#error: MCU frequency ('F_CPU') must be defined in makefile for usage of i2c library!
#endif

#if !defined (F_TWI)
#error: TWI frequency ('F_TWI') must be defined in makefile for usage of i2c library!
#endif

#if defined __avrUsi__
// ----------------------------------------------------------------------------
// multi master using USI
// ----------------------------------------------------------------------------
#include <util/delay.h>

void twiDelay (void)
{
	_delay_us (500000UL/F_TWI);
}

void setupTwiBus (char dummy)
{
	// setup does not disrupt any I²C transfer!
	USICR = (0b11<<USIWM0) | (0b10<<USICS0) | (0b0<<USICLK);
	TWIddr &= ~((1<<TWIsdaBit) | (1<<TWIsclBit));
	TWIport |= (1<<TWIsdaBit) | (1<<TWIsclBit);
	TWIddr |= (1<<TWIsclBit);
	USISR = (1<<USIOIF) | (1<<USIDC) | (1<<USISIF) | (1<<USIPF);
}

#elif defined __avrTwi__
// ----------------------------------------------------------------------------
// multi master using TWI
// ----------------------------------------------------------------------------
#define twiDivisor	(F_CPU/F_TWI-16)/2
#if (twiDivisor < 256)
#if (twiDivisor < 10)
#warning: ratio F_CPU/F_TWI too small, bus speed decreased!
#define twiBitrate	10
#else
#define twiBitrate	twiDivisor
#endif
#define twiPrescale		0x00	/* :1 */
#elif (twiDivisor < 1021)
#define twiBitrate		twiDivisor/4
#define twiPrescale		0x01	/* :4 */
#elif (twiDivisor < 4081)
#define twiBitrate		twiDivisor/16
#define twiPrescale		0x02	/* :16 */
#elif (twiDivisor < 16321)
#define twiBitrate		twiDivisor/64
#define twiPrescale		0x03	/* :64 */
#else
#warning: ratio F_CPU/F_TWI too large, prescaling not possible
#define twiBitrate		0xff
#define twiPrescale		0x03
#endif

void setupTwiBus (char address)
{
	TWSR = twiPrescale;								// only needed on master
	TWBR = twiBitrate;								// only needed on master
	TWAR = address;									// only needed on slave
	TWCR = (1<<TWEN) | (1<<TWEA);					// only needed on slave
}

#endif
#elif defined __use_soft_twi_single_master__
// ----------------------------------------------------------------------------
// software emulated single master prerequisites checking
// ----------------------------------------------------------------------------
#if !defined (F_CPU)
#error: MCU frequency ('F_CPU') must be defined in makefile for usage of i2c library!
#endif

#if !defined (F_TWI)
#error: TWI frequency ('F_TWI') must be defined in makefile for usage of i2c library!
#endif

// ----------------------------------------------------------------------------
// software emulated single master
// ----------------------------------------------------------------------------
#include <util/delay.h>

void softTwiDelay (void)
{
	_delay_us (500000UL/F_TWI);
}

inline void setupSoftTwiBus (void)
{
	// setup does not disrupt any I²C transfer!
	softTWIddr &= ~((1<<softTWIsdaBit) | (1<<softTWIsclBit));
	softTWIport &= ~((1<<softTWIsdaBit) | (1<<softTWIsclBit));
}

inline void requestSoftTwiBus (void)
{
	softTWIddr |= (1<<softTWIsdaBit);		// falling edge on SDA
	softTwiDelay ();						// wait according to bus specification
	softTWIddr |= (1<<softTWIsclBit);		// falling edge on SCL
	softTwiDelay ();						// wait according to bus specification
}

inline void restartSoftTwiBus (void)
{
	softTWIddr &= ~(1<<softTWIsdaBit);		// SDA = '1'
	softTwiDelay ();						// wait according to bus specification
	softTWIddr &= ~(1<<softTWIsclBit);		// rising edge on SCL
	softTwiDelay ();						// wait according to bus specification
	while ((softTWIread & (1<<softTWIsclBit)) == 0) {}// wait for slower slave!
	softTWIddr |= (1<<softTWIsdaBit);		// falling edge on SDA
	softTwiDelay ();						// wait according to bus specification
	softTWIddr |= (1<<softTWIsclBit);		// falling edge on SCL
	softTwiDelay ();						// wait according to bus specification
}

inline char softTwi_sendByteMaster (char data)
{
	char result, bitcount;

	if (data & 0x80)
		softTWIddr &= ~(1<<softTWIsdaBit);	// SDA = '1'
	else
		softTWIddr |= (1<<softTWIsdaBit);	// SDA = '0'
	softTwiDelay ();						// wait according to bus specification
	for (bitcount=0; bitcount<8; bitcount++)
	{
		softTWIddr &= ~(1<<softTWIsclBit);	// rising edge on SCL
		softTwiDelay ();					// wait according to bus specification
		while ((softTWIread & (1<<softTWIsclBit)) == 0) {}// wait for slower slave!
		softTWIddr |= (1<<softTWIsclBit);	// falling edge on SCL
		softTwiDelay ();					// wait according to bus specification
		data = data << 1;
		if (data & 0x80)
			softTWIddr &= ~(1<<softTWIsdaBit);// SDA = '1'
		else
			softTWIddr |= (1<<softTWIsdaBit);// SDA = '0'
	}
	softTWIddr &= ~(1<<softTWIsdaBit);		// release SDA
	softTwiDelay ();						// wait according to bus specification
	softTWIddr &= ~(1<<softTWIsclBit);		// rising edge on SCL
	softTwiDelay ();						// wait according to bus specification
	while ((softTWIread & (1<<softTWIsclBit)) == 0) {}// wait for slower slave!
	if ((softTWIread & (1<<softTWIsdaBit)) == 0)// check device ACK status
		result = 0;							// ACK received
	else
		result = -1;						// no response from device
	softTWIddr |= (1<<softTWIsclBit);		// falling edge on SCL
	return (result);						// return device ACK status
}

inline char softTwi_receiveByteMaster (char ackOrNotAck)
{
	char received, bitcount;

	received = 0;
	softTWIddr &= ~(1<<softTWIsdaBit);		// release SDA
	softTwiDelay ();						// wait according to bus specification
	for (bitcount=0; bitcount<8; bitcount++)
	{
		softTWIddr &= ~(1<<softTWIsclBit);	// rising edge on SCL
		softTwiDelay ();					// wait according to bus specification
		while ((softTWIread & (1<<softTWIsclBit)) == 0) {}// wait for slower slave!
		received = received << 1;
		if ((softTWIread & (1<<softTWIsdaBit)) != 0)// read data bit
			received |= 0x01;					// '1'
		softTWIddr |= (1<<softTWIsclBit);	// falling edge on SCL
		softTwiDelay ();					// wait according to bus specification
	}
	if (ackOrNotAck == __twiAck__)
		softTWIddr |= (1<<softTWIsdaBit);	// SDA = '0'
	else
		softTWIddr &= ~(1<<softTWIsdaBit);	// SDA = '1'
	softTwiDelay ();						// wait according to bus specification
	softTWIddr &= ~(1<<softTWIsclBit);		// rising edge on SCL
	softTwiDelay ();						// wait according to bus specification
	while ((softTWIread & (1<<softTWIsclBit)) == 0) {}// wait for slower slave!
	softTWIddr |= (1<<softTWIsclBit);		// falling edge on SCL
	return (received);						// return data received
}

inline void releaseSoftTwiBus (void)
{
	softTWIddr |= (1<<softTWIsdaBit);		// SDA = '0'
	softTwiDelay ();						// wait according to bus specification
	softTWIddr &= ~(1<<softTWIsclBit);		// rising edge on SCL
	softTwiDelay ();						// wait according to bus specification
	while ((softTWIread & (1<<softTWIsclBit)) == 0) {}// wait for slower slave!
	softTWIddr &= ~(1<<softTWIsdaBit);		// rising edge on SDA
	softTwiDelay ();						// wait according to bus specification
}
#endif

#endif /* #ifndef __i2c_h__ */


/*****************************************************************************\
 *
 * $Id$
 *
 * $Log$
 *
\*****************************************************************************/
