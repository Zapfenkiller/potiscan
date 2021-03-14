/******************************************************************************\
*                                                                              *
* File        : joystick_twi.h                                                 *
* Project     : Roboter 2006                                                   *
* Author      :                                                                *
* Initial date:                                                                *
* Release rev.:                                                                *
* Copyright   :                                                                *
* Credits     :                                                                *
* License     :                                                                *
* Description : Definitions for TWI joystick service.                          *
*                                                                              *
\******************************************************************************/


#ifndef __JOYSTICK_TWI_H__
#define __JOYSTICK_TWI_H__

#ifndef F_CPU
#define F_CPU                   4000000UL
#warning: F_CPU set to 4MHz!
#endif

#define STICK_AT_MIN_RESI   43 /* LSB -   0K, replaced by individual trim */
#define STICK_AT_MAX_RESI 4439 /* LSB - 100k, replaced by individual trim */

#define RESCALING_FACTOR  (int16_t)(6 * \
                          ((STICK_AT_MAX_RESI - STICK_AT_MIN_RESI) / \
                          (DESIRED_MAX_READING - DESIRED_MIN_READING + 1) \
                          + 0.5))

/* ######## properties ######## */
#define   SCAN_PERIOD           2000UL  /* us */
#define   CAPTURE_LIMIT         5332UL  /* clocks - also limit of conversion
                                           input, MAXIMUM is 32767 / 6 = 5461! */
#define   DESIRED_MAX_READING    247L   /* equivalent to max resistance detected */
#define   DESIRED_MIN_READING      8L   /* equivalent to min resistance detected */
#define   ABSOLUTE_MAX_READING   255L   /* e.g. pot not connected */
#define   ABSOLUTE_MIN_READING     0L
#if (STICK_AT_MAX_RESI >= CAPTURE_LIMIT)
#warning: STICK_AT_MAX_RESI beyond timeout - will deny proper function!
#endif

/* ######## MCU-type selection ######## */
#ifdef __AVR_ATtiny2313__
/* =========== ATtiny2313 ============= */
/* - Unused IO pads, not connected on PCB! - */
#define   NC_PORT1              PORTB
#define   NC_DDR1               DDRB
#define   NC_BITS1              (1 << 6)
#define   SET_UNUSED_AS_INPUTS  NC_DDR1 &= ~NC_BITS1
#define   SET_UNUSED_AS_GND     NC_PORT1 &= ~NC_BITS1;\
                                NC_DDR1 |= NC_BITS1
#define   SET_UNUSED_AS_VCC     NC_PORT1 &= NC_BITS1;\
                                NC_DDR1 |= NC_BITS1
/* - Joystick pots -------------------- */
#define   POT_PORT              PORTD
#define   POT_DDR               DDRD
#define   POT1                  PD2     /* Joy 1 X */
#define   POT2                  PD4     /* Joy 1 Y */
#define   POT3                  PD3     /* Joy 2 X */
#define   POT4                  PD5     /* Joy 2 Y */
#define   POT1_BIT              (1 << POT1)
#define   POT2_BIT              (1 << POT2)
#define   POT3_BIT              (1 << POT3)
#define   POT4_BIT              (1 << POT4)
#define   POT_BITS              (POT1_BIT | POT2_BIT | POT3_BIT | POT4_BIT)
#define   FAST_DISCHARGE        POT_PORT &= ~POT_BITS; \
                                POT_DDR |= POT_BITS
#define   STOP_CHARGING         POT_DDR &= ~POT_BITS; \
                                POT_PORT &= ~POT_BITS
#define   CHARGE_POT1           POT_DDR |= POT1_BIT; \
                                POT_PORT |= POT1_BIT
#define   CHARGE_POT2           POT_DDR |= POT2_BIT; \
                                POT_PORT |= POT2_BIT
#define   CHARGE_POT3           POT_DDR |= POT3_BIT; \
                                POT_PORT |= POT3_BIT
#define   CHARGE_POT4           POT_DDR |= POT4_BIT; \
                                POT_PORT |= POT4_BIT
/* - Joystick buttons ----------------- */
#define   BUTTON_INPORT1        PINB
#define   BUTTON_PORT1          PORTB
#define   BUTTON_DDR1           DDRB
#define   BUTTON1               PB4     /* Joy 1 PB 1 */
#define   BUTTON3               PB2     /* Joy 2 PB 1 */
#define   BUTTON4               PB3     /* Joy 2 PB 2 */
#define   BUTTON_INPORT2        PIND
#define   BUTTON_PORT2          PORTD
#define   BUTTON_DDR2           DDRD
#define   BUTTON2               PD6     /* Joy 1 PB 2 */
#define   BUTTON1_BIT           (1 << BUTTON1)
#define   BUTTON2_BIT           (1 << BUTTON2)
#define   BUTTON3_BIT           (1 << BUTTON3)
#define   BUTTON4_BIT           (1 << BUTTON4)
#define   BUTTON_BITS1          (BUTTON1_BIT | BUTTON3_BIT | BUTTON4_BIT)
#define   BUTTON_BITS2          BUTTON2_BIT
#define   INIT_BUTTON_PORTS     BUTTON_DDR1 &= ~BUTTON_BITS1;\
                                BUTTON_PORT1 |= BUTTON_BITS1;\
                                BUTTON_DDR2 &= ~BUTTON_BITS2;\
                                BUTTON_PORT2 |= BUTTON_BITS2
/* - Analog comparator ---------------- */
#define   AINM                  PB1     /* common */
#define   AINP                  PB0     /* reference */
#define   VOLT_BELOW_THR        (ACSR & (1<<ACO))
#define   STOP_DISCHARGING      DDRB &= ~(1<<AINM)
#define   START_DISCHARGING     DDRB |= (1<<AINM)
/* - 16-bit timer --------------------- */
#define   T1_STOP               (0b000 << CS10)
#define   T1_FULL_CLK           (0b001 << CS10)
#define   T1_CAPTURE_NEGEDGE    (0 << ICES1)
#define   T1_CAPTURE_NO_NOISE   (1 << ICNC1)
#define   T1_MODE_REG_A         TCCR1A
#define   T1_MODE_REG_B         TCCR1B
#define   CAPTURE_RESULT_REG    ICR1
#define   CAPTURE_OCCURED       (TIFR & (1 << ICF1))
#define   CLEAR_CAPTURE_FLAG    TIFR = (1 << ICF1)
#define   POT_TIMEOUT           (CAPTURE_LIMIT*1e6/F_CPU)
#define   INIT_T1               OCR1A = ((F_CPU/1e6)*POT_TIMEOUT)-1-IRQ_RESPONSE_CLOCKS; \
                                OCR1B = ((F_CPU/1e6)*SCAN_PERIOD)-1-IRQ_RESPONSE_CLOCKS-IRQ_REINIT_DELAY_CLKS; \
                                TIMSK |= (1 << OCIE1A) | (1 << OCIE1B);
#define   START_T1_OPERATION    TCCR1B = T1_CAPTURE_NO_NOISE | T1_CAPTURE_NEGEDGE | T1_FULL_CLK
#define   STOP_T1_OPERATION     TCCR1B = T1_CAPTURE_NO_NOISE | T1_CAPTURE_NEGEDGE | T1_STOP
#define   CLEAR_T1_COUNT_REG    TCNT1 = 0
/* - 8-bit timer ---------------------- */
#define   T0_CLK_64             (0b011 << CS00)
#define   T0_CLK_256            (0b100 << CS00)
#define   T0_MODE_REG_A         TCCR0A
#define   T0_MODE_REG_B         TCCR0B
#define   INIT_T0               TIMSK |= (1 << TOIE0)
#define   START_T0_OPERATION    TCCR0B = T0_CLK_64
#if (F_CPU != 4000000UL)
#warning: key debouncing rate needs readjustment!!!
#endif
/* - TWI (USI) ------------------------ */
//        SCL                   PB7
//        SDA                   PB5
#define   TWIADDR_PORT          PORTD
#define   TWIADDR_DDR           DDRD
#define   TWIA0                 PD0     /* A0 */
#define   TWIA1                 PD1     /* A1 */
#define   TWIA0_BIT             (1 << TWIA0)
#define   TWIA1_BIT             (1 << TWIA1)
#define   TWIADDR_BITS          (TWIA0_BIT | TWIA1_BIT)
#define   INIT_TWIADDR_PORTS    TWIADDR_DDR &= ~TWIADDR_BITS;\
                                TWIADDR_PORT |= TWIADDR_BITS
#end of __AVR_ATtiny2313__
/* - Interrupts ----------------------- */
#define   IRQ_RESPONSE_CLOCKS   8       /* average - measured with debugger */
#define   IRQ_REINIT_DELAY_CLKS 27      /* average - measured with debugger */

#else
#error:   sorry, MCU type not supported!
#endif // ifdef __AVR_ATtiny2313__

/* ######## interface to key debouncing routines of P. Dannegger ######## */
#define   KEY_DDR1              BUTTON_DDR
#define   KEY_PORT1             BUTTON_PORT
#define   KEY_PIN1              BUTTON_INPORT
#define   KEY_DDR2              BUTTON_DDR
#define   KEY_PORT2             BUTTON_PORT
#define   KEY_PIN2              BUTTON_INPORT

#endif // #ifndef __JOYSTICK_TWI_H__



/******************************************************************************
 *
 * $Id$
 *
 * $Log$
 *
 *****************************************************************************/
