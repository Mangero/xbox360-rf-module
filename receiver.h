/*******************************************************************************
 * File: receiver.h
 *
 * Defines for microcontroller register names.
 * Macros for reading from and writing to individual pins on the controllers.
 ******************************************************************************/

#ifndef XBOX360_RECEIVER_H
#define XBOX360_RECEIVER_H 1

// Include math for log function.
#include <math.h>

// Board specific defines.
#if defined (__AVR_ATtiny25__) || defined (__AVR_ATtiny45__) || defined (__AVR_ATtiny85__)
	// Calculate Timer1 prescaler value. Works for all CPU frequencies which are a multiple of 125 kHz.
	// With avr-gcc this is calculated at compile-time.
	#define PRESCALE_SELECT ((uint8_t) (log(F_CPU / 125000UL) / log(2) + 1))
	#define INT_REG GIMSK    // Register controlling interrupt behaviour.
	#define TIMER_COMP_VECT TIMER1_COMPA_vect // Timer compare vector.

	#define OCR_REG OCR1C    // Output compare register for hardware timer.
	#define TCCRA_REG TCCR1  // Timer/Counter Control Register.
	#define TCCRB_REG TCCR1  // Timer/Counter Control Register.
	#define TIMSK_REG TIMSK  // Timer Interrupt Mask Register.
	#define OCI_BIT OCIE1A   // Output Compare Interrupt bit.
	#define CTC_BIT CTC1     // Clear Timer on Compare bit.
	#define PSR_BIT PSR1     // Prescaler Reset Timer/Counter.

	#define DIR_REG DDRB     // Register controlling the data direction of the pins.
	#define IN_REG  PINB     // Register for reading values from pins.
	#define OUT_REG PORTB    // Register for writing values to pins.

	#define BUTTON_PIN PB2   // Pin connected to button (physical pin 7 on ATtiny, 5 on xbox module).
	#define DATA_PIN   PB4   // Data line (physical pin 3 on ATtiny, 6 on xbox module).
	#define CLOCK_PIN  PB3   // Clock line (physical pin 2 on ATtiny, 7 on xbox module).
#elif defined (__AVR_ATmega48A__)  || defined (__AVR_ATmega48PA__)  || \
      defined (__AVR_ATmega88A__)  || defined (__AVR_ATmega88PA__)  || \
      defined (__AVR_ATmega168A__) || defined (__AVR_ATmega168PA__) || \
      defined (__AVR_ATmega328__)  || defined (__AVR_ATmega328P__)
	#define PRESCALE_SELECT 5 // TODO Make dynamic
	#define INT_REG EIMSK    // Register controlling interrupt behaviour.
	#define TIMER_COMP_VECT TIMER2_COMPB_vect // Timer compare vector.

	#define OCR_REG OCR2A    // Output compare register for hardware timer.
	#define TCCRA_REG TCCR2A // Timer/Counter Control Register A.
	#define TCCRB_REG TCCR2B // Timer/Counter Control Register B.
	#define TIMSK_REG TIMSK2 // Timer Interrupt Mask Register.
	#define OCI_BIT OCIE2B   // Output Compare Interrupt bit.
	#define CTC_BIT WGM21    // Clear Timer on Compare bit.
	#define PSR_BIT PSRASY   // Prescaler Reset Timer/Counter.

	#define DIR_REG DDRD     // Register controlling the data direction of the pins.
	#define IN_REG  PIND     // Register for reading values from pins.
	#define OUT_REG PORTD    // Register for writing values to pins.

	#define BUTTON_PIN PD2   // Pin connected to button.
	#define DATA_PIN   PD4   // Data line.
	#define CLOCK_PIN  PD3   // Clock line.
#endif

// Macros for reading and writing to individual pins.
#define SET_PIN(reg, pin)     ((reg) |= (1 << (pin)))
#define CLR_PIN(reg, pin)     ((reg) &= ~(1 << (pin)))
#define READ_PIN(pin)         ((IN_REG >> (pin)) & 0x01)
#define WRITE_PIN(pin, value) ((value) ? SET_PIN(OUT_REG, (pin)) : CLR_PIN(OUT_REG, (pin)))

#endif // XBOX360_RECEIVER_H
