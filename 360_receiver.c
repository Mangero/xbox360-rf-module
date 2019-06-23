// Define device if not already defined
#ifndef __AVR_ATtiny85__
	#define __AVR_ATtiny85__
#endif

// Define CPU frequency if not already defined
#ifndef F_CPU
	#define F_CPU 1000000UL // 1 MHz
#endif

#if F_CPU == 1000000UL
	#define PRESCALE_SELECT 0x04
#elif F_CPU == 8000000UL
	#define PRESCALE_SELECT 0x07
#elif F_CPU == 16000000UL
	#define PRESCALE_SELECT 0x08
#else
	#define PRESCALE_SELECT 0x04
#endif

#include <inttypes.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define INT_REG GIMSK  // Register controlling interrupt behaviour
#define DIR_REG DDRB   // Register controlling the data direction of the pins
#define IN_REG  PINB   // Register for reading values from pins
#define OUT_REG PORTB  // Register for writing values to pins

#define button_pin PB2 // Pin connected to button (physical pin 7 on ATtiny, 5 on xbox module)
#define data_pin   PB4 // Data line (physical pin 3 on ATtiny, 6 on xbox module)
#define clock_pin  PB3 // Clock line (physical pin 2 on ATtiny, 7 on xbox module)

// Macros for reading and writing to individual pins
#define SET_PIN(reg, pin) (reg |= (1 << pin))
#define CLR_PIN(reg, pin) (reg &= ~(1 << pin))
#define READ_PIN(pin) ((IN_REG >> pin) & 0x01)
#define WRITE_PIN(pin, value) (value ? SET_PIN(OUT_REG, pin) : CLR_PIN(OUT_REG, pin))

// Commands for controlling the xbox controller module
const uint16_t leds_cmd = 0x0084;     // Initializes the LEDs, leaving the center LED lit
const uint16_t anim_cmd = 0x0085;     // Makes the startup animation on the ring of light
const uint16_t sync_cmd = 0x0004;     // Initiates the sync process
const uint16_t ctrl_off_cmd = 0x0009; // Turn off all controllers

volatile uint32_t millis_value = 0;      // Variable for storing current timer value in milliseconds
const uint16_t long_press_length = 2000; // Length of a long press in milliseconds

void send_data(uint16_t cmd)
{
	// Prepare data pin for sending data by setting it as output
	SET_PIN(DIR_REG, data_pin);

	// Synchronize with the clock to ensure data is valid on positive clock flank
	while (READ_PIN(clock_pin) == 0) {}

	// Send command to the xbox module
	for (int8_t i = 9; i >= 0; i--) {
		// Wait for clock pin to go low
		while (READ_PIN(clock_pin) == 1) {}

		// Output next bit in the command to data pin
		WRITE_PIN(data_pin, ((cmd >> i) & 0x0001));

		// Wait for clock pin to go high
		while (READ_PIN(clock_pin) == 0) {}
	}

	// Switch data pin to high impedence mode
	CLR_PIN(DIR_REG, data_pin); // Set as input
	WRITE_PIN(data_pin, 0);     // Pull pin low
}

// Empty interrupt vector for INT0 (button press)
EMPTY_INTERRUPT(INT0_vect);

ISR(TIMER1_COMPA_vect)
{
	millis_value++;
}

void timer_init()
{
	// Set Output Compare Register so that it matches every millisecond TODO Bättre kommentar
	OCR1A = 0x7D;
	OCR1C = 0x7D;
	// Enable Clear Timer on Compare Match
	TCCR1 = (1 << CTC1);
	// Enable interrupt on match with Compare RegisterA
	TIMSK = (1 << OCIE1A);
}

void timer_start()
{
	TCCR1 = (TCCR1 & 0xF0) | PRESCALE_SELECT;
}

void timer_stop()
{
	GTCCR |= (1 << PSR1);
}

void timer_reset()
{
	TCNT1 = 0x00;
}

void sleep()
{
	// Set sleep mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	// Enable interrupt on low INT0 (pin 2, physical pin 6 on ATtiny)
	INT_REG |= (1 << INT0);
	// Enable sleep and enter sleep mode
	sleep_mode();
	// Disable further interrupts on INT0 until entering sleep again
	INT_REG &= ~(1 << INT0);
}

int main(void) {
	// Enable all internal pull-ups except for the data pin, which has an external pull-up
	OUT_REG = (0x3F & ~(1 << data_pin));

	// Wait for xbox module to be ready for communitation
	_delay_ms(100);

	// Initialize xbox module LEDs
	send_data(leds_cmd);

	// Boolean indicating if the current button press is a long press
	bool long_press_active = false;

	// Setup registers controlling Timer/Counter1
	timer_init();

	// Start timer measuring length of button press
	timer_start();

	// Enable interrupts
	sei();

	// Main loop
	while (1) {
		// Sleep awaiting button press
		sleep();

		// Start timer measuring length of button press
		//timer_start();
		// Reset millisecond counter
		millis_value = 0;

		// Simple debounce
		_delay_ms(50);

		// Wait for button to be released or be held long enough for a long press
		while (!long_press_active && READ_PIN(button_pin) == 0) {
			if (millis_value >= long_press_length) {
				long_press_active = true;
			}
		}

		if (long_press_active) {
			// Reset boolean and pair controllers on long press
			long_press_active = false;
			send_data(sync_cmd);
		} else {
			// Turn off controllers on short press
			send_data(ctrl_off_cmd);
		}

		// Wait for button to be released before sleeping again
		while (READ_PIN(button_pin) == 0) {}
	}
}
