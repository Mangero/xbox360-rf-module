// Define device if not already defined
#ifndef __AVR_ATtiny85__
#define __AVR_ATtiny85__
#endif

#include <math.h>
#include <inttypes.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>

#define INT_REG GIMSK  // Register controlling interrupt behaviour.
#define DIR_REG DDRB   // Register controlling the data direction of the pins.
#define IN_REG  PINB   // Register for reading values from pins.
#define OUT_REG PORTB  // Register for writing values to pins.

#define button_pin PB2 // Pin connected to button (physical pin 7 on ATtiny, 5 on xbox module).
#define data_pin   PB4 // Data line (physical pin 3 on ATtiny, 6 on xbox module).
#define clock_pin  PB3 // Clock line (physical pin 2 on ATtiny, 7 on xbox module).

// Calculate Timer1 prescaler value. Works for all CPU frequencies which are a multiple of 125 kHz.
// With avr-gcc this is calculated at compile-time.
#define PRESCALE_SELECT ((uint8_t) (log(F_CPU / 125000UL) / log(2) + 1))

// Macros for reading and writing to individual pins.
#define SET_PIN(reg, pin) (reg |= (1 << pin))
#define CLR_PIN(reg, pin) (reg &= ~(1 << pin))
#define READ_PIN(pin) ((IN_REG >> pin) & 0x01)
#define WRITE_PIN(pin, value) (value ? SET_PIN(OUT_REG, pin) : CLR_PIN(OUT_REG, pin))

// Commands for controlling the xbox controller module.
static const uint16_t leds_cmd = 0x0084;     // Initializes the LEDs, leaving the center LED lit.
static const uint16_t anim_cmd = 0x0085;     // Makes the startup animation on the ring of light.
static const uint16_t sync_cmd = 0x0004;     // Initiates the sync process.
static const uint16_t ctrl_off_cmd = 0x0009; // Turn off all controllers.

static const uint16_t long_press_length = 2000;      // Length of a long press in milliseconds.
static volatile uint32_t button_status = 0xFFFFFFFF; // Button press status for debouncing.
static volatile uint32_t millis_value;               // Button press duration in milliseconds.

void send_data(uint16_t cmd)
{
	// Prepare data pin for sending data by setting it as output.
	SET_PIN(DIR_REG, data_pin);

	// Synchronize with the clock to ensure data is valid on positive clock flank.
	while (READ_PIN(clock_pin) == 0) {}

	// Send command to the xbox module.
	for (int8_t i = 9; i >= 0; i--) {
		// Wait for clock pin to go low.
		while (READ_PIN(clock_pin) == 1) {}

		// Output next bit in the command to data pin.
		WRITE_PIN(data_pin, ((cmd >> i) & 0x0001));

		// Wait for clock pin to go high.
		while (READ_PIN(clock_pin) == 0) {}
	}

	// Switch data pin to high impedence mode.
	CLR_PIN(DIR_REG, data_pin); // Set as input.
	WRITE_PIN(data_pin, 0);     // Pull pin low.
}

// Interrupt vector for INT0 (button press).
ISR(INT0_vect)
{
	// Disable further interrupts on INT0.
	INT_REG &= ~(1 << INT0);
}

// Interrupt vector for Timer1, triggers every millisecond.
ISR(TIMER1_COMPA_vect)
{
	// Increment millisecond counter.
	millis_value++;
	// Update button status (used for debouncing).
	button_status = (button_status << 1) | READ_PIN(button_pin);
}

void timer_init()
{
	// Set Output Compare Register to match with timer counter every millisecond.
	OCR1C = 125;
	// Enable Clear Timer on Compare Match.
	TCCR1 = (1 << CTC1);
	// Enable interrupt on match with Compare Register A.
	TIMSK = (1 << OCIE1A);
}

void timer_start()
{
	TCCR1 = (TCCR1 & 0xF0) | PRESCALE_SELECT; // TODO Fungerar denna mask med Mega 328P?
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
	// Disable global interrupts to avoid race condition with INT0-interrupt.
	cli();
	// Reset millisecond counter.
	millis_value = 0;
	// Set button_status value to avoid premature button release condition.
	button_status = 0xFFFFFFFE;
	// Enable interrupt on low INT0 (button_pin).
	INT_REG |= (1 << INT0);
	sleep_enable();
	// The instruction following sei() will always run before any interrupts are triggered.
	sei();
	sleep_cpu();
	sleep_disable();
}

int main(void) {
	// Enable all internal pull-ups except for the data pin, which has an external pull-up.
	OUT_REG = (0x3F & ~(1 << data_pin));

	// Wait for xbox module to be ready for communitation.
	_delay_ms(100);
	// Initialize xbox module LEDs.
	send_data(leds_cmd);

	// Boolean indicating if the current button press is a long press.
	bool long_press = false;

	// Setup registers controlling Timer/Counter1.
	timer_init();
	// Start timer measuring length of button press.
	timer_start();
	// Set sleep mode.
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	// Sleep awaiting button press.
	sleep();

	// Main loop.
	while (1) {
		// Check for long press condition while button is held down.
		if (!long_press && button_status == 0x00000000) {
			if (millis_value >= long_press_length) {
				// Pair controllers on long press.
				send_data(sync_cmd);
				long_press = true;
			}
		// Check if button has been released.
		} else if (button_status == 0xFFFFFFFF) {
			if (!long_press) {
				// Turn off controllers on short press.
				send_data(ctrl_off_cmd);
			}

			// Reset long press status.
			long_press = false;
			// Sleep awaiting next button press.
			sleep();
		}

	}
}
