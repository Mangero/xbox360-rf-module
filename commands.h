/*******************************************************************************
 * File: commands.h
 *
 * Commands for controlling the xbox controller module.
 ******************************************************************************/

#ifndef XBOX360_MODULE_COMMANDS_H
#define XBOX360_MODULE_COMMANDS_H 1

#define CMD_PAIR 0x0004     // Initiates the controller pairing process.
#define CMD_CTRL_OFF 0x0009 // Turn off all controllers.
#define CMD_LEDS_OFF 0x0080 // Turn off all the LEDs on the module.
#define CMD_LEDS 0x0084     // Initializes the LEDs, leaving the center LED lit.
#define CMD_ANIM 0x0085     // Makes the startup animation on the ring of light.

#endif // XBOX360_MODULE_COMMANDS_H
