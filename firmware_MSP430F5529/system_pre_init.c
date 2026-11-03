/*******************************************************************************
 * system_pre_init.c
 *
 * _system_pre_init() is called by the C startup code (_c_int00) BEFORE the
 * .data/.bss segment initialisation and BEFORE main().
 *
 * WHY THIS FILE IS MANDATORY HERE
 * -------------------------------
 * After reset the watchdog is ENABLED, clocked from SMCLK / 32768, which is
 * about 31 ms at the default ~1 MHz DCO. The auto-init step has to clear the
 * whole .bss (about 1 KB) and copy .data, and once this firmware grew that
 * work took longer than one watchdog period.
 *
 * The result was a permanent reset loop during initialisation:
 *   - .bss was only cleared up to roughly 0x2780, the tail stayed random
 *   - main() was NEVER entered, so not a single line of it ever ran
 *   - the board looked completely dead: no LEDs, no LCD, no UART
 *   - the image was nevertheless programmed correctly and the reset vector
 *     was right, which is why this looked like a hardware / flashing problem
 *
 * Disabling the watchdog here is the standard TI fix for MSP430 projects;
 * the LAB1 reference project ships exactly this file for the same reason.
 *
 * Return 1 so that segment initialisation still takes place.
 ******************************************************************************/
#include <msp430.h>

int _system_pre_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;   /* stop the watchdog before auto-init */

    return 1;
}
