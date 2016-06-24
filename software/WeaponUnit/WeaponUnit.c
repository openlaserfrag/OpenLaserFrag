/**
 * Firmware for OpenLaserFrag 0.9 weapon, www.openlaserfrag.org
 * Firmware revision 0.9.2
 *
 * Copyright (c) 2015
 * - Bianco Zandbergen <zandbergenb AT gmail.com>.
 * - Gerard de Leeuw <gdeleeuw AT leeuwit.nl>.
 *
 * Build with AVR-GCC, target ATmega168, no optimization
 * Target Hardware: http://scout.kvz.tudelft.nl/OpenLaserFrag/OLF0.9/OLF0.9_3.pdf
 *
 * CHANGELOG
 *
 * revision 0.9.2:
 *					- Use ATmega168 instead of ATtiny2313
 *					- Move handling of FragWire to a separate library
 *					- Move handling of MilesTag to a separate library
 * 
 * revision 0.9.1:
 *                  - improved ISR(TIMER0_COMPA_vect)
 *  
 * revision 0.9: 	
 *                  - Initial release
 *                  - Needs to be tested extensively with target hardware
 *                  - Implements UART receive communication and packet handling
 *                  - Implements Infrared transmit protocol
 *                  - Implements weapon trigger handling
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
**/

// CPU speed of 16 Mhz, is used by delay.h
#define F_CPU 16000000UL

#include <stddef.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "../FragWire/FragWire.h"
#include "../MilesTag/MilesTag.h"

// Team ID and player ID
#define TEAM_ID     MT_TID_GREEN
#define PLAYER_ID   0x54 // 7-bits, so should be less than 128
#define DAMAGE		MT_DAMAGE_35

// weapon states
#define DISABLED    0
#define ENABLED     1

// Infrared transmit data lengths for Timer0
#define TIMER0_16MHZ_2400USEC   150
#define TIMER0_16MHZ_1200USEC   75
#define TIMER0_16MHZ_600USEC    38

//#define TRIGGER_PULLUP    // If trigger has pullup resistor, define and undefine TRIGGER_PULLDOWN
#define TRIGGER_PULLDOWN    // If trigger has pulldown resistor, define and undefine TRIGGER_PULLUP

// Status LED 
#define ENABLE_STATUS			PORTD |= (1 << PD7)
#define DISABLE_STATUS			PORTD &= ~(1 << PD7)
#define TOGGLE_STATUS			PORTD ^= (1 << PD7)

// function prototypes
void fw_process_packet(fw_packet fw_rx_packet);
void init_statusled(void);
void init_muzzleled(void);
void init_timer2(void);
void init_extint(void);
void fire(void);

/**
 * The state of the weapon
 * state can be ENABLED of DISABLED
 * If state is DISABLED the weapon will not respond at trigger
 */
unsigned char weapon_state = ENABLED;

/**
 * The weapon is cooling down
 */
unsigned char cooling_down = 0;

/**
 * FragWire packet (used to send)
 */
fw_packet fw_tx_packet;

/**
 * MilesTag shot packet (used to send)
 */
mt_shot_packet mt_tx_shot_packet;

/**
 * USART Receive buffer empty interrupt service routine
 * Handles the transmission of a FragWire packet
 */
ISR(USART_UDRE_vect)
{
	fw_ISR_USART_UDRE();
}

/**
 * Interrupt service routine for UART receive interrupt
 * Checks if UART receive buffer is not full and
 * pushes the received byte to the UART receive buffer
 */ 
ISR(USART_RX_vect) {
	fw_ISR_USART_RX();
}

/**
 * Interrupt service routine for Timer0 Output Compare Match A interrupt.
 * This routine is used by the infrared send protocol.
 */
ISR(TIMER0_COMPA_vect) 
{
	mt_ISR_TIMER0_COMPA();
}

/**
 * Timer2 overflow interrupt routine
 * Allows a new shot to be fired
 */
ISR(TIMER2_OVF_vect) 
{
	TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20)); // disable timer2
	cooling_down = 0;
	ENABLE_STATUS; // enable status LED
}

/**
 * Interrupt service routine for INT0
 * Trigger of the weapon initiates the interrupt
 * If it is allowed, shoot
 */
ISR(INT0_vect) 
{
	fire();
}

/**
 * Main routine
 * Initializes system
 * Goes into endless loop and assembles packets.
 * Calls fw_process_packet() when a packet is assembled.
 */
int main(void) 
{	
    // init system
    fw_init(fw_process_packet);
    mt_tx_init();
    init_statusled();
    init_muzzleled();
	init_timer2();
	init_extint();
	
    wdt_enable(WDTO_1S); // enable watchdog timer, 1 second timeout
		    
	sei(); // enable interrupts
	
	ENABLE_STATUS; // enable status LED
		
    while(1) {	
	
        wdt_reset(); // reset watchdog timer

        fw_assemble_packet();
		
    }
	
    return -1;
}

/**
 * Process an assembled packet
 * This function is called by the packet assembler
 * when it has completed assembling a packet
 */
void fw_process_packet(fw_packet fw_rx_packet) 
{
	switch (fw_rx_packet.data[0]) { // fw_rx_packet.data[0] contains the message identifier
		case FW_MID_DISABLE_WEAPON:
			weapon_state = DISABLED;		
			break;
		case FW_MID_ENABLE_WEAPON:
			weapon_state = ENABLED;		
			break;			
		case FW_MID_NOP:
			// no operation, do nothing
		    break;			
		default:
			break;
    }
}

/**
 * Initialize pin used for status LED
 */
void init_statusled(void)
{
	DDRD |= (1 << PD7); // PD7 as output
	PORTD &= ~(1 << PD7); // set PD7 to low, so status led is off in the beginning
}

/**
 * Initialize pin used for muzzle LED
 */
void init_muzzleled(void)
{
	DDRB |= (1 << PB1); // PB1/OC1A as output
	PORTB &= ~(1 << PB1); // set PB1 to low, so muzzle led is off in the beginning
}

/**
 * Initialize Timer2, used by throttle the rate of fire
 */
void init_timer2(void)
{
    TIMSK2 |= (1 << TOIE2);		// Overflow interrupt enable
}

/**
 * Initialize external interrupt used for trigger
 */
void init_extint(void)
{
    DDRD &= ~ (1 << PD2); // set PD2/INT0 data direction to input	
    EIMSK |= (1 << INT0); // enable INT0 in external interrupt mask register

#ifdef TRIGGER_PULLUP

    EICRA |= (1 << ISC01); // interrupt on falling edge

#endif

#ifdef TRIGGER_PULLDOWN

    EICRA |= (1 << ISC01) | (1 << ISC00); // interrupt on rising edge

#endif

}

/**
 * Fire an IR bullet
 */
void fire(void) 
{
	// PORTB ^= (1 << PB1);
	
	// only fire if allowed
	if (mt_get_tx_state() == MT_TX_IDLE && weapon_state == ENABLED && cooling_down == 0) {
		DISABLE_STATUS;
		
		cooling_down = 1;
		TCNT2 = 1;
		TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // enable timer2, CLK/1024
				
		mt_tx_shot_packet.player_id = PLAYER_ID;
		mt_tx_shot_packet.team_id = TEAM_ID;
		mt_tx_shot_packet.damage = DAMAGE;
		mt_send_shot_packet(mt_tx_shot_packet);
	}
}