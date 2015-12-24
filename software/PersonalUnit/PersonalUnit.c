/**
 * Firmware for OpenLaserFrag 0.9 Personal Unit, www.openlaserfrag.org
 * Firmware revision 0.9.1
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
 * revision 0.9.1:
 *					- Move handling of FragWire to a separate library
 * 
 * revision 0.9: 	
 *                  - Initial release
 *                  - Needs to be tested extensively with target hardware
 *                  - Implements UART receive/transmit communication and packet handling
 *                  - accepts player hit message and gives player a timeout 
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

#define TIMER1_1SEC (F_CPU / 1024) // Output compare register value, 1 second (16000000 / 1024 pre-scaler)

// weapon states
#define DISABLED	0
#define ENABLED		1

#define PLAYER_TIMEOUT_SEC 10 // number of seconds a player gets a timeout after being killed

// player states
#define PLAYER_ALIVE	0
#define PLAYER_TIMEOUT	1

// Status LED
#define ENABLE_STATUS			PORTD |= (1 << PD4)
#define DISABLE_STATUS			PORTD &= ~(1 << PD4)
#define TOGGLE_STATUS			PORTD ^= (1 << PD4)

// function prototypes
void fw_process_packet(fw_packet fw_rx_packet);
void init_statusled(void);
void init_timer1(void);
void player_hit(void);

/**
 * FragWire packet (used to send)
 */
fw_packet fw_tx_packet;

/**
 * The state of the weapon
 * state can be ENABLED of DISABLED
 * If state is DISABLED the weapon will not respond at trigger
 */
unsigned char weapon_state = ENABLED;

/** 
 * Number of times timer1 should count down
 */
unsigned char timer1_countdown;

/**
 * The current state of the player
 */
unsigned char player_state = PLAYER_ALIVE;

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
 * Interrupt service routine for Timer1 Output Compare Match A interrupt.
 * Used for player timeout
 */
ISR(TIMER1_COMPA_vect) 
{
    timer1_countdown--; // decrease counter

    if (timer1_countdown == 0) {
        TCCR1B &= ~((1 << CS10) | (1 << CS12)); // disable timer
        
        // send message to weapon to enable weapon
		fw_tx_packet.source = 0x00;
		fw_tx_packet.destination = 0x00;
		fw_tx_packet.length = 0x01;
		fw_tx_packet.data[0] = FW_MID_ENABLE_WEAPON;
        fw_send_packet(fw_tx_packet);
        
        player_state = PLAYER_ALIVE; // new state        
    }
}

/**
 * Main routine
 * Initializes system
 * Goes into endless loop and assembles packets.
 * Calls fw_process_packet() when a packet is assembled.
 */
int main(void)
{
    DDRB = 0x01;
    PORTB = 0x00;
    
    // init system
    fw_init(fw_process_packet);
    init_statusled();
    init_timer1();
    wdt_enable(WDTO_1S); // enable watchdog timer, 1 second

    sei(); // enable interrupts
    
    ENABLE_STATUS; // enable status LED

    while (1) {     
    
        wdt_reset(); // reset watchdog timer

		fw_assemble_packet();

    }

    return -1;
}

/**
 * Process an assembled FragWire packet
 * This function is called by the packet assembler 
 * when it has completed assembling a packet
 */
void fw_process_packet(fw_packet packet) 
{    
    switch (packet.data[0]) { // packet.data[0] contains the message identifier
        case FW_MID_NOP:
            // no operation, do nothing
            break;
        case FW_MID_PLAYER_HIT:
            player_hit();
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
	DDRD |= (1 << PD4); // PD4 as output
	PORTD &= ~(1 << PD4); // set PD4 to low, so status led is off in the beginning
}

void init_timer1(void)
{
    TIMSK1 |= (1 << OCIE1A); // enable output compare 1A match interrupt
    TCCR1B |= /*(1 << CS10) | (1 << CS12) |*/ (1 << WGM12); // clear timer on compare match mode
    OCR1A = TIMER1_1SEC; // timer1 output compare 1 second
}

/**
 * Processes a player hit message from a sensor module
 * Checks to see if player is alive, if so it sends a message to the weapon
 * to disable it. Then it starts a timer and after the timer 
 * expires it (the timer int routine) enables the weapon again
 */
void player_hit(void) 
{
    // ignore if already been hit
    if (player_state == PLAYER_ALIVE) {

        timer1_countdown = PLAYER_TIMEOUT_SEC; // number of seconds timeout        

		// send message to weapon to disable weapon
		fw_tx_packet.source = 0x00;
		fw_tx_packet.destination = 0x00;
		fw_tx_packet.length = 0x01;
		fw_tx_packet.data[0] = FW_MID_DISABLE_WEAPON;
		fw_send_packet(fw_tx_packet);
		
        while (fw_get_tx_state() != FW_TX_IDLE); // wait till transfer is complete before starting timer

        player_state = PLAYER_TIMEOUT; // new player state

        TCNT1 = 0; // clear timer value
        TCCR1B |= (1 << CS10) | (1 << CS12); // enable timer with 1024 pre-scaler
    }
}
