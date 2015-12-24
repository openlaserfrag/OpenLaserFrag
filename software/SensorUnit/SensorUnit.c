/**
 * Firmware for OpenLaserFrag 0.9 sensor module, www.openlaserfrag.org
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
 *					- Use ATmega168 instead of ATtiny2313
 *					- Move handling of FragWire to a separate library
 *					- Move handling of MilesTag to a separate library
 *
 * revision 0.9: 	
 *                  - Initial release
 *                  - Needs to be tested extensively with target hardware
 *                  - Implements Infrared receive protocol
 *                  - Sends message through FragWire to the PU when player is hit
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

// Status LED
#define ENABLE_STATUS			PORTB |= (1 << PB1)
#define DISABLE_STATUS			PORTB &= ~(1 << PB1)
#define TOGGLE_STATUS			PORTB ^= (1 << PB1)

// function prototypes
void fw_process_packet(fw_packet fw_rx_packet);
void mt_process_shot_packet(mt_shot_packet mt_rx_shot_packet);
void mt_process_data_packet(mt_data_packet mt_rx_data_packet);
void init_statusled(void);

/**
 * FragWire packet (used to send)
 */
fw_packet fw_tx_packet;

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
 * Timer1 overflow interrupt routine
 * Should not happen
 */
ISR(TIMER1_OVF_vect) 
{
	mt_ISR_TIMER1_OVF();
}

/**
 * External interrupt 0 INT0 routine
 * Takes care for receiving IR packets
 */
ISR(INT0_vect) 
{
	mt_ISR_INT0();
}

/**
 * Main routine
 * Initializes system
 * Goes into infinite loop to assemble IR packets and calls 
 * mt_process_shot_packet() when a packet is complete and the CRC8 is correct
 */
int main(void) {	        
	
    // init system
    fw_init(fw_process_packet);
	mt_rx_init(mt_process_shot_packet, mt_process_data_packet);
	init_statusled();
	
    wdt_enable(WDTO_1S); // enable watchdog timer, 1 second

    sei(); // enable interrupts
    
    ENABLE_STATUS; // enable status LED
   
    while(1) {

        wdt_reset();    // clear watchdog timer
		
		mt_assemble_packet();  

    }		    	

	return -1;
}

/**
 * Process an assembled FragWire packet
 * This function is called by the packet assembler 
 * when it has completed assembling a packet
 */
void fw_process_packet(fw_packet fw_rx_packet) 
{    
    switch (fw_rx_packet.data[0]) { // packet.data[0] contains the message identifier
        case FW_MID_NOP:
            // no operation, do nothing
            break;
        default:
            break;
    } 
}

/**
 * Processes an assembled MilesTag shot packet
 * This function is called by the packet assembler
 * when it has completed assembling a packet
 */
void mt_process_shot_packet(mt_shot_packet mt_rx_shot_packet)
{
	//TOGGLE_STATUS;
	
    // send message that the player was hit
    fw_tx_packet.source = 0x00;
    fw_tx_packet.destination = 0x00;
    fw_tx_packet.length = 0x04;
    fw_tx_packet.data[0] = FW_MID_PLAYER_HIT;
    fw_tx_packet.data[1] = mt_rx_shot_packet.player_id;
    fw_tx_packet.data[2] = mt_rx_shot_packet.team_id;
    fw_tx_packet.data[3] = mt_rx_shot_packet.damage;
    fw_send_packet(fw_tx_packet);
}

/**
 * Processes an assembled MilesTag data packet
 * This function is called by the packet assembler
 * when it has completed assembling a packet
 */
void mt_process_data_packet(mt_data_packet mt_rx_data_packet)
{
	//TOGGLE_STATUS;
}

/**
 * Initialize pin used for status LED
 */
void init_statusled(void)
{
	DDRB |= (1 << PB1); // PB1 as output
	PORTB &= ~(1 << PB1); // set PB1 to low, so status led is off in the beginning
}
