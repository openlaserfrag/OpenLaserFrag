/**
 * Library for OpenLaserFrag 0.9 FragWire protocol, www.openlaserfrag.org
 * Library revision 0.1
 *
 * Copyright (c) 2015
 * - Bianco Zandbergen <zandbergenb AT gmail.com>.
 * - Gerard de Leeuw <gdeleeuw AT leeuwit.nl>.
 *
 * Build with AVR-GCC, target ATmega168, no optimization
 *
 * CHANGELOG
 * 
 * revision 0.1: 	
 *                  - Initial release
 *                  - Needs to be tested extensively with target hardware
 *                  - Implements UART receive/transmit communication and packet handling
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

#ifndef FRAGWIRE_H_
#define FRAGWIRE_H_

// FragWire message id's
#define FW_MID_NOP				0x00
#define FW_MID_ENABLE_WEAPON	0x20
#define FW_MID_DISABLE_WEAPON	0x21
#define FW_MID_PLAYER_HIT		0x10

// FragWire packet assembling states
#define FW_RX_IDLE          0
#define FW_RX_PREAMBLE      1
#define FW_RX_DESTINATION   2
#define FW_RX_SOURCE        3
#define FW_RX_LENGTH        4
#define FW_RX_DATA          5
#define FW_RX_CRC           6
#define FW_RX_PROCESSING    7

// FragWire transfer states
#define FW_TX_IDLE				0
#define FW_TX_BUSY				1

// FragWire packet data buffer size (this is not UART buffer!)
#define FW_PACKET_BUFSIZE		128

typedef struct  
{
	/** 
	 * The source bus address 
	 */
	unsigned char source;

	/** 
	 * The destination bus address 
	 */
	unsigned char destination;

	/**
	 * The length field 
	 */
	unsigned char length;

	/**
	 * The data field 
	 */
	unsigned char data[FW_PACKET_BUFSIZE];

	/**
	 * The CRC8 checksum 
	 */
	unsigned char crc8_checksum;
} fw_packet;

// function prototypes
void fw_init(void (*fw_packet_callback)(fw_packet fw_rx_packet));
void fw_ISR_USART_RX(void);
void fw_ISR_USART_UDRE(void);
unsigned char fw_get_tx_state(void);
void fw_assemble_packet(void);
void fw_send_packet(fw_packet fw_tx_packet);

#endif /* FRAGWIRE_H_ */