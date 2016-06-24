/**
 * Library for OpenLaserFrag 0.9 MilesTag protocol, www.openlaserfrag.org
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
 *                  - Implements Infrared transmit protocol
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

#ifndef MILESTAG_H_
#define MILESTAG_H_

// MilesTag team id's (bits 7:6)
#define MT_TID_RED				0x00 << 6
#define MT_TID_BLUE				0x01 << 6
#define MT_TID_YELLOW			0x02 << 6
#define MT_TID_GREEN			0x03 << 6

// MilesTag damage values (bits 6:2). Note that bits 1:0 are not used at all.
#define MT_DAMAGE_1				0x00 << 2
#define MT_DAMAGE_2				0x01 << 2
#define MT_DAMAGE_4				0x02 << 2
#define MT_DAMAGE_5				0x03 << 2
#define MT_DAMAGE_7				0x04 << 2
#define MT_DAMAGE_10			0x05 << 2
#define MT_DAMAGE_15			0x06 << 2
#define MT_DAMAGE_17			0x07 << 2
#define MT_DAMAGE_20			0x08 << 2
#define MT_DAMAGE_25			0x09 << 2
#define MT_DAMAGE_30			0x0A << 2
#define MT_DAMAGE_35			0x0B << 2
#define MT_DAMAGE_40			0x0C << 2
#define MT_DAMAGE_50			0x0D << 2
#define MT_DAMAGE_75			0x0E << 2
#define MT_DAMAGE_100			0x0F << 2

// MilesTag message id's (MSB has to be set to 1, a shot packet has MSB set to 0)
#define MT_MID_ADD_HEALTH		0x80
#define MT_MID_ADD_ROUNDS		0x81
#define MT_MID_COMMAND			0x83
#define MT_MID_SYSTEM_DATA		0x87 // not supported
#define MT_MID_CLIPS_PICKUP		0x8A
#define MT_MID_HEALTH_PICKUP	0x8B
#define MT_MID_FLAG_PICKUP		0x8C

// MilesTag command id's
#define MT_CID_ADMIN_KILL			0x00
#define MT_CID_PAUSE_UNPAUSE		0x01
#define MT_CID_START_GAME			0x02
#define MT_CID_RESTORE_DEFAULTS		0x03
#define MT_CID_RESPAWN				0x04
#define MT_CID_NEW_GAME_IMMEDIATE	0x05
#define MT_CID_FULL_AMMO			0x06
#define MT_CID_END_GAME				0x07
#define MT_CID_RESET_CLOCK			0x08
#define MT_CID_INITIALIZE_PLAYER	0x0A
#define MT_CID_EXPLODE_PLAYER		0x0B
#define MT_CID_NEW_GAME_READY		0x0C
#define MT_CID_FULL_HEALTH			0x0D
#define MT_CID_FULL_ARMOR			0x0F
#define MT_CID_CLEAR_SENSORS		0x14
#define MT_CID_TEST_SENSORS			0x15
#define MT_CID_STUN_PLAYER			0x16
#define MT_CID_DISARM_PLAYER		0x17

// MilesTag packet assembling states
#define MT_RX_IDLE				0
#define MT_RX_SHOT				1
#define MT_RX_DATA				2
#define MT_RX_PROCESSING		4

// MilesTag transmit states
#define MT_TX_IDLE				0
#define MT_TX_TRANSMITTING		1

// MilesTag packet data buffer size
#define MT_PACKET_BUFSIZE		3

typedef struct
{
	/** 
	 * The player ID 
	 */
	unsigned char player_id;
	/** 
	 * The team ID 
	 */
	unsigned char team_id;
	/** 
	 * The damage 
	 */
	unsigned char damage;
} mt_shot_packet;

typedef struct  
{
	/** 
	 * The message ID
	 */
	unsigned char message_id;

	/**
	 * The length field
	 */
	unsigned char length;

	/**
	 * The data field 
	 */
	unsigned char data[MT_PACKET_BUFSIZE];
} mt_data_packet;

void mt_rx_init(void (*mt_shot_packet_callback)(mt_shot_packet mt_rx_shot_packet), void (*mt_data_packet_callback)(mt_data_packet mt_rx_data_packet));
void mt_tx_init(void);
void mt_ISR_INT0(void);
void mt_ISR_TIMER1_OVF(void);
void mt_ISR_TIMER0_COMPA(void);
unsigned char mt_get_tx_state(void);
void mt_assemble_packet(void);
void mt_send_shot_packet(mt_shot_packet mt_tx_shot_packet);
void mt_send_data_packet(mt_data_packet mt_tx_data_packet);

#endif /* MILESTAG_H_ */