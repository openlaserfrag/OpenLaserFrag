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

#define F_CPU 16000000UL // CPU speed, is used by delay.h and setbaud.h
#define BAUD 9600 // Baud rate, is used by setbaud.h

#include <stddef.h>
#include <inttypes.h>
#include <util/delay.h>
#include <util/crc16.h>
#include <util/setbaud.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "FragWire.h"

// a, FragWire preamble
#define PREAMBLE				0x61

// Buffer sizes
#define UART_RXBUF_SIZE			128 // UART Receive buffer size
#define FW_TXBUF_SIZE			128 // FragWire transmit buffer size
#define FW_TXTEMPBUF_SIZE		128 // FragWire transmit temp buffer size

// UART BAUD rates
#define UART_BAUD_16MHZ_9600    103 // 0.2% error
#define UART_BAUD_16MHZ_19200   51  // 0.2% error
#define UART_BAUD_16MHZ_57600   16  // 2.1% error
#define UART_BAUD_16MHZ_115200  8   // -3.5% error
#define UART_BAUD_16MHZ_250000  3   // 0% error

// internal function prototypes
void (*fw_process_packet_callback)(fw_packet fw_rx_packet) = (void*) NULL;
void uart_init(unsigned int baud_divisor);
unsigned char uart_rxbuf_full(void);
unsigned char uart_rxbuf_empty(void);
unsigned char uart_rxbuf_pullbyte(void);
void uart_rxbuf_pushbyte(unsigned char c);
unsigned char CRC8_packet(fw_packet packet);

/**
 * FragWire transmit state
 */
volatile unsigned char fw_tx_state = FW_TX_IDLE;

/**
 * FragWire transmit buffer
 */
volatile unsigned char fw_tx_buf[FW_TXBUF_SIZE];

/**
 * FragWire transmit buffer size
 */
volatile unsigned char fw_tx_bufsize;

/**
 * Number of bytes sent of a FragWire packet
 */
volatile unsigned char fw_tx_count;

/**
 * Temporary buffer to assemble FragWire packet
 */
unsigned char fw_tx_tempbuf[FW_TXTEMPBUF_SIZE];

/**
 * State of packet assembling.
 * State can be idle, processing or any of the packet segments
 */
unsigned char fw_rx_state = FW_RX_IDLE;

/**
 * Used to temp store a byte pulled from UART receive buffer
 */
unsigned char fw_rx_byte;  

/**
 * FragWire packet that is currently being assembled
 */
fw_packet fw_rx_packet;

/**
 * Number of bytes already received from the data field of a packet
 * We need to record this to know where the data field ends
 */
unsigned char fw_rx_packet_datacnt = 0;

/**
 * Checksum of the packet that is currently being assembled
 */
unsigned char fw_rx_packet_checksum;    

/**
 * Used by the UART receive interrupt service routine
 * to temporary store the incoming byte
 */
unsigned char uart_rx_byte;

/**
 * UART receive buffer
 */
unsigned char uart_rx_buf[UART_RXBUF_SIZE];

/**
 * UART receive buffer size
 */
unsigned char uart_rx_bufsize = 0;

/**
 * UART receive buffer read pointer 
 * indice number of last read char
 */
unsigned char uart_rx_buf_rpt = 0;

/**
 * UART receive buffer write pointer
 * indice number of last written char
 */ 
unsigned char uart_rx_buf_wpt = 0;

/**
 * Initialize FragWire
 * @param baud_divisor the baud rate divisor. The baud rate is clk / baud rate divisor
 */
void fw_init(void (*fw_packet_callback)(fw_packet fw_rx_packet)) 
{
	fw_process_packet_callback = fw_packet_callback; // set the callback
	uart_init(UART_BAUD_16MHZ_9600);
}

/**
 * Initialize UART
 * @param baud_divisor the baud rate divisor. The baud rate is clk / baud rate divisor
 */
void uart_init(unsigned int baud_divisor) 
{
	// initialize the USART using the values from setbaud.h
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	
	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif
	
	// enable the USART receiver and transmitter, enable interrupt on receive
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
}

/**
 * Interrupt service routine for UART receive interrupt
 * Checks if UART receive buffer is not full and
 * pushes the received byte to the UART receive buffer
 */ 
void fw_ISR_USART_RX(void)
{
    uart_rx_byte = UDR0;
    
    if ( !uart_rxbuf_full() ) {
        uart_rxbuf_pushbyte(uart_rx_byte);		
    }
}

/**
 * USART Receive buffer empty interrupt service routine
 * Handles the transmission of a FragWire packet
 */
void fw_ISR_USART_UDRE(void)
{
    if (fw_tx_bufsize > 0) {    // something to send
        UDR0 = fw_tx_buf[fw_tx_count++];
        fw_tx_bufsize--;
    } else {
        fw_tx_state = FW_TX_IDLE; // done with sending, so we are idle now
        UCSR0B &= ~(1 << UDRIE0); // disable USART receive buffer empty interrupt       
    }   
}

/**
 * Returns the FragWire transmit state
 */
unsigned char fw_get_tx_state(void) 
{
	return fw_tx_state;
}

/**
 * Assembles the FragWire packet
 */
void fw_assemble_packet(void) 
{	
	// packet assembling
	if (! uart_rxbuf_empty() ) {
		if (fw_rx_state != FW_RX_PROCESSING) {
			
			// needs to be checked if we have to disable int for concurrency problems
			//cli(); // disable interrupts
			fw_rx_byte = uart_rxbuf_pullbyte();
			//sei(); // enable interrupts

			switch(fw_rx_state) {
				case FW_RX_IDLE:
					if (fw_rx_byte == PREAMBLE) {					// received preamble
						fw_rx_state = FW_RX_PREAMBLE;
						fw_rx_packet_datacnt = 0; // reset to zero 
					}
					break;
				case FW_RX_PREAMBLE:								// received destination
					fw_rx_state = FW_RX_DESTINATION;
					fw_rx_packet.destination = fw_rx_byte;
					break;
				case FW_RX_DESTINATION:								// received source
					fw_rx_state = FW_RX_SOURCE;
					fw_rx_packet.source = fw_rx_byte;
					break;
				case FW_RX_SOURCE:									// received length
					fw_rx_state = FW_RX_LENGTH;
					fw_rx_packet.length = fw_rx_byte;
					break;
				case FW_RX_LENGTH:									// received data
					fw_rx_packet.data[fw_rx_packet_datacnt] = fw_rx_byte; // save byte in buffer
					fw_rx_packet_datacnt++; // increase number of received data bytes

					if (fw_rx_packet_datacnt == fw_rx_packet.length) {  // check if last data byte
						fw_rx_state = FW_RX_DATA; // next byte is CRC8
					}
					break;
				case FW_RX_DATA:									// received checksum
					fw_rx_state = FW_RX_PROCESSING; // no other packets can be assembled while in this state
					fw_rx_packet.crc8_checksum = fw_rx_byte;
					fw_rx_packet_checksum = CRC8_packet(fw_rx_packet);

					if (fw_rx_packet.crc8_checksum == fw_rx_packet_checksum) { // check if CRC is correct and process if correct
						if (fw_process_packet_callback != NULL) { // check if the callback is set
							fw_process_packet_callback(fw_rx_packet);
						}
					} else {
						// pretend nothing happened, start over with packet assembling
						//while ( !( UCSRA & (1 << UDRE)) ){} // while transmit buffer not empty, wait
						//UDR = fw_rx_packet_checksum; // send received byte to the serial port for debug
					}
				
					fw_rx_packet_datacnt = 0; // reset to zero for next packet
					fw_rx_state = FW_RX_IDLE;
					break;
				default: // should not happen
					break; 
			}
		}
	}
}

/**
 * Initiates a transfer of a packet on FragWire
 * Assembles a packet in fw_tx_buf
 * Enables USART Data Register Empty interrupt
 * DRE interrupt routine takes care for transfer 
 * and sets fx_tx_state to FW_TX_IDLE when done
 * @param packet the FragWire packet 
 * @param source source FragWire bus address
 * @param length length of data segment
 * @param data array containing data segment
 */
void fw_send_packet(fw_packet fw_tx_packet) {
    
    unsigned char i;

    while (fw_tx_state != FW_TX_IDLE); // wait if previous transfer has not yet completed

    fw_tx_bufsize = 0; // used by interrupt routine to check number of bytes left to transfer
    fw_tx_count = 0;   // used by interrupt routine for array index of fw_tx_buf (counting up)

    fw_tx_buf[0] = PREAMBLE;
    fw_tx_buf[1] = fw_tx_packet.destination;
    fw_tx_buf[2] = fw_tx_packet.source;
    fw_tx_buf[3] = fw_tx_packet.length;

    fw_tx_bufsize = 4;

    // add data bytes to transmit buffer
    for (i=0; i< fw_tx_packet.length; i++) {
        fw_tx_buf[fw_tx_bufsize++] = fw_tx_packet.data[i];
    }
    
    // calculate and add CRC8 to transmit buffer
    fw_tx_buf[fw_tx_bufsize++] = CRC8_packet(fw_tx_packet);

    fw_tx_state = FW_TX_BUSY; // we start with sending so we are busy
    UCSR0B |= (1 << UDRIE0); // enable USART Data Register Empty interrupt
}

/**
 * Checks if UART receive buffer is full
 * @return 1 if buffer is full, 0 if buffer is not full
 */
unsigned char uart_rxbuf_full(void) 
{
    return uart_rx_buf_rpt == uart_rx_buf_wpt &&
           uart_rx_bufsize == UART_RXBUF_SIZE;
}

/**
 * Checks if UART receive buffer is empty
 * @return 1 if buffer is empty, 0 if buffer is not empty
 */
unsigned char uart_rxbuf_empty(void)
{
    return uart_rx_buf_rpt == uart_rx_buf_wpt && 
           uart_rx_bufsize == 0;
}

/**
 * Adds a byte to the UART receive buffer
 * This function may only be called if uart_rxbuf_full() == 0
 */
void uart_rxbuf_pushbyte(unsigned char c)
{
    // increase write pointer, check if at end of array
    if (++uart_rx_buf_wpt >= UART_RXBUF_SIZE) uart_rx_buf_wpt = 0;
 
    uart_rx_buf[uart_rx_buf_wpt] = c;    
    uart_rx_bufsize++;
}

/**
 * Pulls a byte from the UART receive buffer
 * This function may only be called if uart_rxbuf_empty() == 0
 * @return byte pulled from the buffer
 */
unsigned char uart_rxbuf_pullbyte(void) 
{  

    // increase read pointer, check if at end of buffer, if so wrap to the start of the buffer
    if (++uart_rx_buf_rpt >= UART_RXBUF_SIZE) uart_rx_buf_rpt = 0;
     
    uart_rx_bufsize--;  
  
    return uart_rx_buf[uart_rx_buf_rpt];  
}

/**
 * Calculates CRC from a FragWire packet
 * Runs _crc8_ccitt_update() on each byte in the packet to calculate the CRC
 */
unsigned char CRC8_packet(fw_packet packet)
{
    unsigned char i, crc;

    crc = 0xFF;
	crc = _crc8_ccitt_update(crc, packet.destination);
	crc = _crc8_ccitt_update(crc, packet.source);
	crc = _crc8_ccitt_update(crc, packet.length);

    for (i=0; i<packet.length; i++)
    {
        crc = _crc8_ccitt_update(crc, packet.data[i]);
    }
   
    return crc;
}
