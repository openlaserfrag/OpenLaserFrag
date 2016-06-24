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

// CPU speed, is used by delay.h
#define F_CPU 16000000UL

#include <stddef.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "MilesTag.h"

// Masks to use when transmitting a shot packet
#define SHOT_PLAYER_ID_MASK		0x7F
#define SHOT_TEAM_ID_MASK		0xC0
#define SHOT_DAMAGE_MASK		0x3F

// Buffer sizes
#define IR_TXBUF_SIZE			3 // Infrared transmit buffer size
#define IR_RXBUF_SIZE			24 // IR Receive buffer size, 24 = max 8 packets

// Infrared data lengths
// Note that these are not UART BAUD divisors!
#define IR_2400USEC_HEADER		(F_CPU * 0.0024) // CPU speed * duration in seconds
#define IR_1200USEC_BIT_1		(F_CPU * 0.0012) // CPU speed * duration in seconds
#define IR_600USEC_BIT_0		(F_CPU * 0.0006) // CPU speed * duration in seconds
#define IR_600USEC_SPACE		(F_CPU * 0.0006) // CPU speed * duration in seconds
#define IR_150USEC_MARGIN		(F_CPU * 0.00015) // CPU speed * duration in seconds

// Infrared receive data lengths for Timer1
// Used by the IR receive code to check the lower and higher value of timer a received bit should be
#define TIMER1_2400USEC_HEADER	IR_2400USEC_HEADER
#define TIMER1_1200USEC_BIT_1	IR_1200USEC_BIT_1
#define TIMER1_600USEC_BIT_0	IR_600USEC_BIT_0
#define TIMER1_600USEC_SPACE	IR_600USEC_SPACE
#define TIMER1_150USEC_MARGIN	IR_150USEC_MARGIN

// Infrared transmit data lengths for Timer0
// Divided by 256 because Timer0 uses a CLK/256 prescaler
#define TIMER0_2400USEC_HEADER	IR_2400USEC_HEADER / 256 // header pulse
#define TIMER0_1200USEC_BIT_1	IR_1200USEC_BIT_1 / 256 // bit value 1 pulse
#define TIMER0_600USEC_BIT_0	IR_600USEC_BIT_0 / 256 // bit value 0 pulse
#define TIMER0_600USEC_SPACE	IR_600USEC_SPACE / 256 // space pulse

// Infrared transmit PWM modulation
#define PWM_56KHZ				(F_CPU / 56000) // 56 KHz @ 16MHz (286 cycles, 17.88 microseconds period time)

// Infrared receive states
#define IR_RX_IDLE				0
#define IR_RX_HEADER			1
#define IR_RX_DATA				2
#define IR_RX_SPACE				4
#define IR_RX_PROCESSING		8

// Infrared transmit states
#define IR_TX_IDLE				0
#define IR_TX_HEADER			1
#define IR_TX_DATA				2
#define IR_TX_SPACE				4

// IR carrier enable/disable macro's, makes it easier to read and replace
#define ENABLE_IR ir_set_pwm(1)
#define DISABLE_IR ir_set_pwm(0)

// function prototypes
void (*mt_process_shot_packet_callback)(mt_shot_packet mt_rx_shot_packet) = NULL;
void (*mt_process_data_packet_callback)(mt_data_packet mt_rx_data_packet) = NULL;
void ir_init_extint(void);
void ir_init_timer1(void);
void ir_init_timer0(void);
void ir_init_pwm(void);
void ir_set_pwm(unsigned char state);
void ir_tx_clear(void);
void ir_tx_start(void);
void ir_tx_stop(void);
void ir_rx_clear(void);
void ir_rx_reset(void);
unsigned char ir_rxbuf_full(void);
unsigned char ir_rxbuf_empty(void);
unsigned char ir_rxbuf_pullbyte(void);
void ir_rxbuf_pushbyte(unsigned char c);

/**
 * Infrared receive state
 */
unsigned char ir_rx_state = IR_RX_IDLE;

/**
 * Infrared receive buffer
 */
unsigned char ir_rx_buf[IR_RXBUF_SIZE];

/**
* Infrared receive buffer size
 */
unsigned char ir_rx_bufsize = 0;

/**
 * Used to temp store a byte that is being received
 */
unsigned char ir_rx_byte;

/**
 * Used to store bit transmit state of ir_rx_byte
 * starts with 0b10000000 and shifts one to the right after each received bit
 */
volatile unsigned char ir_rx_byte_shift;

/**
 * Number of bits remaining of an Infrared byte
 */
volatile unsigned char ir_rx_bits_remaining;

/**
 * Infrared receive buffer read pointer 
 * indice number of last read char
 */
unsigned char ir_rx_buf_rpt = 0;

/**
 * Infrared receive buffer write pointer
 * indice number of last written char
 */ 
unsigned char ir_rx_buf_wpt = 0;

/**
 * Timer value
 * Used by infrared receive protocol
 */
unsigned int ir_rx_timed;

/**
 * Infrared transmit state
 */
volatile unsigned char ir_tx_state = IR_TX_IDLE;

/**
 * Infrared transmit buffer
 */
volatile unsigned char ir_tx_buf[IR_TXBUF_SIZE];

/**
 * Infrared transmit buffer size
 */
volatile unsigned char ir_tx_bufsize;

/**
 * Number of bytes sent of an infrared packet
 */
volatile unsigned char ir_tx_count;

/**
 * Used to temp store a byte that is being transmitted
 */
unsigned char ir_tx_byte;

/**
 * Used to store bit transmit state of ir_tx_byte
 * starts with 0b10000000 and shifts one to the right after each transmitted bit
 */
volatile unsigned char ir_tx_byte_shift;

/**
 * Number of bits remaining of an Infrared byte
 */
volatile unsigned char ir_tx_bits_remaining;

/**
 * MilesTag transmit state
 */
volatile unsigned char mt_tx_state = MT_TX_IDLE;

/**
 * MilesTag packet assembling state
 */
volatile unsigned char mt_rx_state = MT_RX_IDLE;

/**
 * Used to temp store a byte pulled from Infrared receive buffer
 */
unsigned char mt_rx_byte;  

/**
 * MilesTag shot packet that is currently being assembled
 */
mt_shot_packet mt_rx_shot_packet;

/**
 * MilesTag data packet that is currently being assembled
 */
mt_data_packet mt_rx_data_packet;

/**
 * Number of bytes already received from the data field of a packet
 * We need to record this to know where the data field ends
 */
unsigned char mt_rx_data_packet_datacnt = 0;

void mt_rx_init(void (*mt_shot_packet_callback)(mt_shot_packet mt_rx_shot_packet), void (*mt_data_packet_callback)(mt_data_packet mt_rx_data_packet))
{
	mt_process_shot_packet_callback = mt_shot_packet_callback;
	mt_process_data_packet_callback = mt_data_packet_callback;
	ir_init_timer1();
	ir_init_extint();
}

void mt_tx_init(void)
{
	ir_init_timer0();
	ir_init_pwm();
}

/**
 * External interrupt 0 INT0 routine
 * Takes care for receiving IR packets
 * Puts the received data in ir_rxbuf[]
 * Note that the signal of the IR receiver is inverted!
 */
void mt_ISR_INT0(void)
{
    ir_rx_timed = TCNT1; // store current timer value
    TCNT1 = 0; // clear timer

    switch (ir_rx_state) 
    {
		case IR_RX_IDLE: // should have received a header
			if (PIND & (1 << PD2)) { // rising edge, end of header
				if (ir_rx_timed > (TIMER1_2400USEC_HEADER - TIMER1_150USEC_MARGIN) && ir_rx_timed < (TIMER1_2400USEC_HEADER + TIMER1_150USEC_MARGIN)) {
					ir_rx_clear(); // make sure all variables are cleared
					ir_rx_state = IR_RX_HEADER; // received a header, space follows
				} else { // timing is out of bounds
					ir_rx_reset();
				}
			} else { // falling edge, start of header
				TCCR1B |= (1 << CS10); // enable timer1, CLK/1
			}
			break;
		case IR_RX_HEADER: // should have received a space
		case IR_RX_DATA: // should have received a space
			if (PIND & (1 << PD2)) { // rising edge, something went wrong
				ir_rx_reset();
			} else { // falling edge, end of space
				if(ir_rx_timed > (TIMER1_600USEC_SPACE - TIMER1_150USEC_MARGIN) && ir_rx_timed < (TIMER1_600USEC_SPACE + TIMER1_150USEC_MARGIN)) { 			
					ir_rx_state = IR_RX_SPACE; // received a space, next bit follows
				} else { // timing is out of bounds
					ir_rx_reset();
				}
			}
			break;
		case IR_RX_SPACE: // should have received a bit
			if (PIND & (1 << PD2)) {  // rising edge, end of data
				
				if (ir_rx_timed > (TIMER1_1200USEC_BIT_1 - TIMER1_150USEC_MARGIN) && ir_rx_timed < (TIMER1_1200USEC_BIT_1 + TIMER1_150USEC_MARGIN)) {
					ir_rx_byte ^= ir_rx_byte_shift; // received a 1
				} else if (ir_rx_timed > (TIMER1_600USEC_BIT_0 - TIMER1_150USEC_MARGIN) && ir_rx_timed < (TIMER1_600USEC_BIT_0 + TIMER1_150USEC_MARGIN)) {				
					// received a 0, do nothing
				} else { // should not happen
					ir_rx_reset();
					break;
				}
				
				ir_rx_byte_shift >>= 1; // shift one place to the right for the next bit (MSB first)
				
				if(ir_rx_byte_shift == 0) { // received a complete byte, push it to the buffer					
					if (!ir_rxbuf_full()) {
						ir_rxbuf_pushbyte(ir_rx_byte); // push byte to Infrared receive buffer
					}
					
					ir_rx_byte_shift = 0x80; // reset the shift
				}

				if(ir_rx_bits_remaining == 0) { // first bit received after header
					if(ir_rx_byte & 0x80) { // check the MSB of the (partially) received byte is set, meaning this is a message packet
						ir_rx_bits_remaining = 24; // message packet
					} else {
						ir_rx_bits_remaining = 14; // shot packet
					}
				}

				ir_rx_bits_remaining--; // decrease the amount of bits remaining
				if(ir_rx_bits_remaining == 0) { 
					ir_rx_state = IR_RX_IDLE; // received the last bit, processing follows
				} else {
					ir_rx_state = IR_RX_DATA; // received a bit, space follows
				}
			
			} else { // falling edge, something went wrong
				ir_rx_reset();
			}
			break;
		default: // should not happen
			ir_rx_reset();
			break;     
    }
}

/**
 * Timer1 overflow interrupt routine
 * Should not happen
 */
void mt_ISR_TIMER1_OVF(void)
{
	ir_rx_reset();
}

/**
 * Interrupt service routine for Timer0 Output Compare Match A interrupt.
 * This routine is used by the infrared send protocol.
 * ir_send_packet() initiates transfer and starts the timer.
 * Each time the timer interrupts this routine will check the transfer state and decide what to do.
 * If the transmission has completed it finally disables the timer and sets ir_tx_state to IR_IDLE
 */
void mt_ISR_TIMER0_COMPA(void)
{
    switch (ir_tx_state) {
        case IR_TX_HEADER:							
        case IR_TX_DATA: // current state is sent header or data					
			if(ir_tx_byte_shift == 0 && ir_tx_count < ir_tx_bufsize) { // no more bits available but more bytes are available
				ir_tx_byte = ir_tx_buf[ir_tx_count];		// get the byte to be transferred
				ir_tx_count++;								// increment the amount of bytes send
				ir_tx_byte_shift = 0x80;					// reset the shift
			}
		
			if(ir_tx_bits_remaining == 0) { // first bit after transmitting header
				if(ir_tx_byte & 0x80) { // check the MSB of the byte to transmit is set, meaning this is a message packet
					ir_tx_bits_remaining = 24; // message packet
				} else {
					ir_tx_bits_remaining = 14; // shot packet
				}
			}
			
			if(ir_tx_byte_shift > 0 && ir_tx_bits_remaining > 0) { // more bits are available to transmit
				DISABLE_IR;									// disable carrier
				TCNT0 = 0;									// reset timer
				OCR0A = TIMER0_600USEC_SPACE;				// next state is 600 microseconds space (no carrier)
				ir_tx_state = IR_TX_SPACE;
			} else {
				ir_tx_stop();							// stop transmitting
			}
			break;
        case IR_TX_SPACE: // current state is sent space
			ENABLE_IR;								// disable carrier
			TCNT0 = 0;								// reset timer
			if (ir_tx_byte & ir_tx_byte_shift) {	// if bit is 1, 1200 microseconds carrier
				OCR0A = TIMER0_1200USEC_BIT_1;
			} else {								// if bit is 0, 600 microseconds carrier
				OCR0A = TIMER0_600USEC_BIT_0;
			}
			ir_tx_byte_shift >>= 1;					// shift one place to the right for the next bit (MSB first)
			ir_tx_bits_remaining--;
			ir_tx_state = IR_TX_DATA;
			break;						
		default:
			ir_tx_stop();							// should not happen
			break;
    }
}

unsigned char mt_get_tx_state(void)
{
	return mt_tx_state;
}

void mt_assemble_packet(void)
{
	// check if there is data available
	if (!ir_rxbuf_empty()) {
		
		if (mt_rx_state != MT_RX_PROCESSING) {

			// needs to be checked if we have to disable int for concurrency problems
			//cli(); // disable interrupts
			mt_rx_byte = ir_rxbuf_pullbyte();
			//sei(); // enable interrupts

			switch(mt_rx_state) {
				case MT_RX_IDLE:
					if ((mt_rx_byte & 0x80) == 0) { // received a shot packet
						mt_rx_shot_packet.player_id = mt_rx_byte & SHOT_PLAYER_ID_MASK; // the player id is part of the first byte of a shot packet
						mt_rx_state = MT_RX_SHOT;
					} else { // received a data packet
						mt_rx_data_packet.message_id = mt_rx_byte;					
						mt_rx_data_packet.length = 3;
						mt_rx_data_packet_datacnt = 0; // reset to zero
						mt_rx_state = MT_RX_DATA;					
					}
					break;
				case MT_RX_SHOT:									// received the second byte of a shot packet
					mt_rx_shot_packet.team_id = mt_rx_byte & SHOT_TEAM_ID_MASK;
					mt_rx_shot_packet.damage = mt_rx_byte & SHOT_DAMAGE_MASK;

					mt_rx_state = MT_RX_PROCESSING;
					if (mt_process_shot_packet_callback != NULL) {			// check if the callback is set
						mt_process_shot_packet_callback(mt_rx_shot_packet);
					}
					mt_rx_state = MT_RX_IDLE;
					break;
				case MT_RX_DATA:									// received the second byte of a shot packet
					mt_rx_data_packet.data[mt_rx_data_packet_datacnt] = mt_rx_byte; // save byte in buffer
					mt_rx_data_packet_datacnt++;
				
					if (mt_rx_data_packet_datacnt >= mt_rx_data_packet.length) {  // check if last data byte
						mt_rx_state = MT_RX_PROCESSING;
						if (mt_process_data_packet_callback != NULL) {			// check if the callback is set
							mt_process_data_packet_callback(mt_rx_data_packet);
						}
						mt_rx_state = MT_RX_IDLE;
					}
					break;
				default: // should not happen
					break;
			}
		}
	}
}

void mt_send_shot_packet(mt_shot_packet mt_tx_shot_packet) 
{	
	while (ir_tx_state != IR_TX_IDLE); // wait if previous transfer has not yet completed

	ir_tx_buf[0] = SHOT_PLAYER_ID_MASK & mt_tx_shot_packet.player_id;
	ir_tx_buf[1] = (SHOT_TEAM_ID_MASK & mt_tx_shot_packet.team_id) | (SHOT_DAMAGE_MASK & mt_tx_shot_packet.damage);
	ir_tx_bufsize = 2; // used by interrupt routine to check number of bytes left to transfer

	ir_tx_start();
}

void mt_send_data_packet(mt_data_packet mt_tx_data_packet)
{
	unsigned char i;

	while (ir_tx_state != IR_TX_IDLE); // wait if previous transfer has not yet completed
	
	ir_tx_buf[0] = mt_tx_data_packet.message_id;
	ir_tx_buf[1] = mt_tx_data_packet.length;
	ir_tx_bufsize = 2; // used by interrupt routine to check number of bytes left to transfer

	// add data bytes to transmit buffer
	for (i=0; i< mt_tx_data_packet.length; i++) {
		ir_tx_buf[ir_tx_bufsize] = mt_tx_data_packet.data[i];
		ir_tx_bufsize++;
	}
		
	ir_tx_start();
}

void ir_tx_clear(void) 
{
	ir_tx_count = 0;						// used by interrupt routine for array index of ir_tx_buf (counting up)
	ir_tx_byte_shift = 0;					// used by interrupt routine to shift bit of ir_tx_byte
	ir_tx_bits_remaining = 0;				// used by interrupt routine to determine when to stop sending
}

void ir_tx_start(void) 
{
	PORTB |= (1 << PB1);
	
	mt_tx_state = MT_TX_TRANSMITTING;		// our current state is transmitting
	ir_tx_state = IR_TX_HEADER;				// first step is sending the IR header
	ir_tx_clear();
    TCNT0 = 0;								// clear the timer
	OCR0A = TIMER0_2400USEC_HEADER;			// set timer0 interrupt after 2400 microseconds
	ENABLE_IR;								// enable IR carrier
	TCCR0B |= (1 << CS02);					// start timer, CLK/256
}

void ir_tx_stop(void) {	
	TCCR0B &= ~(1 << CS02);			// disable timer0 clock source
	TCNT0 = 0;						// clear the timer
	DISABLE_IR;						// just to be sure that IR is really disabled
	ir_tx_state = IR_TX_IDLE;		// set ir_tx_state to IR_TX_IDLE
	mt_tx_state = MT_TX_IDLE;		// set mt_tx_state to MT_TX_IDLE
	
	PORTB &= ~(1 << PB1);
}

void ir_rx_clear(void) {
	ir_rx_byte = 0;
	ir_rx_byte_shift = 0x80;		// reset the shift value
	ir_rx_bits_remaining = 0;
}

void ir_rx_reset(void) {
	TCCR1B &= ~(1 << CS10);			// disable timer1 clock source
	TCNT1 = 0;						// clear timer
	ir_rx_clear();
	ir_rx_state = IR_RX_IDLE;		// start over again with receiving packet
}

/**
 * Initialize external interrupt used for IR receiver input
 */
void ir_init_extint(void)
{
    DDRD &= ~ (1 << PD2);     // set PD2/INT0 data direction to input	
    EIMSK |= (1 << INT0);     // enable INT0 in external interrupt mask register
    EICRA |= (1 << ISC00);    // Interrupt INT0 on both falling and rising edge
}

/**
 * Initialize Timer1, used by IR receive protocol
 */
void ir_init_timer1(void)
{
    TIMSK1 |= (1 << TOIE1);		// Overflow interrupt enable
}

/**
 * Initialize Timer0, used by IR send protocol
 */
void ir_init_timer0(void)
{
    TCCR0A |= (1 << WGM01); 	// CTC mode (Clear timer on Compare match), compare with OCR1A
    TIMSK0 |= (1 << OCIE0A);	// Output compare match A interrupt enable
}

/**
 * Initialize PWM for the IR LED, uses OC1B as output pin
 * PWM is not connected to OC1B after initialization!
 * To turn PWM on set bit CS11 of TCCR1B to 1 (use ir_set_pwm())
 */
void ir_init_pwm(void)
{
    // Timer1: Fast PWM, TOP=OCR1A, CLK/1
    TCCR1A = (1 << WGM11) | (1 << WGM10);	
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
	
    OCR1A = PWM_56KHZ; // 56 KHz @ 16MHz
    OCR1B = PWM_56KHZ / 2; // 50% duty cycle

    DDRB |= (1 << PB2); // PB2/OC1B as output
    PORTB &= ~(1 << PB2); // set PB2/OC1B to low, so it is low while PWM is not on at the beginning
}

/**
 * Enables or disables the PWM output on pin PB4/OC1B
 * @param state disables PWM if state is 0, enables PWM if it is any other value
 */
void ir_set_pwm(unsigned char state)
{
    if (state) {
        // reset Timer1 value so we start counting at 0
        // Otherwise the length of the first pulse is unpredictable
        TCNT1 = 0; 
		TCCR1A |= (1 << COM1B1); // enable PWM: clear OC1B on compare match, set OC1B at BOTTOM
        PORTB |= (1 << PB1); // enable muzzle led for debugging
    } else {
		TCCR1A &= ~(1 << COM1B1); // disable PWM
        PORTB &= ~(1 << PB2); // make sure PB2/OC1B is low
        PORTB &= ~(1 << PB1); // disable muzzle led for debugging
    }
}

/**
 * Checks if Infrared receive buffer is full
 * @return 1 if buffer is full, 0 if buffer is not full
 */
unsigned char ir_rxbuf_full(void) 
{
	return ir_rx_buf_rpt == ir_rx_buf_wpt &&
	 		ir_rx_bufsize == IR_RXBUF_SIZE;
}

/**
 * Checks if Infrared receive buffer is empty
 * @return 1 if buffer is empty, 0 if buffer is not empty
 */
unsigned char ir_rxbuf_empty(void)
{
	return ir_rx_buf_rpt == ir_rx_buf_wpt && 
			ir_rx_bufsize == 0;
}

/**
 * Adds a byte to the Infrared receive buffer
 * This function may only be called if ir_rxbuf_full() == 0
 */
void ir_rxbuf_pushbyte(unsigned char c)
{
    // increase write pointer, check if at end of array
    if (++ir_rx_buf_wpt >= IR_RXBUF_SIZE) ir_rx_buf_wpt = 0;
 
    ir_rx_buf[ir_rx_buf_wpt] = c;    
    ir_rx_bufsize++;
}
 

/**
 * Pulls a byte from the Infrared receive buffer
 * This function may only be called if ir_rxbuf_empty() == 0
 * @return byte pulled from the buffer
 */
unsigned char ir_rxbuf_pullbyte(void) 
{  

  // increase read pointer, check if at end of buffer, if so wrap to the start of the buffer
  if (++ir_rx_buf_rpt >= IR_RXBUF_SIZE) ir_rx_buf_rpt = 0;
     
  ir_rx_bufsize--;  
  
  return ir_rx_buf[ir_rx_buf_rpt];  
}

