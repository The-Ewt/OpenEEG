/*
* ModularEEG firmware for one-way transmission, v0.5.4-p2
* Copyright (c) 2002-2003, Joerg Hansmann, Jim Peters, Andreas Robinson
* License: GNU General Public License (GPL) v2
* Compiles with AVR-GCC v3.3.
*
* Note: -p2 in the version number means this firmware is for packet version 2.
*/

//////////////////////////////////////////////////////////////

/*

////////// Packet Format Version 2 ////////////

// 17-byte packets are transmitted from the ModularEEG at 256Hz,
// using 1 start bit, 8 data bits, 1 stop bit, no parity, 57600 bits per second.

// Minimial transmission speed is 256Hz * sizeof(modeeg_packet) * 10 = 43520 bps.

struct modeeg_packet
{
uint8_t		sync0;		// = 0xa5
uint8_t		sync1;		// = 0x5a
uint8_t		version;	// = 2
uint8_t		count;		// packet counter. Increases by 1 each packet.
uint16_t	data[6];	// 10-bit sample (= 0 - 1023) in big endian (Motorola) format.
uint8_t		switches;	// State of PD5 to PD2, in bits 3 to 0.
};

// Note that data is transmitted in big-endian format.
// By this measure together with the unique pattern in sync0 and sync1 it is guaranteed,
// that re-sync (i.e after disconnecting the data line) is always safe.

// At the moment communication direction is only from Atmel-processor to PC.
// The hardware however supports full duplex communication. This feature
// will be used in later firmware releases to support the PWM-output and
// LED-Goggles.

*/

//////////////////////////////////////////////////////////////

/*
* Program flow:
*
* When 256Hz timer expires: goto SIGNAL(SIG_OVERFLOW0)
* SIGNAL(SIG_OVERFLOW0) enables the ADC
*
* Repeat for each channel in the ADC:
* Sampling starts. When it completes: goto SIGNAL(SIG_ADC)
* SIGNAL(SIG_ADC) reads the sample and restarts the ADC.
*
* SIGNAL(SIG_ADC) writes first byte to UART data register
* (UDR) which starts the transmission over the serial port.
*
* Repeat for each byte in packet:
* When transmission begins and UDR empties: goto SIGNAL(SIG_UART_DATA)
*
* Start over from beginning.
*/

// =====================================================
// Last Built with WinAVR-20100110
// Supported processors: ATmega8, ATmega16 and AT90S4434
// =====================================================

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <util/delay.h>

#undef  Crystal_Freq_16MHz		// In all other cases 7.3728MHz crystal is used!

#define NUMCHANNELS 6
#define HEADERLEN 4
#define PACKETLEN (NUMCHANNELS * 2 + HEADERLEN + 1)

#define SAMPFREQ 256
#define TIMER0VAL 256 - ((7372800 / 256) / SAMPFREQ)

//char const channel_order[]= { 0, 3, 1, 4, 2, 5 };
char const channel_order[]= { 0, 1, 2, 3, 4, 5 };

/** The transmission packet */
volatile uint8_t TXBuf[PACKETLEN];

/** Next byte to read or write in the transmission packet. */
volatile uint8_t TXIndex;

/** Current channel being sampled. */
volatile uint8_t CurrentCh;


/** Sampling timer (timer 0) interrupt handler */

//SIGNAL(SIG_OVERFLOW0)
ISR(TIMER0_OVF_vect)
{
	outb(TCNT0, TIMER0VAL); //Reset timer to get correct sampling frequency.
	CurrentCh = 0;
	
	// Write header and footer:
	
	// Increase packet counter (fourth byte in header)
	TXBuf[3]++;
	
	//Get state of switches on PD2..5, if any (last byte in packet).
	TXBuf[2 * NUMCHANNELS + HEADERLEN] = (inp(PIND) >> 2) &0x0F;
	cbi(UCSRB, UDRIE);      //Ensure UART IRQ's are disabled.
	sbi(ADCSR, ADIF);       //Reset any pending ADC interrupts
	sbi(ADCSR, ADIE);       //Enable ADC interrupts.
	
	//The ADC will start sampling automatically as soon
	//as sleep is executed in the main-loop.
}

/** AD-conversion-complete interrupt handler. */

//SIGNAL(SIG_ADC)
ISR(ADC_vect)
{
	volatile uint8_t i;
	i = 2 * CurrentCh + HEADERLEN;
	TXBuf[i+1] = inp(ADCL);
	TXBuf[i] = inp(ADCH);
	CurrentCh++;
	if (CurrentCh < NUMCHANNELS)
	{
		outb(ADMUX, (channel_order[CurrentCh]));  //Select the next channel.
		//The next sampling is started automatically.
	}
	else
	{
		outb(ADMUX, channel_order[0]);      //Prepare next conversion, on channel 0.
		// Disable ADC interrupts to prevent further calls to SIG_ADC.
		cbi(ADCSR, ADIE);
		// Hand over to SIG_UART_DATA, by starting
		// the UART transfer and enabling UDR IRQ's.
		outb(UDR, TXBuf[0]);
		sbi(UCSRB, UDRIE);
		TXIndex = 1;
	}
}


/*** UART data transmission register-empty interrupt handler ***/
ISR(USART_UDRE_vect)//SIGNAL(SIG_UART_DATA)
{
	outb(UDR, TXBuf[TXIndex]);  //Send next byte
	TXIndex++;
	if (TXIndex == PACKETLEN)   //See if we're done with this packet
	{
		cbi(UCSRB, UDRIE);      //Disable SIG_UART_DATA interrupts.
		//Next interrupt will be a SIG_OVERFLOW0.
	}
}

/** Initialize PWM output (PB1 = 14Hz square wave signal) */
void pwm_init(void)
{
	// Set timer/counter 1 to use 10-bit PWM mode.
	// The counter counts from zero to 1023 and then back down
	// again. Each time the counter value equals the value
	// of OCR1(A), the output pin is toggled.
	// The counter speed is set in TCCR1B, to clk / 256 = 28800Hz.
	// Effective frequency is then clk / 256 / 2046 = 14 Hz
	outb(OCR1AH,2);	// Set OCR1A = 512
	outb(OCR1AL,0);
	outb(TCCR1A, ((1<<COM1A1) + (1<<WGM11) + (1<<WGM10))); // Set 10-bit PWM mode
	outb(TCCR1B, (1 << CS12));	// Start and let run at clk / 256 Hz.
}

int main( void )
{
	//Write packet header and footer
	TXBuf[0] = 0xa5;        //Sync 0
	TXBuf[1] = 0x5a;        //Sync 1
	TXBuf[2] = 2;           //Protocol version
	TXBuf[3] = 0;           //Packet counter

	//Set up the ports.
	outb(DDRD,  0xc2);
	outb(DDRB,  0x07);
	outb(PORTD, 0xff);
	outb(PORTB, 0xff);

	//Select sleep mode = idle.
	outb(MCUCR,(inp(MCUCR) | (1<<SE)) & (~(1<<SM0) | ~(1<<SM1) | ~(1<<SM2)));

	//Initialize the ADC

	// Timings for sampling of one 10-bit AD-value:
	//
	// prescaler > ((XTAL / 200kHz) = 36.8 =>
	// prescaler = 64 (ADPS2 = 1, ADPS1 = 1, ADPS0 = 0)
	// ADCYCLE = XTAL / prescaler = 115200Hz or 8.68 us/cycle
	// 14 (single conversion) cycles = 121.5 us (8230 samples/sec)
	// 26 (1st conversion) cycles = 225.69 us

	outb(ADMUX, 0); //Select channel 0

	//Prescaler = 64, free running mode = off, interrupts off.
	outb(ADCSR, ((1<<ADPS2) | (1<<ADPS1)));
	sbi(ADCSR, ADIF);		//Reset any pending ADC interrupts
	sbi(ADCSR, ADEN);		//Enable the ADC

	//Initialize the UART
	outb(UBRRH,0);              //Set speed to 57600 bps
	outb(UBRRL,7);
	outb(UCSRA, 0);
	outb(UCSRC, ((1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0)));	// 8 bits data length
	outb(UCSRB, (1<<TXEN));									// Transmitter Enabled

	//Initialize timer 0  -> used to sample ADC

	outb(TCNT0, 0);             //Clear it.
	outb(TCCR0, 4);             //Start it. Frequency = clk / 256
	outb(TIMSK, (1<<TOIE0));    //Enable the interrupts.

	pwm_init();		//Initialize PWM (optional)
	
	sei();			// Enable all interrupts
	
	#define DEBUG_PINB0 // Enables PINB0 debug flicker

	#ifdef DEBUG_PINB0

	DDRB |= (1 << PINB0);
	PORTB &= ~(1 << PINB0);

	while (1)
	{
		PORTB ^= (1 << PINB0);
		_delay_ms(250);
	}

	#else
	//Now, we wait. This is an event-driven program, so nothing much
	//happens here.
	while (1)
	{
		__asm__ __volatile__ ("sleep");
	}
	#endif
}