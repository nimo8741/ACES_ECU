/** @file Initial_funcs.c
 *  @author Nick Moore
 *  @date March 16, 2018
 *  @brief Implementation of initialization routines
 *  
 *  @bug No known bugs.
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ECU_funcs.h"

/** @brief Performs the waiting cycle until the HCU signals to the ECU to being operating
 *
 *  This performs the following functions:
 *  
 *  1) Waits until the connection pin between the HCU and ECU is driven high
 *
 *  @param void
 *  @return void
 */
void pre_Initial(void)
{
	DDRC |= (0 << HCU_link);      // set this pin for input
	while (bit_is_clear(PINC, HCU_link));    // just wait until the HCU tells the ECU to wake up
}

/** @brief Initializes the microcontroller for its mainline execution.
 *
 *  This performs the following functions:
 *  
 *  1) Initializes the PORT data directions. 0 for input and 1 for output
 *
 *  2) Initializes the asynchronous communication with the Windows GUI
 *
 *  3) Initializes the SPI communication with the ESB
 *
 *  4) Initializes the I2C communication with the on board temperature sensors
 *
 *  5) Initializes the ADC for the Lipo battery voltage measurements
 *
 *  6) Enable global interrupts
 * 
 *  7) Enable a timer for the ECU command cycle and start the timer
 * 
 *  @param void
 *  @return void
 */
void Initial(void)
{
	////////////////////////// Initialize Port Configuration ////////////////////////////////
	DDRD |= (1 << XCK1);          // Enable the SPI output pin for output.  This puts the ECU in master mode for SPI
	DDRC |= (1 << ESB_SS);        // Enable the ESB SS pin as and output.  This will drive low to begin data transfer
	PORTC |= (1 << ESB_SS);       // Drive high for not as to not confuse the ESB
	
	//////////////////// Initialize the Asynchronous Communication //////////////////////////
	UBRR0 = 12;     // With a 16 MHz clock this will make a baud rate of 76800 (error of 0.16%)
	
	// The next things that need to be set are as follows (reference page 220 in datasheet)
	// 1) Enable receive interrupts
	// 2) Enable receiver
	// 3) Enable transmitter
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	
	// Now set the framing information
	// I want the following things
	// 1) Asynchronous USART
	// 2) No Parity
	// 3) 1 Stop bit
	// 4) 8 bit character size
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // Now the USART should be ready to receive
	
	// Now connect with the PC for the first time
	
	/////////////////////////// Initialize SPI with ESB //////////////////////////////////
	
	// The next things that need to be set are as follows
	// 1) Set MSPI mode of operation and SPI data mode to 0
	// 2) Enable the receiver and transmitter
	// 3) Set the baud rate.  This has to be done after the the transmitter has been enabled
	
	UCSR1C = (1<<UMSEL11)|(1<<UMSEL10)|(0<<UCPHA1)|(0<<UCPOL1);
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);	UBRR1 = 103;    // This will have a baud rate of 76800 with the 16MHz oscillator
	
	// Now connect with the ESB, must wait until this step is completed
	ESB_Connect();
	
	////////////////// Initialize I2C with temperature sensors ///////////////////////////
	
	// The implementation of this feature will be filled in later
	
	/////////////////// Initialize ADC for Battery Voltage ///////////////////////////////
	
	// The next things that need to be set are as follows
	// 1) The reference for the ADC needs to be set to the AVCC with external cap
	// 2) The channel for ADMUX needs to be channel 0
	// 3) The ADC needs to be enabled without interrupts
	// 4) Division factor for the ADC needs to be set to 128 to keep the input clock 
	//    frequency between 50kHz and 200kHz
	
	ADMUX = (1 << REFS1) | (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
	batChannel = 0;


	/////////////////////// Enable global interrupts //////////////////////////////////
	sei();
	 
	///////  Set Up Timers for ECU, Flow meter, and Communication Timers  /////////////
	
	// The next things that need to be set are as follows
	// 1) Timer 1 needs a prescalar of 64 and timer register of 3036    for time of 0.25 sec
	// 2) Timer 3 needs a prescalar of 64 and timer register if 3036
	// 3) Timer 3 needs to have interrupts enabled
	TCNT1 = 3036;
	TCNT3 = 3036;
	TIMSK3 = (1 << TOIE3);    // enable overflow interrupts for the flow meter timer and that's it
	// The TIFR1 register and TOV1 flag are the overflow flag for timer1
	TCCR1B = (1 << CS11) | (1 << CS10);    // start timer 1 with prescalar of 64
	TCNT4 = 40536;
	TCNT5 = 40536;
	TIMSK4 = (1 << TOIE4);     // enable overflow interrupts for the GUI communication timer
	TIMSK5 = (1 << TOIE5);     // enable overflow interrupts for the ESB communication timer

	
	// Begin the first conversion for the ADC
	ADCSRA |= (1 << ADSC);
	
	massFlow = 0.0;
	 
	// Now configure the external interrupts for the Flow meters
	EICRA = (1 << ISC20) | (1 << ISC21);     // This will enable rising edge interrupts on INT2, see page 110 in datasheet
	assign_bit(&DDRD, INT2, 0);              // Configure the PD2 pin as an input so that it can receive the signals
	 
	 
	// Now configure the global variables for the flow meter
	float pulse_flow = (density) * K_factor * max_time / 1000;   // number of pulses expected per g/sec
	V_per_pulse = pump_m / pulse_flow;               // number of volts per pulse
	opMode = 0;

 
}




