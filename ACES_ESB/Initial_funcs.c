/** @file Initial_funcs.c
 *  @author Nick Moore
 *  @date March 22, 2018
 *  @brief Implementation of initialization routines
 *  
 *  @bug No known bugs.
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "ESB_funcs.h"

/** @brief Initializes the microcontroller for its mainline execution.
 *
 *  This performs the following functions:
 *  
 *  1) Initializes the PORT data directions. 0 for input and 1 for output
 *
 *  2) Initializes the ADC for use by EGT
 *
 *  3) Initializes the External Interrupt line for use by the Hall effect Sensor
 *
 *  4) Initializes the I2C communication with the on board temperature sensors
 *
 *  5) Initializes the SPI communication with the ECU
 *
 *  6) Enable a timer for use by the Hall effect sensor
 * 
 *  7) Enable a timer to determine if ignition is taking too long
 *
 *  8) Enable global interrupts
 * 
 *  @param void
 *  @return void
 */
void Initial(void)
{
	////////////////////  Step 1: Initialize the Port directions  ///////////////////////////
	DDRB = (1 << MOSI) | (1 << SCK) | (1 << Ethernet_SS);
	DDRE = (1 << CJC_CLK) | (1 << CJC_SS);              // This is the SS line for the CJC, so set to output
	assign_bit(&PORTE, CJC_SS, 1);     // Don't want to start conversion too soon
	
	// Now do port directions for the PWM outputs
	DDRB |= (1 << PB4) | (1 << PB5) | (1 << PB7);   // This sets the output for the pump, starter motor, and solenoids
	DDRE |= (1 << PE4);	
	
	
	////////////       Step 2: Initialize the Master SPI with the CJC         ///////////////
	// The next things that need to be set are as follows
	// 1) Set MSPI mode of operation and SPI data mode to 0
	// 2) Enable the receiver and transmitter
	// 3) Set the baud rate.  This has to be done after the the transmitter has been enabled
	// 4) Set up timer 7 with a 0.1 second delay so that there is time in between sampling 
		
	UCSR0C = (1<<UMSEL01)|(1<<UMSEL00);   // by not explicitly defining the mode, it should be in mode 0
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);	UBRR0 = 7;    // This will have a baud rate of 1MHz with the 16MHz oscillator

	
	///////////////////  Step 3: Initialize External Interrupt line  ////////////////////////
	// This will be used with the Hall effect sensor
	assign_bit(&DDRD, INT2, 0);              // Configure the PD2 pin as an input so that it can receive the signals
	hallCount = 0;
	EICRA = (1 << ISC20) | (1 << ISC21);     // This will enable rising edge interrupts on INT2, see page 110 in datasheet

	
	/////////////////  Step 5: Initialize UART Communication with ECU  ////////////////////////
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

			
	
	// Now initialize the timer for both the ECU communication error checking
	TCNT5 = ECU_timer_val;
	TIMSK5 = (1 << TOIE5);     // enable overflow interrupts on this mode of timer 5
		
	///////////////////////  Step 6: Enable Hall Effect Timer  //////////////////////////////
	// This will be set to 0.25 seconds so there is a reasonable sampling period
	TCNT4 = HallTime;
	TIMSK4 = (1 << TOIE4);                 // enable overflow interrupts for the Hall effect sensor timer
	waitMS(195);                           // wait this portion of time so that the ECU comm and Hall effect interrupts are off phase
	TCCR4B = (1 << CS41) | (1 << CS40);    // start timer 4 with prescalar of 64


	////////////////////  Step 8: Fuel Flow Calculation factors  /////////////////////////////
	pulse_flow = (1.0 / density) * K_factor * max_time / 1000;   // this is the pulses expected per g/s in 0.25 sec
	V_per_pulse = pump_m / pulse_flow;	

	
	//////////////////////  Step 9: Enable Global Interrupts  //////////////////////////////
	sei();
	
	hasInterrupted = 0;
	commandCode = 0;        // This means that that the next received char is a new command
	ECUreceiveCount = 0;
	
	//connected = 1;

}