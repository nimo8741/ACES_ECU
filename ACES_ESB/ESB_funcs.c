/** @file Ethernet.c
 *  @author Nick Moore
 *  @date March 22, 2018
 *  @brief Function implementation for most operations of the ESB
 *
 *  @bug No known bugs
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ESB_funcs.h"


/** @brief Sets the specified bit to the specified value or does nothing if it already set to that.
 *
 *  @param[out] sfr Pointer to Special Function Register the bit is located in.
 *  @param[in] bit Denotes the bit which would like to be changed.
 *  @param[in] val The value, either 1 or 0, that the user would like the bit to be after the function call.
 *  @return void
 */
void assign_bit(volatile uint8_t *sfr,uint8_t bit, uint8_t val)
{
	if (val)      // This is for if I want the value to be a 1
	{
		val = (val << bit);
		*sfr |= val;
	}
	else             // This is for if I want the value to be a 0
	{
		val = ~(1 << bit);
		*sfr &= val;
	}
}

/** @brief Collects the temperature from the Exhaust Gas Thermocouple and then packages/send data to the ECU
 *
 *  @param[in] void
 *  @return void
 */
void EGT_collect(void)
{
	// Now loop through 4 bytes for the receive message
	uint8_t tempString[2];
	SSACTIVE;       // drop the SS line for the CJC
	for (int i = 0; i < 2; i++){
		tempString[i] = SPI_Receive();
	}
	SSPASSIVE;       // raise the SS line again since we are done
	
	// Now interpret this data string into an actual temperature

	getTemp(tempString);
			
	// Now package the message
	package_message();
}

/** @brief Performs the operations needed to communicate with the CJC via SPI
 *
 *  @param[in] void
 *  @return uint8_t
 */
uint8_t SPI_Receive( void )   // This is copied from the datasheet page 193
{
	cli();
	/* Put data into buffer and start the transmission */
	SPDR = 0;   // Since the CJC doesn't receive, we can put anything we want into the buffer
	
	/* Wait for data to be received */
	while ( !(SPSR & (1<<SPIF)) );
	
	/* Enable interrupts again */
	sei();
	
	/* Get and return received data from buffer */
	return SPDR;
}

/** @brief Converts the bytes received from the CJC into a usable temperature
 *
 *  @param[in] tempString Array of chars which contains all the data received by the CJC
 *  @return void
 */
void getTemp(uint8_t *tempString)     // This is definitely Wrong but will fix it later
{
	// First check to see if there is a fault
	if (tempString[1] & 0x04)
		EGT = 0;               // This means that the Thermocouple is open, check connection
	else {                     // If it makes it to here then there are no faults
		int val = ((tempString[0] &) << 5) | (tempString[1] >> 3);
		
		if (!val)
			val = 1;   // set the value to this for anything that is less than 3
		EGT = ((float) val / 4095.0) * 1023.75;        // Since the temperature is on the scale of 0->1023.75 and it has 12 bit resolution
		ref_temp = 0;                                  // This is unimplemented at this time as the MAX6675 does not transmit the reference temperature
		
	}
}

/** @brief Increments the hall effect counter when a pulse is received
 *
 *  @param[in] void
 *  @return void
 */
ISR(INT2_vect)
{
	hallCount++;
}

/** @brief Signals that the sampling time for the Hall effect sensor is over
 *
 *  @param[in] void
 *  @return void
 */
ISR(TIMER4_OVF_vect)
{
	hallEffect = hallCount * 120;  // this gets the number of pulses per 30 seconds
	EGT_collect();
	
	if (hallEffect > 65000 || EGT > 700) { // if either the engine is too hot or spinning too fast, shut it down
		//shutdown();
	}
	hallDone = 1;
	hallCount = 0;                        // reset the hall effect counter
	
	// reload the new value into the timer register
	TCNT4 = HallTime;

	
}

/** @brief Sets all of the Initializations for the PWMs for the Fuel Pump, Solenoids, and Starter Motor
 *
 *  @param[in] void
 *  @return void
 */
void setPWM(void)
{
	// First the Fuel pump will be a PWM on Timer 3
	TCCR3A |= (1 << WGM31);    // set this for mode 14 waveform
	TCCR3B |= (1 << WGM32) | (1 << WGM33);        // this sets the other 2 bits for the waveform generation
	TCCR3A |= (1 << COM3B1) | (1 << COM3B0);      // This sets the other two bits for the waveform generation
	ICR3 = 40000;                                 // With a prescalar of 8, this will have a period of 20 ms
	
	// Now the fuel solenoid will be a PWM on Timer 1
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	TCCR1A |= (1 << COM1B1) | (1 << COM1B0);      // This is the same as for the fuel pump
	ICR1 = 31250;                                 // With a prescalar of 256, this will have a period of 0.5 sec
	
	// Now set up the Starter motor on Timer 0 (this an 8 bit timer instead of 16)
	TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A0) | (1 << COM0A1);   // This will set it to the Fast PWM
	TCCR0B |= (1 << WGM02);
	OCR0A = (uint8_t) (255 - sMotor * 255.0 / pump_tot_V);             // When combine with a prescalar of 1024, this will have a period of 0.016384 seconds
	
	// Now set up the Glow plug on Timer 2 (this is an 8 bit timer instead of 16)
	TCCR2A |= (1 << WGM21) | (1 << WGM20) | (1 << COM2A0) | (1 << COM2A1);    // This is the same as for starter motor code
	TCCR2B |= (1 << WGM22);
	OCR2A = (uint8_t) (255 - gVolts * 255.0 / pump_tot_V);
	
}

void waitMS(uint16_t msec)
{
	// This function utilizes timer 0 and a sequence of delay loops to delay for the desired time
	TCNT0 = 5;
	// begin the timer
	TCCR0B |= (1 << CS01) | (1 << CS00);       // this will start the timer with a prescalar of 64
	for (uint16_t i = 0; i < msec; i++){
		while(bit_is_clear(TIFR0, TOV0));
		TIFR0 |= (1 << TOV0);                  // Clear the overflow flag by writing a 1 to it
	}
	assign_bit(&TCCR0B, CS01, 0);
	assign_bit(&TCCR0B, CS00, 0);              // Turn off the timer
}