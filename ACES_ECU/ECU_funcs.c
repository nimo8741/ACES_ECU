/** @file Engine_funcs.c
 *  @author Nick Moore
 *  @date March 20, 2018
 *  @brief Implementation for functions for local processes on the ECU
 *
 *  @bug No known bugs.
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ECU_funcs.h"


/** @brief Measures the voltage on the Lipo battery from the 3 different ADC channels
 *
 *  This performs the following functions:
 *  
 *  1) Measures the ADC voltage on the current ADC channel
 *  2) Combines this result will the current float value of the voltage
 *  3) Changes the ADC channel to the next one in the rotation 0->1->2->0
 *
 *  @param void
 *  @return void
 */
void batVoltage(void)
{
	// first check to see if the conversion has completed
	if (bit_is_set(ADCSRA,ADSC))
		return;
	
	uint8_t low_bits = ADCL;
	uint8_t high_bits = ADCH;						 // Do the shifting so that there is room made inside of the 16 bit register
	uint16_t result = (high_bits << 8) | low_bits;
	
	// Now I need to convert this 16 bit number into an actual temperature
	voltage += (float) result;   // Convert to float value of the voltage
	
	// Now prepare the ADC for the next channel
	if (batChannel == 2){
		voltageFinal = voltage;
		assign_bit(&ADMUX,MUX1,0);
		batChannel = 0;
	}
	else{  // if there are still at least one more channel to sample before starting over
		ADMUX++;
		batChannel++;
	}
	return;
	
}

/** @brief Measures the fuel flow passing through the Flow meter
 *
 *  This performs the following functions:
 *  
 *  1) If the flow meter measurement has not started, it enables the appropriate interrupts and returns
 *  2) If the flow meter measurement is in progress, it simply returns
 *  3) If the flow meter measurement has concluded, it calculates the recorded pulses
 *
 *  @param void
 *  @return void
 */
void measureFlow(void)
{
	if (flowMode == 0){            // This is the mode in which the flow meter has not started sampling
		pulse_count = 0;

		EIMSK |= (1 << INT2);    // enable the external interrupts
	
		// Now start Timer3 with overflow interrupts
		TCCR3B |= (1 << CS31) | (1 << CS30);
		flowMode = 1;
	}
	else if(flowMode == 2){        // This is the mode in which the flow meter has completed sampling
		// Now need to convert the pulses detected into an actual flow rate
		massFlow = V_per_pulse * (float) pulse_count;
		massFlow = (massFlow - pump_b) / pump_m;
		flowMode = 0;     // prepare for the next measurement
	}
	return;
}

/** @brief Interrupt Service Routine which reads in the pulse train and increments a count.
 *
 *  @param[in] pulse_count This is the number which describes how many pulses have been received for the sampling period.  It is an implicit argument as it is a global variable which is not explicitly passed in.
 *  @return void
 *  @see flowMeter
 */
ISR(INT2_vect)
{
	pulse_count++;  // The interrupt flag will automatically be cleared by hardware
}

/** @brief Interrupt service routine for the overflow of timer 3 (the flow meter timer)
 *
 *  This performs the following functions:
 *  
 *  1) The INT2 external interrupts are disabled
 *  2) Timer 3 is then turned off
 *  3) The value inside of the timer register is reset
 *  4) The flowMode is then changed 
 *
 *  @param void
 *  @return void
 */
ISR(TIMER3_OVF_vect)
{
	// First disable the external interrupt
	assign_bit(&EIMSK, INT2, 0);
	
	// Now turn off Timer 3
	assign_bit(&TCCR3B, CS31, 0);
	assign_bit(&TCCR3B, CS30, 0);
	
	// Now reset the value inside of the TIMER3 register
	TCNT3 = 3036;    // This will allow the timer to run for 0.25 sec once the prescalars are restored
	
	// now chance the flowMode
	flowMode = 2;
}

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

ISR(TIMER4_OVF_vect)
{
	// If it makes it in here then the Computer is presumed to have gotten disconnected from the ECU
	assign_bit(&TCCR4B, CS41, 0);
	assign_bit(&TCCR4B, CS40, 0);            // turn off the timer
	TCNT4 = 40536;                           // reload the timer register
	shutdown();
	connected = 0;
}

ISR(TIMER5_OVF_vect)
{
	// If it makes it in here then the ESB is presumed to have gotten disconnected from the ECU
	opMode = 5;
	assign_bit(&TCCR5B, CS51, 0);
	assign_bit(&TCCR5B, CS50, 0);            // turn off the timer
	TCNT5 = 40536;                           // reload the timer register
}