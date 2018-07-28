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
	voltage = 3.0 * (float) result;            // Convert to float value of the voltage
	voltage = voltage * (4.96 / 1024.0);      // This will return Vin
	
	ADCSRA |= (1 << ADSC);                     // Start the next conversion
		
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
	pulse_count = 0;

	EIMSK |= (1 << INT2);    // enable the external interrupts
	
	// Now start Timer3 to run for 0.25 sec
	TCCR3B |= (1 << CS31) | (1 << CS30);
	
	while (bit_is_clear(TIFR3, TOV3));       // hog execution until the overflow flag has been set
	TIFR3 |= (1 << TOV3);                    // clear the interrupt flag
	
	// now disable the external interrupt
	assign_bit(&EIMSK, INT2, 0);
	
	// Now turn off Timer 3
	assign_bit(&TCCR3B, CS31, 0);
	assign_bit(&TCCR3B, CS30, 0);
	
	// Now reset the value inside of the TIMER3 register
	TCNT3 = FlowTime;    // This will allow the timer to run for 0.25 sec once the prescalars are restored
	
	// Now need to convert the pulses detected into an actual flow rate
	//massFlow.f = V_per_pulse * (float) pulse_count;
	//massFlow.f = (massFlow.f - pump_b) / pump_m;

	// now set the doTransmit flag
	if (doTransmit < 1){
		doTransmit++;
	}
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
	assign_bit(&TCCR4B, CS42, 0);
	TCNT4 = 34286;                           // reload the timer register
	shutdown();
	connected_GUI = 0;
	newCommand = 1;     // this will come in handy when trying to reconnect
}

ISR(TIMER5_OVF_vect)
{
	// If it makes it in here then the ESB is presumed to have gotten disconnected from the ECU
	opMode = 5;
	assign_bit(&TCCR5B, CS52, 0);            // turn off the timer
	TCNT5 = ESB_timer_val;                           // reload the timer register
	connected_GUI = 0;
}

void readTempSensor(void)
{
	i2c_Start(SLA_W);   // this will write the temp address to the register on the sensor
	i2c_write(0x05);    // this references the temperatures location in memory
	
	// now restart I2C so that it will be configured for reading the temperature
	i2c_Start(SLA_R);
	
	// now read the upper and lower bytes
	unsigned char upper = i2c_read(1);   // this will send the acknowledged bit
	unsigned char lower = i2c_read(0);   // this will send the not acknowledged bit
	i2c_Stop();
	
	// now process the data into a usable temperature
	//First Check flag bits
	if ((upper & 0x80) == 0x80){ //TA >= TCRIT
	}
	
	if ((upper & 0x40) == 0x40){ //TA > TUPPER
	}
	
	if ((upper & 0x20) == 0x20){ //TA < TLOWER
	}
	
	upper = upper & 0x1F; //Clear flag bits
	uint8_t decimal = (lower & dec_MSK) >> 2;
	if ((upper & 0x10) == 0x10){ //TA < 0°C
		upper = upper & 0x0F; //Clear SIGN
		ECU_temp = (float)(256 - ((upper << 4) + (lower >> 4)));
		ECU_temp -= (0.25 * decimal);
	}
	else { //TA >= 0°C
		ECU_temp = (float)((upper << 4) + (lower >> 4));     //Temperature = Ambient Temperature (°C)
		ECU_temp += (0.25 * decimal);
	}

}

char calculateParity(char message[], uint8_t start_index)
{
	// This will return the parity byte for a message made up of 6 sequential bytes, starting with start_index
		char parity = 0;
		
		// first need to get the number of high bits in the first three bytes in the set
		for (unsigned set = 0; set < 2; set++){
			unsigned char count = 0;
			for (unsigned char i = 0; i < 3; i++){
				count += countOnes(message[start_index + set*3 + i]);
			}
			// now that I have the count for this set, I need to take the modulo
			count = count % 16;   // Modulo with 16 because I have 4 bits to play with
			
			// now add this into the parity byte
			parity |= count << (4 * set);   // this will make it so that bytes 0-2 will take up the LSB of the parity byte
		}
		// now copy this byte into memory
		return parity;
}

unsigned char countOnes(unsigned char byte)
{
	unsigned char count = 0;
	while (byte)
	{
		count += byte & 1;    // this was essentially lifted from geeksforgeeks.com
		byte >>= 1;
	}
	return count;
}

void dummyData(void)
{
		
	// second dummy data for the hall effect sensor
	Hall_effect += 5000;   // this will overflow all on its own
	
	// third dummy data for the EGT;
	EGT += 30.24;
	if (EGT > 1000)
		EGT = 0;
	
	// fourth dummy data for the mass flow rate
	massFlow.f += 0.05;
	if (massFlow.f > 4.8)
		massFlow.f = 0;
		
	// fifth the glow plug state
	if (Hall_effect > 50000)
		glow_plug = 1;
	else
		glow_plug = 0;
		
	ESB_temp = 69.69;
			
	// sixth need the operational mode
	opMode = 5;  // this means that the engine is not doing anything
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