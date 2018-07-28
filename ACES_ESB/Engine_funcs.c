#include <avr/io.h>
#include <avr/interrupt.h>
#include "ESB_funcs.h"

void shutdown(void)
{
	// For this just need to turn off the pump, glow plug, starter motor, and solenoids
	// Start with the starter motor
	TCCR0B = 0;    // this will force the prescalar values to be zero
	TCCR0A = 0;
	assign_bit(&PORTB, startPin, 0);
	
	// Now the glow plug
	TCCR2B = 0;
	TCCR2A = 0;
	assign_bit(&PORTB, glowPin, 0);
	
	// now the pump
	TCCR3B = 0;
	TCCR3A = 0;  // make sure the PWM loses authority
	assign_bit(&PORTB, pumpPin, 0);
	
	// now the fuel solenoid
	TCCR5B = 0;
	TCCR5A = 0;
	assign_bit(&PORTB, solePin, 0);
	
	// now the lube solenoid
	assign_bit(&PORTB, lubePin, 0);
	
	startUpLockOut = 1;
	opMode = 4;    // An opMode of 4 means that the engine will enter the cooling mode
}

void startup(void)
{
	if (startUpLockOut){
		if (hallEffect < 10 && EGT < 50){
			startUpLockOut = 0;
			startup();              // restart the function so that it has the opportunity to restart
		}
	}
	else{
		return;              // skip all of the actual stuff for now, REMOVE LATER
		setPWM();
		compressor();
		if (opMode == 1)
			return;
			
		fuel_puffs();
		if (opMode == 1)
			return;
			
		if (hallEffect < 35000){  // This means that start up was not achieved
			//shutdown();     // 35,000 RPM is the minimum required for startup
		}
		else{
			heatSoaking();	
		}
	}
}

void throttle(void)   // I only want this function to be called after a new 
{
	// first I need to figure out what mass flow rate is desired for the requested throttle
	desMFlow = 4.8*((float) throttle_val / 255.0);
	uint8_t desPulses = (uint8_t) desMFlow*pulse_flow;
	
	// Now I need to increase the duty cycle depending in the difference from the expected flow rate
	uint8_t pulse_error = desPulses - (uint8_t) pulse_flow * massFlow.f;
	
	float difference = massFlow.f - desMFlow;
	if (difference < 0)
		difference = -difference;
	
	if (difference < errorAllow){
		opMode = 8;                                 // this means that the desired throttle has been reached
	}
	
	float change = (float) pulse_error * V_per_pulse * ((float) ICR3 / pump_tot_V);
	OCR3B -= (uint16_t) change;
	if (opMode != 8)
		opMode = 4;                 // change the opMode so that it doesn't go through this again until there is a new flow measurement
}

void compressor(void)
{
	// For this function, the PD control law needs to be implemented
	//so that the engine gets to 10,000 RPM as quickly as possible
	char slope = 0;     // this will be a linear slope for the current rate of change of RPM
	
	// first turn on the glow plug
	OCR2A = 255 - ((uint8_t) (gVolts / pump_tot_V * 255.0));
	// now turn on the prescalar
	TCCR2B |= (1 << CS22) | (1 << CS20);   // this is a prescalar of 1024
	glowPlug = 1;   // so that the PC can also record that the glow plug is on
	
	// now turn on the starter motor
	OCR0A = 255 - ((uint8_t) (sMotor / pump_tot_V * 255.0));
	TCCR0B |= (1 << CS02) | (1 << CS00);	
	
	while (!(hallEffect < 10500 && hallEffect > 9500 && slope < 10 && slope > -10))
	{
		// now need to find new voltage
		float voltage = Kp*(hallEffect - 10000) + Kd*slope;
		if (voltage > 6.0)
			voltage = 6.0;
		else if (voltage < 0.0)
			voltage = 0.0;         // some error checking to make sure that things to not get unbounded
		// now convert this into a duty cycle 
		float duty = voltage / pump_tot_V;
		
		// now change the duty cycle on the starter motor
		OCR0A = 255 - ((uint8_t) (duty * 255.5));
		hallDone = 0;
		uint16_t hallPrev = hallEffect;
		// now wait for the next hall effect sensor measurement
		while (!hallDone);
		
		if (opMode == 1){  // This means that a shutdown has been invoked
			return;
		}
		
		// now calculate the new slope
		slope = (hallPrev - hallEffect) / 0.25;
		
	}
	
}

void fuel_puffs(void)
{
	// If the code has made it this far then the compressor is up to speed 
	// first I need to apply 2 volts of pressure with the fuel pump
	OCR3B = ICR3 - (unsigned int)(ICR3 * 2.0 / pump_tot_V);          // This will set the duty cycle so that there is 2 volts received by the pump
	
	// now change the prescalar so the pump will turn on, prescalar of 8
	TCCR3B |= (1 << CS31);
	
	// Now begin with increasing the duty cycle
	float duty = 0.0;
	OCR1B = ICR1 - (unsigned int)(ICR1 * duty);
	// now turn on the fuel solenoid with a prescalar of 256
	TCCR4B |= (1 << CS42);
	
	while (duty != 1)
	{
		// now wait for the new value of Hall effect and EGT, wait for 2 cycles so that 0.5 seconds will elapse
		hallDone = 0;
		while (!hallDone);
		hallDone = 0;
		while (!hallDone);
		
		if (opMode == 1)    // This means that a shutdown has been invoked
			return;
			
		if (EGT > 100) {  // if true, turn off the starter motor and glow plug
			TCCR2B &= 0xF8;  // this will turn off the glow plug
			TCCR0B &= 0xF8;  // this will turn off the starter motor
		}

		duty += 0.05;
		OCR4B = ICR4 - (unsigned int)(ICR4 * duty);
	}
	if (!massFlow.f)
		opMode = 9;
	
}

void heatSoaking(void)
{
	// If I have made it to this phase, then startup has been
	// completed and I need to run a timer for 15 seconds to heat soak
	// the whole engine.  During this time, the throttle is not allowed
	// to increase
	
	// during this time the starter motor will not be using its PWM, timer0 (8 bit)
	TCCR0A = 0;    // make this work as a normal timer
	TCCR0B = 0;    // reset everything to 0
	
	// Now figure out the prescalars
	TCNT0 = 100;
	assign_bit(&TIMSK0, TOIE0, 0);  // make sure there are not any overflow interrupts
	TCCR0B |= (1 << CS02) | (1 << CS00);   // have a prescalar of 1024 and starts the timer
	
	for (uint16_t i = 0; i < 1502; i++){
		while (bit_is_clear(TIFR0,TOV0));
		assign_bit(&TIFR0, TOV0, 1);    // clear by writing a 1 to it
		TCNT0 = 100;
	}
	// If it has made it to here then the engine has reached idle
	opMode = 10;
}

void coolingMode(void)
{
	// for this I will make sure that the starter motor receives 4V 
	// This will force cool air through the engine
	
	if (EGT > 100){
		// first need to make sure that the PWM is working 
		TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A0) | (1 << COM0A1);
		TCCR0B |= (1 << WGM02);
		OCR0A = (uint8_t) (255 - sMotor * 255.0 / pump_tot_V);             // When combine with a prescalar of 1024, this will have a period of 0.016384 seconds	
	
		// Now start with the proper duty cycle
		TCCR0B |= (1 << CS00) | (1 << CS02);
	}
	else{    // this means that the engine has cooled sufficiently
		TCCR0A = 0;
		TCCR0B = 0;
		assign_bit(&PORTB, startPin, 0);   // make sure the starter motor is turned off
		opMode = 6;   // this means that the engine is just kind of chilling    
	}
	
	
}