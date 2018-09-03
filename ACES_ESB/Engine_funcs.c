/** @file Engine_funcs.c
 *  @author Nick Moore
 *  @date March 22, 2018
 *  @brief Function implementation functions which are more directly related to the operation of the jet engine
 *
 *  @bug No known bugs, however, this code has not been tested on actual hardware
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ESB_funcs.h"

/** @brief Forces an engine shutdown and closes all output ports which could actuate the engine.
 *
 *  This function performs the following functions:
 *  1) Zeros out the prescalars for the operation of the following pieces of hardware:
 *		a) The starter motor
		b) The fuel solenoid
 *		b) The glow plug
 *		c) The fuel pump
 *		d) The lubrication solenoid
 *		
 *	2) Zeros out the control registers for the same components.
 *		This has the effect of restoring the pin to its normal operation (see page 155 in datasheet)
 *
 *  @param void
 *  @return void
 */
void shutdown(void)
{
	// For this just need to turn off the pump, glow plug, starter motor, and solenoids
	// Start with the starter motor
	TCCR0B = 0;    // this will force the prescalar values to be zero
	TCCR0A = 0;    // This will force normal operation of the pin (so that it can always be pulled low)
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

/** @brief Performs the function calls in order such that and engine startup would occur.
 *
 *	1)	This function checks to make sure that there is not a lockout which prevents the engine from starting.
 *	These engine lockouts are put in place such that the engine cannot be started until it has fully stopped.
 *	This is to prevent a startup in an unsafe situation.
 *
 *	2)	If a lockout is not present then the next set is set the various PWM lines such that the designates I/O
 *		pins can drive the hardware vital to the engine.
 *
 *	3)	The function invokes the compressor function which will begin spinning up the starter motor until it 
 *		reaches the point at which the air is sufficiently compressed for combustion to occur.  Meanwhile, the
 *		glow plug is turned on so that it can begin heating up.
 *
 *	4)	The function begins injecting small puffs of fuel into the combustion chamber so that combustion will
 *		begin taking place and become self-sufficient.
 * 
 *	5)	Assuming there are no issues with the engine operation so far, a heat soaking procedure will then take
 *		place.  This is essentially a process where the engine is just allowed to run without being interfered
 *		with so that the combustion chamber can reach a more optimal temperature.
 *
 *  @param void
 *  @return void
 */
void startup(void)
{
	if (startUpLockOut){
		if (hallEffect < 10 && EGT < 50){
			startUpLockOut = 0;
			startup();              // restart the function so that it has the opportunity to restart
		}
	}
	else{
		setPWM();
		compressor();
		if (opMode == 1)
			return;
			
		fuel_puffs();
		if (opMode == 1)
			return;
			
		if (hallEffect < 35000){  // This means that start up was not achieved
			shutdown();     // 35,000 RPM is the minimum required for startup
		}
		else{
			heatSoaking();	
		}
	}
}

/** @brief Sets the fuel flow rate such that the engine would operate at a desired throttle value
 *
 *	1)	This function converts the desired throttle value into a mass flow rate.  THIS MASS FLOW RATE IS
 *		DETERMINED ONLY FOR THE P90-RXI!
 *
 *	2)	This mass flow rate is then converted into a number of flow rate pulses bases on the known linear 
 *		relationship.  See the published document for more explanation on this relationship.
 *
 *	3)	The error, in terms of pulses, is then determined between the actual and desired flow rate of fuel.
 *
 *	4)	The error is then converted into volts, which then can be used to directly translated into how many 
 *		counts by which the input capture control register.  This directly correlates to the duty cycle for
 *		the PWM signal which powers the fuel pump.
 *
 *  @param void
 *  @return Void
 */
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

/** @brief Sets the duty cycle for the starter motor such that the compressor safely reaches a desired RPM
 *	
 *	1)	This function uses a proportional-derivative control law to quickly get to the desired RPM of 10,000 RPM
 *		The reason this RPM was chosen is because it was listed in one of the data sheets for the JetCat engine
 *
 *	2)	This control law uses gain which have been estimated based on the desired rise time and such.  However,
 *		these gains have not been tested at all and should not be trusted.
 *
 *	3)	If would be recommend that instead of using a fixed value for the voltage that can be supplied to the motor
 *		(see pump_tot_V) 	 
 *
 *  @param void
 *  @return void
 */
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
		if (voltage > 6.0)               // Cap the voltage at the maximum the motor is rated for
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

/** @brief Operates the fuel pump and fuel/lubrication solenoids such that ignition begins in the combustion chamber.
 *	
 *	1)	This function starts by first applying voltage so that there will be some fuel pressure on the back of the 
 *		closed fuel and lubrication solenoids.
 *
 *	2)	Then the solenoid will begin opening with a duty cycle of 5% and increase by 5% every iteration in the loop.
 *		Each iteration of the loop will occupy 0.25 secs of time.  
 *
 *	3)	The Lubrication solenoid will then be toggled at an interval as specified by lube_factor (see header file).
 *		This value corresponds to the factor by which the lubrication solenoid is slower than the fuel solenoid.
 *
 *	4)	Once the exhaust gas temperature gets above 200C, then the starter motor and glow plug will turn off as it
		can be assumed that the ignition has been seeded.
 *
 *  @param void
 *  @return void
 */
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
	
	// NOTE: It would be beneficial to have the output line for the fuel solenoid tied to a PCINT pin (such as PB6) and then
	// toggle the lubrication solenoid through the use of an interrupt.  Becuase of this the actuation of the lubrication 
	// solenoid will be left unimplemented. 
	
	while (duty != 1)
	{
		// now wait for the new value of Hall effect and EGT, wait for 2 cycles so that 0.5 seconds will elapse
		hallDone = 0;
		while (!hallDone);
		hallDone = 0;
		while (!hallDone);
		
		if (opMode == 1)    // This means that a shutdown has been invoked
			return;
			
		if (EGT > 200) {  // if true, turn off the starter motor and glow plug.  Do your own check to make sure that 200C is a good temp to turn this off at
			TCCR2A = 0;      // this will return the pin to its normal state
			TCCR2B &= 0xF8;  // this will turn off the glow plug
			assign_bit(&PORTB, glowPin, 0);   // force the pin low
			TCCR0A = 0;
			TCCR0B &= 0xF8;  // this will turn off the starter motor
			assign_bit(&PORTB, startPin, 0);    // for the pin low
		}

		duty += 0.05;
		OCR4B = ICR4 - (unsigned int)(ICR4 * duty);
	}
	if (!massFlow.f)
		opMode = 9;      // This is the opMode for if the fuel is not flowing
	
}

/** @brief Prevents interruptions from the operation of the engine so that the temperature of the combustion can will increase.
 *
 *	1)	This function is pretty simple, it sets a timer for 15 seconds and hogs execution until the timer has completed.
		During this time, the throttle is not allowed to be changed.
 
	2)	If this step is completed then it can be said that the engine has reached idle*
 *
 *  @param void
 *  @return void
 */
void heatSoaking(void)
{

	// during this time the starter motor will not be using its PWM, timer0 (8 bit)
	TCCR0A = 0;    // make this work as a normal timer
	TCCR0B = 0;    // reset everything to 0
	
	// Now figure out the prescalars
	TCNT0 = 100;
	assign_bit(&TIMSK0, TOIE0, 0);  // make sure there are not any overflow interrupts
	TCCR0B |= (1 << CS02) | (1 << CS00);   // have a prescalar of 1024 and starts the timer
	
	for (uint16_t i = 0; i < 1500; i++){
		while (bit_is_clear(TIFR0,TOV0));
		assign_bit(&TIFR0, TOV0, 1);    // clear by writing a 1 to it
		TCNT0 = 100;    // This will have the timer run for 0.1 seconds
	}
	// If it has made it to here then the engine has reached idle
	opMode = 10;
}

/** @brief Shuts off the engine with the exception of the starter motor to force cool air through the engine.
 *
 *  @param void
 *  @return void
 */
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