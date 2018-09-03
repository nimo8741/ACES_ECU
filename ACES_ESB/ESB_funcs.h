/** @file ESB_funcs.h
 *  @author Nick Moore
 *  @date March 22, 2018
 *  @brief Function prototypes, macros, constants, and global variables for ESB microcontroller
 * 
 *  @bug No known bugs
 */  

#include <avr/sfr_defs.h>
#include <float.h>

#ifndef ESB_FUNCS_H_
#define ESB_FUNCS_H_


/** Description of the opModes:
 *  OpMode 1: An engine shutdown has been requested
 *  OpMode 2: An engine startup is in progress
 *  OpMode 3: An engine throttle adjustment is needed
 *	OpMode 4: The flow meter is being waited on so that the throttle can be adjusted again
 *  OpMode 5: The engine is in the cooling mode
 *  OpMode 6: The engine is doing nothing
 *  OpMode 7: Special Shutdown, the EGT is not working properly and so there cannot be a normal cooling mode
 *  OpMode 8: Engine is operating at the desired throttle, within the designated error tolerance
 *  OpMode 9: Fuel is not flowing when it should be flowing
 *  OpMode 10: Engine has reached idle
 *  OpMode 11: Messages sent from the ECU have failed the parity check
 *  OpMode 12: Engine Temperature limit has been reached, shutting down
 *  OpMode 13: RPM limit has been reached, shutting down
**/


//Function #defines that don't need to have an entire function call
//! Simple function define for testing if the bit is set
#define bit_is_set(sfr,bit) \
(_SFR_BYTE(sfr) & _BV(bit))

//! Simple function define for testing if the bit is clear
#define bit_is_clear(sfr,bit) \
(!(_SFR_BYTE(sfr) & _BV(bit)))

#define SSACTIVE assign_bit(&SPI_PORT, CJC_SS, 0)
#define SSPASSIVE assign_bit(&SPI_PORT, CJC_SS, 1)  // These two defines will control the operation of the Slave Select line


///////////////////////////////////////////////////////////////////////////
//////////////////////// Project Constants ////////////////////////////////
///////////////////////////////////////////////////////////////////////////
#define K_factor 91387
#define density 0.81
#define pump_m 0.382587   // slope for the linear relationship between voltage and mf (put in mf, get volts)
#define pump_b 0.195783   // y intercept for linear relationship between voltage and mf
#define max_time 0.25     // seconds for the sampling time of the flow meter
#define pump_tot_V 9.9
#define gVolts 1.75
#define Kp -0.0006
#define Kd -0.0004         // These will need to be figured out experimentally
#define lube_factor 3     // this multiple of how much less fuel the lubrication solenoid will allow let pass when compared to the fuel solenoid
#define sMotor 5.0        // this is the desired voltage on the starter motor for startup and cooling
#define errorAllow 0.2    // this is the error allowed in g/s
#define max_len 50        // This is the maximum number of bytes which will be read from the ECU
#define allData 9
#define ECU_timer_val 3036    // This is the reload value for the ECU connection timer
#define HallTime 2760         // This was found experimentally to cause the space between the message to be exactly 0.25 sec (by logic analyzer)
#define CJC_MSK 0x7           // This is the mask will will separate the MSB's of the temperature from the dummy sign bit, probably not needed
#define normalDataIn 11         // This the length of a normal data message coming from the ECU


///////////////////////////////////////////////////////////////////////////
///////////////////////// Pin Assignments /////////////////////////////////
///////////////////////////////////////////////////////////////////////////
#define MISO 3     // Pin assignment in PORT B for MISO
#define MOSI 2
#define SCK 1
#define CJC_SS 0
#define glowPin 4
#define solePin 5
#define lubePin 6
#define startPin 7
#define pumpPin 4
#define SPI_PORT PORTB

//////////////////////////////////////////////////////////////////////////
///////////////////////////  Functions  //////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void Initial(void);
void assign_bit(volatile uint8_t *sfr,uint8_t bit, uint8_t val);
void shutdown(void);
void startup(void);
void throttle(void);
void EGT_collect(void);
uint8_t SPI_Receive(void);
void getTemp(uint8_t *tempString);
void package_message(void);
void compressor(void);
void fuel_puffs(void);
void heatSoaking(void);
void setPWM(void);
void coolingMode(void);
void sendToECU(uint8_t len);
void waitMS(uint16_t msec);
void ECUconnect(void);
uint8_t calculateParity(uint8_t message[], uint8_t start_index);
uint8_t countOnes(uint8_t byte);
uint8_t checkParity(void);



//////////////////////////////////////////////////////////////////////////
//////////////////////// Global Variables  ///////////////////////////////
//////////////////////////////////////////////////////////////////////////

//! This will describe whether or not the ESB is connected to the ECU
uint8_t connected;

//! Describes critical information about the current state of the engine
uint8_t opMode;

//! Value 1-100 for the the user wants to set the throttle to
uint8_t throttle_val;

//! Flag which describes if the current hall effect sampling period has concluded
uint8_t hallDone;

//! Flag which determines if an engine startup will currently be prevented
uint8_t startUpLockOut;

//! The number of pulses recorded in the current sampling period
uint16_t hallCount;

//! Flag which describes whether the current data transmission has been interrupted by an ISR
uint8_t hasInterrupted;

//! Describes which type of information the ESB is receiving from the ECU, either normal data or a command
uint8_t commandCode;

//! Counter to keep track of the current index in a message received from the ECU
uint8_t ECUreceiveCount;

//! Flag for if the glow plug is on or off
unsigned char glowPlug;

//! Value of the desired amount of fuel flow
float desMFlow;

//! Value for how much voltage applied to the fuel pump corresponds to a single pulse from the flow meter
float V_per_pulse;

//! The amount of pulses expected per each g/sec of fuel flow
float pulse_flow;    

//! Value of the current lipo battery voltage
float bat_voltage;

//! Array for the message to send to the ECU
uint8_t ECUtransmit[14];

//! Array for the message received from the ECU
uint8_t ECUreceive[7];

//! Current RPM of recorded by the hall effect sensor
uint16_t hallEffect;

//! The current exhaust gas temperature
float EGT;

//! Ambient temperature recorded on the ESB.  This is currently unimplemented
float ref_temp;

//! Current value of mass flow
union{
	uint8_t c[4];
	float f;	
} massFlow;


#endif /* ESB_FUNCS_H_ */