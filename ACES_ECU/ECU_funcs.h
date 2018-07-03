/** @file ECU_funcs.h
 *  @author Nick Moore
 *  @date March 16, 2018
 *  @brief Function prototypes, macros, constants, and global variables for ECU microcontroller
 * 
 *  @bug No known bugs
 */ 

#include <avr/sfr_defs.h>
#include <float.h>

#ifndef ECU_FUNCS_H_
#define ECU_FUNCS_H_


//Function #defines that don't need to have an entire function call
//! Simple function define for testing if the bit is set
#define bit_is_set(sfr,bit) \
		(_SFR_BYTE(sfr) & _BV(bit))

//! Simple function define for testing if the bit is clear
#define bit_is_clear(sfr,bit) \
		(!(_SFR_BYTE(sfr) & _BV(bit)))

///////////////////////////////////////////////////////////////////////////
//////////////////////// Project Constants ////////////////////////////////
///////////////////////////////////////////////////////////////////////////
#define ECU_temp_sensors 0     // The number of temperature sensors on the ECU
#define ESB_temp_sensors 0     // The number of temperature sensors on the ESB


//! Assumed K factor for the flow meter in pulses per liter
#define K_factor 91387      // This is the experimentally determined K factor

//! Density of the kerosene in g/ml
#define density 0.81

//! Maximum amount of time for an 8 bit timer with a prescalar of 1024
#define max_time 0.25

//! Slope for the linear relationship between voltage and mass flow (put in mf, get volts)
#define pump_m 0.382587

//! Y-intercept for the linear relationship between voltage and mass flow
#define pump_b 0.195783

//! defines if the HCU is present in the current configuration or not, 1 for yes, 0 for no
#define HCU_present 0

//! Defines the minimum RPM we say is engine is off
#define offRPM 213    // This translates to 99.7 RPM

//! Defines the maximum RPM of the engine
#define maxRPM 60850      // This translates to 129989 RPM

//! Defines the idle RPM of the engine
#define idleRPM 16384     // This translates to 35000 RPM

#define SLA_W 0x3E
#define SLA_R 0x3F
#define SPI_PORT PORTB
#define flowData 5
#define ESB_timer_val 3036
#define FlowTime 3700              // This was found via logic analyzer to have a flow period of exactly 0.25 seconds

///////////////////////////////////////////////////////////////////////////
///////////////////////// Pin Assignments /////////////////////////////////
///////////////////////////////////////////////////////////////////////////
#define XCK1 5     // Pin Assignment for the output clock line for USART1
#define UCPHA1 1   // bit assignment for clock phase bit for the SPI communication
#define ESB_SS 0   // Pin assignment for the SS line which extends to the ESB
#define HCU_link 1 // Pin assignment for the Command line for the HCU to turn on the ECU
#define MOSI 2     // Pin within PORTB for MOSI
#define SCK 1      // Pin within PORTB for SCK



//////////////////////////////////////////////////////////////////////////
///////////////////////////  Functions  //////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void pre_Initial(void);
void Initial(void);
void shutdown(void);
void startup(void);
void throttle(void);
void batVoltage(void);
void assign_bit(volatile uint8_t *sfr,uint8_t bit, uint8_t val);
void sendToESB(uint8_t len);
void ESB_Connect(void);
void measureFlow(void);
void sendToLaptop(void);
void repeatCommand(void);
void GUI_Connect(void);
void i2c_Start(unsigned char address);
void i2c_Stop(void);
void i2c_write(unsigned char data);
unsigned char i2c_read(unsigned char ack);
void readTempSensor(void);
void calculateParity(char message[]);
unsigned char countOnes(unsigned char byte);
void packageMessage(void);
void waitMS(uint16_t msec);

void dummyData(void);

//////////////////////////////////////////////////////////////////////////
//////////////////////// Global Variables  ///////////////////////////////
//////////////////////////////////////////////////////////////////////////                    

//! Float which contains the current voltage level on the Lipo battery
float voltage;

//! Float which contains the final battery voltage after all three conversions
float voltageFinal;

//! Char which keeps track of which ADC channel we are on
char batChannel;

//! Holds the current operational mode of the engine
char opMode;

//! Char which keeps track of the throttle percentage 0-100
uint8_t throttle_per;

//! Float holding the current mass flow rate
union {
	float f;
	unsigned char c[4];
}massFlow;

//! uint_16 holding the current measurement of the hall effect sensor
uint16_t Hall_effect;

//! uint_16 holding the current measurement of the exhaust gas thermocouple
uint16_t EGT;

//! unsigned char holding whether or not the glow plug is currently on
unsigned char glow_plug;

//! Array of uint_16 holding the temperature of the various temperature sensors
int16_t ESB_temp;

//! This will keep track of how many pulses have been seen in the sampling window
uint8_t pulse_count;

//! Variable to convert the pulses into a voltage
float V_per_pulse;

//! Will keep track of the input format that the ECU will be expecting
uint8_t commandMode;

//! Will keep track of whether or not it is possible for a new command stream to be coming in
uint8_t newCommand;

//! Will keep track of whether or not the message to the ESB has been interrupted by the PC
uint8_t hasInterrupted;

//! will keep track of whether or not the ECU is connected to the windows GUI
uint8_t connected;

char voltage_s[3];    // this will keep track of the three digits for the battery voltage

uint8_t ESBtransmit[5];
uint8_t ESBreceive[8];   // might need to change this number later

float ECU_temp;

uint8_t overrideMode;
uint8_t connect_count;
uint8_t repeatCount;
uint8_t newCommand_ESB;
uint8_t ESBreceiveCount;


#endif /* ECU_FUNCS_H_ */