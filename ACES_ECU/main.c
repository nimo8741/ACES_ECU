/** @file main.c
 *  @author Nick Moore
 *  @date March 16, 2018
 *  @brief Mainline code for the ACES ECU
 *
 *  @bug No known bugs
 *  @note The ECU/ESB Temperature sensors have yet to be implemented
 */ 

#include <avr/io.h>
#include "ECU_funcs.h"

int main(void)
{
	//waitMS(4000);                   // For debugging purposes only REMOVE!
	if (HCU_present){
		pre_Initial();       // This will have the waiting until the HCU tells the ECU to wake up
	}
	Initial();
	while (1) 
    {
		if (!connected){
			ESB_Connect();
		}

		//batVoltage();       // This is the battery voltage measurement function
		measureFlow();      // This is the flow calculation function
		//readTempSensor();
		massFlow.f += 0.05;
		if (massFlow.f > 4.8) {
			massFlow.f = 0;
		}
		
		if (connected){
			packageMessage();
			sendToESB(flowData);           // Send the flow data to the ESB
		}
		
		sendToLaptop();
		
    }
}

