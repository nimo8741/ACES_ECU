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
    if (HCU_present){
		pre_Initial();       // This will have the waiting until the HCU tells the ECU to wake up
	}
	Initial();
	while (1) 
    {
		batVoltage();       // This is the battery voltage measurement function
		measureFlow();      // This is the flow calculation function
		ESBTransmit();      // This is the normal message transmission between the ESU and ESB
    }
}

