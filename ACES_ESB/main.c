/** @file main.c
 *  @author Nick Moore
 *  @date March 22, 2018
 *  @brief Mainline code for the ACES ESB
 *
 *  @bug No known bugs
 *  @note The ECU/ESB Temperature sensors have yet to be implemented
 */ 

#include <avr/io.h>
#include "ESB_funcs.h"

int main(void)
{
    Initial();
	while (1) 
    {	
		connected++;
		connected--;                    // I don't understand why these two lines are needed but it makes it work
		
		if (connected){
			if (opMode == 1){}
				//shutdown();
			else if (opMode == 2)
				startup();
			else if (opMode == 3)
				throttle();
			else if (opMode == 5)
				coolingMode();
			else if (hallDone){
				package_message();
				sendToECU(allData);
			}
		}		
    }
}

