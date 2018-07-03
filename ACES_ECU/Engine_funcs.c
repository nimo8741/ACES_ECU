/** @file Engine_funcs.c
 *  @author Nick Moore
 *  @date March 16, 2018
 *  @brief Implementation for functions directly related to the operation of the jet engine
 *
 *  @bug No known bugs.
 */ 

#include <avr/io.h>
#include <string.h>
#include "ECU_funcs.h"

/** @brief Commands the ESB to shutdown and awaits the confirmation code
 *
 *  This performs the following functions:
 *  
 *  1) Transmits the char to the ESB for the shutdown sequence
 *  2) Waits for the appropriate return message
 *  3) If the function was interrupted part way through by an interrupt, it is restarted
 *
 *  @param void
 *  @return void
 */
void shutdown(void)
{
	ESBtransmit[0] = 'S';
	while (ESBreceive[0] != 'K'){                    // Loop until the ESB says K
		sendToESB(1);
		waitMS(5);
	}
	ESBreceive[0] = 0;                              // clear out the first char so that it actually transmits
}

/** @brief Commands the ESB to startup and awaits the confirmation code
 *
 *  This performs the following functions:
 *  
 *  1) Transmits the char to the ESB for the startup sequence
 *  2) Waits for the appropriate return message
 *  3) If the function was interrupted part way through by an interrupt, it is restarted
 *
 *  @param void
 *  @return void
 */
void startup(void)
{
	ESBtransmit[0] = 'r';
	while (ESBreceive[0] != 'K'){
		sendToESB(1);
		waitMS(5);   
	}
	ESBreceive[0] = 0;              // Change this character so I actually have to receive it in the future	
	
}

/** @brief Commands the ESB to adjust the throttle and awaits the confirmation code
 *
 *  This performs the following functions:
 *  
 *  1) Transmits the char to the ESB to change the throttle
 *  2) Sends the new value the throttle should be changed to
 *  3) Awaits for the appropriate return message
 *  4) If the function was interrupted part way through by an interrupt, it is restarted
 *
 *  @param void
 *  @return void
 */
void throttle(void)
{
	ESBtransmit[0] = 't';
	ESBtransmit[1] = throttle_per;
	while (ESBreceive[0] != 'K'){
		sendToESB(2);
		waitMS(5);
	}
	ESBreceive[0] = 0;                   // clear out the first char so that it actually transmits
}
