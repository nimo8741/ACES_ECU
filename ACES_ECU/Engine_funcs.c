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
	char receive = 'P';                        // This is a letter that isn't K and so it will enter the loop
	hasInterrupted = 0;
	while (receive != 'K'){                    // Loop until the ESB says K
		receive = SPI_Transmit('S');
		receive = SPI_Transmit(0);          // This last message should have the K
	}
	if (hasInterrupted){
		shutdown();                            // just keep calling this function until it doesn't get interrupted
	}
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
	char receive = 'P';                           // This is a char that isn't K and so it will enter the loop
	hasInterrupted = 0;
	while (receive != 'K'){
		receive = SPI_Transmit('r');   
		receive = SPI_Transmit(0);            // This last message should have the K
	}
	// The reason this is a lower case r is so that a single bit error
	// won't make this look like an "S"
	if (hasInterrupted){
		startup();                               // just keep calling this function until it doesn't get interrupted
	}
	
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
	char message[1];
	memcpy(message,&throttle_per,sizeof(uint8_t));      // copy over the memory into an array
	
	char recieve = '0';                                 // This is a char that isn't K and so will enter the loop
	hasInterrupted = 0;
	while (recieve != 'K'){
		recieve = SPI_Transmit('t');
		recieve = SPI_Transmit(message[0]);
		recieve = SPI_Transmit(0);                   // This last message should have the K
	}
	if (hasInterrupted){
		throttle();                                     // Just keep calling this function until it isn't interrupted
	}
}
