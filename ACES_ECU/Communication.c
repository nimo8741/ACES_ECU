/** @file Communication.c
 *  @author Nick Moore
 *  @data March 16, 2018
 *  @brief Implementation for the functions relating to the communication of the ECU/ESB and ECU/Windows GUI
 *
 *
 *  @bug No known bugs.
 *  @note Add notes here
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "ECU_funcs.h"

/** @brief Waits until the timer is done for the ESB command cycle and then swaps data with it
 *
 *  This performs the following functions:
 *  
 *  1) Prepares the message string to be sent
 *  2) Sends the message to the ESB in a prescribed order
 *  3) Continues to resend the message to the ESB until it receives the confirmation code in reply
 *  4) Sends the necessary data onto the Windows GUI
 *  5) Resets the timer for use the next time (0.25 sec later)
 *
 *  @param void
 *  @return void
 */
void ESBTransmit(void)
{
	if (bit_is_clear(TIFR1,TOV1))
		return;                     // if the overflow is not set then just return
		
	// Now I need to package all of the data going to the ESB
	// The format will go like this
	// 1) Operation Mode - N for normal
	// 3) Current Mass flow 0 - first byte of the float
	// 4) Current Mass flow 1 - second byte of the float
	// 5) Current Mass flow 2 - third byte of the float
	// 6) Current Mass flow 3 - fourth byte of the float
	
	char message[4];              // this will be the array for the message string
	message[0] = 'N';              // this is N for normal mode
	memcpy(message+1,&massFlow,sizeof(float));     
	
	// Now comes the transmission part of the function
	char receive;
	hasInterrupted = 0;
	for (unsigned char i = 0; i < 6 + ECU_temp_sensors; i++){
		if (i > 3){
			receive = SPI_Transmit(0);               // transmit nothing but need to fill something in the buffer 
		}
		else{
			receive = SPI_Transmit(message[i]);
		}
		
		// Now process the information
		switch (i){
			case 0:
				opMode = receive;
				break;
			case 1:
				Hall_effect = (receive << 8);
				break;
			case 2:
				Hall_effect |= receive;
				break;
			case 3:
				EGT = (receive << 8);
				break;
			case 4:
				EGT |= receive;
				break;
			case 5:
				glow_plug = receive;
				break;
			default:                // This section should never run unless there are temp sensors on the ESB
				if (i % 2){
					ESB_temp[i - 5] = (receive << 8);
				}
				else{
					ESB_temp[i - 5] |= receive;
				}
		}
		if (hasInterrupted){
			i = 0;   // restart the loop from the beginning because the message got interrupted
			continue;
		}
	}
	sendToLaptop();     // now send all of the data to the laptop
	
	// Since the overflow flag is not clear, we need to clear it by writing a 1 to it
	TIFR1 |= (1 << TOV1);
	TCNT1 = 3036;                 // reset the timer register
		
}

/** @brief Transmits data between the ECU and ESB
 *
 *  This performs the following functions:
 *  
 *  1) Pulls the SS line for the ESB low, this will cause the ESB to listen to the com line
 *  2) Fills the specified data into the transmit buffer
 *  3) Waits for the data to be received from the ESB
 *  4) Pulls the SS line high to end communication with ESB for that byte
 *
 *  @param char The byte that is going to be sent to the ESB
 *  @return char The received byte from the ESB
 */
char SPI_Transmit(char info)
{
	// First wait for an empty transmit buffer
	while (! (UCSR1A & (1 << UDRE1)) );
	
	// Then drive the SS low
	assign_bit(&PORTC, ESB_SS, 0);           // Pull low to start the communication
	
	// Now begin timer 5
	TCCR5B = (1 << CS51) | (1 << CS50);      // start timer 5 with prescalar of 64

	// Now put the data into the buffer, this should send the data too
	UDR1 = info;
	
	// Now wait for the data to be received
	while ( !(UCSR1A & (1 << RXC1)) );
	assign_bit(&PORTC, ESB_SS, 1);           // Pull high to stop communication
	assign_bit(&TCCR5B, CS51, 0);
	assign_bit(&TCCR5B, CS50, 0);            // turn off the timer
	TCNT5 = 40536;
	return UDR1;
}

/** @brief Establishes the connection between the ECU and ESB
 *
 *  This performs the following functions:
 *  
 *  1) Prepares the ECU's password for transmission
 *  2) Sends the password to the ESB
 *  3) Awaits the correct return password from the ESB
 *  4) repeats steps 2-4 until the proper return password is received
 *
 *  @param void
 *  @return void
 */
void ESB_Connect(void)
{
	char message[] = "ACES";
	char receive[5];              // this is the receive buffer
	char done = 0;
	while (!done){
		for (unsigned char i = 0; i <= strlen(message); i++){   // the equals sign will ensure that the terminator is sent
			receive[i] = SPI_Transmit(message[i]);
		}
		// now compare to the pass phrase
		if (!strcmp(receive, "DALE")){
			done = 1;
		}
	}
	
}

/** @brief Establishes the connection between the ECU and Windows GUI
 *
 *  This performs the following functions:
 *  
 *  1) Prepares the ECU's password for transmission
 *  2) Sends the password to the Windows GUI
 *
 *  @param void
 *  @return void
 */
void GUI_Connect(void)
{
	char message[] = "DALE";
	for (uint8_t i = 0; i < 5; i++){                // This will also send the terminator
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) );
		/* Put data into buffer, sends the data */
		UDR0 = message[i];
	}
	connected = 1;
}

/** @brief Transmits periodic data between the ECU and Windows GUI
 *
 *  This performs the following functions:
 *  
 *  1) Prepares the message to send to the Windows GUI
 *  2) Loops through this message, sending it one byte at a time
 *
 *  @param void
 *  @return void
 */
void sendToLaptop(void)
{
	// This function will be responsible for packaging and sending the desired data to the Windows GUI
	char message[14];              // this is the base length of the message
	// now fill the message
	
	if (opMode == 0){
		message[0] = 'H';          // This means that the engine is just sitting there not doing anything
	}
	else if(opMode == 1){
		message[0] = 'S';          // This means that the engine is trying to stop
	}
	else if(opMode == 2){
		message[0] = 'r';          // This means that the engine is trying to start
	}
	else if(opMode == 3){
		message[0] = 'N';          // This means that the engine is running nominally
	}
	else if(opMode == 4){
		message[0] = 't';          // This means that the engine is changing to desired throttle
	}
	else if(opMode == 5){
		message[0] = 'E';          // This means that the ESB has been disconnected
	}
	
	memcpy(message+1,&massFlow,sizeof(float));         // This should fill 1->4 with the float value
	memcpy(message+5,&Hall_effect,sizeof(float));   // This should fill 5->6 with the Hall effect sensor value
	memcpy(message+7,&EGT,sizeof(float));           // This should fill 7->8 with the EGT value 
	memcpy(message+9,&voltageFinal,sizeof(float));
	memcpy(message+13,&glow_plug,sizeof(char));         // This should fill 9 with the glow plug on/off
	
	for (uint8_t i = 0; i < 14; i++){
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) );
		/* Put data into buffer, sends the data */
		UDR0 = message[i];
	}
	// Now start the timer
	TCCR4B = (1 << CS41) | (1 << CS40);      // start timer 5 with prescalar of 64

}

/** @brief Interrupt Service Routine for when a message has been received from the Windows GUI
 *
 *  This performs the following functions:
 *  
 *  1) Sets the newCommand variable based on which type of command is being received
 *  2) Awaits the second a later bytes depending on which type of command is being received
 *  3) If a received command deviated from the expected format, the command is asked to be repeated
 *
 *  commandMode = 0 handles all undefined behavior
 *  commandMode = 1 corresponds to the GUI ordering the ECU to do something (starting or stopping)
 *  commandMode = 2 corresponds to the GUI requesting that the throttle be changed to a specified value (in percent of max)
 *
 *  @param void
 *  @return void
 */
ISR(USART0_RX_vect)
{
	// This will automatically clear the interrupt flag
	char data = UDR0;
	hasInterrupted = 1;                    // Set this flag so that the ESBCommand function knows if it has been interrupted or not
	if (newCommand){
		newCommand = 0;                    // reset this so that the commandMode cannot change until the command string is done
		if (data == 'C'){
			newCommand = 1;
			GUI_Connect();
		}
		else if (data == 'O'){
			commandMode = 1;               // This means the GUI is ordering the ECU to do something
		}
		else if (data == 'T'){
			commandMode = 2;               // This means the GUI is requesting the ECU change the throttle to a specified value
		}
		else if (data == 'K'){             // This means the GUI is sending a confirmation message for receiving the usual data transfer
			newCommand = 1;
			// stop timer 4
			assign_bit(&TCCR4B, CS41, 0);
			assign_bit(&TCCR4B, CS40, 0);            // turn off the timer
			TCNT4 = 40536;                           // reset the timer register
		}
		else{
			commandMode = 0;               // This will handle all undefined behavior
		}
	}
	else if (connected) {
		switch (commandMode){
			case 0:
				repeatCommand();           // repeat the command because something got screwed up  This isn't right because I don't do it over spi, do it over USART
				break;
			case 1:
				if (data == 'S'){          // GUI wants the engine to stop
					shutdown();
				}
				else if (data == 'r'){     // GUI wants the engine to start
					startup();
				}
				else{
					repeatCommand();       // undefined behavior, ask GUI to repeat
				}
				break;
			case 2:
				throttle_per = data;       // GUI wants the throttle to the value specified by data
				throttle();
				break;
		}
		newCommand = 1;                    // reset this so that a new command will be accepted in the way that is expected
		
	}
}

/** @brief Requests the Windows GUI to repeat the last sent command
 *
 *  This performs the following functions:
 *  
 *  1) Sends the char to the GUI which signals a command repetition is required
 *
 *  @param void
 *  @return void
 */
void repeatCommand(void)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	
	/* Put data into buffer, sends the data */
	UDR0 = 'R';
	
	// Will have to see in unit testing how long this takes to get the response from the GUI
}