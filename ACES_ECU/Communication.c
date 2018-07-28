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
void sendToESB(uint8_t len)
{
	// this will send the number of character in ESBmessage up to len
	cli();
	for(uint8_t i = 0; i < len; i++)
	{
		while ( !( UCSR1A & (1<<UDRE1)) );
		/* Put data into buffer, sends the data */
		UDR1 = ESBtransmit[i];
	}
	sei();
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

	ESBtransmit[0] = 'A';
	ESBtransmit[1] = 'C';
	ESBtransmit[2] = 'E';
	ESBtransmit[3] = 'S';
	sendToESB(4);	
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
	for (uint8_t i = 0; i < 4; i++){                // This will also send the terminator
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) );
		/* Put data into buffer, sends the data */
		UDR0 = message[i];
	}
	connected_GUI = 1;
	doTransmit = -1;
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
	//loadESBData();
	dummyData();    // remove this later
	// This function will be responsible for packaging and sending the desired data to the Windows GUI
	char message[28];              // this is the base length of the message with room for the parity bytes
	// now fill the message
	if (!connected_ESB){
		message[0] = 'b';          // This means that the ESB is not connected or it has lost connection
	}
	else if (opMode == 0){
		message[0] = 'S';          // This means that an engine shutdown is being carried out
	}
	else if(opMode == 1){
		message[0] = 'r';          // This means that an engine startup is in progress
	}
	else if(opMode == 2){
		message[0] = 't';          // This means that a throttle adjustment has been requested and is being carried out
	}
	else if(opMode == 4){
		message[0] = 'C';          // This means that the engine is in the cooling mode
	}
	else if(opMode == 5){
		message[0] = 'n';          // The engine is not doing anything
	}
	else if (opMode == 6){
		message[0] = 'g';          // Special shutdown, the EGT is not working properly and there cannot be a normal cooling mode
	}
	else if (opMode == 7){
		message[0] = 'N';          // This means that the engine has reached the desired throttle, within the error tolerance
	}
	else if (opMode == 9){         // This means that fuel is not flowing when it should be flowing
		message[0] = 'P';	
	}
	else if (opMode == 10){
		message[0] = 'I';          // This means that the engine has reached idle
	}
	else if (opMode == 11){
		message[0] = 'c';          // This means that messages from the ECU have failed the parity check
	}
	else if (opMode == 12){
		message[0] = 'R';          // This means that RPM limit has been reached and the engine is shutting down
	}
	else if (opMode == 13){
		message[0] = 'T';          // This means the Temperature limit has been reached and the engine is shutting down
	}
	else if (opMode == 14){
		message[0] = 's';          // This means that messages from the ESB have failed the parity check
	}
		
	memcpy(message+1,&massFlow,sizeof(float));          // This should fill 1->4 with the float value
	memcpy(message+5,&Hall_effect,sizeof(uint16_t));    // This should fill 5->6 with the Hall effect sensor value
	memcpy(message+7,&EGT,sizeof(float));            // This should fill 7->8 with the EGT value 
	memcpy(message+11,&voltage,sizeof(float));      // this should fill 9->12 with the value of the battery voltage
	memcpy(message+15,&glow_plug,sizeof(char));         // This should fill 13 with the glow plug on/off
	memcpy(message+16,&ECU_temp,sizeof(float));         // This should fill 14->17 with the temperature of the ECU
	memcpy(message+20,&ESB_temp,sizeof(float));         // This should fill 18->21 with the ambient temperature of the ESB
	
	// now that the message is made, I need to calculate and populate the parity bytes
	message[24] = calculateParity(message, 0);
	message[25] = calculateParity(message, 6);
	message[26] = calculateParity(message, 12);
	message[27] = calculateParity(message, 18);
	
	// At this point it is advantageous to turn off global interrupts so that this process is not interrupted
	cli();
	for (uint8_t i = 0; i < 28; i++){
		/* Wait for empty transmit buffer */
		while ( !( UCSR0A & (1<<UDRE0)) );
		/* Put data into buffer, sends the data */
		UDR0 = message[i];
	}
	sei();   // Now need to turn global interrupts back on
	// Now start the timer
	
	TCCR4B = (1 << CS42);      // start timer 4 with prescalar of 256


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
		if (data == 'A'){
			connect_count = 1;
			newCommand = 1;
		}
		else if (data == 'C' && connect_count == 1){
			connect_count++;
			newCommand = 1;
		}
		else if (data == 'E' && connect_count == 2){
			connect_count++;
			newCommand = 1;
		}
		else if (data == 'S' && connect_count == 3){
			connect_count++;
			GUI_Connect();
			newCommand = 1;
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
		else if (data == 'R'){    // This means that the data needs to be sent to the GUI one more time
			newCommand = 1;
			sendToLaptop();   // this will just use what ever the values of the data currently are
		}
		else{
			commandMode = 0;               // This will handle all undefined behavior
			if (!connected_GUI)
				newCommand = 1;     // This was found to work during debugging as there is a 0 waiting in the buffer
		}
	}
	else if (connected_GUI) {
		newCommand = 1;
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

// This interrupt will be triggered whenever data is received from the ESB
ISR(USART1_RX_vect)
{
	uint8_t data = UDR1;
	hasInterrupted = 1;      // set this flag so other functions will know if they have been interrupted
	if (newCommand_ESB == 1)
	{
		ESBreceive[0] = data;
		if (data == 'K'){
			newCommand_ESB = 1;
			ESBreceiveCount = 0;
		}
		else if (data == 'D'){
			newCommand_ESB = 2;       // this means that the ESB is sending the connection string back
			ESBreceiveCount = 0;
		}
		else if (data == 'N'){
			newCommand_ESB = 3;
			ESBreceiveCount = 0;
			TCNT5 = ESB_timer_val;
		}
	}
	else if (newCommand_ESB == 2){
		ESBreceiveCount++;
		switch(ESBreceiveCount){
			case 1:
				if (data != 'A'){
					ESBreceiveCount = 0;
					newCommand_ESB = 1;
				}
				break;
			case 2:
				if (data != 'L'){
					ESBreceiveCount = 0;
					newCommand_ESB = 1;	
				}
				break;
			case 3:
				if (data == 'E'){
					connected_ESB = 1;
					TCCR5B |= (1 << CS52);      // start the ESB connection timer with a prescalar of 256
				}
				ESBreceiveCount = 0;
				newCommand_ESB = 1;
				break;

		}
	}
	else if (newCommand_ESB == 3){                     // This will handle the normal data transmission
		ESBreceiveCount++;
		ESBreceive[ESBreceiveCount] = data;
		if (ESBreceiveCount >= 10){
			ESBreceiveCount = 0;
			newCommand_ESB = 1;                       // This indicates the end of the data string
			loadESBData();
		}
	}
	
}

void packageMessage(void)
{
	ESBtransmit[0] = 'N';
	ESBtransmit[1] = massFlow.c[0];
	ESBtransmit[2] = massFlow.c[1];
	ESBtransmit[3] = massFlow.c[2];
	ESBtransmit[4] = massFlow.c[3];
	ESBtransmit[5] = 0;
	ESBtransmit[6] = calculateParity(ESBtransmit, 0);
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
	UDR0 = 'V';
	repeatCount++;
	
	// Will have to see in unit testing how long this takes to get the response from the GUI
}

void i2c_Start(unsigned char address)
{
	TWCR = (1<<TWSTA) | (1<<TWINT) | (1 << TWEN);   // this will issue the start command, enable TWI, and clear the interrupt flag, if needed
	while (!(TWCR & (1<<TWINT)));    // wait for the start condition to be transmitted 
	if ((TWSR & 0xF8) != 0x08 && (TWSR & 0xF8) != 0x10)
		i2c_Start(address);            // it will call this recursively until the start is transmitted
		
	// Now send the addressing byte 
	TWDR = address;
	TWCR = (1<<TWINT) | (1 << TWEN);    // this will start the data transfer
	
	while (!(TWCR & (1<<TWINT)));    // wait for the address byte to be sent
	
	if ((TWSR & 0xF8) != 0x18 && (TWSR & 0xF8) != 0x40){    // the first is if it is in write mode and the second is if it is in read mode
		i2c_Stop();   // stop the current i2c message string
		i2c_Start(address);  // retry.  This will make an infinite loop if something is wrong
			
	}
		
}

void i2c_Stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1 << TWEN);  // this will issue the stop command
	
}

void i2c_write(unsigned char data)
{
	TWDR = data;
	TWCR = (1<<TWINT) | (1 << TWEN);    // this will start the data transmission
	
	while (!(TWCR & (1<<TWINT)));    // wait for the data to be transmitted
	
	if ((TWSR & 0xF8) != 0x28) {   // this is the hex number that should be received if the acknowledge has been received
	     // fill in this later for error checking	
	}
		
}

unsigned char i2c_read(unsigned char ack)
{
	// write the interrupt bit low and set the appropriate acknowledge bit
	TWCR = (1<<TWINT) | (ack << TWEA) | (1 << TWEN);  
	
	// and now we wait for data
	while (!(TWCR & (1<<TWINT)));
	
	if ((TWSR & 0xF8) != 0x50 && ack) {   // this is the hex number that should be received if the acknowledge has been received
		// fill in this later for error checking for in the acknowledge bit was meant to be sent
	}
	else if ((TWSR & 0xF8) != 0x58 && !ack) {
		// fill in this later for error checking when the NOT acknowledge bit was meant to be sent
	}
	
	return TWDR;
}

void loadESBData(void)
{
	// First need to check the parity bytes 
	char parity1_check = calculateParity(ESBreceive, 0);
	char parity2_check = calculateParity(ESBreceive, 6);
	
	if (parity1_check != ESBreceive[12] || parity2_check != ESBreceive[13]){      // This means that the parity bytes do not match
		shutdown();         // hopefully this message still gets through
		ESBreceive[0] = 12;      // this will set the correct opMode	
	}
	
	// This will convert the values that were recorded from the communication into usable variables
	memcpy(&opMode, ESBreceive+1, sizeof(uint8_t));
	memcpy(&Hall_effect, ESBreceive+2, sizeof(uint16_t));
	memcpy(&EGT, ESBreceive+3, sizeof(float));
	memcpy(&glow_plug, ESBreceive+7, sizeof(uint8_t));
	memcpy(&ESB_temp, ESBreceive+8, sizeof(float));
}