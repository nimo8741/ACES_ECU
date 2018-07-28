/** @file Communication.c
 *  @author Nick Moore
 *  @date March 22, 2018
 *  @brief Function implementation for managing the data sent and received from the ENC28J60 Ethernet Module
 *
 *  @bug Sometimes when using the debugger the ECU will not accept the return string "DALE".  This is confirmed by logic analyzer
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include "ESB_funcs.h"


ISR(TIMER5_OVF_vect)   // This means that it has been too long since data has been received from the ECU
{
	// If it makes it in here then it is assumed that the ECU and ESB have gotten disconnected
	assign_bit(&TCCR5B, CS52, 0);    // turn off the timer for now
	connected = 0;
	shutdown();     // shutdown the engine    don't want to do this for now until the timers are flushed out

}

void package_message(void)
{
	hallEffect = 34567;
	EGT = 12345;
	glowPlug = 1;
	ref_temp = 23456;
	
	ECUtransmit[0] = opMode;
	memcpy(ECUtransmit + 1, &hallEffect, sizeof(uint16_t));
	memcpy(ECUtransmit + 3, &EGT, sizeof(float));
	memcpy(ECUtransmit + 7, &glowPlug, sizeof(uint8_t));
	memcpy(ECUtransmit + 8, &ref_temp, sizeof(float));
	ECUtransmit[12] = calculateParity(ECUtransmit, 0);
	ECUtransmit[13] = calculateParity(ECUtransmit, 6);
	
	hallDone = 0;                          // reset this we have already used the new data
}

ISR(USART0_RX_vect)
{
	// This will get invoked if data is received from the ECU
	uint8_t data = UDR0;
	hasInterrupted = 1;            // set this flag so other functions will know if they have been interrupted
	if (!commandCode)
	{
		
		if (data == 'S' && connected){          // Handles if the ECU wants a shutdown
			//shutdown();
			ECUtransmit[0] = 'K';
			sendToECU(1);
		}
		else if (data == 'r' && connected){     // Handles if the ECU wants an engine startup
			startup();
			ECUtransmit[0] = 'K';
			sendToECU(1);
		}
		else if (data == 't' && connected){     // Handles if the ECU wants a specific throttle
			commandCode = 1;
		}
		else if (data == 'N' && connected){     // Handles if the ECU is sending the normal data
			commandCode = 2;
			TCNT5 = ECU_timer_val;              // phew, made it before the timer overflow
		}
		else if (data == 'A'){     // Handles if the ECU wants to connect with the ESB
			commandCode = 3;
		}	
	}
	else if (commandCode == 1){
		throttle_val = data;
		ECUtransmit[0] = 'K';
		sendToECU(1);
		commandCode = 0;
	}
	else if (commandCode == 2){              // This means the ESB is receiving the normal data from the ECU
		ECUreceiveCount++;
		switch (ECUreceiveCount){
			case 1:       // This means it is the LSB of the flow rate
				massFlow.c[0] = data;
				break;
				
			case 2:
				massFlow.c[1] = data;
				break;
				
			case 3:
				massFlow.c[2] = data;
				break;
				
			case 4:
				massFlow.c[3] = data;
				ECUreceiveCount = 0;
				commandCode = 0;
				break;
		}
	}
	else if (commandCode == 3){         // This means that the ECU is trying to connect with the ESB
		ECUreceiveCount++;
		switch (ECUreceiveCount)
		{
			case 1:
				if (data != 'C'){
					commandCode = 0;
					connected = 0;
					ECUreceiveCount = 0;
				}
				break;
				
			case 2:
				if (data != 'E'){
					commandCode = 0;
					connected = 0;
					ECUreceiveCount = 0;
				}
				break;
				
			case 3:
				if (data == 'S'){
					connected = 1;
					ECUtransmit[0] = 'D';
					ECUtransmit[1] = 'A';
					ECUtransmit[2] = 'L';
					ECUtransmit[3] = 'E';
					sendToECU(4);
					TCNT4 = HallTime * 11;    // This will put the comm lines on off phases, the multiplier was found experimentally
					TCCR5B = (1 << CS52);    // This will start timer 5 with a prescalar of 256, makes 1 second timer
					// This will set a maximum time limit until another message is received from the ECU before assuming a disconnect
					
				}
				else{
					connected = 0;
				}
				commandCode = 0;
				ECUreceiveCount = 0;
				break;
		}
	}
}

void sendToECU(uint8_t len)
{
	// this will send the number of character in ESBmessage up to len
	cli();
	for(uint8_t i = 0; i < len; i++)
	{
		while ( !( UCSR0A & (1<<UDRE0)) );
		/* Put data into buffer, sends the data */
		UDR0 = ECUtransmit[i];
	}
	sei();
}

uint8_t calculateParity(uint8_t message[], uint8_t start_index)
{
	// This will return the parity byte for a message made up of 6 sequential bytes, starting with start_index
	uint8_t parity = 0;
	
	// first need to get the number of high bits in the first three bytes in the set
	for (unsigned set = 0; set < 2; set++){
		unsigned char count = 0;
		for (unsigned char i = 0; i < 3; i++){
			count += countOnes(message[start_index + set*3 + i]);
		}
		// now that I have the count for this set, I need to take the modulo
		count = count % 16;   // Modulo with 16 because I have 4 bits to play with
		
		// now add this into the parity byte
		parity |= count << (4 * set);   // this will make it so that bytes 0-2 will take up the LSB of the parity byte
	}
	// now copy this byte into memory
	return parity;
}

unsigned char countOnes(unsigned char byte)
{
	unsigned char count = 0;
	while (byte)
	{
		count += byte & 1;    // this was essentially lifted from geeksforgeeks.com
		byte >>= 1;
	}
	return count;
}