/*
 * enc28j60.c
 *
 * Created: 6/21/2018 11:04:03 AM
 *  Author: Nick
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "Ethernet.h"
#include "ECU_funcs.h"

static uint8_t ECU_mac[] = {0x46,0x55,0x43,0x4B,0x45,0x52};
static uint8_t ESB_mac[] = {0x41,0x53,0x53,0x48,0x41,0x54};

uint8_t readBasic(uint8_t op, uint8_t address)
{
	CSACTIVE;
	// Now tell the chip that I am trying to read
	SPDR = op | (address & ADDR_MSK);     // this will make it so only the last 5 bits are preserved of the address
	waitSPI();                            // wait for the SPI interrupt flag to be set
	
	// read the data
	SPDR = 0;                            
	waitSPI(); 
	 
	// If the address is a MAC or MII register then the byte which will be picked up by SPRD
	// will be dummy data
	if (address & 0x80){
		SPDR = 0;
		waitSPI();
	}
	CSPASSIVE;                           // raise the CS line
	return SPDR;
}

void writeBasic(uint8_t op, uint8_t address, uint8_t data)
{
	CSACTIVE;
	SPDR = op | (address & ADDR_MSK);    // this will tell the Ethernet chip where to write the data
	waitSPI();
	
	// write the data
	SPDR = data;
	waitSPI();
	CSPASSIVE;
}

void readBuffer(uint8_t len, uint8_t *data)
{
	// First pull the CS line low
	CSACTIVE;
	// Now issue the read buffer memory command
	SPDR = READ_BUF_MEM;
	waitSPI();                // Now wait for the message to be sent
	
	while(len)
	{
		len--;
		SPDR = 0;            // Just give the register some data so that it begins the transmission
		waitSPI();
		*data = SPDR;
		data++;
	}
	*data = 0;               // Conclude the data with a null terminator
	CSPASSIVE;               // Raise the CS line to stop the transmission  
	
}

void writeBuffer(uint8_t len, uint8_t *data)
{
	CSACTIVE;                 // First pull the CS line low
	SPDR = WRITE_BUF_MEM;     // Issue the write buffer command
	waitSPI();
	
	while (len)
	{
		len--;
		SPDR = *data;         // Send the current byte of data
		waitSPI();
		data++;
		
	}
	CSPASSIVE;                // Raise the CS line to stop the transmission 
	
}

void setBank(uint8_t address)
{
	// This will set the bank to whichever bank *address* is in
	if (((address & BANK_MASK) >> 5) != bankNumber){
		writeBasic(BIT_FIELD_CLR, ECON1, (ECON1_BSEL0 | ECON1_BSEL1));     // First clear the bank from the relevant register
		writeBasic(BIT_FIELD_SET, ECON1, (address & BANK_MASK) >> 5);      // Get the 2 bits by themselves and then shift to have it read the correct number
		bankNumber = (address & BANK_MASK) >> 5;                           // Save the bank number for future comparison
	}
	
}

uint8_t RegisterRead(uint8_t address)
{
	// first set the bank
	setBank(address);
	// Now read the data from the register
	return readBasic(READ_CONTROL_REG, address);
}

void RegisterWrite(uint8_t address, uint8_t data)
{
	// First set the bank
	setBank(address);
	// now write the data to the register
	writeBasic(WRITE_CONTROL_REG, address, data);
}

uint16_t PhyRead(uint8_t address)
{
	// First I need to set the PHY register address
	RegisterWrite(MIREGADR, address);
	
	// Now set the MICMD.MIIRD bit
	writeBasic(BIT_FIELD_SET, MICMD, MICMD_MIIRD);
	
	// now wait for at least 10.24 us
	waitMS(15);
	
	// Now clear the MIIRD bit
	writeBasic(BIT_FIELD_CLR, MICMD, MICMD_MIIRD);
	
	uint16_t data = 0;
	data |= RegisterRead(MIRDL);
	data |= (RegisterRead(MIRDH) << 8);
	
	return data;
}

void PhyWrite(uint8_t address, uint16_t data)
{
	// first need to set the PHY register address
	RegisterWrite(MIREGADR, PHLCON);
	// Write the PHY data
	RegisterWrite(MIWRL, data & 0xFF);
	RegisterWrite(MIWRH, data >> 8);
		
	// now need to wait for 10.24 us, but I will just poll for the end of the waiting time after 1 waiting cycle
	while(RegisterRead(MISTAT) & MISTAT_BUSY)
		waitMS(15);
	
}

void InitPhy(void)
{
	// Turn the LEDs on
	PhyWrite(PHLCON, LED_on);             // Turn both LEDs on
	waitMS(500);
	// Turn the LEDs off
	PhyWrite(PHLCON, LED_off);
	waitMS(500);
	
	// Turn the LEDs back on
	PhyWrite(PHLCON, LED_on);
	waitMS(500);
	
	// Turn LEDs off again
	PhyWrite(PHLCON, LED_off);
	waitMS(500);
	
	// Set the LEDs for normal operation
	// This means that LEDA (green) will correspond to link status
	// And LEDB (yellow) will correspond to receive/transmit
	PhyWrite(PHLCON, LED_normal);
	waitMS(100);                     // The example code has this in here, don't know if I really need it
}

void waitMS(uint16_t msec)
{
	// This function utilizes timer 0 and a sequence of delay loops to delay for the desired time
	TCNT0 = 5;
	// begin the timer
	TCCR0B |= (1 << CS01) | (1 << CS00);       // this will start the timer with a prescalar of 64
	for (uint16_t i = 0; i < msec; i++){
		while(bit_is_clear(TIFR0, TOV0));
		TIFR0 |= (1 << TOV0);                  // Clear the overflow flag by writing a 1 to it
	}
	assign_bit(&TCCR0B, CS01, 0);
	assign_bit(&TCCR0B, CS00, 0);              // Turn off the timer
}

void InitEthernet(void)
{
	// First perform a system reset.  This will ensure register start with their expected values
	readBasic(SOFT_RESET,SOFT_RESET);        // normally read returns something but this will serve as a write with fewer operations
	waitMS(50);                              // give the chip time to restart
	
	////////// Initialize the Receive and Transmit Buffers /////////////////
	nextPacketPtr = RXSTART_INIT;
	
	// Rx start
	RegisterWrite(ERXSTL, RXSTART_INIT & 0xFF);        // Low byte first and then high byte
	RegisterWrite(ERXSTH, RXSTART_INIT >> 8);
	
	// Now set the current receive pointer address
	RegisterWrite(ERXRDPTL, RXSTART_INIT & 0xFF);
	RegisterWrite(ERXRDPTH, RXSTART_INIT >> 8);   // Again, low byte first
	
	// RX end
	RegisterWrite(ERXNDL, RXSTOP_INIT & 0xFF);    // doing the 0xFF will trim off the higher byte
	RegisterWrite(ERXNDH, RXSTOP_INIT >> 8);
	
	// TX Start
	RegisterWrite(ETXSTL, TXSTART_INIT & 0xFF);
	RegisterWrite(ETXSTH, TXSTART_INIT >> 8);
	
	// TX end
	RegisterWrite(ETXNDL, TXSTOP_INIT & 0xFF);
	RegisterWrite(ETXNDH, TXSTOP_INIT >> 8);
	
	// Now configure the packet filters
	// Everything will be filtered with Unicast
	
	//RegisterWrite(ERXFCON, ERXFCON_UCEN | ERXFCON_ANDOR); 
	
	// The above line will ignore all packets which don't have the destination address
	// matching this device's MAC address
	
	// Now enable the MAC and other features
	// This will enable the MAC to receive packets
	// allow MAC to transmit pause control frame
	// and inhibit transmission when pause control frames are received
	RegisterWrite(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
	RegisterWrite(MACON2, 0);     // this will cause the MAC to enter "normal operation" as per the data sheet p61
	
	// Now do Inter-Packet gap (not back-to-back)
	RegisterWrite(MAIPGL, 0x12);      // This was what the data sheet recommended (p34)
	RegisterWrite(MAIPGH, 0x0C);      // Also what the data sheet recommended
	
	// Set the maximum packet size
	RegisterWrite(MAMXFLL, MAX_FRAMELEN & 0xFF);
	RegisterWrite(MAMXFLH, MAX_FRAMELEN >> 8);
	
	
	// Now write the MAC Address to the device
	// Note, the ENC28J60 is byte backwards, at least that is the consensus on the Internet
	RegisterWrite(MAADR5, ECU_mac[0]);
	RegisterWrite(MAADR4, ECU_mac[1]);
	RegisterWrite(MAADR3, ECU_mac[2]);
	RegisterWrite(MAADR2, ECU_mac[3]);
	RegisterWrite(MAADR1, ECU_mac[4]);
	RegisterWrite(MAADR0, ECU_mac[5]);
	
	// Now prevent loop back of transmitted frames
	PhyWrite(PHCON2, PHCON2_HDLDIS | PHCON2_FRCLINK);
	
	// enable packet reception
	writeBasic(BIT_FIELD_SET, ECON1, ECON1_RXEN);
	
	// now enable the interrupt flags
	writeBasic(BIT_FIELD_SET, EIE, EIE_PKTIE | EIE_TXERIE | EIE_RXERIE);
	// The above line will enable the interrupt flags for Receive Packet Pending, Transmit, Transmit error, and receive packet error	
}

void packetSend(uint8_t len, uint8_t* packet)
{
	// Set the Transmit start pointer to the same location as the write location
	RegisterWrite(ETXSTL, TXSTART_INIT & 0xFF);
	RegisterWrite(ETXSTH, TXSTART_INIT >> 8);
	
	// Set the write pointer to start of transmit buffer area
	RegisterWrite(EWRPTL, TXSTART_INIT & 0xFF);
	RegisterWrite(EWRPTH, TXSTART_INIT >> 8);
	
	// Set the TXND pointer to correspond to the packet size given
	RegisterWrite(ETXNDL, (TXSTART_INIT + len) & 0xFF);
	RegisterWrite(ETXNDH, (TXSTART_INIT + len) >> 8);
		
	// copy the packet into the transmit buffer
	writeBuffer(len, packet);
	
	uint8_t len2 = 20;
	uint8_t test[20];
	test[0] = 0;
	RegisterWrite(ERDPTL, TXSTART_INIT & 0xFF);
	RegisterWrite(ERDPTH, TXSTART_INIT >> 8);
	readBuffer(len2,test);	
	
	// Clear the appropriate interrupt flags
	writeBasic(BIT_FIELD_CLR, EIR, EIR_TXIF | EIR_TXERIF);     // clear the transmit flags
	
	// send the contents of the transmit buffer onto the network
	writeBasic(BIT_FIELD_SET, ECON1, ECON1_TXRTS);
	
	// Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
	if((RegisterRead(EIR) & EIR_TXERIF))
	{
		writeBasic(BIT_FIELD_CLR, ECON1, ECON1_TXRTS);
	}
				
}

uint16_t packetRecieve(uint8_t max_len, uint8_t *packets)
{
	uint16_t rxstat;
	uint16_t len;
	
	// Check to see if a packet has been received, if not, then return
	if (!RegisterRead(EPKTCNT))
		return(0);
		
	// Set the read pointer to the start of the received packet
	RegisterWrite(ERDPTL, nextPacketPtr);
	RegisterWrite(ERDPTH, nextPacketPtr >> 8);
	
	// Read the next packet pointer (see page 43 in data sheet)
	nextPacketPtr = readBasic(READ_BUF_MEM, 0);
	nextPacketPtr |= readBasic(READ_BUF_MEM, 0) << 8;
	// Reading from the memory buffer automatically increments to the next byte
	
	// read the packet length
	len = readBasic(READ_BUF_MEM, 0);
	len |= readBasic(READ_BUF_MEM, 0) << 8;
	// in the example, 4 is subtracted from this but I don't have CRC checking implemented since I don't think it is necessary
	
	// read the receive status
	rxstat = readBasic(READ_BUF_MEM, 0);
	rxstat |= readBasic(READ_BUF_MEM, 0) << 8;
	
	// limit the receive length
	if (len > max_len)
		len = max_len;
		
	// check for symbol errors
	if (!(rxstat & 0x80)){
		// this is an invalid packet
		len = 0;
	}
	else{
		readBuffer(len, packets);
	}
	
	// move the RX pointer to the start of the next received packet
	// This frees the memory we just read out
	RegisterWrite(ERXRDPTL, nextPacketPtr);
	RegisterWrite(ERXRDPTH, nextPacketPtr >> 8);
		
	// decrement the packet counter indicate we are done with this packet
	writeBasic(BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
	return(len);
}

void transmitHeader(uint8_t *buffer)
{
	uint8_t i = 0;
	buffer[0] = 0;    // This will hold the per-packet control byte
	//copy the destination mac from the source and fill my mac into src
	while(i < 6)
	{
		buffer[DEST_MAC + i + 1] = ESB_mac[i];
		buffer[SRC_MAC + i + 1] = ECU_mac[i];
		i++;
	}
}

uint8_t loadData(uint8_t *fullMessage, uint8_t *message, uint8_t len)
{
	uint8_t i = 0;
	
	// First load the length of the message
	fullMessage[LEN_INDEX] = len + 15;   // This does the two MAC addresses, size bytes, and per-packet control byte
	fullMessage[LEN_INDEX + 1] = 0;    // Since I know the length will be much less than 256
	
	// Next load the MAC Addresses into the full message
	transmitHeader(fullMessage);
	
	// Now load the data from the message into the full message
	while (i < len)
	{
		fullMessage[LEN_INDEX + 2 + i] = message[i];
		i++;
	}
	return (15 + len);
}

void readTX_StatusVec(uint8_t *Status)
{
	// This will read the 7 bytes which are located at ETXND+1
	// First need to save the location of the read pointer
	uint16_t savePointer = RegisterRead(ERDPTL);
	savePointer |= RegisterRead(ERDPTH) << 8;
	
	// Now get the current value of ETXND
	uint16_t curTX_end = RegisterRead(ETXNDL);
	curTX_end |= RegisterRead(ETXNDH) << 8;
	curTX_end++;
	
	// Now set the read pointer
	RegisterWrite(ERDPTL, curTX_end & 0xFF);
	RegisterWrite(ERDPTH, curTX_end >> 8);
	
	// Now read the buffer 7 times
	readBuffer(7, Status);
	
	// Now put the read pointer back
	RegisterWrite(ERDPTL, savePointer & 0xFF);
	RegisterWrite(ERDPTH, savePointer >> 8);	
	

}