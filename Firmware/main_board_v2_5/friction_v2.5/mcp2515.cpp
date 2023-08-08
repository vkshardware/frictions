/*
* MCP2515.c
*
* Created: 7/13/2020 12:39:16 PM
* Author: XINGHE_THINKPAD 
* adapted by vks 2023
*/
#include "MCP2515.h"


/*******************************************************************
* function: reset mcp2515
* input: none
* output: none
*******************************************************************/
void mcp2515_reset(void)
{
	MCP2515_SELECT();
	spi_master_transmit(RESET_INSTRUCTION);
	_delay_ms(1);
	MCP2515_UNSELECT();
	_delay_ms(100);
}
/*******************************************************************
* function: mcp2515 read one char
* input: const unsigned char address
* output: unsigned char
*******************************************************************/
unsigned char readRegister(const unsigned char address)
{
	unsigned char ret;
	MCP2515_SELECT();
	spi_master_transmit(READ_INSTRUCTION);
	spi_master_transmit(address);
	ret = spi_read();
	MCP2515_UNSELECT();
	return ret;
}
/*******************************************************************
* function: mcp2515 read one char from RX
* input: const unsigned char address (0x02 RXB0D0)
* output: unsigned char
*******************************************************************/
unsigned char readRXRegister(const unsigned char address)
{
	unsigned char ret;
	MCP2515_SELECT();
	spi_master_transmit(READRX_INSTRUCTION | address);
	ret = spi_read();
	MCP2515_UNSELECT();
	return ret;
}
unsigned char readSTATUSRegister(void)
{
	unsigned char ret;
	MCP2515_SELECT();
	spi_master_transmit(READSTAT_INSTRUCTION);
	ret = spi_read();
	ret = spi_read();
	MCP2515_UNSELECT();
	return ret;
}

/*******************************************************************
* function: mcp2515 read one string
* input: const unsigned char address, unsigned char values[], const unsigned char n
* output: void
*******************************************************************/
void mcp2515_readRegisterS(const unsigned char address, unsigned char values[], const unsigned char n)
{
	unsigned char i;
	MCP2515_SELECT();
	spi_master_transmit(READ_INSTRUCTION);
	spi_master_transmit(address);
	for (i=0; i<n; i++) values[i] = spi_read();
	MCP2515_UNSELECT();
}
/*******************************************************************
* Change value of the register on selected address inside the
* MCP2515. Works for every register.
*
* \see MCP2515 datasheet, chapter 11 - register description
* \see MCP2515 datasheet, chapter 12 - write instruction
* \param address Register address
* \param value New value of the register
*******************************************************************/
void writeRegister(const unsigned char address, const unsigned char value)
{
	MCP2515_SELECT();
	spi_master_transmit(WRITE_INSTRUCTION);
	spi_master_transmit(address);
	spi_master_transmit(value);
	MCP2515_UNSELECT();
}

void mcp2515_setRegisterS(const unsigned char address, const unsigned char values[], const unsigned char n)
{
	unsigned char i;
	MCP2515_SELECT();
	spi_master_transmit(WRITE_INSTRUCTION);
	spi_master_transmit(address);
	for (i=0; i<n; i++) spi_master_transmit(* values);
	MCP2515_UNSELECT();
}

/* Configuration routine */
uint8_t initMCP2515(uint8_t baudrate)
{
	unsigned char c_sended;
	
	/* Initialize SPI as a master device, on frequency 16M/16 = 1M */
	spi_master_init(0, SPI_CLOCK_DIV16);

	/* Send reset instruction */
	mcp2515_reset();
	setMode(CONFIGURATION_MODE);
	
	c_sended = readRegister(CANSTAT);

	
	if (getMode == CONFIGURATION_MODE) {

		c_sended = readRegister(CANSTAT);
	
		changeBits(CANCTRL, (7 << REQOP0), (CONFIGURATION_MODE << REQOP0));
		_delay_ms(100);
	
		/* Set bit timing , masks and rollover mode*/
	
		//Set CFN1=0x03 CFN2=0xEA CFN3=0x45  - 16MHz oscillator,
		//CAN baudrate = Fosc/(2*(BRP+1)*16)
		//BRP=0x03, CAN baudrate = 125kbps
		//BRP=0x01, CAN baudrate = 250kbps
		//BRP=0x00, CAN baudrate = 500kbps
	
		//setBitTiming(0x03,0xEA,0x45);
		setBitTiming(baudrate,(1 << BTLMODE) | (1 << SAM) | (1 << PHSEG12) | (1 << PHSEG10) | (1 << PRSEG1), (1 << WAKFIL) | (1 << PHSEG22) | (1 << PHSEG20));
		
		//c_sended = readRXRegister(CNF1);
		//c_sended = readRXRegister(CNF2);
		//c_sended = readRXRegister(CNF3);
		

		// Disable pins RXnBF pins (High Impedance State)
		//writeRegister( BFPCTRL, 0x00 );

		//Switch TXnRTS bits as inputs
		//writeRegister( TXRTSCTRL, 0x00 );

		//setMask(RXM0, 0x00000000, 1);
		//setMask(RXM1, 0x00000000, 1);
		//setRollover(1);
		// enable interrupt
		//writeRegister(CANINTE , 0x1F);
		//c_sended = readRegister(CANINTE);
		
		/* Get into normal mode and setup communication */
		setMode(NORMAL_OPERATION_MODE);
		// open clock output under normal operation mode
		//writeRegister( CANCTRL, 0x05 );

		//c_sended = readRegister(CANCTRL);

			
		return 0;
	}

	
	return c_sended;
}
/** Send a CAN message
* \param bi transmit buffer index
* \param id message identifier
* \param data pointer to data to be stored
* \param prop message properties, the octet has following structure:
* - bits 7:6 - message priority (higher the better)
* - bit 5 - if set, message is remote request (RTR)
* - bit 4 - if set, message is considered to have ext. id.
* - bits 3:0 - message length (0 to 8 bytes) */
void sendCANmsg(unsigned char bi,unsigned long id,unsigned char * data, unsigned char prop)
{

	/* Initialize reading of the receive buffer */
	MCP2515_SELECT();
	/* Send header and address */
	spi_master_transmit(WRITE_INSTRUCTION);
	spi_master_transmit(TXBnCTRL(bi));
	/* Setup message priority */
	spi_master_transmit(prop >> 6);
	/* Setup standard or extended identifier */
	if(prop & 0x10)
	{
		spi_master_transmit((unsigned char)(id>>3));
		spi_master_transmit((unsigned char)(id<<5)|(1<<EXIDE)|((unsigned char)(id>>27)));
		spi_master_transmit((unsigned char)(id>>19));
		spi_master_transmit((unsigned char)(id>>11));
	}
	else
	{
		spi_master_transmit((unsigned char)(id>>3));
		spi_master_transmit((unsigned char)(id<<5));
		
		spi_master_transmit(0x00);
		spi_master_transmit(0x00);
	}
	/* Setup message length and RTR bit */
	spi_master_transmit((prop & 0x0F) | ((prop & 0x20) ? (1 << RTR) : 0));
	/* Store the message into the buffer */
	for(unsigned char i = 0; i < (prop & 0x0F); i++)
	{
		spi_master_transmit(data[i]);
	}
	/* Release the bus */
	MCP2515_UNSELECT();
	/* Send request to send */
	//sendRTS(bi);
	// CAN message sending by Buffer 0 RST action
	MCP2515_SELECT();
	spi_master_transmit(RTS_INSTRUCTION | 0x01);
	MCP2515_UNSELECT();
}

void can_send_one_message (void)
{
	// clear this bit to request a message abort
	changeBits(TXB0CTRL, 0x08, 0);

	// ID Sending Buffer 0
	writeRegister(TXB0SIDH,0x01);
	writeRegister(TXB0SIDL,0x23);

	// Message lenght Buffer 0
	writeRegister(TXB0DLC,0x02);

	// Data sending

	writeRegister(TXB0D0,0x05);
	writeRegister(TXB0D1,0xF3);
	//unsigned char c_sended;
	// c_sended = readRegister(TXB0D0);
	// USART_printf("%d ", c_sended);
	// c_sended = readRegister(TXB0D1);
	// USART_printf("%d\r\n", c_sended);



	// CAN message sending by Buffer 0 RST action
	MCP2515_SELECT();
	spi_master_transmit( (RTS_INSTRUCTION | 0x01) );
	//
	//changeBits(TXB0CTRL, 1<<TXREQ, 1);
	//spi_master_transmit(0x01);
	MCP2515_UNSELECT();
}

static unsigned char * msgReceived = 0;
static unsigned char rbuffer[2][14]; /* 2 RX buffers, each have 14B */


void interruptMCP2515(void)
{
	/* get receive buffer index (we don't consider that both buffer contain
	message, this situation in our environment cannot happen – message is
	directly copied from the buffer and released in this very IRQ ) */
	unsigned char bi = getRXState() >> 6;
	/* Copy the message from the device and release buffer */
	spi_master_transmit(READ_INSTRUCTION);
	spi_master_transmit(RXBnCTRL(bi));
	/* Make the local copy */
	for(unsigned char i = 0; i < 14; i++)
	rbuffer[bi][i] = spi_read();
	msgReceived = &rbuffer[bi][0];
	
	
}

uint8_t getData(int index)
{
	return msgReceived[6+index];
}

uint8_t getId()
{
	return (uint8_t) ((msgReceived[1]<<3)|(msgReceived[2]>>5));
}

uint8_t getLength()
{
	return (uint8_t) (msgReceived[5] >> 4);
}

unsigned char getRXState(void)
{
	unsigned char ret;
	// CAN message sending by Buffer 0 RST action
	spi_master_transmit(RXSTAT_INSTRUCTION);
	ret = spi_read();
	return ret;
}



unsigned char setBitTiming(unsigned char rCNF1, unsigned char rCNF2, unsigned char rCNF3)
{
	writeRegister(CNF1, rCNF1);
	writeRegister(CNF2, rCNF2);
	changeBits(CNF3, 0x07, rCNF3);
	return 1;
}

void changeBits(unsigned char address, unsigned char mask ,unsigned char value)
{
	// Send bit modify instruction, address, mask and data
	MCP2515_SELECT();
	spi_master_transmit(BITMODIFY_INSTRUCTION);
	spi_master_transmit(address);
	spi_master_transmit(mask);
	spi_master_transmit(value);
	MCP2515_UNSELECT();
}
/** Set up acceptance filters/masks
* \param address starting address of 4 registers to setup. It can be mask
* or filter, doesn't matter
* \param criterion message identifier criterion to be set
* \param is_ext 1 if message is extended, otherwise 0 */
void setAcceptanceCriteria (unsigned char address,
unsigned long criterion,
unsigned char is_ext)
{
	/* Initialize reading of the receive buffer */
	MCP2515_SELECT();
	/* Send header and address */
	spi_master_transmit(WRITE_INSTRUCTION);
	spi_master_transmit(address);
	/* Setup standard or extended identifier */
	if(is_ext)
	{
		spi_master_transmit((unsigned char)(criterion>>3));
		spi_master_transmit( (unsigned char)(criterion<<5)
		|(1<<EXIDE)|((unsigned char)(criterion>>27)) );
		spi_master_transmit((unsigned char)(criterion>>19));
		spi_master_transmit((unsigned char)(criterion>>11));
	}
	else
	{
		spi_master_transmit((unsigned char)(criterion >> 3));
		spi_master_transmit((unsigned char)(criterion << 5));
    	spi_master_transmit(0x00);
        spi_master_transmit(0x00);
	}
	/* Release the bus */
	MCP2515_UNSELECT();
}


// static functions
void MCP2515_SELECT(void)
{
	PORTC &= ~nCS;
	return;
}
void MCP2515_UNSELECT(void)
{
	PORTC |= nCS;
	return;
}