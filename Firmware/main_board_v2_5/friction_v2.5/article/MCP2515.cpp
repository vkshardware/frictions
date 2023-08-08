/*
 * MCP2515.cpp
 *
 * Created: 23.12.2022 13:38:12
 *  Author: vks
 */ 


#include "avr/io.h"
#include "MCP2515.h"
#include "avr/interrupt.h"

#define DDR_SPI DDRB /* Data dir. register for port with SPI */
#define PORT_SPI PORTB /* Port with SPI */
#define PIN_MOSI PB5 /* MOSI pin on the PORTB_SPI */
#define PIN_MISO PB6 /* MISO pin on the PORTB_SPI */
#define PIN_SCK PB7/* SCK pin on the PORTB_SPI */
#define PIN_SS PB4 /* SS pin on the PORTB_SPI */

#define getMode (readRegister(CANSTAT) >> 5)
#define setMode(mode) { changeBits(CANCTRL, (7 << REQOP0), (mode << REQOP0)); while(getMode != mode); }

#define setRollover(v) changeBits(RXB0CTRL, 1 << BUKT, v << BUKT);



/** \brief Initialization of the SPI interface on the MCU
*
* Initialization of the SPI hardware interface - configure this
* device as master, set mode 0,0 and the communication speed (there
* is limitation - 10Mhz, nominal speed should be >8Mhz, for this
* purpose.
*
* \warning This is platform-dependent method!
*
*/
void spiMasterINIT()
{
	/* Set MOSI and SCK output, all others input */
	DDR_SPI = (1<<PIN_MOSI)|(1<<PIN_SCK)|(1<<PIN_SS);
	PORT_SPI |= (1 << PIN_SS);
	/* Enable SPI, Master, set clock rate fck/4, mode 0,0 */
	SPCR = (1<<SPE) | (1<<MSTR);
	SPSR = (1<<SPI2X);
}
/** \brief Transmiting databytes via the SPI
*
* This function is transmitting data via the SPI interface. Input
* parameter is uns. char array. Data are transmited from the zero
* index
*
* \warning This is platform-dependent method!
* \param data[] Source data array
* \param length Array length
*
*/

unsigned char spiMasterTRANSMIT(unsigned char data)
{
	/* Start transmission */
	SPDR = data;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	/* SPDR must be stored as quickly
	as possible (ref. ATMegaX ds) */
	return SPDR;
}
/** \brief Settings of the CS pin
*
* This function is used for setting of the CS pin. CS signal
* is inverted, so input 1 (true) means zero on the output.
* Otherwise is analogically the same.
*
* \warning This is platform-dependent method!
* \param state Wished state
*/
void spiMasterChipSelect(unsigned char state)
{
	/* What the user wants? (remember that the CS signal is inverted) */
	if(!state) {
		/* Upper the CS pin */
		PORT_SPI |= (1<<PIN_SS);
		DDR_SPI |= (1<<PIN_SS);
		} else {
		/* Lower the CS pin */
		PORT_SPI &= ~(1<<PIN_SS);
		DDR_SPI |= (1<<PIN_SS);
	}
}

/* Pointer to function which handle change on INT pin handler */
void (*int_handler)(void);
/** Initialization of hardware ext. interrupts
* \param *handler pointer to a function which handle occured interrupt.
* \return nothing
*/
void extInterruptINIT(void (*handler)(void))
{
	/* Set function pointer */
	int_handler = handler;
	/* Initialize external interrupt on pin INT0 on failing edge */
	MCUCR |= (1 << ISC01);
	GICR |= (1 << INT0);
}
/* System interrupt handler */
SIGNAL(INT0_vect)
{
	int_handler();
}

/**
* Read value of the register on selected address inside the
* MCP2515. Works for every register.
*
* \see MCP2515 datasheet, chapter 11 - register description
* \see MCP2515 datasheet, chapter 12 - read instruction
* \param address Register address
*/
unsigned char readRegister(unsigned char address)
{
	/* Send read instruction, address, and receive result */
	spiMasterChipSelect(1);
	spiMasterTRANSMIT(READ_INSTRUCTION);
	spiMasterTRANSMIT(address);
	unsigned char buffer = spiMasterTRANSMIT(0);
	spiMasterChipSelect(0);
	return buffer;
}

/**
* Change value of the register on selected address inside the
* MCP2515. Works for every register.
*
* \see MCP2515 datasheet, chapter 11 - register description
* \see MCP2515 datasheet, chapter 12 - write instruction
* \param address Register address
* \param value New value of the register
*/
void writeRegister(unsigned char address, unsigned char value)
{
	/* Send write instruction, address, and data */
	spiMasterChipSelect(1);
	spiMasterTRANSMIT(WRITE_INSTRUCTION);
	spiMasterTRANSMIT(address);
	spiMasterTRANSMIT(value);
	spiMasterChipSelect(0);
}
/**
* Send reset instruction to the MCP2515. Device should
* reinitialize yourself and go to the configuration mode
*/
void resetMCP2515()
{
	/* Send reset instruction */
	spiMasterChipSelect(1);
	spiMasterTRANSMIT(RESET_INSTRUCTION);
	spiMasterChipSelect(0);
}

unsigned char setBitTiming(unsigned char rCNF1,
unsigned char rCNF2,
unsigned char rCNF3)
{
	if(getMode == CONFIGURATION_MODE) {
		writeRegister(CNF1, rCNF1);
		writeRegister(CNF2, rCNF2);
		changeBits(CNF3, 0x07, rCNF3);
		return 1;
	}
	return 0;
}

/** Set up acceptance filters/masks
* \param address starting address of 4 registers to setup. It can be mask
* or filter, doesn't matter
* \param criterion message identifier criterion to be set
* \param is_ext 1 if message is extended, otherwise 0 */
void setAcceptanceCriteria (unsigned char address,unsigned long criterion,unsigned char is_ext)
{
	/* Initialize reading of the receive buffer */
	spiMasterChipSelect(1);
	/* Send header and address */
	spiMasterTRANSMIT(WRITE_INSTRUCTION);
	spiMasterTRANSMIT(addr);
	/* Setup standard or extended identifier */
	if(is_ext) {
		spiMasterTRANSMIT((unsigned char)(criterion>>3));
		spiMasterTRANSMIT((unsigned char)(criterion<<5)|(1<<EXIDE)|((unsigned char)(criterion>>27));
		spiMasterTRANSMIT((unsigned char)(criterion>>19));
		spiMasterTRANSMIT((unsigned char)(criterion>>11));
		} else {
		spiMasterTRANSMIT((unsigned char)(criterion >> 3));
		spiMasterTRANSMIT((unsigned char)(criterion << 5));
	}
	/* Release the bus */
	spiMasterChipSelect(0);
}

#define setMask(n, c, e) setAcceptanceCriteria(RXMnSIDH(n), c, e)
#define setFilter(n, c, e) setAcceptanceCriteria(RXFnSIDH(n), c, e)

/* Configuration routine */
void initMCP2515(void)
{
	/* Initialize SPI as a master device, on frequency < 10Mhz */
	spiMasterINIT();
	/* Initialize external interrupt service on this device */
	extInterruptINIT(interruptMCP2515);
	/* Send reset instruction */
	resetMCP2515();
	/* Set configuration mode */
	setMode(CONFIGURATION_MODE);
	/* Set bit timing , masks and rollover mode*/
	setBitTiming(0x04, 0xD2, 0x42);
	setMask(RXM0, 0x00000000, 1);
	setMask(RXM1, 0x00000000, 1);
	setRollover(1);
	/* Get into normal mode and setup communication */
	setMode(NORMAL_MODE)
}