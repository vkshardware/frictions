#include <avr\io.h>
#include <avr\interrupt.h>
#include <stdint.h>

#include "twi.h"



#define TWI_READ		1 // �������� ������
#define TWI_WRITE		0 // �������� ������

/*==================================================================================================
			���� ��������� TWI � ������ Master
==================================================================================================*/
#define TWI_BUSERROR	0x00 // ������ ����
#define TWI_START		0x08 // START ���������
#define TWI_REPSTART	0x10 // ��������� START ���������
#define TWI_SLAWACK		0x18 // SLA+W ��������� � ������� ACK
#define TWI_SLAWNACK	0x20 // SLA+W ��������� � ������� NACK
#define TWI_DATWACK		0x28 // DATA ��������� � ������� ACK
#define TWI_DATWNACK	0x30 // DATA ��������� � ������� NACK
#define TWI_COLLISION	0x38 // �������� Slave
#define TWI_SLARACK		0x40 // SLA+R ��������� � ������� ACK
#define TWI_SLARNACK	0x48 // SLA+R ��������� � ������� NACK
#define TWI_DATRACK		0x50 // DATA ������� � ��������� ACK
#define TWI_DATRNACK	0x58 // DATA ������� � ��������� NACK


volatile uint8_t TWI_DataCnt = 0;
volatile uint8_t TWI_Status;
uint8_t TWI_DataAddr = 0;
uint8_t TWI_DataSize = 0;
volatile uint8_t TWI_Mode;
volatile uint8_t TWI_DataBuff[TWI_BUFFERSIZE];


//////////////////////////////////////////////////////////////////////////////////////////////////////
//				�������������
//
void twi_init(void)
{
	TWI_Status = TWI_FREE; // ��������

	TWBR = TWI_TWBR;
	TWSR = 0;
	TWDR = 0xFF;
	TWCR = (1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // �������� TWI
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//				������ ������
//	�� �����
//		TWI_DataAddr - ����� � Slave-� ��� ������
//		TWI_DataSize - ������� ���� ������
//	�� ������
//		TWI_DataBuff - ��������� ������
void twi_read(void)
{
	TWI_Status = TWI_BUSY; // �����
	TWI_Mode = TWI_READ; // �����
	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ���������� START
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//				������ ������
//	�� �����
//		TWI_DataAddr - ����� � Slave-� ��� ������
//		TWI_DataSize - ������� ���� ������
//		TWI_DataBuff - ������ ��� ������
void twi_write(void)
{
	TWI_Status = TWI_BUSY; // �����
	TWI_Mode = TWI_WRITE; // ����
	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ���������� START
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//				������ ���������� TWI - ����� ������� ����� ����
//
ISR(TWI_vect)
{
	switch( TWSR )
	{
	case TWI_START: // START ���������
		TWI_DataCnt = 0; // �������� ������� ������
		TWDR = TWI_SLAVEADDR; // ����� ����� Slave-�
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ��������� SLA+W
		break;

	case TWI_SLAWACK: // SLA+W ��������� � ������� ACK
		TWDR = TWI_DataAddr; // ����� ����� ������
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ��������� �����
		break;

	case TWI_DATWACK: // DATA ��������� � ������� ACK
		if( TWI_Mode == TWI_READ ) // ������?
		{
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ��������� ��������� START
		}
		else // ���, �����
		{
			if( TWI_DataCnt < TWI_DataSize ) // � ��� ���� ��� ������?
			{
				TWDR = TWI_DataBuff[TWI_DataCnt++]; // ����� ������
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ��������� ������
			}
			else // ������ ������ ���
			{
				TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC); // ��������� STOP
				TWI_Status = TWI_FREE; // � ��������
			}
		}
		break;

	case TWI_REPSTART: // ��������� START ���������
		TWDR = (TWI_SLAVEADDR|0x01); // ����� ����� Slave-�
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ��������� SLA+R
		break;

	case TWI_DATRACK: // DATA ������� � ��������� ACK
		TWI_DataBuff[TWI_DataCnt++] = TWDR; // ������ ������
	case TWI_SLARACK: // SLA+R ��������� � ������� ACK
		if( TWI_DataCnt < (TWI_DataSize-1) ) // ��� ����� ��� ������?
		{
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ������� ���� � ACK
		}
		else // ���, �� �����
		{
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // ������� ���� � NACK
		}
		break;

	case TWI_DATRNACK: // DATA ������� � ��������� NACK
		TWI_DataBuff[TWI_DataCnt++] = TWDR; // ������ ������
	case TWI_DATWNACK: // DATA ��������� � ������� NACK
	default: // ��� ���� ��������� ����������� STOP
		TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC); // ��������� STOP
		TWI_Status = TWI_FREE; // ��������
		break;
	}
}
