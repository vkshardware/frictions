#include <avr\io.h>
#include <avr\interrupt.h>
#include <stdint.h>

#include "twi.h"



#define TWI_READ		1 // операция чтения
#define TWI_WRITE		0 // операция записи

/*==================================================================================================
			Коды состояния TWI в режиме Master
==================================================================================================*/
#define TWI_BUSERROR	0x00 // Ошибка шины
#define TWI_START		0x08 // START отправлен
#define TWI_REPSTART	0x10 // повторный START отправлен
#define TWI_SLAWACK		0x18 // SLA+W отправлен и получен ACK
#define TWI_SLAWNACK	0x20 // SLA+W отправлен и получен NACK
#define TWI_DATWACK		0x28 // DATA отправлен и получен ACK
#define TWI_DATWNACK	0x30 // DATA отправлен и получен NACK
#define TWI_COLLISION	0x38 // потеряли Slave
#define TWI_SLARACK		0x40 // SLA+R отправлен и получен ACK
#define TWI_SLARNACK	0x48 // SLA+R отправлен и получен NACK
#define TWI_DATRACK		0x50 // DATA получен и отправлен ACK
#define TWI_DATRNACK	0x58 // DATA получен и отправлен NACK


volatile uint8_t TWI_DataCnt = 0;
volatile uint8_t TWI_Status;
uint8_t TWI_DataAddr = 0;
uint8_t TWI_DataSize = 0;
volatile uint8_t TWI_Mode;
volatile uint8_t TWI_DataBuff[TWI_BUFFERSIZE];


//////////////////////////////////////////////////////////////////////////////////////////////////////
//				инициализация
//
void twi_init(void)
{
	TWI_Status = TWI_FREE; // свободен

	TWBR = TWI_TWBR;
	TWSR = 0;
	TWDR = 0xFF;
	TWCR = (1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // включить TWI
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//				чтение данных
//	на входе
//		TWI_DataAddr - адрес в Slave-е для чтения
//		TWI_DataSize - сколько байт читать
//	на выходе
//		TWI_DataBuff - считанные данные
void twi_read(void)
{
	TWI_Status = TWI_BUSY; // занят
	TWI_Mode = TWI_READ; // читаю
	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // отправляем START
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//				запись данных
//	на входе
//		TWI_DataAddr - адрес в Slave-е для записи
//		TWI_DataSize - сколько байт писать
//		TWI_DataBuff - данные для записи
void twi_write(void)
{
	TWI_Status = TWI_BUSY; // занят
	TWI_Mode = TWI_WRITE; // пишу
	TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // отправляем START
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//				вектор прерывания TWI - самый главный кусок кода
//
ISR(TWI_vect)
{
	switch( TWSR )
	{
	case TWI_START: // START отправлен
		TWI_DataCnt = 0; // обнуляем счетчик данных
		TWDR = TWI_SLAVEADDR; // пишем адрес Slave-а
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // отправить SLA+W
		break;

	case TWI_SLAWACK: // SLA+W отправлен и получен ACK
		TWDR = TWI_DataAddr; // пишем адрес данных
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // отправить адрес
		break;

	case TWI_DATWACK: // DATA отправлен и получен ACK
		if( TWI_Mode == TWI_READ ) // читаем?
		{
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // отправить повторный START
		}
		else // нет, пишем
		{
			if( TWI_DataCnt < TWI_DataSize ) // у нас есть еще данные?
			{
				TWDR = TWI_DataBuff[TWI_DataCnt++]; // пишем данные
				TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // отправить данные
			}
			else // больше данных нет
			{
				TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC); // отправить STOP
				TWI_Status = TWI_FREE; // и свободен
			}
		}
		break;

	case TWI_REPSTART: // повторный START отправлен
		TWDR = (TWI_SLAVEADDR|0x01); // пишем адрес Slave-а
		TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // отправить SLA+R
		break;

	case TWI_DATRACK: // DATA получен и отправлен ACK
		TWI_DataBuff[TWI_DataCnt++] = TWDR; // читаем данные
	case TWI_SLARACK: // SLA+R отправлен и получен ACK
		if( TWI_DataCnt < (TWI_DataSize-1) ) // нам нужны еще данные?
		{
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // требуем байт и ACK
		}
		else // нет, не нужны
		{
			TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); // требуем байт и NACK
		}
		break;

	case TWI_DATRNACK: // DATA получен и отправлен NACK
		TWI_DataBuff[TWI_DataCnt++] = TWDR; // читаем данные
	case TWI_DATWNACK: // DATA отправлен и получен NACK
	default: // при всем остальном формировать STOP
		TWCR = (1<<TWEN)|(0<<TWIE)|(1<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(1<<TWSTO)|(0<<TWWC); // отправить STOP
		TWI_Status = TWI_FREE; // свободен
		break;
	}
}
