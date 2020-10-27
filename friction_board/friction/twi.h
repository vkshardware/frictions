#ifndef __TWI_H__
#define __TWI_H__


#define TWI_WPORT		PORTC
#define TWI_DPORT		DDRC
#define SCL_PIN			5
#define SDA_PIN			4

// скорость шины BaudRate (100 kHz) 400 kHz = 2
#define TWI_TWBR		8

// максимальный объем буфера данных
#define TWI_BUFFERSIZE	8

// адрес Slave-устройства
#define TWI_SLAVEADDR	0x67 << 1  //I2C Led Display

/*==================================================================================================
			коды состояния шины
==================================================================================================*/
#define TWI_BUSY	0 // занят
#define TWI_FREE	1 // свободен




extern volatile uint8_t TWI_Status; // состояние шины
extern uint8_t TWI_DataAddr; // адрес данных чтения/записи
extern uint8_t TWI_DataSize; // объем данных чтения/записи
extern volatile uint8_t TWI_DataBuff[TWI_BUFFERSIZE]; // буфер данных чтения/записи


extern void twi_init(void);
extern void twi_read(void);
extern void twi_write(void);



#endif // __TWI_H__
