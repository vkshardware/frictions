#ifndef __TWI_H__
#define __TWI_H__


#define TWI_WPORT		PORTC
#define TWI_DPORT		DDRC
#define SCL_PIN			5
#define SDA_PIN			4

// �������� ���� BaudRate (100 kHz) 400 kHz = 2
#define TWI_TWBR		8

// ������������ ����� ������ ������
#define TWI_BUFFERSIZE	8

// ����� Slave-����������
#define TWI_SLAVEADDR	0x67 << 1  //I2C Led Display

/*==================================================================================================
			���� ��������� ����
==================================================================================================*/
#define TWI_BUSY	0 // �����
#define TWI_FREE	1 // ��������




extern volatile uint8_t TWI_Status; // ��������� ����
extern uint8_t TWI_DataAddr; // ����� ������ ������/������
extern uint8_t TWI_DataSize; // ����� ������ ������/������
extern volatile uint8_t TWI_DataBuff[TWI_BUFFERSIZE]; // ����� ������ ������/������


extern void twi_init(void);
extern void twi_read(void);
extern void twi_write(void);



#endif // __TWI_H__
