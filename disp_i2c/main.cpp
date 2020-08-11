/*
 * disp_i2c.cpp
 *
 * Created: 20.05.2019 1:19:30
 * Author : vks
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#define I2C_SLAVE_ADDR 0x67 << 1

#define I2C_REG_SIZE 5
volatile uint8_t i2cdata_[I2C_REG_SIZE];
volatile uint8_t i2c_reg_addr_;



	/*
	Hardware configuration Atmega8A with four 7-segments led SUNLIGHT SLR0284DWA1BD
	
	Cathode a = pin30 = PD0;
	Cathode b = pin31 = PD1;
	Cathode c = pin10 = PD6;
	Cathode d = pin2 = PD4;
	Cathode e = pin1 = PD3;
	Cathode f = pin32 = PD2;
	Cathode g = pin11 = PD7;
	Cathode dp = pin9 = PD5;
	
	Anode 1 seg = pin23 = PB2;
	Anode 2 seg = pin24 = PB1;
	Anode 3 seg = pin25 = PA2;
	Anode 4 seg = pin26 = PA3;
	
	Leds
	
	Led3 (left) = pin14 = PB2
	Led2 (right) = pin16 = PB4
	
	Keys
	
	Key1 (left) = pin12 = PB0
	Key2 (right) = pin13 = PB1
	
	*/
	
#define  DIG1 1 << PC3
#define  DIG2 1 << PC2
#define  DIG3 1 << PC1
#define  DIG4 1 << PC0

#define SEG_A 1 << PD0
#define SEG_B 1 << PD1
#define SEG_C 1 << PD6
#define SEG_D 1 << PD4
#define SEG_E 1 << PD3
#define SEG_F 1 << PD2
#define SEG_G 1 << PD7
#define SEG_DP 1 << PD5

#define  LED2 1 << PB4  //right led
#define  LED3 1 << PB2  //left led

#define KEY_1 1 << PB0    // left button
#define KEY_2 1 << PB1    // right button



#define UPDATE_DISPLAY 5
#define KEY_PUSH_DELAY 3
#define KEY_LONGPUSH_DELAY 25
#define BLINK_BUS_DELAY 12


#define ADDR_WAIT_SEC_UP     0x01
#define ADDR_WAIT_SEC_DOWN	 0x02
#define ADDR_WAIT_SEC_LOCK	 0x03
#define ADDR_PERSENTAGE_L	 0x04
#define ADDR_CMD_L			 0x05
#define ADDR_ENABLE_L		 0x06
#define ADDR_PERSENTAGE_R	 0x07
#define ADDR_CMD_R			 0x08
#define ADDR_ENABLE_R		 0x09


//status byte 0
#define STATUS_BIT0_L_ON 0
#define STATUS_BIT0_R_ON 1
#define STATUS_BIT0_L_D  2
#define STATUS_BIT0_L_U  3
#define STATUS_BIT0_R_D  4
#define STATUS_BIT0_R_U  5
#define STATUS_BIT0_L_BUT 6
#define STATUS_BIT0_R_BUT 7

//status byte 1
#define STATUS_BIT1_L_EN   0
#define STATUS_BIT1_R_EN   1
#define STATUS_BIT1_L_ON_U 2
#define STATUS_BIT1_L_ON_D 3
#define STATUS_BIT1_R_ON_U 4
#define STATUS_BIT1_R_ON_D 5
#define STATUS_BIT1_L_TRIP 6   // L TRIP current protection
#define STATUS_BIT1_R_TRIP 7   // R TRIP current protection

#define SCREENS 33


char actval[4] = "";

unsigned char Key1_lock,Key2_lock, Key2_lock_long, CurrScreen = 0;
unsigned char statusbyte0, statusbyte1, paramtodisp, paramtodisp_addr, paramfromdisp, paramfromdisp_addr = 0;
bool Setup_param, blink_bus = false;
unsigned char blink = 0;

struct SwitchFilter{
	bool outstate;
	bool long_outstate;
	bool prev;
	unsigned char scanrate;
	unsigned char long_scanrate;
	volatile unsigned char *port;
	unsigned char pin;
	unsigned char curr_scan;
};

SwitchFilter Key1, Key2;

bool _switch_filter(SwitchFilter * source)
{
	bool result;
	
	result = false;
	
	
	if (!((*source->port) & source->pin))
	{
		if (source->curr_scan >= source->scanrate){
			source->prev = true;
		} 
		
    	if (source->curr_scan >= source->long_scanrate){
			source->prev = false;	
			source->long_outstate = true;					
       } else
       source->curr_scan++;
		
		
		if  (source->scanrate == 0xFF) source->curr_scan = 0;
		
	} else
	{
		if (source->prev) result = true;
		
		source->prev = false;		
		source->long_outstate = false;
		source->curr_scan = 0;
	}
	
	
	return result;
}

void i2c_init_slave(uint8_t addr)
{
	// setup i2c slave addr in TWI register
	TWAR = addr;//(addr << 1);
	// setup TWCR register: , enable addr matching, enable TWI, clear TWINT, interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
	// enable interrupt
	sei();
}

void i2c_stop_slave(void)
{
	// clear ack and enable bits
	TWCR &= ~((1<<TWEA) | (1<<TWEN));
}

ISR(TWI_vect)
{
	uint8_t rx_data=0;
	
	// addr ack
	if((TWSR & 0xF8) == TW_SR_SLA_ACK)
	{
		// set register buffer index to end
		i2c_reg_addr_ = 0xff;
		// clear TWI interrupt flag, ack for recv next byte
		TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	}
	
	// handle rx data in rx mode
	else if((TWSR & 0xF8) == TW_SR_DATA_ACK)
	{
		rx_data = TWDR;
		
		// handle rx data
		if(i2c_reg_addr_ == 0xff)
		{
			// check and set i2c register addr, 2nd byte is register addr
			if(rx_data < I2C_REG_SIZE)
			{
				i2c_reg_addr_ = rx_data;
			}
			else
			{
				// error routine
				// set register addr to start
				i2c_reg_addr_ = 0x00;
			}
			
			// TWCR ACK
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		}
		else
		{
			// handle rx user data
			if(i2c_reg_addr_ < I2C_REG_SIZE) {
				i2cdata_[i2c_reg_addr_] = rx_data;
				i2c_reg_addr_++;
				
				// TWCR ACK
				TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
			}
			else
			{
				// rx last byte, no ack, clear TWI interrupt flag
				TWCR |= (1<<TWIE) | (1<<TWINT) | (0<<TWEA) | (1<<TWEN);
			}
		}
	}
	
	// handle slave as transmitter
	else if((TWSR & 0xF8) == TW_ST_DATA_ACK)
	{
		rx_data = TWDR;
		
		// check data register to transmit
		if(i2c_reg_addr_ == 0xff)
		{
			if(rx_data < I2C_REG_SIZE)
			{
				i2c_reg_addr_ = rx_data;
			}
			else
			{
				// error routine
				// set register addr to start
				i2c_reg_addr_ = 0x00;
			}
		}
		
		TWDR = i2cdata_[i2c_reg_addr_];
		i2c_reg_addr_++;
		
		// handle ack
		if(i2c_reg_addr_ < I2C_REG_SIZE){
			// clear TWI interrupt flag, ack
			TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		}
		else
		{
			// clear TWI interrupt flag, no ack
			TWCR |= (1<<TWIE) | (1<<TWINT) | (0<<TWEA) | (1<<TWEN);
		}
	}
	
	// default handle
	else
	{
		TWCR |= (1<<TWIE) | (1<<TWEA) | (1<<TWEN);
	}
	
}

ISR (TIMER2_OVF_vect) // 30.5 Hz
{	
    Key1.outstate = _switch_filter(&Key1);
    Key2.outstate = _switch_filter(&Key2);

	
	if (blink < BLINK_BUS_DELAY) blink++;
	else
	   blink = 0;

   if (blink == BLINK_BUS_DELAY) blink_bus = !blink_bus;	
	
	TCNT2 = 0;

}

void setseg(unsigned char seg)
{
	//clear segment
	
	PORTD = 0xFF;
	
	switch(seg)
	{
	
		case 0:  PORTD &= ~(SEG_F | SEG_E | SEG_D | SEG_C | SEG_B | SEG_A);
		break;
		case 1:  PORTD &= ~(SEG_B | SEG_C);
		break;
		case 2:  PORTD &= ~(SEG_A | SEG_B | SEG_G | SEG_E | SEG_D);
		break;
		case 3:  PORTD &= ~(SEG_A | SEG_B | SEG_G | SEG_C | SEG_D);
		break;
		case 4:  PORTD &= ~(SEG_F | SEG_G | SEG_B | SEG_C);
		break;
		case 5:  PORTD &= ~(SEG_A | SEG_F | SEG_G | SEG_C | SEG_D);
		break;
		case 6:  PORTD &= ~(SEG_A | SEG_F | SEG_G | SEG_E | SEG_D | SEG_C);
		break;
		case 7:  PORTD &= ~(SEG_A | SEG_B | SEG_C);
		break;
		case 8:  PORTD &= ~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G);
		break;
		case 9:  PORTD &= ~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G);
		break;
		case 10: PORTD &= ~(SEG_G); //print "-"
		break;
		case 11:  PORTD &= ~(SEG_F | SEG_E | SEG_D | SEG_B | SEG_C); //print U
		break;		
		case 12:  PORTD &= ~(SEG_G | SEG_E | SEG_D | SEG_C | SEG_B); //print d
		break;
		case 13:  PORTD &= ~(SEG_F | SEG_E | SEG_D );                //print L
		break;
		case 14: PORTD &= ~(SEG_G | SEG_E | SEG_D | SEG_B | SEG_C); //print r 		
		break;
		case 15: PORTD &= ~(SEG_A | SEG_B | SEG_C | SEG_E | SEG_F); //print A
		break;
		case 16: PORTD &= ~(SEG_D);                                  //print _
		break;		
		case 17: PORTD &= ~(SEG_A);                                  //print - (upper hyphen)
		break;
		case 18: PORTD &= ~(SEG_A | SEG_D);                          //print - & _ (upper hyphen and _)
		break;		
	    case 19: PORTD &= ~(SEG_A | SEG_F | SEG_E | SEG_D);          //print C
		break;	
		default: //empty segment
		break;
	}
} 

void selectseg(unsigned char selseg)
{
	//clear 7seg led anodes
	
    PORTC  |= DIG1 | DIG2 | DIG3 | DIG4;
	
	switch(selseg)
	{
		case 1: PORTC &= ~(DIG1);
		break;
		
		case 2: PORTC &= ~(DIG2);
		break;
		
		case 3: PORTC &= ~(DIG3);
		break;
		
		case 4: PORTC &= ~(DIG4);
		break;
		
		default:
		break;
	}
}

void init_switches(){
	
	Key1.outstate=false;
	Key1.long_outstate = false;
	Key1.curr_scan=0;
	Key1.scanrate=KEY_PUSH_DELAY;
	Key1.long_scanrate = KEY_LONGPUSH_DELAY;
	Key1.prev = false;
	Key1.port = &PINB;
	Key1.pin = KEY_1;
	
		
	Key2.outstate=false;
	Key2.long_outstate = false;
	Key2.curr_scan=0;
	Key2.scanrate=KEY_PUSH_DELAY;
    Key2.long_scanrate = KEY_LONGPUSH_DELAY;
	Key2.prev = false;
	Key2.port = &PINB;
	Key2.pin = KEY_2;
}



void print_decimal(unsigned int val, char point_pos)
{
	unsigned char i,n = 0;
	
	actval[3] = val % 10;
	actval[2] = (val/=10) % 10;
	actval[1] = (val/=10) % 10;
	actval[0] = (val/=10) % 10;
	
	while ((n<3) && (actval[n] == 0)) n++;  //remove excess zeros
	
	if (4-point_pos < n) n = 4-point_pos;
	
	
	for(i=n+1;i<=4;i++)  {
		
		selectseg(i);
		
	
				
		setseg(actval[i-1]);
				
		if (point_pos == 5-i)  PORTD &= ~(SEG_DP);
		
		_delay_ms(UPDATE_DISPLAY);
	}
} 


void print_number_decimal(unsigned char number, unsigned int val, bool setup, unsigned char dot_pos){
	unsigned char i,n = 0;
		
	selectseg(1);
	if (number < 10){
		  setseg(number);
          PORTD &= ~(SEG_DP);
	} else
	{
	   setseg(number / 10);
	    _delay_ms(UPDATE_DISPLAY);
	   selectseg(2);
	   setseg(number % 10);
	   PORTD &= ~(SEG_DP);
	}
		
    _delay_ms(UPDATE_DISPLAY);
	
	
	actval[2] = val % 10;
	actval[1] = (val/=10) % 10;
	actval[0] = (val/=10) % 10;
    
	n = 1;
    if (actval[0] == 0) n=2;
	

	if (setup)
    {
	   for(i=n+1;i<=4;i++)  {
		  selectseg(i);
		  setseg(actval[i-2]);
		  if ((i == 3) && (dot_pos == 1)) PORTD &= ~(SEG_DP);
	    	else
		   PORTD |= (SEG_DP);
		
		
		  _delay_ms(UPDATE_DISPLAY);
	  }
	}
}

void print_statusbyte(unsigned char _statusbyte0, unsigned char _statusbyte1)
{
	
	if (_statusbyte1 & (1 << STATUS_BIT1_L_TRIP))
	{
		selectseg(1);
		setseg(0);
		PORTD |= (SEG_DP);
		_delay_ms(UPDATE_DISPLAY);
		
		selectseg(2);
		setseg(19);
		PORTD |= (SEG_DP);
		_delay_ms(UPDATE_DISPLAY);
	}
	else
	{
		
		selectseg(1);
		setseg(255);
		if (_statusbyte1 & (1 << STATUS_BIT1_L_ON_U)) setseg(11); 
		if (_statusbyte1 & (1 << STATUS_BIT1_L_ON_D)) setseg(12);
		PORTD |= (SEG_DP);	
		_delay_ms(UPDATE_DISPLAY);
	
		selectseg(2);
		setseg(255);
		if (_statusbyte0 & (1 << STATUS_BIT0_L_D)) setseg(16); 
		if (_statusbyte0 & (1 << STATUS_BIT0_L_U)) setseg(17);
		if ((_statusbyte0 & (1 << STATUS_BIT0_L_D)) && (_statusbyte0 & (1 << STATUS_BIT0_L_U))) setseg(18);
		if (_statusbyte1 & (1 << STATUS_BIT1_L_EN)) PORTD &= ~(SEG_DP);
	
		_delay_ms(UPDATE_DISPLAY);
	}
	
	if (_statusbyte1 & (1 << STATUS_BIT1_R_TRIP))
	{	
		selectseg(3);
		setseg(0);
		PORTD |= (SEG_DP);
		_delay_ms(UPDATE_DISPLAY);
				
		selectseg(4);
		setseg(19);
		PORTD |= (SEG_DP);
		_delay_ms(UPDATE_DISPLAY);
		
	} else
	{
		
		selectseg(3);
		setseg(255);
		if (_statusbyte1 & (1 << STATUS_BIT1_R_ON_U)) setseg(11);
		if (_statusbyte1 & (1 << STATUS_BIT1_R_ON_D)) setseg(12);
		PORTD |= (SEG_DP);
		_delay_ms(UPDATE_DISPLAY);
	
	
		selectseg(4);
	
		setseg(255);
		if (_statusbyte0 & (1 << STATUS_BIT0_R_D)) setseg(16); 
		if (_statusbyte0 & (1 << STATUS_BIT0_R_U)) setseg(17); 
		if ((_statusbyte0 & (1 << STATUS_BIT0_R_D)) && (_statusbyte0 & (1 << STATUS_BIT0_R_U))) setseg(18);
		if (_statusbyte1 & (1 << STATUS_BIT1_R_EN)) PORTD &= ~(SEG_DP);
		_delay_ms(UPDATE_DISPLAY);
	}
}


void print_nodata(void)
{
	int i;
	
	for(i=1;i<=4;i++)  {
		selectseg(i);
		
		
		setseg(10);
		_delay_ms(UPDATE_DISPLAY);
	}
	
}


int main(void)
{

	
	DDRC = DIG1 | DIG2 | DIG3 | DIG4;
	DDRD = 0xFF; // All segments in a D port connected to cathodes
	
	DDRB = LED3 | LED2;
	
	// set up timer0
    TCCR2 = (1 << CS22)|(1 << CS21)|(1 << CS20); //1024 divider
		
	TCNT2 = 0;
		
	TIFR = (1<<TOV2);
	TIMSK = (1<<TOIE2);
	
	CurrScreen = 0;	
    
	i2c_init_slave(I2C_SLAVE_ADDR);
	
	init_switches();
	    
    sei();
    
	
	while (1) 
    {
		//asm("wdr"); 
		
		 statusbyte0 = i2cdata_[1];
		 statusbyte1 = i2cdata_[2];
		 if (!Setup_param) paramtodisp = i2cdata_[3];
		 i2cdata_[4] = paramtodisp_addr;
		 i2cdata_[5] = paramfromdisp;
		 i2cdata_[6] = paramfromdisp_addr;
		
		
		if (Key1.outstate && (!Key1_lock)) {
			Key1_lock = 1;
			
		    if (Setup_param)
	        {
			     if (paramtodisp > 0) paramtodisp--;
	        }   else
	        {
		         if (CurrScreen > 0) CurrScreen--;
		         else
		        CurrScreen = SCREENS;
	       }
		 }
		 if (!Key1.outstate) Key1_lock = 0;
		 
		 if (Key2.outstate && (!Key2_lock)) {
			 
			 Key2_lock = 1;
			 
			 if (Setup_param)
			 {
				 paramtodisp++;
			 } else
			 {
			    if (CurrScreen < SCREENS) CurrScreen++;
			    else
			       CurrScreen = 0;
			 } 
				  
		 }	 
		 
		 if (!Key2.outstate) Key2_lock = 0;
		 
		 
		 if (Key2.long_outstate && (!Key2_lock_long) && (CurrScreen > 0)) 
		 {
		      Key2_lock_long = 1;
			  Setup_param = !Setup_param;
		 }
		 
		 if (!Key2.long_outstate) Key2_lock_long = 0;
		 

		 if (statusbyte0 & (1 << STATUS_BIT0_L_ON)) 
		 { 
			 PORTB |=LED3;
		 }
		 
		 else PORTB &= ~ (LED3);
		 
		if (statusbyte0 & (1 << STATUS_BIT0_R_ON))
		{
			PORTB |=LED2;
		}
		
		else PORTB &= ~ (LED2);		 
		 
		switch(CurrScreen)
		{
			case 0: 	print_statusbyte(statusbyte0,statusbyte1);
			            break;
			
			case 1: 	paramtodisp_addr = ADDR_WAIT_SEC_UP;
			            print_number_decimal(1, paramtodisp,!(Setup_param && blink_bus),1);
						break;
						
			case 2:     paramtodisp_addr = ADDR_WAIT_SEC_DOWN;
			            print_number_decimal(2, paramtodisp,!(Setup_param && blink_bus),1);
						break;
			
			case 3:     paramtodisp_addr = ADDR_WAIT_SEC_LOCK;
			            print_number_decimal(3, paramtodisp,!(Setup_param && blink_bus),1);
						break;
			case 4:     paramtodisp_addr = ADDR_PERSENTAGE_L;
			            print_number_decimal(4, paramtodisp,!(Setup_param && blink_bus),0);
			            break;				
			case 5:     paramtodisp_addr = ADDR_CMD_L;
			            print_number_decimal(5, paramtodisp,!(Setup_param && blink_bus),1);
			            break;	
			case 6:     paramtodisp_addr = ADDR_ENABLE_L;
			            print_number_decimal(6, paramtodisp,!(Setup_param && blink_bus),0);
			            break;				
			case 7:     paramtodisp_addr = ADDR_PERSENTAGE_R;
			            print_number_decimal(7, paramtodisp,!(Setup_param && blink_bus),0);
			            break;	
			case 8:     paramtodisp_addr = ADDR_CMD_R;
			            print_number_decimal(8, paramtodisp,!(Setup_param && blink_bus),1);
			            break;	
			case 9:     paramtodisp_addr = ADDR_ENABLE_R;
			            print_number_decimal(9, paramtodisp,!(Setup_param && blink_bus),0);
			            break;	
			case 10 ... 19:     paramtodisp_addr = CurrScreen;
			            print_number_decimal(paramtodisp_addr, paramtodisp,!(Setup_param && blink_bus),0);
			            break;	
			case 20 ... 29:     paramtodisp_addr = CurrScreen;
						print_number_decimal(paramtodisp_addr, paramtodisp,!(Setup_param && blink_bus),0);
						break;
			case 30:    paramtodisp_addr = CurrScreen;
			            print_number_decimal(paramtodisp_addr, paramtodisp,!(Setup_param && blink_bus),0);
						break;
			case 31:    paramtodisp_addr = CurrScreen;
			            print_number_decimal(paramtodisp_addr, paramtodisp,!(Setup_param && blink_bus),2);
			            break;
			case 32:    paramtodisp_addr = CurrScreen;
						print_number_decimal(paramtodisp_addr, paramtodisp,!(Setup_param && blink_bus),0);
						break;
			case 33:    paramtodisp_addr = CurrScreen;
						print_number_decimal(paramtodisp_addr, paramtodisp,!(Setup_param && blink_bus),2);
						break;
		}
		if (Setup_param) { 
			
			paramfromdisp = paramtodisp;
			paramfromdisp_addr = paramtodisp_addr;
		} else
		{
			paramfromdisp = 0;
			paramfromdisp_addr = 0;
		}
		
    }
}

