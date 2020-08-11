/*
 * friction.cpp
 *
 * Created: 11/12/18 17:30:46
 * Author : vks
 */ 

#define F_CPU                  16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "twi.h"


/*

GAZ-71 Frictions control board v2.0

Firmware v2.2.

Channel 1 = LEFT:

PB0 -  SW_LEFT - button
PD0 - NP_ON11 - forward
PB2 - NP_ON12 - backward

PC4 - ANA11 - current Q1 (forward)
PC5 - ANA12 - current Q2 (backward)

PD4 - SW_L_DOWN - switch DOWN
PD5 - SW_L_UP   - switch UP


Channel 2 = RIGHT:

PB1 -  SW_RIGHT - button
PD2 -  NP_ON21 - forward
PD3 -  NP_ON22 - backward

PE2 - ANA21 - current Q12 (forward)
PE3 - ANA22 - current Q11 (backward)

PD6 - SW_R_DOWN - switch DOWN
PD7 - SW_R_UP   - switch UP



*/

#define ON_L1 (1 << PD0)
#define ON_L2 (1 << PB2)
#define SW_L (1 << PB0)

#define ON_R1 (1 << PD2)
#define ON_R2 (1 << PD3)
#define SW_R (1 << PB1)


#define LEFT_DOWN (1 << PD4)   //концевик левый низ
#define LEFT_UP (1 << PD5)     //концевик левый верх
#define RIGHT_DOWN (1 << PD6)  //концевик правый низ
#define RIGHT_UP (1 << PD7)    //концевик правый верх

#define CURRENT_OFF_DELAY 20   //беcтоковая пауза 50 мс
#define CURRENT_COEFFICIENT 5

#define WAIT_100HZ 0xF63B
#define WAIT_5Hz 30
#define WAIT_1Hz 100

#define DEF_WAIT_SEC_UP 11   //время подачи напряжения на привод НА ВЫЖИМ х100мс
#define DEF_WAIT_SEC_DOWN 30 //время подачи напряжения на привод НА ОТПУСКАНИЕ х100мс
#define DEF_WAIT_SEC_LOCK 70 //время перехода в выдвинутое состояние без удерживания кнопки х100мс Должно быть > WAIT_SEC_UP

#define ADDR_WAIT_SEC_UP	 0x01
#define ADDR_WAIT_SEC_DOWN   0x02
#define ADDR_WAIT_SEC_LOCK	 0x03
#define ADDR_PERSENTAGE_L	 0x04
#define ADDR_CMD_L			 0x05
#define ADDR_ENABLE_L		 0x06
#define ADDR_PERSENTAGE_R	 0x07
#define ADDR_CMD_R			 0x08
#define ADDR_ENABLE_R		 0x09

#define MIN_WAIT_SEC_UP 3
#define MIN_WAIT_SEC_DOWN 3
#define MIN_WAIT_SEC_LOCK 0

#define MAX_WAIT_SEC_UP 100
#define MAX_WAIT_SEC_DOWN 100
#define MAX_WAIT_SEC_LOCK 200

#define TIMER1_MAXCOUNT 10

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

#define AN_X_L (0 << MUX3)|(0 << MUX2)|(0 << MUX1)|(0 << MUX0) //ADC0
#define AN_X_R (0 << MUX3)|(0 << MUX2)|(0 << MUX1)|(1 << MUX0) //ADC1
#define ANA11 (0 << MUX3)|(0 << MUX2)|(1 << MUX1)|(0 << MUX0)  //ADC2
#define ANA12 (0 << MUX3)|(0 << MUX2)|(1 << MUX1)|(1 << MUX0)  //ADC3
#define ANA21 (0 << MUX3)|(1 << MUX2)|(1 << MUX1)|(0 << MUX0)  //ADC6
#define ANA22 (0 << MUX3)|(1 << MUX2)|(1 << MUX1)|(1 << MUX0)  //ADC7

struct SwitchFilter{
	bool outstate;
	unsigned char scanrate;
	volatile unsigned char *port;
	unsigned char pin;
	unsigned char curr_scan;
	bool reset;
	bool off;
	bool on_trigger;
};

struct JoystickFilter{
	unsigned int in_value;
	unsigned char step_scan[10];
	unsigned char scanrate;
	unsigned char steps[10];
	unsigned int joystick_step;
	unsigned int mirror_joy_step;
	bool enabled;
};

struct AnalogFilter{
	uint8_t in_value;
	uint8_t time_rate;
	uint8_t analog_limit;
	uint8_t time_count;
	bool trip;
};

class ChannelControl{	
	private:
	   
	public:
      bool switch_up;
      bool switch_down;
	  bool control_source; //1 - joystick, 0 - button

      uint16_t timer_count;
	  uint8_t goal_step;
	  uint8_t curr_step;
	  bool timer_count_en;
	  bool moving_fwd;
	  bool moving_bwd;
	  bool jog_direction;
	  bool lock; 

	  volatile unsigned char *port1;
	  uint8_t pin1;
	  
	  volatile unsigned char *port2;
	  uint8_t pin2;
	  	  
	  void InitChannel();
	  void Forward();
	  void Backward();
	  void Stop();
	  int8_t Process();
};

ChannelControl Left, Right;

bool count_en_L, count_en_R, stL, stR, ledL_on_up, ledR_on_up, ledL_on_down, ledR_on_down, operation_L, operation_R = false;
unsigned int channelL_on, channelR_on = 0;


bool lock1, lock2, twi_process_read, twi_process_write = false;

SwitchFilter in_left, in_right, sw_leftdown, sw_leftup, sw_rightdown, sw_rightup;
JoystickFilter an_R, an_L;
AnalogFilter val11, val12, val21, val22;

char w100Hz, readbyte_disp = 0;

char statebyte0, statebyte1, paramtodisp, paramtodisp_addr, paramfromdisp, paramfromdisp_addr = 0;

unsigned int val_L, val_R, val_ANA11, val_ANA12, val_ANA21, val_ANA22 = 0;
unsigned int jogL_on, jogR_on = 0;
bool jogR_count_en, jogL_count_en = true;
bool jog_stR, jog_stL = false;
unsigned int curr_R_step, curr_L_step = 0;
bool jog_started_R_fwd, jog_started_R_bwd = false;
bool jog_started_L_fwd, jog_started_L_bwd = false;

unsigned int def_analog[] = {10,30,35,40,45,56,61,66,71,90}; //default analog values

													  //30-33 parameter, protection data:
													  //Default value, minimal value, maximal value, decimal point position
unsigned int def_protection[4][4] = {{8,1,50,0},     //L current (A),
	                                 {10,1,99,2},     // L times x 10ms,
									 {8,1,50,0},   //R current (A),
									 {10,1,99,2}};  //R times x 10ms

uint8_t EEMEM eeprom_wait_sec_down;
uint8_t EEMEM eeprom_wait_sec_lock;
uint8_t EEMEM eeprom_wait_sec_up;
uint8_t EEMEM an_L_eeprom[10];
uint8_t EEMEM an_R_eeprom[10];
uint8_t EEMEM protection_eeprom[4];

unsigned char  wait_sec_up, wait_sec_down, wait_sec_lock = 0;
uint8_t timer1_count = 0;
uint8_t protection[4];

bool control_L, control_R = false; // Control source (JOYSTICK/BUTTON)


void ChannelControl::InitChannel()
{
	lock = false;
	switch_up = false;
	control_source = false;
	switch_down = false;
	timer_count = 0;
	moving_fwd = false;
	moving_bwd = false;
	timer_count_en = true;
	curr_step = 0;
	goal_step = 0;
}

void ChannelControl::Backward()
{
	*port1 |= pin1;    
	*port2 &= ~(pin2); 
	moving_fwd = false;
	moving_bwd = true;
	
}

void ChannelControl::Forward()
{
	*port1 &= ~(pin1);
	*port2 |=  pin2;
	moving_fwd = true;
	moving_bwd = false;
}

void ChannelControl::Stop()
{
	*port1 &= ~(pin1);
	*port2 &= ~(pin2);
	moving_fwd = false;
	moving_bwd = false;
}

int8_t ChannelControl::Process()
{
		uint16_t relative_step = 0;
	    uint16_t condition = 0;
		
		if ((goal_step > curr_step) || moving_fwd)  // Forward
		{
			relative_step = goal_step - curr_step;
			
			if (moving_fwd && (relative_step == 0)) relative_step = 1;
			
			
			
			if (!jog_direction) {
				Stop();
				_delay_ms(CURRENT_OFF_DELAY);
				timer_count_en = true;
				moving_fwd = true;
				moving_bwd = false;
				timer_count = 0;
				
				if (control_source) curr_step = 0;
			}
			
			jog_direction = true;
			
			condition = wait_sec_up*10/4*relative_step;
			
			if ((timer_count <= condition) && (!switch_up))
			{
				Forward(); //Channel RIGHT ON turn Forward
				
				timer_count_en = true;
			} else
			{
				if (timer_count_en) curr_step +=relative_step;

				//timer_count_en = false;
				//timer_count = 0;
				
				Stop();
			}
		}
		
		if ((goal_step < curr_step) || moving_bwd)   // Backward
		{
			relative_step = curr_step - goal_step;
			
			
			if (moving_bwd && (relative_step == 0)) relative_step = 1;
			
			if (jog_direction) {
				Stop();
				_delay_ms(CURRENT_OFF_DELAY); // current off delay
				timer_count_en = true;
				moving_bwd = true;
				moving_fwd = false;
				timer_count = 0;
			}
			
			jog_direction = false;
			
			
			
			condition = wait_sec_up*10/4*relative_step;
			
			if (goal_step == 0) {
				condition = wait_sec_down*10;  // big backward time guaranteed to return to actuator starting position
			}
			if ((timer_count <= condition) && (!switch_down))
			{
				Backward();  //Channel RIGHT ON turn Backward
				timer_count_en = true;
				
			} else
			{
				
				
				if ((timer_count_en) && (goal_step > 0)) curr_step -=relative_step;
				
				if ((timer_count_en) && (goal_step == 0)) curr_step = 0; //fuse condition
				
				timer_count_en = false;
				
				timer_count = 0;
				
				Stop();			
			}
			
		}
		
	if (timer_count >= wait_sec_lock*10)
	{
		lock = true;
		timer_count_en = false;
		timer_count = 0;
	}
		
	
	return curr_step;
}

unsigned char CheckMinMax(unsigned char addr, unsigned char param)
{
	switch (addr)
	{
    	case ADDR_WAIT_SEC_UP:   if ((param > MAX_WAIT_SEC_UP) || (param < MIN_WAIT_SEC_UP)) param = DEF_WAIT_SEC_UP;
		                         break;
	    case ADDR_WAIT_SEC_DOWN: if ((param > MAX_WAIT_SEC_DOWN) || (param < MIN_WAIT_SEC_DOWN)) param = DEF_WAIT_SEC_DOWN;
		                         break;
		case ADDR_WAIT_SEC_LOCK: if ((param > MAX_WAIT_SEC_LOCK) || (param < MIN_WAIT_SEC_LOCK)) param = DEF_WAIT_SEC_LOCK;
		                         break;
	}
	
	return param;
}


void LoadEeprom(){
	
   unsigned char tmp = 0;
   int i = 0;

	wait_sec_up = eeprom_read_byte(&eeprom_wait_sec_up);
	
	_delay_ms(5);
	
	tmp = CheckMinMax(ADDR_WAIT_SEC_UP, wait_sec_up);
	if (wait_sec_up != tmp)
	{
		wait_sec_up = tmp;
		eeprom_write_byte(&eeprom_wait_sec_up, wait_sec_up);
		_delay_ms(5);
	}
	
	
	wait_sec_down = eeprom_read_byte(&eeprom_wait_sec_down);
	
	_delay_ms(5);
	
	tmp = CheckMinMax(ADDR_WAIT_SEC_DOWN, wait_sec_down);
	
	if (wait_sec_down != tmp)
	{
		wait_sec_down = tmp;
		eeprom_write_byte(&eeprom_wait_sec_down, wait_sec_down);
		_delay_ms(5);
	}
	
	wait_sec_lock = eeprom_read_byte(&eeprom_wait_sec_lock);
	
	_delay_ms(5);
	
	tmp = CheckMinMax(ADDR_WAIT_SEC_LOCK, wait_sec_lock);
	if  (wait_sec_lock != tmp)
	{
		wait_sec_lock =tmp;
		eeprom_write_byte(&eeprom_wait_sec_lock, wait_sec_lock);
		_delay_ms(5);
	}
	 
	 //reading  10..19 joystick analogs values (LEFT) 
	eeprom_read_block((void*)&an_L.steps, (const void*)an_L_eeprom, 10);	 
	 _delay_ms(5);
	
	tmp = 0;
	for (i = 0; i <=9; i++)
	if ((an_L.steps[i] > 99) || (an_L.steps[i]  == 0)) 
	{
		tmp = 1;
		an_L.steps[i] = def_analog[i];
	}
	
	if (tmp){
		eeprom_write_block((void*)&an_L.steps,(void*)&an_L_eeprom, 10);
		_delay_ms(5);	
	}
	
	
	//reading  20..29 joystick analogs values (RIGHT)
	eeprom_read_block((void*)&an_R.steps, (const void*)an_R_eeprom, 10);	 
	 _delay_ms(5);
	
	tmp = 0;
	for (i = 0; i <=9; i++)
	if ((an_R.steps[i] > 99) || (an_R.steps[i]  == 0)) 
	{
		tmp = 1;
		an_R.steps[i] = def_analog[i];
	}
	
	if (tmp){
		eeprom_write_block((void*)&an_R.steps,(void*)&an_R_eeprom, 10);	
	}
	
	
	//reading  30..33 protection data
	
	eeprom_read_block((void*)&protection, (const void*)protection_eeprom,4);
	_delay_ms(5);
	
	tmp = 0;
	for (i = 0; i <=3; i++)
	if ((protection[i] > def_protection[i][2]) || (protection[i] < def_protection[i][1]))
	{
		tmp = 1;
		protection[i] = def_protection[i][0];
	}
	
	if (tmp){
		eeprom_write_block((void*)&protection,(void*)&protection_eeprom, 4);
		_delay_ms(5);
	}
}

void WriteEeprom(char paramfromdisp_addr)
{
	cli();
	switch (paramfromdisp_addr)
	{
		case ADDR_WAIT_SEC_UP:   eeprom_write_byte(&eeprom_wait_sec_up, wait_sec_up);
		                         break;
		case ADDR_WAIT_SEC_DOWN: eeprom_write_byte(&eeprom_wait_sec_down, wait_sec_down);
	                        	 break;
		case ADDR_WAIT_SEC_LOCK: eeprom_write_byte(&eeprom_wait_sec_lock, wait_sec_lock);
			                     break;		
		case 10 ... 19: 	     eeprom_write_block((void*)&an_L.steps,(void*)&an_L_eeprom, 10);
		                         break;
		case 20 ... 29: 	     eeprom_write_block((void*)&an_R.steps,(void*)&an_R_eeprom, 10);
		                         break;					
		case 30 ... 33: 	     eeprom_write_block((void*)&protection,(void*)&protection_eeprom, 4);
	                         	 break;			 					 							 
	}
	
    sei();
	
	_delay_ms(5);
		
}

bool _switch_filter(SwitchFilter * source)
{
	bool result = false;
	
	source->outstate = 0;
		
	if (!((*source->port) & source->pin))
	{
		if (source->curr_scan >= source->scanrate){
			
		    if (source->off) source->on_trigger = true;
		    source->off = false;
			
			result = true;
		} else
	
		source->curr_scan++;
		
		
		if  (source->scanrate == 0xFF) source->curr_scan = 0;
		
	} else
	{
	    source->curr_scan = 0;	
		source->off = true;
	}
	
	if (source->reset) result = false;
	
	return result;
}


void _joystick_filter(JoystickFilter * source)
{
	int i = 0;
	
	if ((source->in_value < source->steps[0]) || (source->in_value > source->steps[9]))
	{
		source->joystick_step = 0;
		source->enabled = false;
		
	} 
	else
	
	{
	
		source->enabled = true;	
	
		for (i = 0; i <= 4; i++) // scan 0-50 base joystick bandwidth
		if ((source->in_value > source->steps[i]) && (source->in_value <= source->steps[i+1]))
		{
		
			if (source->step_scan[i] >= source->scanrate){
			
				source->joystick_step = 4-i;
					
			} else
					
			source->step_scan[i]++;
				
			if  (source->scanrate == 0xFF) source->step_scan[i] = 0;
		
		}
		else
		source->step_scan[i] = 0;
		
		
		for (i = 9; i >= 5; i--) // scan 50-100 reserve mirror joystick bandwidth
		if ((source->in_value < source->steps[i]) && (source->in_value >= source->steps[i-1]))
		{
		
			if (source->step_scan[i] >= source->scanrate){
			
				source->mirror_joy_step = i-5;
					
			} else
					
			source->step_scan[i]++;
				
			if  (source->scanrate == 0xFF) source->step_scan[i] = 0;
		
		}
		else
		source->step_scan[i] = 0;
	
	}


}

bool CurrentProtection(AnalogFilter* ana)
{
	
	if (ana->in_value > ana->analog_limit)
	{
		if (ana->time_count >= ana->time_rate)
		{
			ana->trip = true;
		} else
		  ana->time_count++;
	} else 
	  ana->time_count = 0;
	
	return ana->trip;
}


ISR (TIMER1_OVF_vect) // 100 Hz
{
    if (Left.timer_count_en) Left.timer_count++;
    if (Left.timer_count == 0xFFFF) Left.timer_count = 0;

    if (Right.timer_count_en) Right.timer_count++;
    if (Right.timer_count == 0xFFFF) Right.timer_count = 0;

    w100Hz++;
	

   
    if (((w100Hz % WAIT_5Hz) == 0) && (w100Hz > 0))
	{
		twi_process_read = false;
		twi_process_write = true;
	}
	
	if (w100Hz >= WAIT_1Hz)
	{
		        	
		twi_process_read = true;
		twi_process_write = false;
		w100Hz = 0;
		
	}
	
	
    in_left.outstate = _switch_filter(&in_left);
	in_right.outstate = _switch_filter(&in_right);
	

	sw_leftdown.outstate = _switch_filter(&sw_leftdown);
	sw_leftup.outstate = _switch_filter(&sw_leftup);
	   
	sw_rightdown.outstate = _switch_filter(&sw_rightdown);
	sw_rightup.outstate = _switch_filter(&sw_rightup);
	
	an_R.in_value = val_R;
	_joystick_filter(&an_R);
	
	an_L.in_value = val_L;
	_joystick_filter(&an_L);
	
	val11.in_value = val_ANA11;
	CurrentProtection(&val11);
	val12.in_value = val_ANA12;
	CurrentProtection(&val12);
	val21.in_value = val_ANA21;
	CurrentProtection(&val21);
	val22.in_value = val_ANA22;	
	CurrentProtection(&val22);
	
	Left.switch_down = sw_leftdown.outstate;
	Left.switch_up = sw_leftup.outstate;
	Right.switch_down = sw_rightdown.outstate;
	Right.switch_up = sw_rightup.outstate;
	
	if ((timer1_count > 0) && (timer1_count < TIMER1_MAXCOUNT))
	{
			timer1_count++;
	}

    TCNT1 = WAIT_100HZ;
}
/*

bool Left_Channel_Button()
{
	bool result = false;
	
	if (in_left.outstate) {
		
			ledL_on_down = 0;
			if (!stL) {
				count_en_L = 1; //count enable
				channelL_on = 0;
			}
			
			stL = 1;
			operation_L = true;
			
			
			if ((channelL_on >= wait_sec_lock*10) && (channelR_on >= wait_sec_lock*10) && stR){
				lock1 = 1;
			} else
			{
				lock1 = 0;
			}
			
			
			PORTD &= ~(ON_L1);
			

			if ((channelL_on < wait_sec_up*10) && (!sw_leftup.outstate) && (count_en_L)){
				PORTB |=  ON_L2; //Channel Left ON turn Forward
				ledL_on_up=1;
			} else
			{		
				PORTB &= ~(ON_L2);
				ledL_on_up=0;		
			}
	
	} else
	{
		ledL_on_up = 0;
		
		if (stL) {
			_delay_ms(CURRENT_OFF_DELAY); // current off delay
			count_en_L = 1;
			channelL_on = 0;
		}
		
		stL = 0;
		
		PORTD |=  ON_L1;
		
		if ((channelL_on < wait_sec_down*10) && (!lock1) && (!sw_leftdown.outstate) && (count_en_L)){
			PORTB &= ~(ON_L2); //Channel Left ON turn Backward
			ledL_on_down=1;
		} else
		{
			PORTB |=  ON_L2;
			ledL_on_down=0;
			operation_L = false;
		}
		
	}
	
	if (channelL_on >= wait_sec_lock*10) 
	{
		count_en_L=0;
	}
	
	if (ledL_on_up || ledL_on_down) result = true;
		
	return result;
}

bool Right_Channel_Button()
{
	bool result = false;
	
	if (in_right.outstate) {

            ledR_on_down = 0;
			
			if (!stR) {
				count_en_R = 1;
				channelR_on = 0;
			}
			
			stR = 1;
			operation_R = true;

			if ((channelL_on >= wait_sec_lock*10) && (channelR_on >= wait_sec_lock*10)  && stL){
				lock2 = 1;
			} else
			{
				lock2 = 0;
			}
			
			PORTD &= ~(ON_R1);

			
			if ((channelR_on < wait_sec_up*10) && (!sw_rightup.outstate) && (count_en_R)){
				PORTD |=  ON_R2; //Channel 2 RIGHT ON turn Forward
				ledR_on_up= 1;
			} else
			{
				
				PORTD &= ~(ON_R2);
				ledR_on_up=0;
				
			}
	
	} else
	{
		ledR_on_up = 0;
		
		if (stR) {
			_delay_ms(CURRENT_OFF_DELAY); // current off delay
			count_en_R = 1;
			channelR_on = 0;
		}
		
		stR = 0;
		
		PORTD |= ON_R1;
		
		if ((channelR_on < wait_sec_down*10) && (!lock2) && (!sw_rightdown.outstate) && (count_en_R)) {
			PORTD &= ~(ON_R2);  //Channel RIGHT ON turn Backward
			ledR_on_down=1;
		} else
		{
			PORTD |=ON_R2;
			ledR_on_down=0;
			operation_R = false;
		}
		
	}
	
	if (channelR_on >= wait_sec_lock*10) 
	{
		count_en_R=0;
	}
	
	if (ledR_on_up || ledR_on_down) result = true;
	
	return result;
}

void Jog_R_Drive(unsigned char step) //jog right drive to step position
{
	unsigned int relative_step;
	unsigned int condition;
	
    if ((step > curr_R_step) || jog_started_R_fwd)  // Forward
	{
		relative_step = step - curr_R_step;
		
		if (jog_started_R_fwd && (relative_step == 0)) relative_step = 1;
		
		PORTD &= ~(ON_R1);
		
		if (!jog_stR) {
			
			jogR_count_en = true;
			jog_started_R_fwd = true;
			jog_started_R_bwd = false;
			jogR_on = 0;
		}
		
		jog_stR = true;
		
		condition = wait_sec_up*10/4*relative_step;
		
    	if ((jogR_on <= condition) && (!sw_rightup.outstate))
	    {   		
			PORTD |=  ON_R2; //Channel RIGHT ON turn Forward
			ledR_on_up= 1;
			jogR_count_en = true;
		} else
		{   
			if (jogR_count_en) curr_R_step +=relative_step;
			jog_started_R_fwd = false;
			jogR_count_en = false;
			jogR_on = 0;
			PORTD &= ~(ON_R2);
			ledR_on_up= 0;
		}
	 }
	 
    if ((step < curr_R_step) || jog_started_R_bwd)   // Backward
	{
		relative_step = curr_R_step - step;
		
		ledR_on_up= 0;
		
		if (jog_started_R_bwd && (relative_step == 0)) relative_step = 1;
		
		if (jog_stR) {
			_delay_ms(CURRENT_OFF_DELAY); // current off delay
			jogR_count_en = true;
			jog_started_R_bwd = true;
			jog_started_R_fwd = false;
			jogR_on = 0;
		}
				
		jog_stR = false;
		
		PORTD |= ON_R1;
		
		condition = wait_sec_up*10/4*relative_step;
		
		if (step == 0) { 
			  condition = wait_sec_down*10;  // big backward time guaranteed to return to actuator starting position 
		}
		if ((jogR_on <= condition) && (!sw_rightdown.outstate))
		{
		    PORTD &= ~(ON_R2);  //Channel RIGHT ON turn Backward
			ledR_on_down=1;
			jogR_count_en = true;
			
		} else
		{
			
		    
			if ((jogR_count_en) && (step > 0)) curr_R_step -=relative_step;
			
			if ((jogR_count_en) && (step == 0)) curr_R_step = 0; //fuse condition
		    jogR_count_en = false;
			jog_started_R_bwd = false;
		    jogR_on = 0;
		   
			PORTD |=ON_R2;
			ledR_on_down=0;
		
		}
		   
	 }
	
}

bool ADC_Set_R_Drive()
{ 
	bool result = false;
	unsigned int step = 0;
	
 
	if (an_R.enabled)
	{
		step = an_R.joystick_step;
	} else
	    step = an_L.mirror_joy_step;
	
	
	if (an_L.enabled || an_R.enabled)
	if ((step > 0) || (curr_R_step > 0) || jog_started_R_bwd || jog_started_R_fwd)  
	{
		Jog_R_Drive(step);
		result = true;
	}
	
	     
	
	return result;
}


void Jog_L_Drive(unsigned char step) //jog left drive to step position
{
	unsigned int relative_step;
	unsigned int condition;
	
	if ((step > curr_L_step) || jog_started_L_fwd)  // Forward
	{
		relative_step = step - curr_L_step;
		
		if (jog_started_L_fwd && (relative_step == 0)) relative_step = 1;
		
		PORTD &= ~(ON_L1);
		
		if (!jog_stL) {
			
			jogL_count_en = true;
			jog_started_L_fwd = true;
			jog_started_L_bwd = false;
			jogL_on = 0;
		}
		
		jog_stL = true;
		
		condition = wait_sec_up*10/4*relative_step;
		
		if ((jogL_on <= condition) && (!sw_leftup.outstate))
		{
			PORTB |=  ON_L2; //Channel RIGHT ON turn Forward
			ledL_on_up= 1;
			jogL_count_en = true;
		} else
		{
			if (jogL_count_en) curr_L_step +=relative_step;
			jog_started_L_fwd = false;
			jogL_count_en = false;
			jogL_on = 0;
			PORTB &= ~(ON_L2);
			ledL_on_up= 0;
		}
	}
	
	if ((step < curr_L_step) || jog_started_L_bwd)   // Backward
	{
		relative_step = curr_L_step - step;
		
		ledL_on_up= 0;
		
		if (jog_started_L_bwd && (relative_step == 0)) relative_step = 1;
		
		if (jog_stL) {
			_delay_ms(CURRENT_OFF_DELAY); // current off delay
			jogL_count_en = true;
			jog_started_L_bwd = true;
			jog_started_L_fwd = false;
			jogL_on = 0;
		}
		
		jog_stL = false;
		
		PORTD |= ON_L1;
		
		condition = wait_sec_up*10/4*relative_step;
		
		if (step == 0) {
			condition = wait_sec_down*10;  // big backward time guaranteed to return to actuator starting position
		}
		if ((jogL_on <= condition) && (!sw_leftdown.outstate))
		{
			PORTB &= ~(ON_L2);  //Channel LEFT ON turn Backward
			ledL_on_down=1;
			jogL_count_en = true;
			
		} else
		{
			
			
			if ((jogL_count_en) && (step > 0)) curr_L_step -=relative_step;
			
			if ((jogL_count_en) && (step == 0)) curr_L_step = 0; //fuse condition
			jogL_count_en = false;
			jog_started_L_bwd = false;
			jogL_on = 0;
			
			PORTB |=ON_L2;
			ledL_on_down=0;
			
		}
		
	}
	
}

bool ADC_Set_L_Drive()
{  
	
	unsigned int step = 0;
	bool result = false;
	
	
	if (an_L.enabled) 
	{
		step = an_L.joystick_step;
	} else
	    step = an_R.mirror_joy_step;
		
	if (an_L.enabled || an_R.enabled)	
	if ((step > 0) || (curr_L_step > 0) || jog_started_L_bwd || jog_started_L_fwd)
	{
		Jog_L_Drive(step);
		result = true;
	}
	
	
	return result;
}
*/

void init_switches(){
		
		in_left.outstate=false;
		in_left.reset = false;
		in_left.on_trigger = false;
		in_left.curr_scan=0;
		in_left.scanrate=5;
		in_left.port = &PINB;
		in_left.pin = SW_L;
		
		in_right.outstate=false;
		in_right.on_trigger = false;
		in_right.reset = false;
		in_right.curr_scan=0;
		in_right.scanrate=5;
		in_right.port = &PINB;
		in_right.pin = SW_R;		
		
		sw_leftdown.outstate=false;
		sw_leftdown.curr_scan=0;
		sw_leftdown.scanrate=2;
		sw_leftdown.port = &PIND;
		sw_leftdown.pin = LEFT_DOWN;
		
		sw_leftup.outstate=false;
		sw_leftup.curr_scan=0;
		sw_leftup.scanrate=2;
		sw_leftup.port = &PIND;
		sw_leftup.pin = LEFT_UP;

		sw_rightdown.outstate=false;
		sw_rightdown.curr_scan=0;
		sw_rightdown.scanrate=2;
		sw_rightdown.port = &PIND;
		sw_rightdown.pin = RIGHT_DOWN;		

		sw_rightup.outstate=false;
		sw_rightup.curr_scan=0;
		sw_rightup.scanrate=2;
		sw_rightup.port = &PIND;
		sw_rightup.pin = RIGHT_UP;	
		
}


/*

ADC0 - AN_X_L
ADC1 - AN_X_R
ADC2 - ANA11 (left forward)
ADC3 - ANA12 (left backward)
ADC4 - not used
ADC5 - not used
ADC6 - ANA21 (right forward)
ADC7 - ANA22 (right backward)

*/

void init_protections()
{
	val11.trip = false;
	val11.in_value = 0;
	val11.time_rate = protection[1];
	val11.time_count = 0;
	val11.analog_limit = protection[0]*CURRENT_COEFFICIENT; 

	val12.trip = false;
	val12.in_value = 0;
	val12.time_rate = protection[1];
	val12.time_count = 0;
	val12.analog_limit = protection[0]*CURRENT_COEFFICIENT; 	
	
	val21.trip = false;
	val21.in_value = 0;
	val21.time_rate = protection[3];
	val21.time_count = 0;
	val21.analog_limit = protection[2]*CURRENT_COEFFICIENT; 
	
	val22.trip = false;
	val22.in_value = 0;
	val22.time_rate = protection[3];
	val22.time_count = 0;
	val22.analog_limit = protection[2]*CURRENT_COEFFICIENT; 
}

ISR (ADC_vect) 
{
	
	cli();
	unsigned char _admux = ADMUX & ((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
	
	ADMUX &= 0b11110000;
	
	switch(_admux)
	{
		case AN_X_L:
	                 val_L = ADCW/10;
					 ADMUX |=AN_X_R;
					 break;
	    case AN_X_R: val_R =   ADCW/10;
                     ADMUX |=ANA11;
		             break;
	    case ANA11: val_ANA11 =   ADCW;
		    	     ADMUX |= ANA12;
			         break;	
	    case ANA12: val_ANA12 =   ADCW;
	                 ADMUX |=ANA21;
	                 break;	
	    case ANA21: val_ANA21 =   ADCW;
	                 ADMUX |=ANA22;
	                 break;			
	    case ANA22: val_ANA22 =   ADCW;
			         ADMUX |=AN_X_L;
			         break;		 	 
	    default:     ADMUX &= 0b11110000;
	                 ADMUX |=AN_X_L;
					 break;
	}
	 
   sei();

	ADCSRA |=(1<<ADSC); 
}

void ADC_Init()
{
	// Joystick L resistor initialization
	
	an_L.scanrate = 2;	
	an_L.joystick_step = 0;
	an_L.mirror_joy_step = 0;
	an_L.enabled = false;	
	
	// Joystick R resistor initialization

	an_R.scanrate = 2;	
	an_R.joystick_step = 0;
	an_R.mirror_joy_step = 0;
	an_R.enabled = false;
	
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0) | (1 << ADIE);  //ADC 32 divider
	ADMUX = (0 << REFS1) | (1 << REFS0) | AN_X_L; 
	ADCSRA |=(1<<ADSC);
}
/*

void Reset_Jog_L()
{
	jogL_count_en = false;
	jog_started_L_fwd = false;
	jog_started_L_bwd = false;
	jogL_on = 0;
}

void Reset_Jog_R()
{
	jogR_count_en = false;
	jog_started_R_fwd = false;
	jog_started_R_bwd = false;
	jogR_on = 0;
}

void Stop_L_channel()
{
		PORTD &= ~(ON_L1); // stop L channel
		PORTB &= ~(ON_L2);
		ledL_on_down = false;
		ledL_on_up = false;
		stL = 0;
		channelL_on = 0;
		count_en_L = false;		
		
		Reset_Jog_L();
	
}

void Stop_R_channel()
{
		PORTD &= ~(ON_R1); //Stop R channel
		PORTD &= ~(ON_R2);
	    ledR_on_down = false;
		ledR_on_up = false;
		stR = 0;
		channelR_on = 0;
		count_en_R = false;
		
		Reset_Jog_R();
}
*/

void init_channels()
{
	Left.port1 = &PORTD;
	Left.pin1 = ON_L1;
	Left.port2 = &PORTB;
	Left.pin2 = ON_L2;
	Left.InitChannel();

	Right.port1 = &PORTD;
	Right.pin1 = ON_R1;
	Right.port2 = &PORTD;
	Right.pin2 = ON_R2;
	Right.InitChannel();
}

int main(void)
{	 
	uint8_t step = 0;
	 
    DDRD = ON_L1 | ON_R1 | ON_R2;
	DDRB = ON_L2; 
		
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP1);
	
	TCCR1A = 0;
	TCCR1B = (1 << CS11)|(1 << CS10); //64 divider
	
	TCNT1 = WAIT_100HZ;
	
	TIFR1 = (1<<TOV1);
	TIMSK1 = (1<<TOIE1);
	
	sei();
	
	
	LoadEeprom();
    init_switches();
	init_channels();
	init_protections();
	ADC_Init();
	twi_init();
  
    while (1) 
    {
		asm("wdr"); 

        if ((!val11.trip) && (!val12.trip))
		{
			if (in_left.outstate)  // Button L process
			{
				Left.control_source = true;
				Left.goal_step = 4;				
			}	
			else   //Joystick Process
			{
				if (Left.control_source)
				{
				    Left.goal_step = 0;
				    
					if ((Left.curr_step == 0) && (!Left.moving_bwd)) Left.control_source = false;
					 
				} else
				{
					if (an_L.enabled)
					{
						step = an_L.joystick_step;
					} else
						step = an_R.mirror_joy_step;
						
						
					if (an_L.enabled || an_R.enabled)
					if ((step > 0) || (Left.curr_step > 0) || Left.moving_bwd || Left.moving_fwd )
					{
							Left.goal_step = step;
							
					}
						
				}		
				
			}
			

			if (in_left.on_trigger)
			{
				Left.lock = false;
				in_left.on_trigger = false;
			}

			if (Left.lock && Right.lock) Left.goal_step = 4;
			
			Left.Process(); // Process L channel
					
		} else
		{
            //Stop Left Channel
			Left.Stop();
			in_left.reset = true;
		}


        if ((!val21.trip) && (!val22.trip))
        {
			
			if (in_right.outstate)  // Button R process
			{
				Right.control_source = true;
				Right.goal_step = 4;				
			}	
			else   //Joystick Process
			{
				if (Right.control_source)
				{
				    Right.goal_step = 0;
				    
					if ((Right.curr_step == 0) && (!Right.moving_bwd)) Right.control_source = false;
				} else
				{
					if (an_R.enabled)
					{
						step = an_R.joystick_step;
					} else
						step = an_L.mirror_joy_step;
						
						
					if (an_L.enabled || an_R.enabled)
					if ((step > 0) || (Right.curr_step > 0) || Right.moving_bwd || Right.moving_fwd)
					{
							Right.goal_step = step;
							
					}
						
				  
				}			
			}

			if (in_right.on_trigger)
			{
				Right.lock = false;
				in_right.on_trigger = false;
			}
			
			if (Right.lock && Left.lock) Right.goal_step = 4;
			Right.Process(); // Process R channel		
			
		} else
		{
            // Stop Right Channel
			Right.Stop();
			in_right.reset = true;
		}
		
		if (in_left.off && in_left.reset) {
				val11.trip = false;
				val12.trip = false;
				in_left.reset = false;
		}


		if (in_right.off && in_right.reset) {
				val21.trip = false;
				val22.trip = false;
				in_right.reset = false;
		}
		
	    
		ledL_on_down = Left.moving_bwd;
     	ledL_on_up = Left.moving_fwd;
		ledR_on_down = Right.moving_bwd;
		ledR_on_up = Right.moving_fwd;
		
		statebyte0 = ((ledL_on_down | ledL_on_up) << STATUS_BIT0_L_ON) | ((ledR_on_down | ledR_on_up)  << STATUS_BIT0_R_ON) | 
		            (sw_leftdown.outstate << STATUS_BIT0_L_D) | (sw_leftup.outstate << STATUS_BIT0_L_U) | 
		            (sw_rightdown.outstate << STATUS_BIT0_R_D) | (sw_rightup.outstate << STATUS_BIT0_R_U) | (in_left.outstate << STATUS_BIT0_L_BUT) | (in_right.outstate << STATUS_BIT0_R_BUT);
        
		statebyte1 = (1 << STATUS_BIT1_L_EN) | (1 << STATUS_BIT1_R_EN) | (ledL_on_up << STATUS_BIT1_L_ON_U) | (ledL_on_down << STATUS_BIT1_L_ON_D) | 
																		 (ledR_on_up << STATUS_BIT1_R_ON_U) | (ledR_on_down << STATUS_BIT1_R_ON_D) |
																		 ((val11.trip | val12.trip) << STATUS_BIT1_L_TRIP) |
																		 ((val21.trip | val22.trip) << STATUS_BIT1_R_TRIP)  ;

		if  (twi_process_read)
		{
			
			TWI_DataAddr = 0x04;
			TWI_DataSize  = 4;
			
			
			twi_read();  // Read Data from Display board
			
			twi_process_read = false;
					
			timer1_count = 1; // startup timer
			
			while ((TWI_Status != TWI_FREE) && (timer1_count < TIMER1_MAXCOUNT)) //fuse condition
			{
				// wait TWI reading
			}
			
			timer1_count = 0;
			
			paramtodisp_addr =   TWI_DataBuff[1];
			paramfromdisp =      TWI_DataBuff[2];
			paramfromdisp_addr = TWI_DataBuff[3];
			
			
			
			switch (paramfromdisp_addr)
			{
				case ADDR_WAIT_SEC_UP: if (paramfromdisp != wait_sec_up)
				{
				    wait_sec_up = CheckMinMax(ADDR_WAIT_SEC_UP, paramfromdisp); 
					WriteEeprom(paramfromdisp_addr);	                  
				}
				break;
				case ADDR_WAIT_SEC_DOWN: 
				if (paramfromdisp != wait_sec_down)
				{
					wait_sec_down = CheckMinMax(ADDR_WAIT_SEC_DOWN,paramfromdisp); 
					WriteEeprom(paramfromdisp_addr);
				}
				break;	
				
				case ADDR_WAIT_SEC_LOCK: 
				if (paramfromdisp != wait_sec_lock)
				{
					wait_sec_lock = CheckMinMax(ADDR_WAIT_SEC_LOCK,paramfromdisp); 
					WriteEeprom(paramfromdisp_addr);
				}
				
				break;	
				case 10 ... 19:
				if (paramfromdisp != an_L.steps[paramfromdisp_addr-10])		
				{
					if ((paramfromdisp < 100) || (paramfromdisp > 0))
					{
						an_L.steps[paramfromdisp_addr-10] = paramfromdisp;
						WriteEeprom(paramfromdisp_addr);
					}
				}			
				break;		
				
				case 20 ... 29:
				if (paramfromdisp != an_R.steps[paramfromdisp_addr-20])		
				{
					if ((paramfromdisp < 100) || (paramfromdisp > 0))
					{
						an_R.steps[paramfromdisp_addr-20] = paramfromdisp;
						WriteEeprom(paramfromdisp_addr);
					}
				}	
				break;	
				
				case 30 ... 33:
				if (paramfromdisp != protection[paramfromdisp_addr-30])
				{
				  	if ((protection[paramfromdisp_addr-30] <= def_protection[paramfromdisp_addr-30][2]) || 
					    (protection[paramfromdisp_addr-30] >= def_protection[paramfromdisp_addr-30][1]))
				  	{
				        protection[paramfromdisp_addr-30] = paramfromdisp;
						WriteEeprom(paramfromdisp_addr);
						init_protections();
					}
				
				}
				break;
				default: paramfromdisp = 0;
			}
			
		}
		
		if (twi_process_write)
		{

			TWI_DataAddr = 0x01;
			TWI_DataSize  = 3;
			TWI_DataBuff[0] = statebyte0;
			TWI_DataBuff[1] = statebyte1;
			
			switch (paramtodisp_addr)
			{
				case ADDR_WAIT_SEC_UP: paramtodisp = wait_sec_up;
				           break;
			    case ADDR_WAIT_SEC_DOWN: paramtodisp = wait_sec_down;
			               break;
			    case ADDR_WAIT_SEC_LOCK: paramtodisp = wait_sec_lock;
			               break;
				case ADDR_PERSENTAGE_L: paramtodisp = val_L;
						  break;
			    case ADDR_CMD_L: paramtodisp = Left.goal_step*10+Left.curr_step;
				          break;
				case ADDR_ENABLE_L: paramtodisp = an_L.enabled;		 
					      break;
				case ADDR_PERSENTAGE_R: paramtodisp =val_R;
						  break;
				case ADDR_CMD_R: paramtodisp = Right.goal_step*10+Right.curr_step;
				          break;
				case ADDR_ENABLE_R: paramtodisp = an_R.enabled;  
				          break;	
				case 10 ... 19: paramtodisp = an_L.steps[paramtodisp_addr-10];	
				          break;
				case 20 ... 29: paramtodisp = an_R.steps[paramtodisp_addr-20];
						  break;		  	
				case 30 ... 33: paramtodisp = protection[paramtodisp_addr-30];	
				          break;
			    default:   paramtodisp = 0;
			}
			
			TWI_DataBuff[2] = paramtodisp; 
			
			twi_write(); // Send Data to Display board
			
			twi_process_write = false;
		}
    }
}

