/*
 * friction.cpp
 * v2.5.2. Friction Clutch Controller
 *
 * Created: 19/01/2021 
 * Author : vks
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "twi.h"
#include "mcp2515.h"


//#define F_CPU                  16000000UL
#include <util/delay.h>

/*

GAZ-71 Frictions control board v3.0 & v4.0

Firmware v2.5.
PCBs v3.0 & Module v4.0


*/

#define RESERVE1 (1 << PB0)  //Reserve 1 Output
#define RESERVE2 (1 << PB1)  //Reserve 2 Input

#define ON_R1 (1 << PB2)  // Right Forward Output
#define ON_R2 (1 << PB3)  //Right Backward Output

#define SW_R (1 << PD0) // Release Right
#define RIGHT_DOWN (1 << PD1) // Right Down limit switch
#define RIGHT_UP (1 << PD2) // Right Up limit switch
#define SQUEEZE (1 << PD3) // Release and stay both sides
#define WATER_MODE (1 << PD4) // Water Mode - slowly going back
#define LEFT_DOWN (1 << PD5) // Left Down limit switch
#define LEFT_UP (1 << PD6) // Left Up limit switch
#define SW_L (1 << PD7) // Release Left

#define ON_L1 (1 << PA6)  //Left Forward Output
#define ON_L2 (1 << PA7)  //Left Backward Output


#define CURRENT_OFF_DELAY 20   //current off delay 20 ms
#define CURRENT_COEFFICIENT 5

#define WAIT_10kHZ 0x37
#define WAIT_100HZ 0xF63B
#define WAIT_5Hz 20
#define WAIT_1Hz 100

#define DEF_WAIT_SEC_UP 9   //forward moving default time
#define DEF_WAIT_SEC_DOWN 20 //backward moving default time
#define DEF_MOVING_UP_STOP 10 //

#define ADDR_WAIT_SEC_UP	 0x01
#define ADDR_WAIT_SEC_DOWN   0x02
#define ADDR_MOVING_UP_STOP	 0x03
#define ADDR_PERSENTAGE_L	 0x04
#define ADDR_CMD_L			 0x05
#define ADDR_ENABLE_L		 0x06
#define ADDR_PERSENTAGE_R	 0x07
#define ADDR_CMD_R			 0x08
#define ADDR_ENABLE_R		 0x09

#define MIN_WAIT_SEC_UP 3
#define MIN_WAIT_SEC_DOWN 3
#define MIN_MOVING_UP_STOP 0

#define MAX_WAIT_SEC_UP 100
#define MAX_WAIT_SEC_DOWN 100
#define MAX_MOVING_UP_STOP 100

#define TIMER1_MAXCOUNT 10

#define  PROTECTIONS_COUNT 6

#define SQUELCH 5; // CURRENT OFF protection: reject noise if PWM on

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
#define STATUS_BIT1_L_TRIP_STAGE1 6   // L TRIP current protection jamming
#define STATUS_BIT1_R_TRIP_STAGE1 7   // R TRIP current protection jamming

//status byte 2

#define STATUS_BIT2_L_TRIP_STAGE2 0  // L TRIP over current protection short circuit
#define STATUS_BIT2_R_TRIP_STAGE2 1  // R TRIP over current protection short circuit
#define STATUS_BIT2_WATERMODE 2		 // Water Mode
#define STATUS_BIT2_SQUEEZE 3        // Release both sides
#define STATUS_BIT2_HOLD_ON_LOAD 4   // hold drive under load

#define ANA22 (0 << MUX3)|(0 << MUX2)|(0 << MUX1)|(0 << MUX0) //ADC0
#define ANA21 (0 << MUX3)|(0 << MUX2)|(0 << MUX1)|(1 << MUX0) //ADC1
#define ANA11 (0 << MUX3)|(0 << MUX2)|(1 << MUX1)|(0 << MUX0)  //ADC2
#define ANA12 (0 << MUX3)|(0 << MUX2)|(1 << MUX1)|(1 << MUX0)  //ADC3
#define AN_X_L (0 << MUX3)|(1 << MUX2)|(0 << MUX1)|(0 << MUX0)  //ADC4
#define AN_X_R (0 << MUX3)|(1 << MUX2)|(0 << MUX1)|(1 << MUX0)  //ADC5

struct SwitchFilter{
	bool outstate;
	unsigned char scanrate;
	volatile unsigned char *port;
	unsigned char pin;
	unsigned char curr_scan;
	bool reset;
	bool off;
	bool on_trigger;
	bool off_trigger;
	bool protection_reset;
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
	  bool control_source; //0 - joystick, 1 - button
	  bool water_mode;  
	  bool long_release;
	  bool forward_end_trigger;

      uint8_t timer_pwm_backward;
	  uint8_t timer_pwm_hold_on;
	  uint8_t water_dc;
	  uint8_t hold_on_dc;
	  uint8_t normal_mode_dc;
	  
	  uint16_t timer_count;
	  uint8_t goal_step;
	  uint8_t curr_step;
	  uint16_t prev_timer_count_position;
	  float timer_count_position;

	  bool timer_count_en;
	  bool moving_up_stop_trip;
	  bool moving_fwd;
	  bool moving_bwd;
	  bool jog_direction;
	  bool lock; 
	  bool prev_lock_state;

	  volatile unsigned char *port1;
	  uint8_t pin1;
	  
	  volatile unsigned char *port2;
	  uint8_t pin2;
	  	  
	  void InitChannel();
	  void Forward();
	  void ForwardHoldOn();
	  void Backward();
	  void VoltageOff();
	  void Stop();
	  int8_t Process();
};


ChannelControl Left, Right;

bool count_en_L, count_en_R, stL, stR, ledL_on_up, ledR_on_up, ledL_on_down, ledR_on_down, operation_L, operation_R, hold_on_load = false;
uint16_t channelL_on, channelR_on = 0;


bool twi_process_read, twi_process_write = false;

SwitchFilter in_left, in_right, sw_leftdown, sw_leftup, sw_rightdown, sw_rightup, sw_water, sw_squeeze;
JoystickFilter an_R, an_L;
AnalogFilter val11, val12, val21, val22; //jamming protection
AnalogFilter oc_val11, oc_val12, oc_val21, oc_val22;  //over current protection (short circuit)

uint8_t w100Hz, readbyte_disp = 0;

uint8_t statebyte0, statebyte1, statebyte2, paramtodisp, paramtodisp_addr, paramfromdisp, paramfromdisp_addr = 0;

uint16_t val_L, val_R, val_ANA11, val_ANA12, val_ANA21, val_ANA22 = 0;
uint16_t jogL_on, jogR_on = 0;
bool jogR_count_en, jogL_count_en = true;
bool jog_stR, jog_stL = false;
uint16_t curr_R_step, curr_L_step = 0;
bool jog_started_R_fwd, jog_started_R_bwd = false;
bool jog_started_L_fwd, jog_started_L_bwd = false;

uint8_t def_analog[] = {10,25,30,35,40,61,66,71,76,90}; //default analog values

													  //30-33 parameter, protection data:
													  //Default value, minimal value, maximal value, decimal point position
uint8_t def_protection[PROTECTIONS_COUNT][4] = {{10,1,50,0},     //L current (A) jamming,
	                                 {10,1,99,2},     // L times x 10ms jamming,
									 {10,1,50,0},     //R current (A) jamming,
									 {10,1,99,2},     //R times x 10ms jamming
									 {45,0,80,0},     //L current (A), short circuit
									{45,0,80,0}};     //R current (A), short circuit
										
										// 36-39 parameter, pulse moving parameters
uint8_t def_pulsemoving[4][4] = {{0,0,10,0},     //Hold on L drive on release *5%
								 {0,0,10,0},     //Hold on R drive on release *5%
								 {10,1,10,0},    //Backward speed  *10%
								 {4,1,10,0}};   //Watermode speed *10% 
									 
uint8_t def_logic_release[4] = {0,0,1,0}; //40 HAN logic release
	
													//41-42 CAN BUS parameters
uint8_t def_can_bus[2][4] = {{4,0,6,0},				//CAN BUS baudrate 4 = 125kbps
	                         {0x11,0x01,0xFE,0}};	//CAN BUS address
								   

uint8_t EEMEM eeprom_wait_sec_down;
uint8_t EEMEM eeprom_moving_up_stop;
uint8_t EEMEM eeprom_wait_sec_up;
uint8_t EEMEM an_L_eeprom[10];
uint8_t EEMEM an_R_eeprom[10];
uint8_t EEMEM protection_eeprom[PROTECTIONS_COUNT];
uint8_t EEMEM pulsemoving_eeprom[4];
uint8_t EEMEM logic_release_eeprom;
uint8_t EEMEM can_bus_eeprom[2];

unsigned char  wait_sec_up, wait_sec_down, moving_up_stop = 0;
uint8_t timer1_count = 0;
uint8_t protection[PROTECTIONS_COUNT];
uint8_t pulsemoving[4];
uint8_t logic_release;
uint8_t can_bus[2];

bool control_L, control_R = false; // Control source (JOYSTICK/BUTTON)


void ChannelControl::InitChannel()
{
	lock = false;
	prev_lock_state = false;
	prev_timer_count_position = 0;
	timer_count_position = 0;
	moving_up_stop_trip = false;
	switch_up = false;
	control_source = false;
	switch_down = false;
	timer_count = 0;
	moving_fwd = false;
	moving_bwd = false;
	timer_count_en = false;
	long_release = false;
	
	timer_pwm_backward = 0;
	timer_pwm_hold_on = 0;
	
	water_dc = 0;
	hold_on_dc = 0;
	normal_mode_dc = 0;
	
	water_mode = false;
	curr_step = 0;
	goal_step = 0;
}

void ChannelControl::Backward()
{
	//188Hz PWM 
	
	if (((!water_mode) && (timer_pwm_backward <= normal_mode_dc)) || ((water_mode) && (timer_pwm_backward <= water_dc)))
	{
		*port1 |= pin1; // Moving backward
		*port2 &= ~(pin2);
	} 
	else 
	
	VoltageOff();
	
	
}

void ChannelControl::VoltageOff()
{
	*port1 &= ~(pin1); // Turn off voltage on motor
	*port2 &= ~(pin2);
}

void ChannelControl::ForwardHoldOn()
//  94 Hz PWM
{
    if (hold_on_dc > 0)
    {
		 moving_fwd = true;
		 moving_bwd = false;
		  
		if (timer_pwm_hold_on < hold_on_dc)
		{
		  *port1 &= ~(pin1);
		  *port2 |=  pin2;
		  

	   } else VoltageOff();
	} 
	
	else Stop();

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
    VoltageOff();
	moving_fwd = false;
	moving_bwd = false;
}


int8_t ChannelControl::Process()
{
		uint16_t relative_step = 0;
	    uint16_t condition = 0;
		uint16_t condition2 = 0;
		
		uint16_t coefficient_fwd, coefficient_bwd = 10;
		
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
			
			
			if (!control_source) coefficient_fwd = 12;   // If it is Joystick control, increase forward time
			  
		    else
			    coefficient_fwd = 10;
				
			if (long_release) coefficient_fwd = 12; // Increase release time if SQUEEZE input active
			
			condition2 = moving_up_stop*coefficient_fwd;
			condition = wait_sec_up*coefficient_fwd/4*relative_step;

			
	        if (timer_count != prev_timer_count_position)
	 	    {
                 timer_count_position++;                    // Increase real actuator position timer
			      prev_timer_count_position = timer_count;
			} 
			
			moving_up_stop_trip = (timer_count_position >= (float) condition2) && (moving_up_stop >= wait_sec_up);
			
			  	
			
			
			if ((timer_count <= condition) && (curr_step < 4) && (!switch_up) && (!moving_up_stop_trip))
			{
				Forward(); //Channel turn Forward
				
				timer_count_en = true;
				timer_pwm_hold_on = 0;
				forward_end_trigger = false;
				
			
			} else
			{

				//if (!switch_up) curr_step = 4;
			
				if ((timer_count_en) && (curr_step < 4)) curr_step +=relative_step;

				if ((curr_step < 4) || (!control_source))
				{
					
				  timer_count_en = false;
				  timer_count = 0;
				
				}
				
				if ((!forward_end_trigger) && (curr_step == 4) && (hold_on_dc > 0))
				{
					VoltageOff();
					_delay_ms(CURRENT_OFF_DELAY);
					
					forward_end_trigger = true;
				}
				 		
				if ((!switch_up) && (curr_step == 4) && (hold_on_dc > 0)) {
					
					timer_count_en = false;					  
					ForwardHoldOn(); // Hold on pulsing on released actuator under load
				} else
				
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
			
			
			if (water_mode) 
				coefficient_bwd = 100/water_dc;
			else
			   coefficient_bwd = 10;
			
			
			if (goal_step == 0) {
				
				condition = wait_sec_down*coefficient_bwd;  // big backward time guaranteed to return to actuator starting position
			} 
			else
			condition = wait_sec_up*coefficient_bwd/4*relative_step;
						
			
	        if (timer_count !=  prev_timer_count_position)
	        {
		        if (timer_count_position > 0) timer_count_position = timer_count_position - (10 / (float) coefficient_bwd);  //Decrease real actuator position timer
		        
				if (timer_count_position < 0) timer_count_position = 0;
				prev_timer_count_position = timer_count;
	        }
			
			if ((timer_count <= condition) && (!switch_down))
			{
				Backward();  //Channel turn Backward
				timer_count_en = true;
				
			} else  
			{
				
				
				if ((timer_count_en) && (goal_step > 0)) curr_step -=relative_step; //
				
				if ((timer_count_en) && (goal_step == 0)) curr_step = 0; //fuse condition
				
				timer_count_en = false;
				
				timer_count = 0;
				
				Stop();			
			}
			
		} 
		
		if (switch_down)
		{
			timer_count_position = 0;
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
		case ADDR_MOVING_UP_STOP: if ((param > MAX_MOVING_UP_STOP) || (param < MIN_MOVING_UP_STOP)) param = DEF_MOVING_UP_STOP;
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
	
	moving_up_stop = eeprom_read_byte(&eeprom_moving_up_stop);
	
	_delay_ms(5);
	
	tmp = CheckMinMax(ADDR_MOVING_UP_STOP, moving_up_stop);
	if  (moving_up_stop != tmp)
	{
		moving_up_stop =tmp;
		eeprom_write_byte(&eeprom_moving_up_stop, moving_up_stop);
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
	
	
	//reading  30..35 protection data
	
	eeprom_read_block((void*)&protection, (const void*)protection_eeprom,PROTECTIONS_COUNT);
	_delay_ms(5);
	
	tmp = 0;
	for (i = 0; i <=PROTECTIONS_COUNT-1; i++)
	if ((protection[i] > def_protection[i][2]) || (protection[i] < def_protection[i][1]))
	{
		tmp = 1;
		protection[i] = def_protection[i][0];
	}
	
	if (tmp){
		eeprom_write_block((void*)&protection,(void*)&protection_eeprom, PROTECTIONS_COUNT);
		_delay_ms(5);
	}
	
	//reading  36..39 drive speed settings on normal and WATER mode, HOLD ON function
	
	eeprom_read_block((void*)&pulsemoving, (const void*)pulsemoving_eeprom,4);
	_delay_ms(5);
	
	tmp = 0;
	for (i = 0; i <=3; i++)
	if ((pulsemoving[i] > def_pulsemoving[i][2]) || (pulsemoving[i] < def_pulsemoving[i][1]))
	{
		tmp = 1;
		pulsemoving[i] = def_pulsemoving[i][0];
	}
	
	if (tmp){
		eeprom_write_block((void*)&pulsemoving,(void*)&pulsemoving_eeprom, 4);
		_delay_ms(5);
	}

	//reading  40 HAN specific logic on actuators release 
	
	logic_release = eeprom_read_byte(&logic_release_eeprom);
	_delay_ms(5);
	
	tmp = 0;
	if ((logic_release > def_logic_release[2]) || (logic_release < def_logic_release[1]))
	{
		tmp = 1;
		logic_release = def_logic_release[0];
	}
	
	if (tmp){
		eeprom_write_byte(&logic_release_eeprom, logic_release);
		_delay_ms(5);
	}
	
	
	//CAN BUS parameters
	eeprom_read_block((void*)&can_bus, (const void*)can_bus_eeprom,2);
	_delay_ms(5);
		
	tmp = 0;
	for (i = 0; i <=1; i++)
	if ((can_bus[i] > def_can_bus[i][2]) || (can_bus[i] < def_can_bus[i][1]))
	{
			tmp = 1;
			can_bus[i] = def_can_bus[i][0];
	}
		
	if (tmp){
			eeprom_write_block((void*)&can_bus,(void*)&can_bus_eeprom, 2);
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
		case ADDR_MOVING_UP_STOP: eeprom_write_byte(&eeprom_moving_up_stop, moving_up_stop);
			                     break;		
		case 10 ... 19: 	     eeprom_write_block((void*)&an_L.steps,(void*)&an_L_eeprom, 10);
		                         break;
		case 20 ... 29: 	     eeprom_write_block((void*)&an_R.steps,(void*)&an_R_eeprom, 10);
		                         break;					
		case 30 ... 35: 	     eeprom_write_block((void*)&protection,(void*)&protection_eeprom, 6);
	                         	 break;	
		case 36 ... 39: 	     eeprom_write_block((void*)&pulsemoving,(void*)&pulsemoving_eeprom, 4);
								 break;	
	    case 40:				 eeprom_write_byte(&logic_release_eeprom, logic_release);
	                             break;					
	    case 41 ... 42:		     eeprom_write_block((void*)&can_bus,(void*)&can_bus_eeprom, 2);
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
			
		    if (source->off) 
			{
				source->on_trigger = true;
			    source->protection_reset = true;
			}
			
		    
			source->off = false;
			
			result = true;
		} else
	
		source->curr_scan++;
		
		
		if  (source->scanrate == 0xFF) source->curr_scan = 0;
		
	} else
	{
		if (!source->off) source->off_trigger = true;
		
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

bool CurrentProtection(AnalogFilter * ana)
{
    if (ana->analog_limit == 0) return false;
 	
	if (ana->in_value > ana->analog_limit)
	{
		if (ana->time_count >= ana->time_rate)
		{
			ana->trip= true;
		} else
		  ana->time_count++;
	} else 
	  ana->time_count = 0;
	
	return ana->trip;
}


ISR (TIMER0_OVF_vect) // 1.8 kHz
{
		
	if (Left.timer_pwm_backward < 10) Left.timer_pwm_backward++; else Left.timer_pwm_backward = 0;
	if (Right.timer_pwm_backward < 10) Right.timer_pwm_backward++; else Right.timer_pwm_backward = 0;

	if (Left.timer_pwm_hold_on < 20) Left.timer_pwm_hold_on++; else Left.timer_pwm_hold_on = 0;
	if (Right.timer_pwm_hold_on < 20) Right.timer_pwm_hold_on++; else Right.timer_pwm_hold_on = 0;			
		
    TCNT0 = 0;
}

ISR (TIMER1_OVF_vect) // 100 Hz
{
    if (Left.timer_count_en) Left.timer_count++;
    if (Left.timer_count == 0xFFFF) Left.timer_count = 0;

    if (Right.timer_count_en) Right.timer_count++;
    if (Right.timer_count == 0xFFFF) Right.timer_count = 0;

    w100Hz++;
	
	
   
    if ((w100Hz % WAIT_5Hz) == 0)
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
	
	sw_water.outstate = _switch_filter(&sw_water);
	sw_squeeze.outstate = _switch_filter(&sw_squeeze);
	
	
	an_R.in_value = val_R;
	_joystick_filter(&an_R);
	
	an_L.in_value = val_L;
	_joystick_filter(&an_L);
	
	val11.in_value = val_ANA11;
	CurrentProtection(&val11);
	val12.in_value = val_ANA12;
	CurrentProtection(&val12);
	
	
	
	if (val11.trip || val12.trip)
	{
		// Stop Left Channel
		Left.Stop();
		in_left.reset = true;
	}
	
	val21.in_value = val_ANA21;
	CurrentProtection(&val21);
	val22.in_value = val_ANA22;	
	CurrentProtection(&val22);
	
	if (val21.trip || val22.trip)
	{
		// Stop Right Channel
		Right.Stop();
		in_right.reset = true;
	} 
	
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

void init_switches(){
		
		in_left.outstate=false;
		in_left.reset = false;
		in_left.on_trigger = false;
		in_left.off_trigger = false;
		in_left.protection_reset = false;
		in_left.curr_scan=0;
		in_left.scanrate=5;
		in_left.port = &PIND;
		in_left.pin = SW_L;
		
		in_right.outstate=false;
		in_right.on_trigger = false;
		in_right.protection_reset = false;
		in_right.off_trigger = false;
		in_right.reset = false;
		in_right.curr_scan=0;
		in_right.scanrate=5;
		in_right.port = &PIND;
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
		
		sw_water.outstate=false;
		sw_water.curr_scan=0;
		sw_water.scanrate=2;
		sw_water.off_trigger = false;
		sw_water.port = &PIND;
		sw_water.pin = WATER_MODE;
		
		sw_squeeze.outstate=false;
		sw_squeeze.off_trigger = false;
		sw_squeeze.curr_scan=0;
		sw_squeeze.scanrate=2;
		sw_squeeze.port = &PIND;
		sw_squeeze.pin = SQUEEZE;			
		
}



void init_protections()
{
	// Jamming protections
	
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

    // Short circuit overcurrent protection
 
    oc_val11.trip = false;
	oc_val11.in_value = 0;
	oc_val11.analog_limit = protection[4]*CURRENT_COEFFICIENT;
	oc_val11.time_count=0;

	oc_val12.trip = false;
	oc_val12.in_value = 0;
	oc_val12.analog_limit = protection[4]*CURRENT_COEFFICIENT;
	oc_val12.time_count=0;
	
	oc_val21.trip = false;
	oc_val21.in_value = 0;
	oc_val21.analog_limit = protection[5]*CURRENT_COEFFICIENT;
	oc_val21.time_count=0;
		
	oc_val22.trip = false;
	oc_val22.in_value = 0;
	oc_val22.analog_limit = protection[5]*CURRENT_COEFFICIENT;
	oc_val22.time_count=0;
}



ISR (ADC_vect) 
{
	
	cli();
	unsigned char _admux = ADMUX & ((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
	
	ADMUX &= 0b11110000;
	
	switch(_admux)
	{

	    case ANA22: val_ANA22 =   ADCW;
				oc_val22.in_value = val_ANA22;
				
				if (Right.water_mode || (Right.hold_on_dc > 0))
				{
					oc_val22.time_rate = SQUELCH; 
				}	
				else
					oc_val22.time_rate = 0;
				
				 CurrentProtection(&oc_val22); //Fast short circuit
				  if (oc_val22.trip)
				  {
					 // Stop Right Channel
				    Right.Stop();
					in_right.reset = true;
				  }
				 ADMUX |=ANA21;
	    break;
			
	    case ANA21: val_ANA21 =   ADCW;
					oc_val21.in_value = val_ANA21;
					
				    if (Right.water_mode || (Right.hold_on_dc > 0))
					{
						  oc_val21.time_rate = SQUELCH; 
					} else
						  oc_val21.time_rate = 0;
					
					CurrentProtection(&oc_val21); //Fast short circuit
					if (oc_val21.trip)
					{
						// Stop Right Channel
						Right.Stop();
						in_right.reset = true;
					}
	                
					ADMUX |=ANA11;
	                break;		
						
	    case ANA11: val_ANA11 =   ADCW;
				oc_val11.in_value = val_ANA11;
				
				if (Left.water_mode || (Left.hold_on_dc > 0))
				{
				    oc_val11.time_rate = SQUELCH; 
				} else
				    oc_val11.time_rate = 0;
				
				CurrentProtection(&oc_val11); //Fast short circuit
				if (oc_val11.trip)
			    {
				  // Stop Left Channel
				  Left.Stop();
				  in_left.reset = true;
			    }
			    ADMUX |= ANA12;
		break;
	    
		case ANA12: val_ANA12 =   ADCW;
	    
					 oc_val12.in_value = val_ANA12;
					 
			    	 if (Left.water_mode || (Left.hold_on_dc > 0))
					 {
						  oc_val12.time_rate = SQUELCH; 
					 } else
						  oc_val12.time_rate = 0;
						  
					 CurrentProtection(&oc_val12); //Fast short circuit
					 if (oc_val12.trip)
				     {
					    // Stop Left Channel
					    Left.Stop();
					    in_left.reset = true;
					  }
	    
				    ADMUX |=AN_X_L;
	    break; 
		
		case AN_X_L:
	    		     val_L = ADCW/10;
					 ADMUX |=AN_X_R;
		break;
		
		case AN_X_R: val_R =   ADCW/10;
				
				     ADMUX |=ANA22;
				     break; 	 
	   
	    default:     ADMUX &= 0b11110000;
	                 ADMUX |=ANA22;
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
	
	ADCSRA |= (1 << ADEN) | (1 << ADPS2)  | (0 << ADPS1)  | (1 << ADPS0) | (1 << ADIE);  //ADC 32 divider
	ADMUX = (0 << REFS1) | (1 << REFS0) | ANA22; 
	ADCSRA |=(1<<ADSC);
}

void init_channels()
{
	
	Left.port1 = &PORTA;
	Left.pin1 = ON_L1;
	Left.port2 = &PORTA;
	Left.pin2 = ON_L2;
	Left.InitChannel();
	Left.timer_pwm_backward = 0;
	Left.timer_pwm_hold_on = 0;

	Right.port1 = &PORTB;
	Right.pin1 = ON_R1;
	Right.port2 = &PORTB;
	Right.pin2 = ON_R2;
	Right.InitChannel();
	Right.timer_pwm_backward = 4; // Making time interval between Left and Right Channels
	Right.timer_pwm_hold_on = 10;
}

bool CheckLimitsArray(uint8_t source, uint8_t defs[4])
{
	bool result = false;
	
	if ((source <= defs[2]) && (source >= defs[1])) result = true;
						
   return result;
}

/*

ISR(PCINT1_vect) 
{
	interruptMCP2515();
} */


int main(void)
{	 
	uint8_t step = 0;
	bool sq_off_left, sq_off_right = false;
	unsigned char data[8];

    DDRA = ON_L1 | ON_L2;
	
	DDRB = ON_R1 | ON_R2 | RESERVE1; 
	
	DDRC = nCS;
		
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP1);	
	
	TCCR0A = 0;
	TCCR0B = (1 << CS01)|(0 << CS00); //32 divider
	TCNT0 = 0;
	
	TCCR1A = 0;
	TCCR1B = (1 << CS11)|(1 << CS10); //64 divider
    TCNT1 = WAIT_100HZ;	
	
	//PCICR |= (1 << PCIE1); 
	//PCMSK1 |= (1 << PCINT12);

	LoadEeprom();
	
	if (can_bus[0] > 0)
	  initMCP2515((uint8_t)(can_bus[0] - 1));
	 
	sei();
    init_switches();
	init_channels();
	init_protections();
	ADC_Init();
	twi_init();
	

	TIFR0 = (1<<TOV0);
	TIMSK0 = (1<<TOIE0);
		
	TIFR1 = (1<<TOV1);
	TIMSK1 = (1<<TOIE1);


  
    while (1) 
    {
		asm("wdr"); 
		

        if ((!val11.trip) && (!val12.trip) && (!oc_val11.trip) && (!oc_val12.trip))
		{
			if (sw_squeeze.off_trigger) //SQEEZE switch input
			{
				sq_off_left = true; //return drives to start position on negative edge SQUEEZE input
				sq_off_right = true;
				sw_squeeze.off_trigger = false;
			}
			
			if (in_left.outstate && !(logic_release && in_right.outstate))  // Button L process
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
			
			if ((!Left.prev_lock_state) && Left.lock && (!in_right.outstate)) Left.lock = false; 
			
			Left.prev_lock_state = Left.lock;
			
			
			//if (Left.lock && Right.lock) Left.goal_step = 4; // blocking left drive if WAIT_LOCK time exceeded (left and right buttons pressed)
		
			Left.water_mode = sw_water.outstate;
			Left.long_release = sw_squeeze.outstate;
			
			
			
			
			if (sw_squeeze.outstate) {
				
				Left.control_source = true;
				Left.goal_step = 4;

			}
			
            if (sq_off_left){
				Left.goal_step = 0;
				sq_off_left = false;
			}
			
			Left.hold_on_dc = pulsemoving[0];
			Left.normal_mode_dc = pulsemoving[2];
			Left.water_dc = pulsemoving[3];
			
			Left.Process(); // Process L channel
					
		} 


        if ((!val21.trip) && (!val22.trip) && (!oc_val21.trip) && (!oc_val22.trip))
        {
			
			if (in_right.outstate && !(logic_release && in_left.outstate))  // Button R process
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
			
			if ((!Right.prev_lock_state) && Right.lock && (!in_left.outstate)) Right.lock = false;
			
			Right.prev_lock_state = Right.lock;
			
			//if (Right.lock && Left.lock) Right.goal_step = 4; // blocking left drive if WAIT_LOCK time exceeded (left and right buttons pressed)

	
			Right.water_mode = sw_water.outstate;
			Right.long_release = sw_squeeze.outstate; 

			if (sw_squeeze.outstate) {
				Right.control_source = true;
				Right.goal_step = 4; 
			}
			if (sq_off_right)
			{
				Right.goal_step = 0;
				sq_off_right = false;
			}
			
			
			Right.hold_on_dc = pulsemoving[1];
			Right.normal_mode_dc = pulsemoving[2];
			Right.water_dc = pulsemoving[3];
			
			Right.Process(); // Process R channel		
			
		}
		
		if (in_left.protection_reset && in_left.reset) {
			val11.trip = false;
			oc_val11.trip = false;
			val12.trip = false;
			oc_val12.trip = false;
			in_left.reset = false;
			in_left.protection_reset = false;
		}


		if (in_right.protection_reset && in_right.reset) {
			val21.trip = false;
			oc_val21.trip = false;
			val22.trip = false;
			oc_val22.trip = false;
			in_right.reset = false;
			in_right.protection_reset = false;
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
																		 ((val11.trip | val12.trip) << STATUS_BIT1_L_TRIP_STAGE1) |
																		 ((val21.trip | val22.trip) << STATUS_BIT1_R_TRIP_STAGE1)  ;
		
		statebyte2 = ((oc_val11.trip | oc_val12.trip) << STATUS_BIT2_L_TRIP_STAGE2) | ((oc_val21.trip | oc_val22.trip) << STATUS_BIT2_R_TRIP_STAGE2) | 
		              (sw_squeeze.outstate << STATUS_BIT2_SQUEEZE) | (sw_water.outstate << STATUS_BIT2_WATERMODE);		
		
		if (oc_val11.trip || oc_val12.trip || oc_val21.trip || oc_val22.trip || val11.trip || val12.trip || val21.trip || val22.trip)	
		{
			PORTB |= RESERVE1; // IN_RES1 used as FAULT OUTPUT on external led (1 ma)
		} else
			PORTB &= ~RESERVE1;

		if  (twi_process_read)
		{
			
			TWI_DataAddr = 0x05;
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
				
				case ADDR_MOVING_UP_STOP: 
				if (paramfromdisp != moving_up_stop)
				{
					moving_up_stop = CheckMinMax(ADDR_MOVING_UP_STOP,paramfromdisp); 
					WriteEeprom(paramfromdisp_addr);
				}
				
				break;	
				case 10 ... 19:
				if (paramfromdisp != an_L.steps[paramfromdisp_addr-10])		
				{
					if ((paramfromdisp < 100) && (paramfromdisp > 0))
					{
						an_L.steps[paramfromdisp_addr-10] = paramfromdisp;
						WriteEeprom(paramfromdisp_addr);
					}
				}			
				break;		
				
				case 20 ... 29:
				if (paramfromdisp != an_R.steps[paramfromdisp_addr-20])		
				{
					if ((paramfromdisp < 100) && (paramfromdisp > 0))
					{
						an_R.steps[paramfromdisp_addr-20] = paramfromdisp;
						WriteEeprom(paramfromdisp_addr);
					}
				}	
				break;	
				
				case 30 ... 35:
				if (paramfromdisp != protection[paramfromdisp_addr-30])
				{
				  	if (CheckLimitsArray(paramfromdisp, def_protection[paramfromdisp_addr-30]))
				  	{
				        protection[paramfromdisp_addr-30] = paramfromdisp;
						WriteEeprom(paramfromdisp_addr);
						init_protections();
					}
				
				}
				break;
				
				case 36 ... 39:
				if (paramfromdisp != pulsemoving[paramfromdisp_addr-36])
				{
				  	if (CheckLimitsArray(paramfromdisp, def_pulsemoving[paramfromdisp_addr-36]))
				  	{
				        pulsemoving[paramfromdisp_addr-36] = paramfromdisp;
						WriteEeprom(paramfromdisp_addr);
					}
				
				}
				break;
				
			   case 40: 
			   if (paramfromdisp != logic_release)
			   {
				  	if (CheckLimitsArray(paramfromdisp, def_logic_release))
				  	{
					  	logic_release = paramfromdisp;
					  	WriteEeprom(paramfromdisp_addr);
				  	}
			   }
			   break;
			   
			   case  41 ... 42:
			   if (paramfromdisp != can_bus[paramfromdisp_addr-41])
			   {
                    if (CheckLimitsArray(paramfromdisp, def_can_bus[paramfromdisp_addr-41]))
				   	{
					     can_bus[paramfromdisp_addr-41] = paramfromdisp;
					   	 WriteEeprom(paramfromdisp_addr);
				   	}
			   }
			   
			   break;
				
			   default: paramfromdisp = 0;
			}
			
		}
		
		if (twi_process_write)
		{
			data[0] = statebyte0;
			data[1] = statebyte1;
			data[2] = statebyte2;
			data[3] = 0x00;
			data[4] = 0x00;
			data[5] = 0x00;
			data[6] = 0x00;
			data[7] = 0x00;	
			
			if (can_bus[0] > 0)					
			   sendCANmsg(0, can_bus[1], data, 8);

			TWI_DataAddr = 0x01;
			TWI_DataSize  = 4;
			TWI_DataBuff[0] = statebyte0;
			TWI_DataBuff[1] = statebyte1;
			TWI_DataBuff[2] = statebyte2;
			
			switch (paramtodisp_addr)
			{
				case 1: paramtodisp = wait_sec_up;
				           break;
			    case 2: paramtodisp = wait_sec_down;
			               break;
			    case 3: paramtodisp = moving_up_stop;
			               break;
				case 4: paramtodisp = val_L;
						  break;
			    case 5: paramtodisp = Left.goal_step*10+Left.curr_step;
				          break;
				case 6: paramtodisp = an_L.enabled;		 
					      break;
				case 7: paramtodisp =val_R;
						  break;
				case 8: paramtodisp = Right.goal_step*10+Right.curr_step;
				          break;
				case 9: paramtodisp = an_R.enabled;  
				          break;	
				case 10 ... 19: paramtodisp = an_L.steps[paramtodisp_addr-10];	
				          break;
				case 20 ... 29: paramtodisp = an_R.steps[paramtodisp_addr-20];
						  break;		  	
				case 30 ... 35: paramtodisp = protection[paramtodisp_addr-30];	
				          break;
			    case 36 ... 39: paramtodisp = pulsemoving[paramtodisp_addr-36];
					      break;	
				case 40: 	    paramtodisp = logic_release;
				          break;	
				case 41 ... 42: 			
								paramtodisp = can_bus[paramtodisp_addr - 41];
						  break;  				
				
			    default:   paramtodisp = 0;
			}
			
			TWI_DataBuff[3] = paramtodisp; 
			
			twi_write(); // Send Data to Display board
			
			twi_process_write = false;
		}
    }
}

