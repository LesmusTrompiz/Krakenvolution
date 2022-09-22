#include "encoder_driver.hpp"
extern volatile int counts_right;
extern volatile int counts_left;


extern volatile int right_odom;
extern volatile int left_odom;

void increment_right_odometry(void)
{
	++right_odom;
	return;
}

void increment_left_odometry(void)
{
	++left_odom;
	return;
}




EncoderDriver::EncoderDriver(
					const uint8_t     encoder_pin,
					const uint8_t     interrupt_pin,
					uint16_t *    cnt,
					void(*cb)(void)
					)
{
	total_cnt      =  cnt;
	*total_cnt = 0;
	pinMode(encoder_pin,INPUT);
	pinMode(interrupt_pin,INPUT);

	attachPCINT(digitalPinToPCINT(interrupt_pin), cb, CHANGE);
	return;
}

uint16_t EncoderDriver::read_pulses(){
	return *total_cnt;
}

void EncoderDriver::reset_pulses(){
	*total_cnt  = 0;
	return;
}

/* Preguntar a los chicos de control la configuración del timer 1 en 
	github es esto una configuración asimétrica?
	https://github.com/LesmusTrompiz/UAHR_SOFTWARE/blob/main/control/src/configuraciones.cpp
*/


/*
void config_timer(register8 timer_register, register16 match_count_register, register8 actual_count_register, register8 interrupt_register ){
	*timer_register          = 0;                	//limpia registros
 	*actual_count_register   = 0;                	//Inicializa el contador
	*match_count_register    = MAX_TIMER_WAIT;		// 
	*actual_count_register   = (1<<WGM12) | (1<<CS11);
	*interrupt_register     |= (1<<OCIE1A);   //Set the interrupt request
}
*/

