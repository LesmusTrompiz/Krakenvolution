#include "encoder_driver.hpp"

void cb_increment_count(void){
	return;
}



EncoderDriver::EncoderDriver(
					uint8_t     encoder_pin,
					uint8_t     interrupt_pin,
					register8  	timer_register, 
					register16 	match_count_register,
					register8   actual_count_register, 
					register8 	interrupt_register)
{
	total_cnt      = 0;
	count_register = count_register;
	pinMode(encoder_pin,INPUT);
	pinMode(interrupt_pin,INPUT);
	attachPCINT(digitalPinToPCINT(interrupt_pin), cb_increment_count, RISING);
	config_timer(timer_register, match_count_register, actual_count_register, interrupt_register);
	return;
}

uint16_t EncoderDriver::read_pulses(){
	return total_cnt;
}

void EncoderDriver::reset_pulses(){
	*count_register  = 0;
	return;
}

/* Preguntar a los chicos de control la configuración del timer 1 en 
	github es esto una configuración asimétrica?
	https://github.com/LesmusTrompiz/UAHR_SOFTWARE/blob/main/control/src/configuraciones.cpp
*/


void  EncoderDriver::config_timer(register8 timer_register, register16 match_count_register, register8 actual_count_register, register8 interrupt_register ){
	*timer_register          = 0;                //limpia registros
 	*actual_count_register   = 0;                //Inicializa el contador
	*match_count_register    = MAX_TIMER_WAIT;		// 
	*actual_count_register   = (1<<WGM12) | (1<<CS11);
	*interrupt_register     |= (1<<OCIE1A);   //Set the interrupt request
}


