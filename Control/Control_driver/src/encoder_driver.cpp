#include "encoder_driver.hpp"

extern volatile int right_odom;
extern volatile int left_odom;


/** @todo:
 * Los registros de las interrupciones estan prefijados sin
 * ser ninguna Constexpresion, no mola....
*/

/**
 * @todo: Sacar el desplazamiento de bits de las  
 * mascaras fuera de la interrupción
 */

void increment_right_odometry_channel_a(void)
{
	if(((PINB >> PINB5) & 1) == ((PINB >> PINB6) & 1)) --right_odom;
	else ++right_odom;
	return;
}

void increment_right_odometry_channel_b(void)
{
	if((PINB & (1 << PINB6)) != (PINB & (1 << PINB5))) --right_odom;
	else ++right_odom;
	return;
}

void increment_left_odometry_chanel_a(void)
{
	if(((PINB >> PINB4) & 1) == ((PINE >> PINE5) & 1)) --left_odom;
	else ++left_odom;
	return;
}

void increment_left_odometry_chanel_b(void)
{
	if(((PINB >> PINB4) & 1) != ((PINE >> PINE5) & 1)) --left_odom;
	else ++left_odom;
	return;
}


EncoderDriver::EncoderDriver(
					const uint8_t     chanel_A,
					const uint8_t     chanel_B,
					const uint16_t    resolucion_encoder_,
					const uint16_t    reductora,
					volatile int16_t *cnt,
					void(*cb)(void)
					):
					resolucion_encoder{resolucion_encoder_},
					angle_per_count{2 * PI / (resolucion_encoder_ * reductora)}
					{
	total_cnt      = cnt;
	*total_cnt     = 0;

	pinMode(chanel_A,INPUT);
	pinMode(chanel_B,INPUT);
	
	attachPCINT(digitalPinToPCINT(chanel_A), cb, CHANGE);
	return;
}

int16_t EncoderDriver::read_pulses(){
	return *total_cnt;
}

float EncoderDriver::get_angle_increment(){
	return *total_cnt * angle_per_count;
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

