#include "encoder_driver.hpp"

extern volatile int right_odom;
extern volatile int left_odom;


void increment_right_odometry_chanel_a(void)
{
	if(digitalRead(R_ENC_A) == digitalRead(R_ENC_B)) ++right_odom;
	else --right_odom;
	//Serial.print("R");
  	//Serial.println(right_odom);
	return;
}

void increment_right_odometry_chanel_b(void)
{
	if(digitalRead(R_ENC_A)!=digitalRead(R_ENC_B)) ++right_odom;
	else --right_odom;
	return;
}

void increment_left_odometry_chanel_a(void)
{
	if(digitalRead(L_ENC_A) == digitalRead(L_ENC_B)) ++left_odom;
	else --left_odom;
	
  	//Serial.print("L");
  	//Serial.println(left_odom);
	return;
}

void increment_left_odometry_chanel_b(void)
{
	if(digitalRead(L_ENC_A)!=digitalRead(L_ENC_B)) ++left_odom;
	else --left_odom;
	return;
}


EncoderDriver::EncoderDriver(
					const    uint8_t     chanel_A,
					const    uint8_t     chanel_B,
					volatile int16_t *        cnt,
					void(*cb)(void)
					)
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

