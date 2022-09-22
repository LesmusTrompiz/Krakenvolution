#include "motor_driver.hpp"

MotorDriver::MotorDriver(register8           pwm_register, 
						 uint8_t             pwm_pin,
						 uint8_t             dir_pin,
						 uint8_t             en_pin,
						 uint8_t             en_port,
						 clearing_mode       cl_mode, 
						 uint8_t             cl_mask,
						 wavegeneration_mode wgm_mode, 
						 prescaler           prscl
						){

    Penable    = en_pin;
    Pdir       = dir_pin;
    Rpwm       = pwm_register;
	Portenable = en_port;
	config_timer(cl_mode, cl_mask, wgm_mode, prscl);
	pinMode(pwm_pin, OUTPUT);					//Enable output
	pinMode(dir_pin, OUTPUT);
	pinMode(en_pin,  OUTPUT);
}

MotorDriver::MotorDriver(register8           pwm_register, 
						 uint8_t             pwm_pin,
						 uint8_t             dir_pin,
						 uint8_t             en_pin,
						 uint8_t             en_port,
						 uint8_t             cl_mask
						 ){

    Penable    = en_pin;
    Pdir       = dir_pin;
    Rpwm       = pwm_register;
	Portenable = en_port;
	config_timer(CLEAR_PIN_AT_MATCH, cl_mask, FAST_PWM , PRSCL_64);
	pinMode(pwm_pin, OUTPUT);					//Enable output
	pinMode(dir_pin, OUTPUT);
	pinMode(en_pin,  OUTPUT);
}



void MotorDriver::config_timer(const clearing_mode       cl_mode, 
							   const uint8_t             cl_mask, 
							   const wavegeneration_mode wgm_mode,
							   const prescaler           prscl){
	TCCR0A |=  (cl_mode  << cl_mask)  								// Clearing on compare, if the value is matched the register value is assing to 0
		   |   (wgm_mode << WGM00);    								// Set the Fast PWM mode on both channels 
	TCCR0B |=  (prscl    << CS00);    								// Set the prescale 1/64 clock
	set_pwm(0);
	return;
}

void MotorDriver::set_pwm(const float &duty_cycle){
    float temp_duty;
	temp_duty= duty_cycle * 0.8 + MIN_PWM;			//El duty se calcula como el 10% de m�nimo mas el valor absoluto de
																		    //la velocidad entre mil. La funci�n fabs recibe y devuelve un double
																			//y es por eso por lo que necesitamos las conversiones de tipo.
	*Rpwm = temp_duty * 255;
	return;
}

void inline MotorDriver::set_direction(const bool direction){
	digitalWrite(Pdir, HIGH);
	return;
}

void inline MotorDriver::enable_motor(){
	Portenable |= 1 << Penable;
	return;
}

void inline MotorDriver::disable_motor(){
	Portenable &= 0 <<  Penable;
	return;
}


void MotorDriver::set_speed(float duty_cycle){
	bool dir = true;
	
	if(duty_cycle < 0.0){
		dir = false;
		duty_cycle = -duty_cycle;
	}

	set_direction(dir);
	set_pwm(duty_cycle);	
    enable_motor();	
	return;
}
