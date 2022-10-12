#include "motor_driver.hpp"
#define RADS_TO_RPM(x) x*60/(2*PI)

constexpr int MAX_RPM = 8100;	// MAX_VEL_RECOMMENDED_BY_JQP_09_10_22

inline float speed_to_duty_cycle(float speed){
	// Serial.println(MIN_PWM + ((RADS_TO_RPM(speed) * 0.8) / MAX_RPM)); 
	return MIN_PWM + (RADS_TO_RPM(speed)/MAX_RPM)*0.8;
}


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
	pinMode(pwm_pin, OUTPUT);					//Enable output
	pinMode(dir_pin, OUTPUT);
	pinMode(en_pin,  OUTPUT);
	disable_motor();
	config_timer(CLEAR_PIN_AT_MATCH, cl_mask, FAST_PWM , PRSCL_64);
	enable_motor();
}



void MotorDriver::config_timer(const clearing_mode       cl_mode, 
							   const uint8_t             cl_mask, 
							   const wavegeneration_mode wgm_mode,
							   const prescaler           prscl){
	TCCR0A |=  (cl_mode  << cl_mask)  								// Clearing on compare, if the value is matched the register value is assing to 0
		   |   (wgm_mode << WGM00);    								// Set the Fast PWM mode on both channels 
	TCCR0B |=  (prscl    << CS00);    								// Set the prescale 1/64 clock
	set_pwm(MIN_PWM);
	return;
}

void MotorDriver::set_pwm(const float &duty_cycle){
	// (float)fabs((double)duty_cycle);
	*Rpwm = duty_cycle * 255;
	return;
}

void inline MotorDriver::set_direction(const bool direction){
	digitalWrite(Pdir, !direction);		
	// ESTOS COMEPINGAS NO QUIEREN QUE PONGA ESA EXCLAMACION
	// NO ME APETECE CONFIGURAR LAS MAXON
	// Serial.println(direction);
	return;
}

void inline MotorDriver::enable_motor(){
	//Portenable |= 1 << Penable;
	digitalWrite(Penable,HIGH);
	return;
}

void inline MotorDriver::disable_motor(){
	Portenable &= 0 <<  Penable;
	digitalWrite(Penable,LOW);
	
	return;
}



void MotorDriver::set_speed(float speed){
	bool dir = true;
	float duty_cycle = 0;
	duty_cycle = speed_to_duty_cycle(speed);
	
	if(duty_cycle < 0.0){
		dir = false;
		duty_cycle = -duty_cycle;
	}

	set_direction(dir);
	set_pwm(duty_cycle);	
    enable_motor();	

	// Serial.print("dc:");
	// Serial.println(duty_cycle);

	return;
}
