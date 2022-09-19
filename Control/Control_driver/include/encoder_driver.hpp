#pragma once
#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "register_types.hpp"


constexpr int32_t MAX_TIMER_WAIT = 39999;


struct EncoderDriver{
    public:
		EncoderDriver::EncoderDriver(
					uint8_t             encoder_pin,
					uint8_t             interrupt_pin,
					register8  			timer_register, 
					register16 			match_count_register,
					register8           actual_count_registers, 
					register8 			interrupt_register);
	    uint16_t read_pulses();
		uint16_t total_cnt;
	private:
		void config_timer(clearing_mode cl_mode, uint8_t cl_mask, wavegeneration_mode wgm_mode, prescaler prscl);
};

void cb_increment_count(void){
	return;
}



EncoderDriver::EncoderDriver(
					uint8_t     encoder_pin,
					uint8_t     interrupt_pin,
					register8  	timer_register, 
					register16 	match_count_register,
					register8   actual_count_registers, 
					register8 	interrupt_register)
{
	total_cnt = 0;
	pinMode(encoder_pin,INPUT);
	pinMode(interrupt_pin,INPUT);
	attachPCINT(digitalPinToPCINT(interrupt_pin), cb_increment_count, RISING);
	return;
}

uint16_t EncoderDriver::read_pulses(){
	return total_cnt;
}


void configTimer(register8 timer_register, register16 match_count_register, register8 actual_count_registers, register8 interrupt_register ){
	*timer_register          = 0;                //limpia registros
 	*actual_count_registers  = 0;                //Inicializa el contador
	*match_count_register    = MAX_TIMER_WAIT;		// 
	*actual_count_registers  = (1<<WGM12) | (1<<CS11);
	*interrupt_register     |= (1<<OCIE1A);   //Set the interrupt request
}


