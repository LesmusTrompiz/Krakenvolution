#pragma once
#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "register_types.hpp"


constexpr int32_t MAX_TIMER_WAIT = 39999;


struct EncoderDriver{
    public:
		EncoderDriver(
					uint8_t             encoder_pin,
					uint8_t             interrupt_pin,
					register8  			timer_register, 
					register16 			match_count_register,
					register8           actual_count_registers, 
					register8 			interrupt_register);
	    uint16_t read_pulses();
	    void     reset_pulses();

	private:
		uint16_t    total_cnt;
		register8   count_register;
		void config_timer(register8 timer_register, register16 match_count_register, register8 actual_count_register, register8 interrupt_register);
};
