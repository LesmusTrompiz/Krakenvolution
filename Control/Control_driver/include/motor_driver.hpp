#pragma once
#include <Arduino.h>
#include "register_types.hpp"

constexpr float MIN_PWM = 0.095;


struct MotorDriver{
    public:
		MotorDriver(register8           pwm_register, 
					uint8_t             pwm_pin,
					uint8_t             dir_pin,
					uint8_t             en_pin,
					uint8_t             en_port,
					clearing_mode       cl_mode, 
					uint8_t             cl_mask,
					wavegeneration_mode wgm_mode, 
					prescaler           prscl
					);
		MotorDriver(register8           pwm_register, 
					uint8_t             pwm_pin,
					uint8_t             dir_pin,
					uint8_t             en_pin,
					uint8_t             en_port,
					uint8_t             cl_mask
					);

		void set_speed(float duty_cycle);
	    void set_pwm(const float &duty_cycle);
	private:
		void inline set_direction(const bool direction);
		void inline enable_motor();
		void inline disable_motor();
		void config_timer(clearing_mode cl_mode, uint8_t cl_mask, wavegeneration_mode wgm_mode, prescaler prscl);

    	register8 Rpwm;
        uint8_t   Penable;
        uint8_t   Portenable;
        uint8_t   Pdir;
};
