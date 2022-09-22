#pragma once
#include <stdint.h>
typedef volatile uint8_t *register8;
typedef volatile uint16_t *register16;

enum clearing_mode{
	NOTHING_AT_MATCH,
	TOGGLE_PIN_AT_MATCH,
	CLEAR_PIN_AT_MATCH,
	SET_PIN_AT_MATCH
}; 

enum wavegeneration_mode{
	NORMAL,
	PWM_PHASECORRECT,
	CTC,
	FAST_PWM
};

enum prescaler{
	NO_CLK,
	PRSCL_0,
	PRSCL_8,
	PRSCL_64,
	PRSCL_256,
	PRSCL_1024,
	EXTERN_FALLING_EDGE,
	EXTERN_RISING_EDGE

};
