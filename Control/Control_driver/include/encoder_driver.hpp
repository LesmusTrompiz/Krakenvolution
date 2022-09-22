#pragma once
#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "register_types.hpp"


void increment_right_odometry(void);
void increment_left_odometry(void);

struct EncoderDriver{
    public:
		EncoderDriver(
					const uint8_t     encoder_pin,
					const uint8_t     interrupt_pin,
					uint16_t *    cnt,
					void(*cb)(void)
					);
	    uint16_t read_pulses();
	    void     reset_pulses();

	private:
		uint16_t   *total_cnt;
		register8   count_register;
};
