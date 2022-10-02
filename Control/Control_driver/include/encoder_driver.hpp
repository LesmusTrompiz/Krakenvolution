#pragma once
#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "register_types.hpp"

void increment_right_odometry_chanel_a(void);
void increment_right_odometry_chanel_b(void);

void increment_left_odometry_chanel_a(void);
void increment_left_odometry_chanel_b(void);

constexpr uint8_t R_ENC_A   = 11;
constexpr uint8_t R_ENC_B   = 12;
constexpr uint8_t L_ENC_A   = 10;
constexpr uint8_t L_ENC_B   = 3;

struct EncoderDriver{
    public:
		EncoderDriver(
					const uint8_t     chanel_A,
					const uint8_t     chanel_B,
					volatile int16_t *        cnt,
					void(*cb)(void)
					);
	    int16_t read_pulses();
	    void    reset_pulses();

	private:
		volatile int16_t    *total_cnt;
		register8  count_register;
};
