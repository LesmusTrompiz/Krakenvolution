#pragma once
#include <stdint.h>
typedef volatile uint8_t *register8;
typedef volatile uint16_t *register16;

enum clearing_mode{
	CLMODE1,
	CLMODE2
}; 

enum wavegeneration_mode{
	WGMMODE1,
	WGMMODE2
};

enum prescaler{
	PSMODE1,
	PSMODE2
};
