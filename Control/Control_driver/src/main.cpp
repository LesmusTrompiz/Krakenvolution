#include <Arduino.h>
#include "motor_driver.hpp"

// Pines de controladoras Maxon Derecha OCR0B
constexpr uint8_t R_STOP = 2;
constexpr uint8_t R_DIR  = 3;
constexpr uint8_t R_EN   = 4;
constexpr uint8_t R_PWM  = 5;

// Pines de controladoras Maxon Izquierda OCR0A
constexpr uint8_t L_PWM  = 6;
constexpr uint8_t L_EN   = 7;
constexpr uint8_t L_DIR  = 8;
constexpr uint8_t L_STOP = 9;


void setup() {
  // put your setup code here, to run once:
  
  MotorDriver rmotor(&OCR0A, 
                     R_PWM,
                     R_DIR,
                     R_EN,
                     PORTD,
                     COM0A1);
  MotorDriver lmotor(&OCR0B, 
                     L_PWM,
                     L_DIR,
                     L_EN,
                     PORTB,
                     COM0B1);
}



void loop() {
  // put your main code here, to run repeatedly:
  uint8_t a=0;





  //PWM -> PWM_CH_NUM[5].PWM_CDTY =  MAX_DC*(1-duty);
}