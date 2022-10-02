#include <Arduino.h>
#include "motor_driver.hpp"
#include "encoder_driver.hpp"
#include "PinChangeInterrupt.h"


// Pines de controladoras Maxon Derecha OCR0B
constexpr uint8_t R_STOP = 2;
constexpr uint8_t R_DIR  = 3;
constexpr uint8_t R_EN   = 4;
constexpr uint8_t R_PWM  = 13;

// Pines de controladoras Maxon Izquierda OCR0A
constexpr uint8_t L_PWM  = 4;
constexpr uint8_t L_EN   = 7;
constexpr uint8_t L_DIR  = 8;
constexpr uint8_t L_STOP = 9;
// Pines de controladoras Maxon Derecha OCR0B
//constexpr uint8_t R_ENC_A   = 11;
//constexpr uint8_t R_ENC_B   = 11;



int16_t volatile right_odom = 0;
int16_t volatile left_odom = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
	
}



void loop() {
  // put your main code here, to run repeatedly:
  MotorDriver rmotor(&OCR0A,                    // pwm_register
                     R_PWM,                     // pwm_pin
                     R_DIR,                     // dir_pin
                     R_EN,                      // en_pin
                     PORTD,                     // en_port
                     COM0A0);                   // cl_mask


  MotorDriver lmotor(&OCR0B, 
                     L_PWM,
                     L_DIR,
                     L_EN,
                     PORTB,
                     COM0B0);

  rmotor.set_pwm(1);
  lmotor.set_pwm(1);


  EncoderDriver rencoder(R_ENC_A,
                        R_ENC_B,
                        &right_odom,
                        increment_right_odometry_chanel_a
                        );

  EncoderDriver lencoder(
                        L_ENC_A,
                        L_ENC_B,
                        &left_odom,
                        increment_left_odometry_chanel_a
                        );

  //pinMode(R_PCINT, INPUT);
  //attachPCINT(digitalPinToPCINT(R_PCINT), cb_increment_count, CHANGE);
  
  while (true)
  {
    //rmotor.set_pwm(0);
    //delay(1000);
	  //Serial.print("R");
    // 
  	//Serial.println(right_odom);
    //Serial.print("L");
  	//Serial.println(left_odom);
  }





  //PWM -> PWM_CH_NUM[5].PWM_CDTY =  MAX_DC*(1-duty);
}