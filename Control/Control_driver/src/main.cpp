#include <Arduino.h>
#include "motor_driver.hpp"
#include "encoder_driver.hpp"
#include "PinChangeInterrupt.h"
#include "pose_controller.hpp"


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

constexpr uint16_t ENCODER_RESOLUTION = 256;

int16_t volatile right_odom = 0;
int16_t volatile left_odom = 0;

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


  EncoderDriver rencoder(R_ENC_A,
                        R_ENC_B,
                        ENCODER_RESOLUTION,
                        &right_odom,
                        increment_right_odometry_channel_a
                        );

  EncoderDriver lencoder(
                        L_ENC_A,
                        L_ENC_B,
                        ENCODER_RESOLUTION,
                        &left_odom,
                        increment_left_odometry_chanel_a
                        );

PoseController robot(rmotor,lmotor,rencoder,lencoder,25,13);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
	
}



void loop() {
 
  
  
  while (true)
  {
   
  }
}