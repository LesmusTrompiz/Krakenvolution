#include <Arduino.h>
#include "motor_driver.hpp"
#include "encoder_driver.hpp"
#include "PinChangeInterrupt.h"
#include "pose_controller.hpp"

#define RPM2RAD(x) x * (2 * PI)/60 
// Pines de controladoras Maxon Derecha OCR0B
constexpr uint8_t R_DIR  = 2;
constexpr uint8_t R_EN   = 5;
constexpr uint8_t R_PWM  = 13;

// Pines de controladoras Maxon Izquierda OCR0A
constexpr uint8_t L_PWM  = 4;
constexpr uint8_t L_EN   = 7;
constexpr uint8_t L_DIR  = 8;

// Parámetros físicos/mecánicos
constexpr uint16_t L         = 370;
constexpr uint16_t R         = 60;
constexpr uint16_t REDUCTORA = 26;


// Variables globales con las cuentas de odometría
int16_t volatile right_odom = 0;
int16_t volatile left_odom = 0;

// Constante con el valor de la resolución del encoder
constexpr uint16_t ENCODER_RESOLUTION = 256 * 2;          //  pulsos / vuelta es decir radianes
                                                          // 256 es la resolución del encoder pero como leemos en los dos canales se multiplica por dos 2

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
                        REDUCTORA,
                        &right_odom,
                        increment_right_odometry_channel_a
                        );

  EncoderDriver lencoder(
                        L_ENC_A,
                        L_ENC_B,
                        ENCODER_RESOLUTION,
                        REDUCTORA,
                        &left_odom,
                        increment_left_odometry_chanel_a
                        );

  PoseController robot(rmotor,
                       lmotor,
                       rencoder,
                       lencoder,
                       L,
                       R,
                       REDUCTORA
                       );

void setup() {

  // Configuración del TIMER1
  robot.config_TIMER1();
  // Inicialización del puerto serie
  Serial.begin(115200);
}

void loop() 
{
  // robot.ref_pose = Pose(100,0,0);
  while (true)
  {
  }
}
