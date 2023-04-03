#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class RobotServo
{
  public:
    // Constructores
    RobotServo(const uint8_t _ID_AnalogRead, const uint8_t _PWM_ID, Adafruit_PWMServoDriver* _ServoHandler);
    RobotServo(const uint8_t _PWM_ID, Adafruit_PWMServoDriver* _ServoHandler);
    // Calibración
    void calibration(float angle_init, float angle_final);
    // Método
    void set_angle(float angle);
    // Lectura ángulo
    float read_angle();
    // Pin lectura feedback
    const uint8_t ID_AnalogRead;
    // Pin de salida PWM
    const uint8_t PWM_ID;
    // Servos handler 
    Adafruit_PWMServoDriver* ServoHandler;
    // Función de transferencia nominal
    float m      = 2;
    float n      = 100;
    // Función de lectura
    const float m_read = 0.47;
    const float n_read = -33.4;
};
