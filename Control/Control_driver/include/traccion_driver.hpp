#pragma once

#include "motor_driver.hpp"

class TraccionDriver{

    // Funciones interfaz
    void spin(int degrees);
    void straight(int mm);
    void stop(int mm);
    void muevete_coordenada(int xyo);


    TraccionDriver(MotorDriver motor_derecho, MotorDriver motor_izquierdo);

    // Funciones de control
    private:
        float x;
        float y;
        float o;
        MotorDriver motor_derecho;
        MotorDriver motor_izquierdo;
        void set_duty_cycle(int duty_cycle);


};




