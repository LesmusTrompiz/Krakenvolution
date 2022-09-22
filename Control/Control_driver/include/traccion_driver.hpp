#pragma once

#include "motor_driver.hpp"
#include "encoder_driver.hpp"
#include "control_fns.hpp"



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
        MotorDriver     &rmotor;
        MotorDriver     &lmotor;
        EncoderDriver   &rencoder;
        EncoderDriver   &lencoder;
        CinematicParams &cinematic_params;
        ConsignaMotor   &consigna_derecha;
        ConsignaMotor   &consigna_izquierda;
        ErrorMotor      &error_derecha;
        ErrorMotor      &error_izquierda;
};




void TraccionDriver::straight(int mm){

    /*
    calcula_parametros_recta(mm,cinematic_params,consigna);
    rmotor.set_speed(consigna_derecha)
    rmotor.set_speed(consigna_izquierda)
    rmotor.set_speed(consigna_derecha)
    */
}


