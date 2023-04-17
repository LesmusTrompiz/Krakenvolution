/**
 * Autor: Navil Abeselam Abdel-lah y Javier Quintanar
 * 
 * @brief Esta biblioteca contiene las funciones necesarias para
 * la configuración de timers y pwms para la placa Arduino Due.
 * El proyecto está destinado a la competición Eurobot2023, por 
 * lo que las funciones no tienen por qué ser genéricas.
 * 
*/

#pragma once
#include <Arduino.h>

#define MAX_DC 8400

/* Configuraciones básicas */
void config_pwms();
void config_control_timer();

/* Modificación de ciclo de trabajo PWMs */
void PWMA_SetDuty(float ciclo);
void PWMB_SetDuty(float ciclo);


