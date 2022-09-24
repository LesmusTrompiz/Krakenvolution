#include "control_fns.hpp"
#include <stdint.h>


/*
void calcula_parametros_recta(const int16_t &dist_mm, const CinematicParams &params,ConsignaMotor &consigna ){
    // Calculamos el número de cuentas hasta alcanzar la velocidad en steady state
    consigna.counts_Transient   = fabs(dist_mm/10)/(mecanica->diametro/2);
    consigna.PwmDuty_Transient  = fabs(dist_mm/10)/(mecanica->diametro/2);

    // Calculamos el número de vueltas que tiene que realizar en steady state
    consigna.counts_SteadyState  = calculo_de_frenada(variable,mecanica);
    consigna.PwmDuty_SteadyState = fabs(dist_mm/10)/(mecanica->diametro/2);
}


void calcula_parametros_giro(int spin_degrees, CinematicParams params,consigna ConsignaMotor){
    // Calculamos el número de cuentas hasta alcanzar la velocidad en steady state
    distancia_total_rad = fabs(distancia/10)/(mecanica->diametro/2);

    // Calculamos el número de vueltas que tiene que realizar en steady state
    calculo_de_frenada(variable,mecanica);

    // Devolvemos la velocidad actual de los motores
    

}

*/