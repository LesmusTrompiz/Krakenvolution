#include <Arduino.h>

ISR(TIMER1_COMPA_vect)    //This is the interrupt request
{	
	traccion_driver.calcular_error(&lazo_abierto, &maxon);
	traccion_driver.update_odom(&Robot,&cuadratura,&maxon);
	traccion_driver.calcular_nueva_consigna();
}


void calcula_error_rueda_derecha (cinematica *variable, param_mecanicos *mecanica)
{
	variable->error_posicion_actual_derecha = variable->distancia_acel_vel_cte - ( ( (cuadratura.contador_derecho_total) * 2 * PI ) /
	( mecanica->pulsos_por_rev * mecanica->reductora ) );
	//distancia_acel_vel_cte
}

void calcula_error_rueda_izquierda (cinematica *variable, param_mecanicos *mecanica)
{
	variable->error_posicion_actual_izquierda = variable->distancia_acel_vel_cte - ( ( (cuadratura.contador_izquierdo_total) * 2 * PI ) /
	( mecanica->pulsos_por_rev * mecanica->reductora ) );
	//distancia_acel_vel_cte
}
void calcula_error_rueda_derecha_final (cinematica *variable, param_mecanicos *mecanica)
{
	variable->error_posicion_actual_derecha_total = variable->distancia_total_rad - ( ( (cuadratura.contador_derecho_total) * 2 * PI ) /
	( mecanica->pulsos_por_rev * mecanica->reductora ) );
}

void calcula_error_rueda_izquierda_final (cinematica *variable, param_mecanicos *mecanica)
{
	variable->error_posicion_actual_izquierda_total = variable->distancia_total_rad - ( ( (cuadratura.contador_izquierdo_total) * 2 * PI ) /
	( mecanica->pulsos_por_rev * mecanica->reductora ) );
}