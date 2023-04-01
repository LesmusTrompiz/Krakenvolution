// #define debug_mode

#include <Arduino.h>
#include <eurouart.hpp>
#include <motion_controller.hpp>
#include <timer_&_pwm.hpp>
#include <pines_&_constexpr.hpp>

/* Definición completa del robot */

Param_mecanicos mecanica_tactico(
 tactico_acel,
 tactico_decel,
 tactico_reductora,
 tactico_vel_eje_max,
 tactico_vel_max,
 vel_giro_tactico,
 vel_freno_tactico,
 tactico_pulsos_por_rev,
 tactico_L,
 tactico_diametro
);

Odom odom_tactico;

Motores motores_tactico(
	tactico_vel_max,
	D_EN,
	D_DIR,
	PWM6_SetDuty,
	I_EN,
	I_DIR,
	PWM5_SetDuty
);

PerfilVelocidad perfilador_tactico(
	ajuste_dist_recta_tactico,
	ajuste_dist_giro_tactico,
	mecanica_tactico
);

motion_controller controlador_tactico(
	mecanica_tactico,
	odom_tactico,
	motores_tactico,
	perfilador_tactico
);

/* Interrupciones para odometría */
void int_odom_derecha()
{
	// controlador_tactico.odom.cuentas_derecha_total++;

	if(digitalReadDirect(Enc_D))
		controlador_tactico.odom.cuentas_derecha--;
	else
		controlador_tactico.odom.cuentas_derecha++;
}
void int_odom_izquierda()
{
	// controlador_tactico.odom.cuentas_izquierda_total++;

	if(digitalReadDirect(Enc_I))
		controlador_tactico.odom.cuentas_izquierda++;
	else
		controlador_tactico.odom.cuentas_izquierda--;
}

/* Configuraciones y bucle de control */
void setup() 
{
	// Serial conf
	Serial.begin(115200);
    setup_serial();

	// Pines para el motor
	pinMode(D_EN, OUTPUT);
	pinMode(I_EN, OUTPUT);
	pinMode(D_DIR, OUTPUT);
	pinMode(I_DIR, OUTPUT);
	// Int conf
	pinMode(PCInt_D,INPUT);
	pinMode(Enc_D,INPUT);
	pinMode(PCInt_I,INPUT);	
	pinMode(Enc_I,INPUT);

	attachInterrupt(digitalPinToInterrupt(PCInt_I), int_odom_izquierda, RISING);
	attachInterrupt(digitalPinToInterrupt(PCInt_D), int_odom_derecha, RISING); 

	// PWMs and Timer (50Hz)
	config_pwms();
	config_timer(TC0, 2, TC2_IRQn, 50);

	// Reset de odometría para empezar
	controlador_tactico.odom.reset_odom();

	// Habilitar las controladoras
	controlador_tactico.motores.encender_motores();

	// Check...
}

void loop() 
{
	serialEvent();
}

/* Bucle de control -> TC2 Handler*/
void TC2_Handler(void)
{
  TC_GetStatus(TC0, 2);

	#ifdef debug_mode	
		static int debugger = 0;
		if(debugger >= 1*(50))
		{
			Serial.print("CR: ");Serial.println(controlador_tactico.odom.cuentas_derecha);
			Serial.print("CL: ");Serial.println(controlador_tactico.odom.cuentas_izquierda);
			// if(controlador_tactico.recta_en_curso)
			// Serial.println("Recta en curso...");	
			// if(controlador_tactico.giro_en_curso)
			// Serial.println("Giro en curso...");
			// if(controlador_tactico.parado)
			// Serial.println("Robot parado...")
			Serial.print("X: ");Serial.println(controlador_tactico.odom.pose_actual.x);
			Serial.print("Y: ");Serial.println(controlador_tactico.odom.pose_actual.y);
			Serial.print("O: ");Serial.println(controlador_tactico.odom.pose_actual.alfa);
			debugger = 0;
		}
		debugger++;
	#endif

	// Ejecutar control
  controlador_tactico.move_control();
}
