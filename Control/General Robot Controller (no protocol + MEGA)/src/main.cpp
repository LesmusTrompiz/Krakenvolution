#define debug_mode

#include <Arduino.h>
#include <eurouart.hpp>
#include <motion_controller.hpp>
#include <timer_&_pwm.hpp>
#include <pines_&_constexpr.hpp>
#include <PinChangeInterrupt.h>
/* Definición completa del robot */

Param_mecanicos mecanica_enzima(
 enzima_acel,
 enzima_decel,
 enzima_reductora,
 enzima_vel_eje_max,
 enzima_vel_max,
 vel_giro_enzima,
 vel_freno_enzima,
 enzima_pulsos_por_rev,
 enzima_L,
 enzima_diametro
);

Odom odom_enzima;

Motores motores_enzima(
	enzima_vel_max,
	D_EN,
	D_DIR,
	PWMA_SetDuty,
	I_EN,
	I_DIR,
	PWMB_SetDuty
);

PerfilVelocidad perfilador_enzima(
	ajuste_dist_recta_enzima,
	ajuste_dist_giro_enzima,
	mecanica_enzima
);

motion_controller controlador_enzima(
	mecanica_enzima,
	odom_enzima,
	motores_enzima,
	perfilador_enzima
);

/* Interrupciones para odometría */
void int_odom_derecha()
{
	// controlador_enzima.odom.cuentas_derecha_total++;

	if(digitalRead(Enc_D))
		controlador_enzima.odom.cuentas_derecha++;
	else
		controlador_enzima.odom.cuentas_derecha--;
}
void int_odom_izquierda()
{
	// controlador_enzima.odom.cuentas_izquierda_total++;

	if(digitalRead(Enc_I))
		controlador_enzima.odom.cuentas_izquierda++;
	else
		controlador_enzima.odom.cuentas_izquierda--;
}

/* Configuraciones y bucle de control */
void setup() 
{
	// Serial conf
	Serial.begin(115200);

	// Pines para el motor
	pinMode(D_EN, OUTPUT);
	pinMode(I_EN, OUTPUT);
	pinMode(D_DIR, OUTPUT);
	pinMode(I_DIR, OUTPUT);
	pinMode(D_PWM, OUTPUT);
	pinMode(I_PWM, OUTPUT);
	// Int conf
	pinMode(PCInt_D,INPUT);
	pinMode(Enc_D,INPUT);
	pinMode(PCInt_I,INPUT);	
	pinMode(Enc_I,INPUT);

	// attachInterrupt(digitalPinToInterrupt(PCInt_I), int_odom_izquierda, RISING);
	attachPCINT(digitalPinToPCINT(PCInt_D), int_odom_derecha, RISING); 

	// PWMs and Timer (50Hz)
	config_pwms();
	config_control_timer();

	// Reset de odometría para empezar
	controlador_enzima.odom.reset_odom();

	// Habilitar las controladoras
	controlador_enzima.motores.encender_motores();
	Serial.print("Registro PWM A: ");Serial.println(OCR0A);
	Serial.print("Registro PWM A: ");Serial.println(OCR0B);
	// Check...
	Serial.println("Configuration done...");
}

void loop() 
{
	serialEvent();
}

/* Bucle de control -> TC2 Handler*/
ISR(TIMER1_COMPA_vect)
{
	#ifdef debug_mode	
		static int debugger = 0;
		if(debugger >= 1*(50))
		{
			Serial.print("CR: ");Serial.println(controlador_enzima.odom.cuentas_derecha);
			Serial.print("CL: ");Serial.println(controlador_enzima.odom.cuentas_izquierda);
			// if(controlador_enzima.recta_en_curso)
			// Serial.println("Recta en curso...");	
			// if(controlador_enzima.giro_en_curso)
			// Serial.println("Giro en curso...");
			// if(controlador_enzima.parado)
			// Serial.println("Robot parado...")
			// Serial.print("X: ");Serial.println(controlador_enzima.odom.pose_actual.x);
			// Serial.print("Y: ");Serial.println(controlador_enzima.odom.pose_actual.y);
			// Serial.print("O: ");Serial.println(controlador_enzima.odom.pose_actual.alfa);
			// Serial.print("RB: ");Serial.println(OCR0A);
			// Serial.print("RA: ");Serial.println(OCR0B);
			debugger = 0;
		}
		debugger++;
	#endif

	// Ejecutar control
  controlador_enzima.move_control();
}