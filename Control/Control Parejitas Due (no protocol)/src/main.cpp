// #define debug_mode
#define servo_debug

#include <Arduino.h>
#include <eurouart.hpp>
#include <motion_controller.hpp>
#include <timer_&_pwm.hpp>
#include <pines_&_constexpr.hpp>
#include <Adafruit_PWMServoDriver.h>
#include <RobotServos.hpp>

/* Servos */
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);
constexpr uint8_t BRAZO_DER = 0;
auto Servo_brazo_der = RobotServo(BRAZO_DER, servos);
constexpr uint8_t BRAZO_CEN = 10;
auto Servo_brazo_cen = RobotServo(BRAZO_CEN, servos);
constexpr uint8_t BRAZO_IZQ = 11;
auto Servo_brazo_izq = RobotServo(BRAZO_IZQ, servos);

/* Definición mecánica del robot */
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
	PWM6_SetDuty,
	I_EN,
	I_DIR,
	PWM5_SetDuty
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

	if(digitalReadDirect(Enc_D))
		controlador_enzima.odom.cuentas_derecha++;
	else
		controlador_enzima.odom.cuentas_derecha--;
}
void int_odom_izquierda()
{
	// controlador_enzima.odom.cuentas_izquierda_total++;

	if(digitalReadDirect(Enc_I))
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
	// Int conf
	pinMode(PCInt_D,INPUT);
	pinMode(Enc_D,INPUT);
	pinMode(PCInt_I,INPUT);	
	pinMode(Enc_I,INPUT);

	// attachInterrupt(digitalPinToInterrupt(PCInt_I), int_odom_izquierda, RISING);
	attachInterrupt(digitalPinToInterrupt(PCInt_D), int_odom_derecha, RISING); 

	// PWMs and Timer (50Hz)
	config_pwms();
	config_timer(TC0, 2, TC2_IRQn, 50);

	// Reset de odometría para empezar
	controlador_enzima.odom.reset_odom();

	// Habilitar las controladoras
	controlador_enzima.motores.encender_motores();

	// Servos
	servos.begin();
	servos.setPWMFreq(50);

	#ifdef servo_debug
		Servo_brazo_cen.set_angle(15);
		Servo_brazo_izq.set_angle(35);
		Servo_brazo_der.set_angle(75);
	#endif

	// Check...
	Serial.println("Configuration done...");
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
			Serial.print("CR: ");Serial.println(controlador_enzima.odom.cuentas_derecha);
			Serial.print("CL: ");Serial.println(controlador_enzima.odom.cuentas_izquierda);
			// if(controlador_enzima.recta_en_curso)
			// Serial.println("Recta en curso...");	
			// if(controlador_enzima.giro_en_curso)
			// Serial.println("Giro en curso...");
			// if(controlador_enzima.parado)
			// Serial.println("Robot parado...")
			Serial.print("X: ");Serial.println(controlador_enzima.odom.pose_actual.x);
			Serial.print("Y: ");Serial.println(controlador_enzima.odom.pose_actual.y);
			Serial.print("O: ");Serial.println(controlador_enzima.odom.pose_actual.alfa);
			debugger = 0;
		}
		debugger++;
	#endif

	// Ejecutar control
  controlador_enzima.move_control();
}