// #define debug_mode

#include <RobotServos.hpp>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <eurouart.hpp>
#include <protocol.hpp>
#include <motion_controller.hpp>
#include <timer_&_pwm.hpp>
#include <pines_&_constexpr.hpp>

/* Iniciamos el protocolo */
uahruart::parser::Protocol protocol;

/* Servos */
Adafruit_PWMServoDriver ServoHandlerMaster 	= Adafruit_PWMServoDriver(0x40);
auto servo_brazo_derecha 					= RobotServo(brazo_derecho, ServoHandlerMaster);
auto servo_brazo_izquierda	    			= RobotServo(brazo_izquierdo, ServoHandlerMaster);
//... auto servo_

/* Definición completa de la mecánica del robot */
Param_mecanicos mecanica_parejitas(
 parejitas_acel,
 parejitas_decel,
 parejitas_reductora,
 parejitas_vel_eje_max,
 parejitas_vel_max,
 vel_giro_parejitas,
 vel_freno_parejitas,
 parejitas_pulsos_por_rev,
 parejitas_L,
 parejitas_diametro
);

Odom odom_parejitas;
bool pending_last_odom = true;

Motores motores_parejitas(
	parejitas_vel_max,
	D_EN,
	D_DIR,
	PWM6_SetDuty,
	I_EN,
	I_DIR,
	PWM5_SetDuty
);

PerfilVelocidad perfilador_parejitas(
	ajuste_dist_recta_parejitas,
	ajuste_dist_giro_parejitas,
	mecanica_parejitas
);

motion_controller controlador_parejitas(
	mecanica_parejitas,
	odom_parejitas,
	motores_parejitas,
	perfilador_parejitas
);

/* Interrupciones para odometría */
void int_odom_derecha()
{
	// controlador_parejitas.odom.cuentas_derecha_total++;

	if(digitalReadDirect(Enc_D))
		controlador_parejitas.odom.cuentas_derecha--;
	else
		controlador_parejitas.odom.cuentas_derecha++;
}
void int_odom_izquierda()
{
	// controlador_parejitas.odom.cuentas_izquierda_total++;

	if(digitalReadDirect(Enc_I))
		controlador_parejitas.odom.cuentas_izquierda++;
	else
		controlador_parejitas.odom.cuentas_izquierda--;
}

/* Registro de métodos en el protocolo de comunicación */
void setup_serial_protocol() 
{
    protocol.on_write([](const char* msg) 
		{
        SerialUSB.println(msg);
    });

    // Register methods
    protocol.register_method("traction", "turn", [](int32_t arg) 
		{
        controlador_parejitas.ref_ang = static_cast<float>(arg);
        controlador_parejitas.prev_move_calculus(0);
        return uahruart::messages::ActionFinished::TRACTION;
    });

    protocol.register_method("traction", "advance", [](int32_t arg)
		{
        controlador_parejitas.ref_distancia = static_cast<float>(arg);
        controlador_parejitas.prev_move_calculus(1);
        return uahruart::messages::ActionFinished::TRACTION;
    });

    protocol.register_method("admin", "reset", [](int32_t arg) 
		{
        rstc_start_software_reset(RSTC);
        return uahruart::messages::ActionFinished::NONE;
    });

    protocol.register_method("actuadores", "servo_brazo_derecha", [](int32_t arg) 
		{
        servo_brazo_derecha.set_angle(arg);
        return uahruart::messages::ActionFinished::SERVO;
    });

    on_finished([]() 
		{
			uahruart::messages::ActionFinished action;
			action.action = uahruart::messages::ActionFinished::TRACTION;
      protocol.send(action);
      odom_parejitas = controlador_parejitas.odom;
      pending_last_odom = true;
    });
}

/* Configuraciones y bucle de control */
void setup() 
{
	// Adafruit driver conf
	ServoHandlerMaster.begin();
	ServoHandlerMaster.setPWMFreq(50);
	// Serial conf
	SerialUSB.begin(115200);
  setup_serial_protocol();
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
	controlador_parejitas.odom.reset_odom();

	// Habilitar las controladoras
	controlador_parejitas.motores.encender_motores();

	// Check...
}

unsigned long long last_odom_update = millis();
constexpr unsigned long long ODOM_UPDATE_TIME = 1000;
void loop() 
{
	serialEvent();
	if (pending_last_odom) {
		uahruart::messages::Odometry odom;
		odom.x = odom_parejitas.pose_actual.x;
		odom.y = odom_parejitas.pose_actual.y;
		odom.o = odom_parejitas.pose_actual.alfa;
		if (protocol.send(odom))
			pending_last_odom = false;
	}
	unsigned long long current_time = millis();
	if (current_time > (last_odom_update + ODOM_UPDATE_TIME)) {
		last_odom_update = current_time;
		uahruart::messages::Odometry odom;
		odom.x = controlador_parejitas.odom.pose_actual.x;
		odom.y = controlador_parejitas.odom.pose_actual.y;
		odom.o = controlador_parejitas.odom.pose_actual.alfa;
		protocol.send(odom);
	}

}

/* Bucle de control -> TC2 Handler*/
void TC2_Handler(void)
{
  TC_GetStatus(TC0, 2);

	#ifdef debug_mode	
		static int debugger = 0;
		if(debugger >= 1*(50))
		{
			Serial.print("CR: ");Serial.println(controlador_parejitas.odom.cuentas_derecha);
			Serial.print("CL: ");Serial.println(controlador_parejitas.odom.cuentas_izquierda);

			Serial.print("X: ");Serial.println(controlador_parejitas.odom.pose_actual.x);
			Serial.print("Y: ");Serial.println(controlador_parejitas.odom.pose_actual.y);
			Serial.print("O: ");Serial.println(controlador_parejitas.odom.pose_actual.alfa);
			debugger = 0;
		}
		debugger++;
	#endif

	// Ejecutar control
  controlador_parejitas.move_control();
}
