#include <Arduino.h>
#include <eurouart.hpp>
#include <motion_controller.hpp>
#include <timer_&_pwm.hpp>
#include <pines_&_constexpr.hpp>
#include "Adafruit_PWMServoDriver.h"
#include "RobotServos.hpp"

/* Iniciamos el protocolo */
uahruart::parser::Protocol protocol;

/* Servos */
auto subir_toldo 				= RobotServo(0);
auto apertura_toldo_der = RobotServo(1);
auto apertura_toldo_izq = RobotServo(2);
auto carter_der					= RobotServo(4);
auto carter_izq					= RobotServo(5);
auto mano_der 					= RobotServo(6);
auto mano_izq						= RobotServo(7);
auto cargador 					= RobotServo(8);

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

/* Interrupciones para carter */
void int_carter_der()
{
	carter_der.set_angle(110);
}
void int_carter_izq()
{
	carter_izq.set_angle(98);
}

bool subiendo_carter 	= false;
bool bajando_carter		= false;

/* Interrupciones para odometría */
void int_odom_derecha()
{
	// controlador_tactico.odom.cuentas_derecha_total++;

	if(digitalReadDirect(Enc_D))
		controlador_tactico.odom.cuentas_derecha++;
	else
		controlador_tactico.odom.cuentas_derecha--;
}
void int_odom_izquierda()
{
	// controlador_tactico.odom.cuentas_izquierda_total++;

	if(digitalReadDirect(Enc_I))
		controlador_tactico.odom.cuentas_izquierda++;
	else
		controlador_tactico.odom.cuentas_izquierda--;
}

/* Odom updates */
unsigned long long last_odom_update = millis();
constexpr unsigned long long ODOM_UPDATE_TIME = 1000;
bool pending_last_odom = true;

/* Registro de métodos en el protocolo de comunicación */
void setup_serial_protocol() 
{
    protocol.on_write([](const char* msg) 
		{
        Serial.println(msg);
    });

    // Register methods
    protocol.register_method("traction", "turn", [](int32_t arg) 
		{
        controlador_tactico.ref_ang = static_cast<float>(arg);
        controlador_tactico.prev_move_calculus(0);
        return uahruart::messages::ActionFinished::TRACTION;
    });

    protocol.register_method("traction", "advance", [](int32_t arg)
		{
        controlador_tactico.ref_distancia = static_cast<float>(arg);
        controlador_tactico.prev_move_calculus(1);
        return uahruart::messages::ActionFinished::TRACTION;
    });

    protocol.register_method("traction", "stop", [](int32_t arg)
		{
				controlador_tactico.stop_movement();
        return uahruart::messages::ActionFinished::TRACTION;
    });

		protocol.register_method("servos", "subir_carter", [](int32_t arg)
		{
			carter_der.set_angle(70);
			carter_izq.set_angle(140);
			subiendo_carter = true;
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "bajar_carter", [](int32_t arg)
		{
			carter_der.set_angle(140);
			carter_izq.set_angle(70);
			bajando_carter = true;
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "abrir_toldo", [](int32_t arg)
		{
			apertura_toldo_der.set_angle(90);
			apertura_toldo_izq.set_angle(180);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "cerrar_toldo", [](int32_t arg)
		{
			apertura_toldo_der.set_angle(110);
			apertura_toldo_izq.set_angle(165);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "subir_toldo", [](int32_t arg)
		{
			subir_toldo.set_angle(200);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "bajar_toldo", [](int32_t arg)
		{
			subir_toldo.set_angle(60);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "bajar_mano_der", [](int32_t arg)
		{
			mano_der.set_angle(190);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "subir_mano_izq", [](int32_t arg)
		{
			mano_der.set_angle(120);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "bajar_mano_der", [](int32_t arg)
		{
			mano_der.set_angle(190);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "subir_mano_der", [](int32_t arg)
		{
			mano_der.set_angle(120);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "disparar", [](int32_t distancia)
		{
			// Faltaría encender el motor de disparo...
			PWM4_SetDuty(0.5);
			cargador.set_angle(190);
			return uahruart::messages::ActionFinished::NONE;
		});

		protocol.register_method("servos", "recoger_cargador", [](int32_t arg)
		{
			// Faltaría apagar el motor de disparo...
			PWM4_SetDuty(0);
			cargador.set_angle(0);
			return uahruart::messages::ActionFinished::NONE;
		});

    protocol.register_method("admin", "reset", [](int32_t arg) 
		{
        rstc_start_software_reset(RSTC);
        return uahruart::messages::ActionFinished::NONE;
    });

    on_finished([]() 
		{
			uahruart::messages::ActionFinished action;
			action.action = uahruart::messages::ActionFinished::TRACTION;
      protocol.send(action);
      odom_tactico = controlador_tactico.odom;
      pending_last_odom = true;
    });
}

/* Configuraciones y bucle de control */
void setup() 
{
	// Serial conf
	Serial.begin(115200);
	setup_serial_protocol();
	servos_config();

	// Pines para el motor
	pinMode(D_EN, OUTPUT);
	pinMode(I_EN, OUTPUT);
	pinMode(D_DIR, OUTPUT);
	pinMode(I_DIR, OUTPUT);
	// Finales de carrera
	pinMode(SUBIR_DER, INPUT_PULLUP);
	pinMode(BAJAR_DER, INPUT_PULLUP);
	pinMode(SUBIR_IZQ, INPUT_PULLUP);
	pinMode(BAJAR_IZQ, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(SUBIR_DER),int_carter_der,FALLING);
	attachInterrupt(digitalPinToInterrupt(BAJAR_DER),int_carter_der,FALLING);
	attachInterrupt(digitalPinToInterrupt(SUBIR_IZQ),int_carter_izq,FALLING);
	attachInterrupt(digitalPinToInterrupt(BAJAR_IZQ),int_carter_izq,FALLING);

	carter_izq.set_angle(98);
	carter_der.set_angle(110);

	// Encoders conf
	pinMode(PCInt_D,INPUT);
	pinMode(Enc_D,INPUT);
	pinMode(PCInt_I,INPUT);	
	pinMode(Enc_I,INPUT);

	attachInterrupt(digitalPinToInterrupt(PCInt_I), int_odom_izquierda, RISING);
	// attachInterrupt(digitalPinToInterrupt(PCInt_D), int_odom_derecha, RISING); 

	// PWMs and Timer (50Hz)
	config_pwms();
	config_timer(TC0, 2, TC2_IRQn, 50);

	// Reset de odometría para empezar
	controlador_tactico.odom.reset_odom();

	// Habilitar las controladoras
	controlador_tactico.motores.encender_motores();

}

void loop() 
{

	serialEvent();
	// Update odometry
	if (pending_last_odom) {
		uahruart::messages::Odometry odom;
		odom.x = odom_tactico.pose_actual.x;
		odom.y = odom_tactico.pose_actual.y;
		odom.o = odom_tactico.pose_actual.alfa;
		if (protocol.send(odom))
			pending_last_odom = false;
	}
	unsigned long long current_time = millis();
	if (current_time > (last_odom_update + ODOM_UPDATE_TIME)) {
		last_odom_update = current_time;
		uahruart::messages::Odometry odom;
		odom.x = controlador_tactico.odom.pose_actual.x;
		odom.y = controlador_tactico.odom.pose_actual.y;
		odom.o = controlador_tactico.odom.pose_actual.alfa;
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
			// Serial.print("CR: ");Serial.println(controlador_tactico.odom.cuentas_derecha);
			// Serial.print("CL: ");Serial.println(controlador_tactico.odom.cuentas_izquierda);
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
