// #define debug_mode

#include <Arduino.h>
#include <eurouart.hpp>
#include <motion_controller.hpp>
#include <timer_&_pwm.hpp>
#include <pines_&_constexpr.hpp>
#include <Adafruit_PWMServoDriver.h>
#include <RobotServos.hpp>

/* Iniciamos el protocolo */
uahruart::parser::Protocol protocol;

/* Servos */
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);
constexpr uint8_t BRAZO_DER = 0;
constexpr uint8_t BRAZO_CEN = 10;
constexpr uint8_t BRAZO_IZQ = 11;
auto Servo_brazo_der = RobotServo(BRAZO_DER, servos);
auto Servo_brazo_cen = RobotServo(BRAZO_CEN, servos);
auto Servo_brazo_izq = RobotServo(BRAZO_IZQ, servos);

/* Definición completa del robot */
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

/* Odom updates */
unsigned long long last_odom_update = millis();
constexpr unsigned long long ODOM_UPDATE_TIME = 100;
bool pending_last_odom = true;
bool finished_movement = false;

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

    protocol.register_method("traction", "stop", [](int32_t arg)
		{
				controlador_parejitas.stop_movement();
        return uahruart::messages::ActionFinished::TRACTION;
    });

    protocol.register_method("admin", "reset", [](int32_t arg) 
		{
        rstc_start_software_reset(RSTC);
        return uahruart::messages::ActionFinished::NONE;
    });

		protocol.register_method("servos", "open", [](int32_t arg)
		{
			Servo_brazo_cen.set_angle(90);
			Servo_brazo_izq.set_angle(140);
			Servo_brazo_der.set_angle(15);
      return uahruart::messages::ActionFinished::NONE;
		}); 

		protocol.register_method("servos", "close", [](int32_t arg)
		{
			Servo_brazo_cen.set_angle(60);
			Servo_brazo_izq.set_angle(35);
			Servo_brazo_der.set_angle(80);
      return uahruart::messages::ActionFinished::NONE;
		}); 

		protocol.register_method("servos", "move", [](int32_t arg)
		{
			Servo_brazo_cen.set_angle(90);
			Servo_brazo_izq.set_angle(110);
			Servo_brazo_der.set_angle(15);	
      return uahruart::messages::ActionFinished::NONE;
		}); 

    on_finished([]() 
		{
			finished_movement = true;
    	odom_parejitas = controlador_parejitas.odom;
    });
}

/* Configuraciones y bucle de control */
void setup() 
{
	// Serial conf
	Serial.begin(115200);
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
	// attachInterrupt(digitalPinToInterrupt(PCInt_D), int_odom_derecha, RISING); 

	// PWMs and Timer (50Hz)
	config_pwms();
	config_timer(TC0, 2, TC2_IRQn, 50);

	// Reset de odometría para empezar
	controlador_parejitas.odom.reset_odom();

	// Habilitar las controladoras
	controlador_parejitas.motores.encender_motores();

	// Servos
	servos.begin();
	servos.setPWMFreq(50);

	#ifdef servo_debug
		Servo_brazo_cen.set_angle(15);
		Servo_brazo_izq.set_angle(35);
		Servo_brazo_der.set_angle(75);
	#endif
}


void loop() 
{
	serialEvent();
	if (finished_movement) {
		finished_movement = false;
		delay(50);
		uahruart::messages::ActionFinished action;
		action.action = uahruart::messages::ActionFinished::TRACTION;
    protocol.send(action);
    pending_last_odom = true;
	}
	// Update odometry
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
			// Serial.print("CR: ");Serial.println(controlador_parejitas.odom.cuentas_derecha);
			// Serial.print("CL: ");Serial.println(controlador_parejitas.odom.cuentas_izquierda);
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
