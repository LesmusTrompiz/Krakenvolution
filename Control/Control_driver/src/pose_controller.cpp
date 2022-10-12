#include <Arduino.h>
#include "pose_controller.hpp"

constexpr uint8_t Kx 	= 5;		// 1/s
constexpr uint8_t Ky 	= 1;		
constexpr uint8_t Kalfa = 2;		// rad/s
constexpr uint8_t ERROR_MINIMO_MM  = 5;
constexpr uint8_t ERROR_MINIMO_RAD = PI/1000; 

#define RAD2DEG(X) X * 180 / PI
#define DEG2RAD(X) X * PI  / 180



extern PoseController robot;

void config_TIMER1(void) //Timer medida tiempos largos con Ts = 20 ms
{
	TCCR1A = 0;                //limpia registros
 	TCCR1B = 0;                //limpia registros
 	TCNT1  = 0;                //Inicializa el contador
	OCR1A  = 39999;
	TCCR1B = (1<<WGM12) | (1<<CS11);
	TIMSK1|=(1<<OCIE1A);   //Set the interrupt request
}


PoseController::PoseController(
		MotorDriver     &rmotor_,
		MotorDriver     &lmotor_,
		EncoderDriver   &rencoder_,
		EncoderDriver   &lencoder_,
		uint16_t         L_,
		uint16_t         R_,
        uint16_t         reductora_):
		L{L_},
		R{R_},
        REDUCTORA{reductora_}
	{
		rmotor = &rmotor_;
		lmotor = &lmotor_;
		rencoder = &rencoder_;
		lencoder = &lencoder_;
		config_TIMER1();
		return;
	}


void PoseController::reset_counts(){
	rencoder->reset_pulses();
	lencoder->reset_pulses();

	return;
}

void PoseController::reset_controller(){
	Pose zero;

	ref_pose   = zero;
	robot_pose = zero;

	return;
}

bool PoseController::check_stop(){
	return (rencoder->read_pulses() <= 3 || lencoder->read_pulses() <= 3);
}

bool PoseController::in_goal(){
	float x_error, alfa_error;
	
	// Calculo de error coordenadas iniciales
    x_error    = ref_pose.x    - robot_pose.x;
    alfa_error = ref_pose.alfa - robot_pose.alfa;

	return (fabs(x_error) < ERROR_MINIMO_MM
			&&
			fabs(alfa_error) < ERROR_MINIMO_RAD);
}


/*
void PoseController::cuentas_to_odom_2(){
	float delta_d, delta_a, deltaX, deltaY = 0;
	int rcounts, lcounts;


	rcounts = rencoder->read_pulses();			// Rads 
	lcounts = rencoder->read_pulses();


	// delta_d = (R * 2 * PI / rencoder->resolucion_encoder*2) * ((rcounts + lcounts) / 2);
	
	delta_d = ((rcounts + lcounts) / 2)*((2*R*PI)/(rencoder->resolucion_encoder*REDUCTORA));
	
	// delta_a = (180 / PI * rencoder->resolucion_encoder*2)   * ((rcounts - lcounts) / (L));
	
	delta_a = (((rcounts - lcounts) / 2)*((2*R*PI)/(rencoder->resolucion_encoder*REDUCTORA)))/L;

	robot_pose.alfa += delta_a * (180/PI);

	deltaX  =	delta_d*cos((robot_pose.alfa)*(PI/180));
	deltaY	=	delta_d*sin((robot_pose.alfa)*(PI/180));

	Serial.print("X"); Serial.println(robot_pose.alfa);

	robot_pose.x += deltaX;
	robot_pose.y += deltaY;

	return;
}
*/

void PoseController::cuentas_to_odom(){
	float delta_d, delta_a, deltaX, deltaY = 0;
	float right_twist, left_twist;


	right_twist = rencoder->get_angle_increment() * R;			// mm
	left_twist  = lencoder->get_angle_increment() * R;			// mm


	delta_d = ((right_twist + left_twist) / 2);					// Rads
	delta_a = (right_twist - left_twist) / L;					// Rads

	robot_pose.alfa += RAD2DEG(delta_a);						// Grados porque es como nos llega la referencia	

	deltaX  =	delta_d*cos(DEG2RAD(robot_pose.alfa));
	deltaY	=	delta_d*sin(DEG2RAD(robot_pose.alfa));

	//Serial.print("X"); Serial.println(robot_pose.alfa);

	robot_pose.x += deltaX;
	robot_pose.y += deltaY;

	return;
}

void PoseController::ley_de_control(const int vd, const int wd){
	float x_error, y_error, alfa_error, x_error_r, y_error_r;
	Consigna cons;
	
	// Calculo de error coordenadas iniciales
    x_error    = ref_pose.x    - robot_pose.x;
    y_error    = ref_pose.y    - robot_pose.y;
    alfa_error = ref_pose.alfa - robot_pose.alfa;

	// Calculo de error respecto a las coordenadas del robot_pose
    x_error_r  =  cos(DEG2RAD(robot_pose.alfa)) * x_error + sin(DEG2RAD(robot_pose.alfa)) * y_error;
    y_error_r  = -sin(DEG2RAD(robot_pose.alfa)) * x_error + cos(DEG2RAD(robot_pose.alfa)) * y_error;

    // Ley de control
    cons.v = Kx    * x_error_r       + vd * cos(DEG2RAD(alfa_error));
	cons.w = Kalfa * sin(DEG2RAD(alfa_error)) + Ky * vd * y_error_r + wd;
	
	return;
}


/*
	Rectas y giros (Velocidad final = 0)
*/
void PoseController::ley_de_control(){
	float x_error, y_error, alfa_error, x_error_r;
	
	// Calculo de error coordenadas iniciales
    x_error    = ref_pose.x    - robot_pose.x;					// mm
    y_error    = ref_pose.y    - robot_pose.y;					// mm
    alfa_error = ref_pose.alfa - robot_pose.alfa;				// mm

	// Calculo de error respecto a las coordenadas del robot_pose
    x_error_r  =  cos(DEG2RAD(robot_pose.alfa)) * x_error + sin(DEG2RAD(robot_pose.alfa)) * y_error;		// mm

    // Ley de control
    cons.v = Kx    * x_error_r;						// mm/s 
	cons.w = Kalfa * sin(DEG2RAD(alfa_error));		// rad/s

	return;
}

void PoseController::consigna_to_velocidad(){

    // Serial.println("V2");
    // Serial.println(cons.v);
	// Serial.println(cons.w);

	v_ruedas.r = (2*cons.v + cons.w * L)/(2 * R);
	v_ruedas.l = (2*cons.v - cons.w * L)/(2 * R);

    // Serial.println("Vr");
    // Serial.println(v_ruedas.r);
	// Serial.println(v_ruedas.l);
	return;	
}
void PoseController::update_motor_speed(){
	rmotor->set_speed(v_ruedas.r);
	lmotor->set_speed(v_ruedas.l);
	
	return;	
}


ISR(TIMER1_COMPA_vect){    // This is the interrupt request
	
	// 1º Calcular el error:
	//  right_odom y left_odom -> cuentas -> mm -> (x,y,alfa)
	robot.cuentas_to_odom();

	// 2º Calcular la consigna:
	robot.ley_de_control(); 	// m/s y rad/s
	robot.consigna_to_velocidad();	// rad/s

	// 3º Actualizar las salidas:
	robot.update_motor_speed();


	// 4º Check de fin de movimiento
	if(robot.check_stop()
		&& 
		robot.in_goal()) robot.reset_controller();

	robot.reset_counts();
}
