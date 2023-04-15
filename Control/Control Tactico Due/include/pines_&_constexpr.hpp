#pragma once

// Pines de control de la tracción
// PWMH5
#define I_PWM 		44
#define I_EN 		46
#define I_DIR  		48
// PWMH6
#define D_DIR  		49
#define D_EN 		47
#define D_PWM 		45
// Pines de encoder de motores
#define Enc_I 		50
#define PCInt_I 	52
#define PCInt_D 	51
#define Enc_D 		53

// Parámetros mecánicos
constexpr float tactico_reductora = 26;
constexpr float tactico_diametro = 72; //67;
constexpr float tactico_vel_eje_max = 8100 / tactico_reductora;
constexpr float tactico_vel_max = tactico_vel_eje_max * PI / 30;
constexpr float tactico_pulsos_por_rev = 256;
constexpr float tactico_L = 264;
constexpr float tactico_acel = 4500;
constexpr float tactico_decel = 4500;

// Constantes para el perfil de velocidades
constexpr float vel_freno_tactico         = 0.8;
constexpr float vel_giro_tactico          = 8;
constexpr float ajuste_dist_recta_tactico = 0.25;
constexpr float ajuste_dist_giro_tactico  = 0.1;

// Servos
enum
{
	pwm_disparador,
	pwm_carter_izq, 		// Trucado
	pwm_carter_der,			// Trucado
	pwm_carter_apertura,
	pwm_toldo_izq,
	pwm_toldo_der,
	pwm_toldo_move
};

