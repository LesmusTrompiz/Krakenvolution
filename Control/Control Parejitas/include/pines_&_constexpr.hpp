#pragma once

// Pines de control de la tracción
// PWMH5
#define I_PWM 		44
#define I_EN 			46
#define I_DIR  		48
#define I_STOP 		50
// PWMH6
#define D_STOP 		51
#define D_DIR  		49
#define D_EN 			47
#define D_PWM 		45
// Pines de encoder de motores
#define Enc_I 		50
#define PCInt_I 	52
#define PCInt_D 	51
#define Enc_D 		53

// Parámetros mecánicos
constexpr float parejitas_reductora = 26.0*26.0/39.0;	// Engranajes
constexpr float parejitas_diametro = 52.3; 			// Tactico: 72; //67;
constexpr float parejitas_vel_eje_max = 8100 / parejitas_reductora;
constexpr float parejitas_vel_max = parejitas_vel_eje_max * PI / 30;
constexpr float parejitas_pulsos_por_rev = 256.0;
constexpr float parejitas_L = 248.0;
constexpr float parejitas_acel = 	4000.0; // 4500;
constexpr float parejitas_decel = 4000.0; // 4500;

// Constantes para el perfil de velocidades
constexpr float vel_freno_parejitas         = 0.8;
constexpr float vel_giro_parejitas          = 8;
constexpr float ajuste_dist_recta_parejitas = 0.25;
constexpr float ajuste_dist_giro_parejitas  = 0.1;

// Servos
enum
{
	brazo_derecho,
	brazo_izquierdo
};

