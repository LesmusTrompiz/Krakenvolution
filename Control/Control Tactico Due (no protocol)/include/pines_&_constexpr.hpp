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
#define Enc_I 		26
#define PCInt_I 	24
#define PCInt_D 	27
#define Enc_D 		25

// Parámetros mecánicos
constexpr float tactico_reductora = 26; 
constexpr float tactico_diametro = 72.5; //67; 
constexpr float tactico_vel_eje_max = 8100 / tactico_reductora;																										
constexpr float tactico_vel_max = (tactico_vel_eje_max * PI / 30);
constexpr float tactico_pulsos_por_rev = 256;
constexpr float tactico_L = 258; 
constexpr float tactico_acel = 4500;
constexpr float tactico_decel = 4500;

// Constantes para el perfil de velocidades
constexpr float vel_freno_tactico         = 0.8;
constexpr float vel_giro_tactico          = 8;
constexpr float ajuste_dist_recta_tactico = 0.75;
constexpr float ajuste_dist_giro_tactico  = 0.1;
constexpr float ajuste_error_tactico      = 0.02;
