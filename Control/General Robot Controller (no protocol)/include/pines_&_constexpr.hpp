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
constexpr float enzima_reductora = 26; 
constexpr float enzima_diametro = 72; //67; 
constexpr float enzima_vel_eje_max = 8100 / enzima_reductora;																										
constexpr float enzima_vel_max = enzima_vel_eje_max * PI / 30;
constexpr float enzima_pulsos_por_rev = 256;
constexpr float enzima_L = 150.63;
constexpr float enzima_acel = 4500;
constexpr float enzima_decel = 4500;

// Constantes para el perfil de velocidades
constexpr float vel_freno_enzima         = 0.8;
constexpr float vel_giro_enzima          = 8;
constexpr float ajuste_dist_recta_enzima = 0.25;
constexpr float ajuste_dist_giro_enzima  = 0.1;