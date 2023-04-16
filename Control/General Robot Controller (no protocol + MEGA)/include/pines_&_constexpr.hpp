#pragma once

// Pines de control de la tracción
// PWMH5
#define I_PWM 		4
#define I_EN 			7
#define I_DIR  		8
// PWMH6
#define D_PWM 		13
#define D_EN 			5
#define D_DIR  		2
// Pines de encoder de motores
#define Enc_I 		3
#define PCInt_I 	10
#define PCInt_D 	11
#define Enc_D 		12

// Parámetros mecánicos
constexpr float enzima_reductora = 26*26/39; 
constexpr float enzima_diametro = 52.3; // 72; //67; 
constexpr float enzima_vel_eje_max = 8100 / enzima_reductora;																										
constexpr float enzima_vel_max = enzima_vel_eje_max * PI / 30;
constexpr float enzima_pulsos_por_rev = 256;
constexpr float enzima_L = 150.63;
constexpr float enzima_acel = 1500;
constexpr float enzima_decel = 1500;

// Constantes para el perfil de velocidades
constexpr float vel_freno_enzima         = 0.8;
constexpr float vel_giro_enzima          = 8;
constexpr float ajuste_dist_recta_enzima = 0.25;
constexpr float ajuste_dist_giro_enzima  = 0.1;
