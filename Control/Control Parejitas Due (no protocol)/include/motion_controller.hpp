/**
 * Autor: Navil Abeselam Abdel-lah y Javier Quintanar
 * 
 * @brief Biblioteca destinada a realizar las tareas 
 * de control de tracción del robot.
*/

#pragma once

#include <Arduino.h>

/* Estructuras y funciones auxiliares */
inline int digitalReadDirect(int pin){
 return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

inline void digitalWriteDirect(int pin, boolean val)
{
  /**
  * @brief Función auxiliar para reducir los tiempos
  * de ejecución al escribir en los pines de la placa.
  * Función válida solo para Arduino Due (y eq).
  */
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

#define DEG2RAD(x) x*((2*PI)/360)
#define RAD2DEG(x) x*(360/(2*PI))

struct Pose
{
  /**
   * @brief Estructura auxiliar para almacenar pose
   * 
  */
  float x;
  float y;
  float alfa;

  Pose() : x{0}, y{0}, alfa{0}{};
  Pose(float _x, float _y, float _alfa) : x{_x}, y{_y}, alfa{_alfa}{};
};

struct Consigna
{
  /**
   * @brief Estructura auxiliar para almacenar consginas
   * de velocidades absolutas con respecto al robot.
   * 
  */
  float v;                      // Velocidad lineal
  float w;                      // Velocidad angular
  Consigna() : v{0}, w{0} {};
  Consigna(float _v, float _w) : v{_v}, w{_w}{};
};

struct Motores
{
  /**
   * @brief Estructura auxiliar para mandar las 
   * consignas de velocidad para cada rueda.
   * 
  */
  
  // Pines de la placa
  const uint8_t R_EN;
  const uint8_t R_DIR;
  const uint8_t L_EN;
  const uint8_t L_DIR;
  // Sentidos de acción
  bool sentido_der;
  bool sentido_izq;
  // Velocidades
  float rmotor_vel;
  float lmotor_vel;
  float vel_max;
  // Habilitar y deshabilitar motores
  void apagar_motores();
  void encender_motores();
  // Wrapper para mandar velocidades
  void set_vel_rmotor();
  void set_vel_lmotor();
  // Recibo por puntero funciones para PWM
  void (*set_pwm_rmotor)(float velocidad_rads);
  void (*set_pwm_lmotor)(float velocidad_rads);
  // Constructor
  Motores(float _vel_max,
          uint8_t _R_EN,
          uint8_t _R_DIR,
          void (*_set_pwm_rmotor)(float),
          uint8_t _L_EN,
          uint8_t _L_DIR,
          void (*_set_pwm_lmotor)(float)) 
  : rmotor_vel{0}, lmotor_vel{0}, vel_max{_vel_max},
  R_EN{_R_EN}, R_DIR{_R_DIR}, sentido_der{0}, set_pwm_rmotor{_set_pwm_rmotor},
  L_EN{_L_EN}, L_DIR{_L_DIR}, sentido_izq{0}, set_pwm_lmotor{_set_pwm_lmotor}{};
};

struct Param_mecanicos
{
  /**
   * @brief Estructura utilizada para definir las
   * características mecánicas del robot (diferencial),
   * de los motores y encoders.
  */

  const float  acel;         // controladoras -> 3000 rpm
  const float  decel;        
  const float  reductora;    // reducción de vel de 26
  const float  vel_eje_max;  // velocidad máxima del eje según datasheet
  const float  vel_max;      // velociad máxima de la rueda 
  const float  pulsos_rev;   // pulsos por rev de los encoders
  const float  L_eje;        // longitud del eje del robot -> en mm por favor
  const float  diam_rueda;   // diametro de la rueda -> en mm por favor
  const float  vel_giro;
  const float  vel_freno;

  // Constructor
  Param_mecanicos(float _acel, float _decel,
                  float _reductora,
                  float _vel_eje_max, float _vel_max,
                  float _vel_giro, float _vel_freno,
                  float _pulsos_rev,
                  float _L_eje, float _diam_rueda):
                  acel{_acel}, decel{_decel},
                  reductora{_reductora},
                  vel_eje_max{_vel_eje_max}, vel_max{_vel_max},
                  vel_giro{_vel_giro}, vel_freno{_vel_freno},
                  pulsos_rev{_pulsos_rev},
                  L_eje{_L_eje}, diam_rueda{_diam_rueda}{};
};

struct Odom
{
  // Ruedas derecha
  long cuentas_derecha;
  long cuentas_derecha_total;
  long cuentas_derecha_total_prev;
  // Ruedas izquierda
  long cuentas_izquierda;
  long cuentas_izquierda_total;
  long cuentas_izquierda_total_prev;
  // Flags para saber si estamos parados
  bool parado_absoluto;
  bool parado;
  // Pose del robot
  Pose pose_actual;
  // Métodos para toma de medidas
  void act_odom(Param_mecanicos mecanica, bool inverse);
  void check_mov();
  void reset_odom();
  // Constructor
  Odom():cuentas_derecha{0},    cuentas_derecha_total{0},
         cuentas_izquierda{0},  cuentas_izquierda_total{0},
         pose_actual{0,0,0}{};
};

struct PerfilVelocidad
{
  /**
   * @brief Estructura auxiliar para el cálculo de perfil
   * de velocidad trapezoidal.
   * 
  */
  // Parámetros mecánicos
  Param_mecanicos mecanica; 
  // Distancias -> tanto para rectas como giros
  float distancia_total_rad;    // distancia total a recorrer en rad
  float distancia_frenada;      // distancia a recorrer durante frenada
  float distancia_init_frenada; // distancia en que empiezo a frenar -> total - frenada
  // Ángulo
  float distancia;              // distancia a recorrer en rectas
  float alfa;                   // grados a girar
  // Velocidades
  float vel_final;            // velocidad de crucero
  float vel_final_rads;       // velocidad de crucero en rad/s
  // OJO -> Cuando las distancias son cortas se hará un triángulo
  bool  perfil_triangular;
  // Ajuste de distancia para la frenada
  const float ajuste_distancia_recto;
  const float ajuste_distancia_giro;
  // Constructor
  PerfilVelocidad(float _ajuste_distancia_recto, float _ajuste_distancia_giro, Param_mecanicos _mecanica)
                    :mecanica{_mecanica}, distancia_total_rad{0}, distancia_frenada{0}, distancia_init_frenada{0},
                    distancia{0}, alfa{0},
                    vel_final{0}, vel_final_rads{0}, 
                    perfil_triangular{0},
                    ajuste_distancia_recto{_ajuste_distancia_recto},
                    ajuste_distancia_giro{_ajuste_distancia_giro}{};

  // Métodos para ejecución del perfil
  void calculo_frenada(float velocidad);
  void calculo_recta(float distancia_ref, float vel_ref);
  void calculo_giro(float grados_giro, float velocidad_ref);
};

// @TODO: Falta meter el motion controller
struct motion_controller
{
  // Estructuras utilizadas para toma de medidas y generación de consignas
  Param_mecanicos param_mecanicos;
  Odom            odom;
  Motores         motores; 
  PerfilVelocidad cal_trapecio;
  // Referencia
  float ref_distancia;
  float ref_ang;
  // Acción de control
  bool recta_en_curso;
  bool giro_en_curso;
  bool parado;
  void prev_move_calculus(bool movimiento, float vel_ref);
  void prev_move_calculus(bool movimiento);
  void move_control();
  // Constructor
  motion_controller(Param_mecanicos _param_mecanicos,
                    Odom _odom,
                    Motores _motores,
                    PerfilVelocidad _cal_trapecio):
                    param_mecanicos{_param_mecanicos},
                    odom{_odom},
                    motores{_motores},
                    cal_trapecio{_cal_trapecio},
                    ref_distancia{0}, ref_ang{0},
                    recta_en_curso{0},
                    giro_en_curso{0},
                    parado{1}{odom.reset_odom();};
};



