#pragma once
#include <unordered_map>
#include <functional>
#include "serial_bridge/pose2d.hpp"
extern "C"{
    #include <math.h>
}


// Constantes de simulación:
constexpr float TICK_TIME   {0.1};
constexpr float SPEED       {2000};
constexpr float ANGLE_SPEED {360};
constexpr float advance_tick {SPEED * TICK_TIME};
constexpr float spin_tick {ANGLE_SPEED * TICK_TIME};

// Estas constantes se han dejado de utilizar para
// facilitar la lectura.
constexpr float MIN_DIST = 100;
constexpr float MIN_SPIN = 5;
constexpr int   DIST_MAX_NOISE = 50;
constexpr int   SPIN_MAX_NOISE =  5;

// Dicionario que asinga a cada función de simulación un string con el nombre del RMI.
extern const std::unordered_map<std::string, std::function<bool(Pose2d &, int16_t &)>> simulate;

// Funciones de smulación
bool advance_thread(Pose2d &p, int16_t &distance);
bool spin_thread(Pose2d &p, int16_t &distance);

// Función que convierte de grados a radianes.
inline float DEG2RAD(const float deg)
{
  return deg* M_PI / 180;
}





