#include "serial_bridge/simulate_orders.hpp"
#include <sstream>      // std::stringstream
#include <iostream>


// Dicionario que asinga a cada función de simulación un string con el nombre del RMI.
const std::unordered_map<std::string, std::function<bool(Pose2d &, int16_t &)>> simulate = {
  {"spin" , spin_thread},
  {"advance" , advance_thread}
};


bool advance_thread(Pose2d &p, int16_t &distance){
  /*
    This function simulates the advance of 
    a robot, advancing every call a determined
    distance, this function may be call various
    times to advance the distance goal.

    This function modifies the distance
    value and the pose of the robot, returning 
    True when the distance is advanced.
  */
  float d = 0;

  // Si la distancia que hay que avanzar es positiva
  if(distance > 0){
    
    // Disminuyo la distancia que hay que avanzar
    distance -= advance_tick;

    // Almaceno la distancia que hay que avanzar
    // en metros.
    d = (float)advance_tick/1000;

    // Si me he pasado avanzando reduzco el avance
    // para no pasarme
    if(distance <= 0){
      d += (float)(distance)/1000;
      distance = 0;
    } 
   }  

  else{
    // Disminuyo la distancia que hay que avanzar, 
    // ojo en valor absoluto
    distance += advance_tick;

    // Almaceno la distancia que hay que avanzar
    // en metros.
    d = -(float)advance_tick/1000;

    // Si me he pasado avanzando reduzco el avance
    // para no pasarme
    if(distance >= 0){
      d -= (float)(distance)/1000;
      distance = 0;
    } 
  }
  
  // Calculo la nueva posición del robot
  p.x += d * cos(DEG2RAD(p.a));
  p.y += d * sin(DEG2RAD(p.a));

  // La acción ha acabado cuando la distancia
  // que hay que avanzar es igual a cero.
  return (distance==0);
}

bool spin_thread(Pose2d &p, int16_t &spin){
  /*
    This function simulates the spin of 
    a robot, spining every call a determined
    angle, this function may be call various
    times to spin the spin goal.

    This function modifies the spin
    value and the pose of the robot, returning 
    True when the spin goal is reduced to zero.
  */

  // Si el giro que hay que hacer es positivo
  if(spin > 0){
    // Disminuyo el giro que tengo como objetivo
    spin -= spin_tick;

    // Aumento el ángulo del robot    
    p.a += spin_tick;

    // Si me he pasado retrocedo el ángulo que
    // hasta llegar ala posición deseada.
    if(spin <= 0){
      p.a  += (spin);
      spin = 0;
    } 

  }  
  else if(spin < 0){
    // Disminuyo el giro que tengo como objetivo
    // Ojo en valor absoluto
    spin += spin_tick;

    // Disminuyo el angulo del robot
    p.a -= spin_tick;

    // Si me he pasado avanzo el ángulo que
    // hasta llegar ala posición deseada.
    if(spin >= 0){
      p.a += spin;
      spin = 0;
    } 
  }

  // Normalizo el ángulo entre 0 y 360:
  if (p.a >  180) p.a -= 360;
  if (p.a < -180) p.a += 360;

  // La acción ha acabado cuando el giro
  // que hay que girar es igual a cero.
  return (spin == 0);
}


