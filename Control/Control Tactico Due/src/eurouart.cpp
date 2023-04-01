#include "Eurouart.hpp"
#include <Arduino.h>
#include "motion_controller.hpp"

extern motion_controller controlador_tactico;

String inputString = "";
bool stringComplete = false;

void (*reset_arduino)(void) = 0;

// void serialEvent() {
//   int i=0; 
//   while (Serial.available()>0)
//   {
//     char inChar = (char)Serial.read();
//     // Serial.println(inChar);
//     inputString[i] = inChar;
//     Serial.print(inChar);
    
//     if (inChar == '\r') 
//     {
//         Traduccion_Variables();
//         i = 0; 
//         return;       
//     }
//     i++;
//   }
// }

void serialEvent()
{
  while(Serial.available()>0)
  {
    inputString = Serial.readString();

    if(inputString!="")
    {
      Serial.println(inputString);
      Traduccion_Variables();
      return;
    }
  }
}

void Traduccion_Variables()       
{ //Se encarga de leer el mensaje recibido, actualizar las variables y levantar los flags.
  char Instruccion_Codigo = inputString.charAt(0);
  char Signo = inputString.charAt(1);
  float grados_giro = 0;
  float distancia = 0;
  // Serial.println(Instruccion_Codigo);
    switch (Instruccion_Codigo)
    {
    case ('G'):
        grados_giro = (inputString.charAt(2) - '0') * 100 + (inputString.charAt(3) - '0') * 10 + (inputString.charAt(4) - '0');
        if(inputString.charAt(1) == '-')
          grados_giro = -grados_giro;       
        controlador_tactico.ref_ang = grados_giro;
        controlador_tactico.prev_move_calculus(0);
        break;
    case ('D'):
        distancia = (inputString.charAt(2) - '0') * 1000 + (inputString.charAt(3) - '0') * 100 + (inputString.charAt(4) - '0') * 10 + (inputString.charAt(5) - '0');
        Serial.println("Distancia recibida: "); Serial.println(distancia);
        if(inputString.charAt(1) == '-')
          distancia = -distancia;
        controlador_tactico.ref_distancia = distancia;
        controlador_tactico.prev_move_calculus(1);
        break;
		case ('R'):
				reset_arduino();
				break;
    default:
        Serial.println("EI" + inputString);
        Instruccion_Codigo = 1;
        break;
    }
  inputString = "";
}
