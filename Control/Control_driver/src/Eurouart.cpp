
#include "Eurouart.h" //Enlazo con la biblioteca
#include <Arduino.h>
#include "pose_controller.hpp"

extern PoseController robot;
char inputString[10];

void serialEvent() {
  int i=0; 
  while (Serial.available()>0)
  {
    char inChar = (char)Serial.read();
    // Serial.println(inChar);
    Serial.println(i);
    inputString[i] = inChar;
    Serial.println(inputString[i]);
    
    if (inChar == '\r') 
    {
        Serial.println("ENTRO");
        // Serial.println(inputString[0]);

        Traduccion_Variables();
        i = 0; 
        return;       
    }
    i++;
  }
}


void Traduccion_Variables()       
{
    int grados_giro = 0;
    int distancia = 0;

    // Serial.println(inputString[0]);

    switch(inputString[0])
    {
    case ('G'):
        grados_giro = (inputString[2] - '0') * 100 + (inputString[3] - '0') * 10 + (inputString[4] - '0');
	    Serial.print("A_ref: "); Serial.println(grados_giro);
        if (inputString[1] == '-')
        {
            grados_giro = -1 * grados_giro;
        }
        
        robot.ref_pose = Pose(0,0,grados_giro);
        break;
    case ('D'):
        distancia = (inputString[2] - '0') * 1000 + (inputString[3] - '0') * 100 + (inputString[4] - '0') * 10 + (inputString[5] - '0');
	    Serial.print("X_ref: "); Serial.println(distancia);
        if (inputString[1] == '-')
        {
            distancia = -1 * distancia;
        }

        robot.ref_pose = Pose(distancia,0,0);
        break;		
    default:
        Serial.println("EI");
    }
}

