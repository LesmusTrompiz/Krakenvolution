/**
 * Autor: Navil Abeselam Abdel-lah y Javier Quintanar
 * 
 * @brief Esta biblioteca contiene las funciones necesarias para
 * la configuración de timers y pwms para la placa Arduino Due.
 * El proyecto está destinado a la competición Eurobot2023, por 
 * lo que la funciones no tienen por qué ser genéricas.
 * 
*/
#include <timer_&_pwm.hpp>

/* Configuraciones básicas */
void config_pwms()
{
	// Configuración PWM(A)
	TCCR0A |=  (2  	<< COM0A0)  								// Clearing on compare, if the value is matched the register value is assing to 0
		   	 |   (3 	<< WGM00);    							// Set the Fast PWM mode on both channels 
	TCCR0B |=  (3   << CS00);    								// Set the prescale 1/64 clock
	// Configuración PWM(B)
	TCCR0A |=  (2  	<< COM0B0)  								// Clearing on compare, if the value is matched the register value is assing to 0
		   	 |   (3 	<< WGM00);    							// Set the Fast PWM mode on both channels 
	TCCR0B |=  (3   << CS00);    								// Set the prescale 1/64 clock
}

void config_control_timer()
{
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

/**
 * @brief La fórmula del registro de comparación es la siguiente
 * 
 * f_{OCnA} = f_{clk_{I/O}}/ (2· N·(1 + OCRnA))
 * 
 * Siendo f_{clk_{I/O}} la frecuencia prescalada, y N el valor del prescaler.
 * Por otro lado el primer término de la ecuación representa la frecuencia deseada.
*/

/**
 * @brief Arduino Mega: Ponemos la frecuencia de f_{OCnA} a 50Hz, y para ello no podemos acceder directamente
 * a OCRnA, sino que se accede PRIMERO a OCRnAH y SEGUNDO a OCRnAL para escritura. En esta placa han aparecido problemas varios
 * con la frecuencia de la interrupción en modo CTC.
*/

//   OCR1AH = 0xFF;
//   OCR1AL = 0xFF;

/**
 * @brief Arduino Uno: Ponemos la frecuencia de f_{OCnA} a 50Hz, en el caso de la Arduino Uno sí podemos realizar operaciones de escritura 
 * en registros de 16bits.
*/  

  // Frecuencia = 50Hz
  OCR1A = 39999; 

  // CTC y preescaler a 8
  TCCR1B = (1 << WGM12) | (1 << CS11);

  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

}

/* Modificación de ciclo de trabajo PWMs */
void PWMA_SetDuty(float ciclo)
{
  float duty;
  duty = (float)fabs((double)ciclo) * 0.8 + 0.097;
	OCR0A = 255*duty;
}
void PWMB_SetDuty (float ciclo)
{
  float duty;
  duty = (float)fabs((double)ciclo) * 0.8 + 0.097;
	OCR0B = 255*duty;
}


