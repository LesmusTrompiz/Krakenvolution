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
 /**
 * @brief Función para configurar PWM de los canales 5 y 6.
 * Corresponde con los pines D35 y 37. En este caso nos aseguramos 
 * de configurar todos los registros, incluidos los "opcionales".
 * 
 */

	// Alimentamos el módulo PWM
	REG_PMC_PCER1     	|= 	PMC_PCER1_PID36;
	// Deshabilitamos los GPIO's
	PIOC -> PIO_PDR		=	PIO_PDR_P19 | PIO_PDR_P18;
	// Habilitamos periféricos tipo B
	PIOC -> PIO_ABSR	|= 	PIO_ABSR_P19 | PIO_ABSR_P18;
	// Configuramos CLK_A
	REG_PWM_CLK   = PWM_CLK_PREA(0) | PWM_CLK_DIVA(10);	
	/* Nos interesa un CPRD alto, ya que determina la precisión del ciclo de trabajo.
	*  
	*   1. Preescalamos -> MCK/256
	*   2. Calculamos el valor deseado de CPRD -> fpwm = MCK/(CPRD*X) -> Donde X es 256 (paso 1).
	*   3. Iniciamos todos los ciclos de trabajo a 0.
	*   4. Habilitamos acceso a las salidas al módulo PWM.
	*/
	for(int i=5; i<7; i++)
	{
		REG_PWM_WPSR;
		PWM -> PWM_CH_NUM[i].PWM_CMR    = PWM_CMR_CPRE_CLKA;
		PWM -> PWM_CH_NUM[i].PWM_CPRD   = MAX_DC;
		PWM -> PWM_CH_NUM[i].PWM_CDTY   = 0.9*MAX_DC; 
	}
	// Paso 4
	REG_PWM_ENA   = PWM_ENA_CHID5 | PWM_ENA_CHID6; 
	// Deshabilitar interrupciones -> No estrictamente necesario
	PWM->PWM_IDR2 = PWM_IDR2_CMPM5 | PWM_IDR2_CMPM6;
	NVIC_DisableIRQ(PWM_IRQn);

}

void config_timer( Tc *tc, 
                    uint32_t channel, 
                    IRQn_Type irq, 
                    uint32_t frecuencia)
{
 /**
 * @brief Configuración genérica para cualquiera de los timers 
 * disponibles para la placa Arduino Due. Esta configuración es
 * estrictamente para modo TC. 
 * 
 * Ejemplo de uso:
 *    >>> config_timer(TC0, 2, TC2_IRQn, 50);
 */

  // Habilitamos escritura en los registros pertinentes 
  pmc_set_writeprotect(false);
  // Habilitamos canal
  pmc_enable_periph_clk((uint32_t)irq);
  // Configuración automática del modo TC
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t RC = (84000000/128)/frecuencia;
  TC_SetRA(tc, channel, RC/2); 
  TC_SetRC(tc, channel, RC);
  // Iniciamos el TCO canal 2
  TC_Start(tc, channel);
  // Habilitamos interrupciones
  tc -> TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc -> TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

/* Modificación de ciclo de trabajo PWMs */
void PWM5_SetDuty(float ciclo)
{
 /**
  * @brief Se define la variable duty, que sirve para calcular de forma
	* sencilla el valor del MR2. El mínimo valor del ciclo de trabajo es
	* aproximadamente un 10%.
  * Canal PWM 5
 */
	float duty;																										
	duty=(float)fabs((double)ciclo) * 0.8	+	0.099;								
	PWM -> PWM_CH_NUM[5].PWM_CDTY =  MAX_DC*(1-duty);
	return;
}
void PWM6_SetDuty (float ciclo)
{
 /**
  * @brief Se define la variable duty, que sirve para calcular de forma
	* sencilla el valor del MR2. El mínimo valor del ciclo de trabajo es
	* aproximadamente un 10%.
  * Canal PWM 6
 */
	float duty;																										
	duty=(float)fabs((double)ciclo) * 0.8	+	0.099;								
	PWM -> PWM_CH_NUM[6].PWM_CDTY =  MAX_DC*(1-duty);
	return;
}


