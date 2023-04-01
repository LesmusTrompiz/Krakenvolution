#include "RobotServos.hpp"

RobotServo::RobotServo(const uint8_t _ID_AnalogRead, const uint8_t pwm_id, Adafruit_PWMServoDriver _ServoHandler)
: ID_AnalogRead(_ID_AnalogRead), PWM_ID(pwm_id), ServoHandler(_ServoHandler)
{
  // Lectura analógica
  pinMode(ID_AnalogRead, INPUT);
  // Inicializar el ServoHandler por si no hubiera sido inicializado previamente
  ServoHandler.begin();
  ServoHandler.setPWMFreq(50);
}

RobotServo::RobotServo(const uint8_t PWM_ID, Adafruit_PWMServoDriver _ServoHandler)
: ID_AnalogRead(-1), PWM_ID(PWM_ID), ServoHandler(_ServoHandler)
{
  // Sin lectura analógica.
  // Inicializar el ServoHandler por si no hubiera sido inicializado previamente
  ServoHandler.begin();
  ServoHandler.setPWMFreq(50);
}

float RobotServo::read_angle()
{
  /**
   * @brief Función para lectura del ángulo actual 
  */
	if(ID_AnalogRead != -1)
	{
		int n_reads = 10;
		// 1. Hacemos n_reads lecturas analógicas.
		int read_value[n_reads];
		for(int i=0; i<n_reads; i++){read_value[i] = analogRead(ID_AnalogRead);}
		// 2. Media de las medidas tomadas.
		float final_r = 0;
		for(int i=0; i<n_reads; i++){final_r += read_value[i];}
		final_r = final_r/n_reads;
		// 3. Transformamos la lectura a ángulo -> Transformación dada por fabricante de los servos.
		return final_r*m_read + n_read;
	}
	else
	{
		return -1;
	}
}

void RobotServo::set_angle(float angle)
{
  // 1. Ticks nominal
  float nominal_tick_value = m*angle + n;
  // 2. Mandar la señal y esperar a que se estabilice la posiciónd el servo
  ServoHandler.setPWM(PWM_ID, 0, (int)(nominal_tick_value));
}

void RobotServo::calibration(float angle_init, float angle_final)
{
	if(ID_AnalogRead != -1)
	{
		float tick_init   = m*angle_init + n;
		float tick_final  = m*angle_final + n;

		// 1. Valor inicial de la recta.
		set_angle(angle_init);
		delay(1000);
		float angle_init_real = read_angle();
		// 2. Valor final de la recta.
		set_angle(angle_final);
		delay(1000);
		float angle_final_real = read_angle();
		// 3. Definición de la recta.
		m = (tick_final-tick_init)/(angle_final_real-angle_init_real);
		n = ((tick_final-m*angle_final_real)+(tick_init-m*angle_init_real))/2;  
	}
	else
		return;
}
