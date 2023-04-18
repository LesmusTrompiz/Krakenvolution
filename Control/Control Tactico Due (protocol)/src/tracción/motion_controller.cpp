/**
 * Autor: Navil Abeselam Abdel-lah y Javier Quintanar
 * 
 * @brief Biblioteca destinada a realizar las tareas 
 * de control de tracción del robot.
*/

// #define debug_controller

#include <motion_controller.hpp>

functor<void()> on_finished_callback;
void on_finished(functor<void ()> callback) {
    on_finished_callback = callback;
}

/* Odom */
void Odom::act_odom(Param_mecanicos mecanica, bool inverse)
{
  /**
   * @brief Medida de variación de cuentas en los encoders. 
   * Se transforma de cuentas a coordenadas cartesianas.
   * 
  */
  // Solo medimos un encoder
  cuentas_derecha = cuentas_izquierda;
  if(inverse) cuentas_derecha = -cuentas_izquierda;
  // Para pasar de pulsos a revoluciones usamos una variable estática
  float pulsos2mm = mecanica.diam_rueda*PI/(mecanica.pulsos_rev*mecanica.reductora);
  // Variaciones de las coordenadas cartesianas y avance
  float deltaO = 0, deltaX = 0, deltaY = 0;
  float avance = 0;
  // 1. Avance en el último ciclo (mm)
  avance  = ((cuentas_derecha+cuentas_izquierda)*pulsos2mm)/2;
  // 2. Variación de orientación en el último ciclo (rad)
  deltaO  = ((cuentas_derecha-cuentas_izquierda)*pulsos2mm)/mecanica.L_eje;
  // 3. Limpiamos las variables de cuentas y acumulamos las totales
  // cuentas_derecha_total     += cuentas_derecha;
  // cuentas_izquierda_total   += cuentas_izquierda;
  cuentas_derecha           = 0;
  cuentas_izquierda         = 0;  
  // Serial.println(deltaO);
  // Serial.println(avance);

  // 4. Variaciones en coordenadas cartesianas
  deltaX  = avance*cos(DEG2RAD(pose_actual.alfa)+deltaO);
  deltaY  = avance*sin(DEG2RAD(pose_actual.alfa)+deltaO);
  // 5. Actualizamos la pose del robot
  pose_actual.x     += deltaX;
  // Serial.print("X: "); Serial.println(pose_actual.x);
  pose_actual.y     += deltaY;
  pose_actual.alfa  += RAD2DEG(deltaO);
  // 6. Limitamos el valor de la orientación
  // while(pose_actual.alfa>=180)
  //   pose_actual.alfa -= 360;
  // while(pose_actual.alfa<=-180)
  //   pose_actual.alfa += 360;
}

void Odom::check_mov()
{
  /**
   * @brief Detección variaciones mínimas que aseguran que el
   * robot se halla parado. Tenemos dos posibilidades:
   *   
   *      · Parado absoluto: el encoder no detecta variaciones de cuentas
   *      · Parado: las variaciones son tan pequeñas que podemos considerar
   *                que está parado.
   * 
  */

 // Parada absoluta
 parado_absoluto = cuentas_derecha <= 1 || cuentas_izquierda <= 1;
 // Parada estimada
 parado = cuentas_derecha <= 15 || cuentas_izquierda <= 15;

}

void Odom::reset_odom()
{
  // Ruedas derecha
  cuentas_derecha = 0;
  cuentas_derecha_total = 0;
  cuentas_derecha_total_prev = 0;
  // Ruedas izquierda
  cuentas_izquierda = 0;
  cuentas_izquierda_total = 0;
  cuentas_izquierda_total_prev = 0;  
  // Flags para saber si estamos parados
  parado_absoluto = true;
  parado = true;
  // Pose del robot
  pose_actual.x     = 0;
  pose_actual.y     = 0;
  pose_actual.alfa  = 0;
}

/* Motores */
void Motores::set_vel_rmotor()
{
  if(rmotor_vel>=vel_max)
    set_pwm_rmotor(1);
  else
    set_pwm_rmotor(rmotor_vel/vel_max);
} 

void Motores::set_vel_lmotor()
{
  if(lmotor_vel>=vel_max)
    set_pwm_lmotor(1);
  else
    set_pwm_lmotor(lmotor_vel/vel_max);
}

void Motores::apagar_motores()
{
	digitalWriteDirect(L_EN, LOW);
	digitalWriteDirect(R_EN, LOW);  
}

void Motores::encender_motores( )
{
  rmotor_vel = 0; set_vel_rmotor();
  lmotor_vel = 0; set_vel_lmotor();
	digitalWriteDirect(L_EN, HIGH);
	digitalWriteDirect(R_EN, HIGH);
}

/* Perfil de velocidades */
void PerfilVelocidad::calculo_frenada(float velocidad)
{
  /**
   * @brief Método para calcular la distancia de frenada para una distancia y velocidad
   * concretos. Además se comprueba la posibilidad de un perfil triángular en caso de distancias 
   * muy cortas. 
   * 
  */
  distancia_frenada = pow(velocidad,2)/(2*PI*mecanica.decel/30*(1/mecanica.reductora));
  perfil_triangular = distancia_frenada > 0.5*distancia_total_rad;
}

void PerfilVelocidad::calculo_recta(float distancia_ref, float velocidad_ref)
{
  /**
   * @brief Método para calcular los parámetros del perfil en movimientos rectos 
   * 
  */

  // Calculo de distancia y velocidades pasado a las ruedas
  distancia_total_rad = fabs(distancia_ref/(mecanica.diam_rueda/2));
  // Obtenenemos distancia de frenado
  calculo_frenada(velocidad_ref);
  // Distancia a la que comienza el frenado
  distancia_init_frenada    = distancia_total_rad - distancia_frenada;
	// Si la distancia es muy corta computamos un perfil triangular
  if(perfil_triangular)
  {
    // En este caso la mitad es acelerando y la mitad frenando
    distancia_frenada       = 0.5*distancia_total_rad;
    distancia_init_frenada  = distancia_frenada;
    vel_final_rads          = sqrt(((2*mecanica.decel/30*(1/mecanica.reductora))*distancia_frenada));
  }
}

void PerfilVelocidad::calculo_giro(float grados_giro, float velocidad_giro)
{
  /**
   * @brief Método para calcular los parámetros del perfil en giros 
   * 
  */

  // Calculo de distancia y velocidades pasado a las ruedas
  distancia_total_rad = DEG2RAD(grados_giro)*mecanica.L_eje/(mecanica.diam_rueda);
  vel_final_rads      = velocidad_giro;
  // Obtenenemos distancia de frenado
  calculo_frenada(velocidad_giro);
	// Distancia a la que comienza el frenado
  distancia_init_frenada    = distancia_total_rad - distancia_frenada;// Si la distancia es muy corta computamos un perfil triangular
	// Si la distancia es muy corta computamos un perfil triangular
  if(perfil_triangular)
  {
    // En este caso la mitad es acelerando y la mitad frenando
    distancia_frenada       = 0.5*distancia_total_rad;
    distancia_init_frenada  = distancia_frenada;
    vel_final_rads          = sqrt(((2*mecanica.decel/30*(1/mecanica.reductora))*distancia_frenada));
  }
}

/* Motion controller loop */
void motion_controller::prev_move_calculus(bool movimiento)
{

  if(movimiento)
  {
    if(ref_distancia > 0)
    {
      digitalWriteDirect(I_DIR, LOW);
      digitalWriteDirect(D_DIR, HIGH);
    }
    else
    {
      digitalWriteDirect(I_DIR, HIGH);
      digitalWriteDirect(D_DIR, LOW);
      ref_distancia = -ref_distancia;
    }
    cal_trapecio.calculo_recta(ref_distancia, param_mecanicos.vel_max*0.6);
    motores.encender_motores();
    recta_en_curso = 1;
    giro_en_curso = 0; 
    odom.parado = 0;
    odom.parado_absoluto = 0;
    odom.reset_odom();
    #ifdef debug_controller      
      Serial.println("Dis_rad: ");Serial.println(cal_trapecio.distancia_total_rad);
      Serial.println("Frenada: ");Serial.println(cal_trapecio.distancia_frenada);
      Serial.println("Comienzo frenada: ");Serial.println(cal_trapecio.distancia_init_frenada);
      Serial.print("Odom: ");Serial.println(odom.pose_actual.x);
    #endif
  }
  else 
  {
    if(ref_ang > 0)
    {
      digitalWriteDirect(I_DIR, HIGH);
      digitalWriteDirect(D_DIR, HIGH);
    }
    else
    {
      digitalWriteDirect(I_DIR, LOW);
      digitalWriteDirect(D_DIR, LOW);
      ref_ang = -ref_ang;
    }
    cal_trapecio.calculo_giro(ref_ang, param_mecanicos.vel_giro);
    motores.encender_motores();
    giro_en_curso = 1;
    recta_en_curso = 0;
    odom.parado = 0;
    odom.parado_absoluto = 0;
    odom.reset_odom();
    #ifdef debug_controller      
      Serial.println("Dis_rad: ");Serial.println(cal_trapecio.distancia_total_rad);
      Serial.println("Frenada: ");Serial.println(cal_trapecio.distancia_frenada);
      Serial.println("Comienzo frenada: ");Serial.println(cal_trapecio.distancia_init_frenada);
      Serial.print("Odom: ");Serial.println(odom.pose_actual.x);
    #endif  
    }
}

void motion_controller::move_control()
{
  bool sentido_inverso = (giro_en_curso && ref_ang > 0) || (recta_en_curso && ref_distancia < 0);
  /* Toma de medidas en movimiento */
	if(giro_en_curso || recta_en_curso) odom.act_odom(this->param_mecanicos, sentido_inverso);
  else odom.reset_odom();
  /* Check parado */
  odom.check_mov();
  /* Control */ 
  if(recta_en_curso)
  {
    if(fabs(odom.pose_actual.x) < fabs(cal_trapecio.distancia_init_frenada - fabs(cal_trapecio.ajuste_distancia_recto))*(param_mecanicos.diam_rueda/2))
    {
      // Velocidad de crucero
      // Actualizamos la velocidad
      // Serial.println("Vc");
      motores.rmotor_vel = param_mecanicos.vel_max*0.6;
      motores.lmotor_vel = param_mecanicos.vel_max*0.6;
      motores.set_vel_rmotor();
      motores.set_vel_lmotor();
    }
    else if(fabs(odom.pose_actual.x) < fabs(cal_trapecio.distancia_total_rad-fabs(ajuste_error_tactico))*(param_mecanicos.diam_rueda/2))
    {
      // Serial.println("F");
      // Velocidad de freno
      motores.rmotor_vel = param_mecanicos.vel_freno;
      motores.lmotor_vel = param_mecanicos.vel_freno;
      motores.set_vel_rmotor();
      motores.set_vel_lmotor();
    }
    else
    {    
      // Parar los motores
      // Serial.println("P");
      motores.rmotor_vel = 0;
      motores.lmotor_vel = 0;
      motores.set_vel_rmotor();
      motores.set_vel_lmotor();      
      }
  }
  else if(giro_en_curso)
  {
    if(fabs(DEG2RAD(odom.pose_actual.alfa)*param_mecanicos.L_eje/(param_mecanicos.diam_rueda)) < fabs(cal_trapecio.distancia_init_frenada - fabs(cal_trapecio.ajuste_distancia_giro)))
    {
      // Velocidad de crucero
      // Actualizamos la velocidad
      motores.rmotor_vel = param_mecanicos.vel_giro;
      motores.lmotor_vel = param_mecanicos.vel_giro;
      motores.set_vel_rmotor();
      motores.set_vel_lmotor();
    }
    else if(fabs(DEG2RAD(odom.pose_actual.alfa)*param_mecanicos.L_eje/(param_mecanicos.diam_rueda)) < fabs(cal_trapecio.distancia_total_rad - fabs(ajuste_error_tactico)))
    {
      // Velocidad de freno
      motores.rmotor_vel = param_mecanicos.vel_freno;
      motores.lmotor_vel = param_mecanicos.vel_freno;
      motores.set_vel_rmotor();
      motores.set_vel_lmotor();
    }
    else
    {
      // Parar los motores
      motores.rmotor_vel = 0;
      motores.lmotor_vel = 0;
      motores.set_vel_rmotor();
      motores.set_vel_lmotor();
    }
  }

  if((giro_en_curso || recta_en_curso) && odom.parado_absoluto
      && motores.lmotor_vel == 0 && motores.rmotor_vel == 0)
  {
    // Serial.println("Parada..."); 
    // Serial.print("X: ");Serial.println(odom.pose_actual.x);
    // Serial.print("Y: ");Serial.println(odom.pose_actual.y);
    // Serial.print("O: ");Serial.println(odom.pose_actual.alfa);
    on_finished_callback();
    parado = true;
    odom.parado = true;
    odom.parado_absoluto = true;
    giro_en_curso = false;
    recta_en_curso = false;
    ref_ang = 0;
    ref_distancia = 0;
    odom.reset_odom();
    // Serial.print("X: ");Serial.println(odom.pose_actual.x);
    // Serial.print("Y: ");Serial.println(odom.pose_actual.y);
    // Serial.print("O: ");Serial.println(odom.pose_actual.alfa);   
  }
}
