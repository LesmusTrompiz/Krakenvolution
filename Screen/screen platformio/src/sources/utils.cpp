#include "utils.hpp"

#include <avr/interrupt.h>
#include <avr/io.h>

constexpr uint16_t DELTA_TIME = 20;

void (*tick)();

void utils::setup(void(*_tick)()) {
  // Configuramos el Timer 1 a 50Hz
  tick = _tick;

  // Reseteamos los registros de configuración
  TCCR1A = 0;
  TCCR1B = 0;

  // Estos registros se usan para determinar
  // el modo en que usaremos el timer.
  // En nuestro caso CTC.
  TCCR1B |= (1 << WGM12);   // Modo CTC
  TCCR1B |= (1 << CS11);    // Prescaleer a 8 -> 2MHz

  // Reseteamos las cuentas del Timer.
  TCNT1 = 0;

  // Valor de comparación -> Ciclo de trabajo 
  // expresado en cuentas.
  // OCR1A = (T*(16^6/8))-1;
  OCR1A = 39999;

  // Habilitamos la interrupción del Timer 1
  TIMSK1 |= (1 << OCIE1A);
}

void utils::update_timer() {
  Timer::m_current_time += DELTA_TIME;
}

ISR(TIMER1_COMPA_vect) {
  utils::update_timer();
  tick();
}

// Timers
uint64_t utils::Timer::m_current_time = 0;

utils::Timer::Timer(uint16_t interval)
  : m_last_activation(m_current_time), m_interval(interval) {

}

bool utils::Timer::triggered() {
  bool was_triggered = m_current_time > (m_last_activation + m_interval);
  if (was_triggered) {
    m_last_activation = m_current_time;
  }
  return was_triggered;
}

void utils::Timer::set_interval(uint16_t interval) {
  m_interval = interval;
}
