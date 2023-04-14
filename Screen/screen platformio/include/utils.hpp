#pragma once

#include <stdint.h>

namespace utils {
  class Timer {
    public:
      Timer(uint16_t interval);

      bool triggered();
      void set_interval(uint16_t interval);

    private:
      uint64_t m_last_activation;
      uint64_t m_interval;

      static uint64_t m_current_time;

      friend void update_timer();
  };

  void update_timer();
  void setup(void(*tick)());
}
