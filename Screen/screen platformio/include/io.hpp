#pragma once

#include <stdint.h>

namespace io {
  void setup();
  void tick();

  class ButtonEvent {
    public:
      ButtonEvent(bool triggered, uint8_t index) :
        triggered(triggered), index(index) {}

    operator bool();
    uint8_t unwrap();

    private:
      const bool triggered;
      const uint8_t index;
  };

  ButtonEvent pressed_context_index();
  ButtonEvent pressed_primary_index();
}
