#include "io.hpp"
#include <Arduino.h>

struct Button {
  bool prev_state = false;
  uint8_t pin;
  bool triggered = false;

  Button(uint8_t pin) : pin(pin) {}
};

Button PRIMARY_MENU_BUTTONS[] = {43, 41, 39, 37, 35};
Button CONTEXT_MENU_BUTTONS[] = {47, 49, 51, 53};
constexpr size_t NUMBER_PRIMARY_BUTTONS = sizeof(PRIMARY_MENU_BUTTONS) / sizeof(Button);
constexpr size_t NUMBER_CONTEXT_BUTTONS = sizeof(CONTEXT_MENU_BUTTONS) / sizeof(Button);

void io::setup()
{

  // Setup buttons
  for (auto& button : PRIMARY_MENU_BUTTONS)
    pinMode(button.pin, INPUT_PULLUP);

  for (auto& button : CONTEXT_MENU_BUTTONS)
    pinMode(button.pin, INPUT_PULLUP);
}

void isr_check_button(Button* buttons, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Button& button = buttons[i];
    bool state = !digitalRead(button.pin);
    if ((state != button.prev_state) && state) {
      button.triggered = true;
      button.prev_state = state;
      break; // Apply priority 
    }
    button.prev_state = state;
  }
}

void io::tick() {
  // Check buttons
  isr_check_button(CONTEXT_MENU_BUTTONS, NUMBER_CONTEXT_BUTTONS);
  isr_check_button(PRIMARY_MENU_BUTTONS, NUMBER_PRIMARY_BUTTONS);
}

// Events
io::ButtonEvent::operator bool() {
  return triggered;
}

uint8_t io::ButtonEvent::unwrap() {
  return index;
}

io::ButtonEvent get_pressed_index(Button* buttons, size_t len) {
  bool found = false;
  uint8_t index = -1;
  for (size_t i = 0; i < len; i++) {
    Button& button = buttons[i];
    if (button.triggered) {
      button.triggered = false;
      index = i;
      found = true;
      break;
    }
  }

  return io::ButtonEvent(found, index);
}

io::ButtonEvent io::pressed_primary_index() {
  return get_pressed_index(PRIMARY_MENU_BUTTONS, NUMBER_PRIMARY_BUTTONS);
}

io::ButtonEvent io::pressed_context_index() {
  return get_pressed_index(CONTEXT_MENU_BUTTONS, NUMBER_CONTEXT_BUTTONS);
}
