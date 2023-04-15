#include "reactive.hpp"

using namespace reactive;

template<typename T>
ReactiveValue<T>::ReactiveValue(const T& other)
  : m_value(other), m_changed(true) {
  
}

template<typename T>
ReactiveValue<T>& ReactiveValue<T>::operator=(const T &other) {
  m_value = other;
  m_changed = true;
  return *this;
}

template<typename T>
ReactiveValue<T>::operator T() const {
  return m_value;
}

template<typename T>
void ReactiveValue<T>::trigger() {
  m_changed = true;
}

// OrCombinator

___impl::OrCombinator::OrCombinator(const bool starting_value)
  : m_updated(starting_value) {}

___impl::OrCombinator::operator bool() {
  return m_updated;
}
