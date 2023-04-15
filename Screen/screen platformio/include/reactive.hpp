#pragma once

#include "WString.h"
namespace reactive {

namespace ___impl {
  class OrCombinator;
}

template<typename T>
class ReactiveValue {
  public:
  ReactiveValue<T>() = default;
  ReactiveValue<T>(const T& other);
  ReactiveValue<T>& operator=(const T& other);
  operator T() const;

  void trigger();
  bool check() const;

  // ___impl::OrCombinator changed() const;

  private:
  T m_value;
  mutable bool m_changed = true;

  friend class ___impl::OrCombinator;
};

namespace ___impl {
  class OrCombinator {
    public:
      OrCombinator(bool starting_value);
      template<typename T>
      OrCombinator operator||(const ReactiveValue<T>& other) const {
        bool prev_changed = other.m_changed;
        other.m_changed = false;
        return {m_updated || prev_changed};
      }
      operator bool();
    private:
      bool m_updated = false;
  };
}

template class ReactiveValue<String>;
template class ReactiveValue<int>;
template class ReactiveValue<float>;
template class ReactiveValue<bool>;

using String = ReactiveValue<String>;
using Int = ReactiveValue<int>;
using Float = ReactiveValue<float>;
using Bool = ReactiveValue<bool>;

  // Neutral element for monoidal construction
  inline const ___impl::OrCombinator CHECK(false);
};

