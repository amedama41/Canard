#ifndef CANARD_INDIRECTED_LESS_HPP
#define CANARD_INDIRECTED_LESS_HPP

#include <functional>

namespace Canard {
  template <class T>
  struct indirected_less
    : public std::binary_function<const T*, const T*, bool>
  {
    bool operator()(const T *lhs, const T *rhs) const
    {
      return std::less<T>()(*lhs, *rhs);
    }
  };
}

#endif  // CANARD_INDIRECTED_LESS_HPP

