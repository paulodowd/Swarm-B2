
#ifndef FIXED16_H
#define FIXED16_H

#include <Arduino.h>
#include <stdint.h>

class Fixed16 {
  public:
    // Constructor: specify number of integer bits (excluding sign)
    explicit Fixed16(uint8_t integerBits)
      : _intBits(integerBits),
        _fracBits(15 - integerBits),
        _scale(1 << _fracBits),
        _max((int16_t)((1 << (integerBits + _fracBits)) - 1)),
        _min((int16_t)(-1 << (integerBits + _fracBits))) {}

    // ----------------------------
    // Conversions
    // ----------------------------

    inline int16_t fromInt(int16_t x) const {
      return saturate((int32_t)x << _fracBits);
    }

    inline int16_t fromFloat(float x) const {
      return saturate((int32_t)(x * _scale));
    }

    inline int16_t toInt(int16_t x) const {
      return x >> _fracBits;
    }

    inline float toFloat(int16_t x) const {
      return (float)x / _scale;
    }

    // ----------------------------
    // Arithmetic
    // ----------------------------

    inline int16_t add(int16_t a, int16_t b) const {
      return saturate((int32_t)a + b);
    }

    inline int16_t sub(int16_t a, int16_t b) const {
      return saturate((int32_t)a - b);
    }

    inline int16_t mul(int16_t a, int16_t b) const {
      int32_t r = (int32_t)a * (int32_t)b;
      return saturate(r >> _fracBits);
    }

    inline int16_t div(int16_t a, int16_t b) const {
      int32_t r = ((int32_t)a << _fracBits);
      return saturate(r / b);
    }

    // ----------------------------
    // Accessors
    // ----------------------------

    inline uint8_t integerBits() const {
      return _intBits;
    }
    inline uint8_t fractionalBits() const {
      return _fracBits;
    }

  private:
    uint8_t  _intBits;
    uint8_t  _fracBits;
    int16_t  _scale;
    int16_t  _max;
    int16_t  _min;

    inline int16_t saturate(int32_t x) const {
      if (x > _max) return _max;
      if (x < _min) return _min;
      return (int16_t)x;
    }
};
#endif
