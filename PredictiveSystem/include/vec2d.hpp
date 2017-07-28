/*
 * Author: Nathaniel Rupprecht
 * Start Data: May 12, 2017
 *
 */

// July 27, 2017 - NOTE: Brought in from GFlow, some changes for simplicity made including namespace designations.

#ifndef __VEC_2D_HPP__
#define __VEC_2D_HPP__

#include "Utility.hpp"

namespace Predictive {
  
  /*
   * @class vec2
   * Two dimensional vector class
   *
   */
  struct vec2 {
    // Default constructor
    vec2() : x(0), y(0) {};
    
    // Initialized constructor
    vec2(RealType x, RealType y) : x(x), y(y) {};
    
    // The actual vector data
    RealType x, y;
    
    // Operators
    vec2 operator-() const;
    vec2 operator-(const vec2&) const;
    vec2& operator-=(const vec2&);
    vec2 operator+(const vec2&) const;
    vec2& operator+=(const vec2&);
    RealType operator*(const vec2&) const;
    RealType operator^(const vec2&) const;
    friend vec2 operator*(const RealType, const vec2&);
    
    friend std::ostream& operator<<(std::ostream&, const vec2&);
    
    bool operator==(const vec2&) const;
    bool operator!=(const vec2&) const;
  };
  
  const vec2 Zero(0,0);
  
  // Special squaring function for vectors (not a reference so we can use lvalues)
  inline RealType sqr(const vec2 v) { return v*v; }
  
  // Normalization function
  inline void normalize(vec2 &v) {
    RealType mag = sqrt(sqr(v));
    v.x /= mag; v.y /= mag;
  }
  
  // (not a reference so we can use lvalues)
  inline RealType length(const vec2 v) {
    return sqrt(sqr(v));
  }
  
  inline vec2 randV() {
    RealType angle = 2*PI*drand48();
    return vec2(cos(angle), sin(angle));
  }
  
}
#endif // __VEC_2D_HPP__
