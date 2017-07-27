/*
 * Author: Nathaniel Rupprecht
 * Start Data: July 27, 2017
 *
 */

#ifndef __FIELD_BASE_HPP__
#define __FIELD_BASE_HPP__

#include "../include/Utility.hpp"
#include "../include/Bounds.hpp"
#include "../include/aligned_array.hpp"

namespace Predictive {
  
  template<typename T> class FieldBase {
  public:
    FieldBase();
    FieldBase(const Bounds);
    ~FieldBase();

    // Initialization
    void initialize();

    // Grid point access
    T& at(int, int);

    // Interpolative field acces
    T get(vec2);

    // Derivatives
    T DX(vec2);
    T DY(vec2);

  private:
    RealType dx, dy;   // Grid spacing
    RealType idx, idy; // Inverse of grid spacing
    int nx, ny;        // Number of points in the x, y directions
    Bounds bounds;     // The field (physical) bounds
    bool wrap;         // Periodic b/c or not
    aligned_array<T> data; // Field data
  };

#include "FieldBase.cpp"
  
}
#endif // __FIELD_BASE_HPP
