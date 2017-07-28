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
  
  /*
   * @class FieldBase
   * The base class for all (physical) field - like classes. Contains access,
   *   derivatives, and some basic arithmetic support
   */
  template<typename T> class FieldBase {
  public:
    FieldBase();
    FieldBase(const Bounds&);
    // Destructor
    ~FieldBase();

    // Initialization
    void initialize();

    // Field setting
    void set(T (*func) (vec2));
    FieldBase& operator=(const FieldBase&);

    // Grid point access
    T& at(int, int);
    T at(int, int) const;

    // Grid point location
    vec2 getPosition(int, int) const;

    // Interpolative field acces
    T get(vec2) const;

    // Derivatives
    T DX(vec2) const;
    T DY(vec2) const;
    T DX(int, int) const;
    T DY(int, int) const;
    T DX2(vec2) const;
    T DY2(vec2) const;
    T DX2(int, int) const;
    T DY2(int, int) const;
    
    // Arithmetic
    void minusEq(FieldBase<T>&, RealType=1.);

    // Exception classes
    struct UnalignedSpacing {};

  protected:
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
