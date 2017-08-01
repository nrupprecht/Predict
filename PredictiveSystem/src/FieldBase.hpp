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

#include <functional>

namespace Predictive {
  
  /*
   * @class FieldBase
   * The base class for all (physical) field - like classes. Contains access,
   *   derivatives, and some basic arithmetic support
   * The data is stored for the point at the center of the grid division,
   *   like | * | * | * | * | * |
   *        0 a 1 b 2 c 3 d 4 e 5
   *   --> a = 0.5, b = 1.5, etc., dx = 1
   */
  template<typename T> class FieldBase {
  public:
    FieldBase();
    FieldBase(const Bounds&);
    FieldBase(const Bounds&, int);
    // Destructor
    ~FieldBase();

    // Initialization
    void initialize();

    // Field setting
    void set(std::function<RealType(vec2)>);
    void set(T);
    FieldBase& operator=(const FieldBase&);

    // Grid point access
    T& at(int, int);
    T at(int, int) const;

    // Interpolative field acces
    T get(vec2) const;
    // Closest point access
    T& get(vec2);

    // Accessors
    int getNX() { return nx; }
    int getNY() { return ny; }
    Bounds getBounds() { return bounds; }

    // Mutators
    void setNX(int);
    void setNY(int);
    void setN(int);
    void setWrap(bool w);

    // Grid point location
    vec2 getPosition(int, int) const;

    // Derivatives
    T DX(vec2) const;
    T DY(vec2) const;
    T DX(int, int) const;
    T DY(int, int) const;
    T DX2(vec2) const;
    T DY2(vec2) const;
    T DX2(int, int) const;
    T DY2(int, int) const;
    T Lap(int, int) const;
    
    // Arithmetic
    void minusEq(FieldBase<T>&, RealType=1., bool=false);
    void plusEq (FieldBase<T>&, RealType=1., bool=false);

    // Exception classes
    struct UnalignedPoints  {};
    struct UnalignedSpacing {};

    friend inline bool printToCSV(string filename, const FieldBase& field) {
      ofstream fout(filename);
      if (fout.fail()) return false;

      for(int y=0; y<field.ny; ++y)
	for(int x=0; x<field.nx; ++x)
	  fout << toCSV(field.getPosition(x,y)) << "," << toCSV(field.at(x,y)) << "\n";
      
      fout.close();
      return true;
    }

    // Find the maximum value in the field
    friend inline T max(const FieldBase& field) {
      T m = T(-1e7);
      for(int y=0; y<field.ny; ++y)
        for(int x=0; x<field.nx; ++x)
	  if (field.at(x,y)>m) m = field.at(x,y);
      return m;
    }

    friend inline vec2 max_pos(const FieldBase& field) {
      T m = T(-1e7);
      int X(-1), Y(-1);
      for(int y=0; y<field.ny; ++y)
        for(int x=0; x<field.nx; ++x)
	  if (field.at(x,y)>m) {
	    m = field.at(x,y);
	    X = x; Y = y;
	  }
      return field.getPosition(X,Y);
    }

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
