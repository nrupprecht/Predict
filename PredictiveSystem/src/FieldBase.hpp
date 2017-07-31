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
    void set(std::function<RealType(vec2)>);
    void set(T);
    FieldBase& operator=(const FieldBase&);

    // Grid point access
    T& at(int, int);
    T at(int, int) const;

    // Accessors
    int getNX() { return nx; }
    int getNY() { return ny; }

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

    friend inline bool printToCSV(string filename, const FieldBase& field) {
      ofstream fout(filename);
      if (fout.fail()) return false;

      for(int y=0; y<field.ny; ++y)
	for(int x=0; x<field.nx; ++x)
	  fout << toCSV(field.getPosition(x,y)) << "," << toCSV(field.at(x,y)) << "\n";
      
      fout.close();
      return true;
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

  /*
  inline bool printToCSV(string filename, const FieldBase& field) {
    ofstream fout(filename);
    if (four.fail()) return false;
    
    for (int y=0; y<field.ny; ++y)
      for (int x=0; x<field.nx; ++x) 
    
    fout.close();
    return true;
  }
  */
  
}
#endif // __FIELD_BASE_HPP
