#ifndef __FIELD_HPP__
#define __FIELD_HPP__

#include "FieldBase.hpp"

namespace Predictive {
  
  /*
   * @class Field
   * Scalar field class
   */
  class Field : public FieldBase<RealType> {
  public:
    // Constructors
    Field();
    Field(const Bounds&);
    Field(const Bounds&, int);

    // Calculus
    vec2 grad(vec2);
    vec2 grad(int, int);

    // Set this field to be the laplacian of another field
    void laplacian(const Field&);

    friend void clamp(Field& field) {
      for (int y=0; y<field.ny; ++y)
	for (int x=0; x<field.nx; ++x) {
	  if (field.at(x,y)<0) field.at(x,y) = 0.;
	}
    }
  };

}

#endif // __FIELD_HPP__
