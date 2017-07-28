#include "Field.hpp"

namespace Predictive {
  Field::Field() : FieldBase() {};

  Field::Field(Bounds b) : FieldBase(b) {};

  void Field::laplacian(Field& field) {
    // Set this field to be the laplacian of [field]
    for (int x=0; x<nx; ++x)
      for (int y=0; y<ny; ++y) {
	vec2 p = getPosition(x,y);
	RealType d2x = DX2(x,y);
	RealType d2y = DY2(x,y);
	at(x,y) = d2x+d2y;
      }
  }
}
