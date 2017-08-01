#include "Field.hpp"

namespace Predictive {
  Field::Field() : FieldBase() {};

  Field::Field(const Bounds& b) : FieldBase(b) {};

  Field::Field(const Bounds& b, int n) : FieldBase(b, n) {};

  void Field::laplacian(const Field& field) {
    // Set this field to be the laplacian of [field]
    for (int x=0; x<nx; ++x)
      for (int y=0; y<ny; ++y) {
	/*
	RealType d2x = field.DX2(x,y);
	RealType d2y = field.DY2(x,y);
	at(x,y) = d2x+d2y;
	*/
	at(x,y) = field.Lap(x,y);
      }
  }
}
