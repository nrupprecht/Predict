#include "VField.hpp"
#include "Field.hpp"

namespace Predictive {
  VField::VField() : FieldBase() {};

  VField::VField(const Bounds &b) : FieldBase(b) {};

  void VField::gradient(const Field& field) {
    // Sets this field to be the gradient field of the supplied field
    for (int y=0; y<ny; ++y) 
      for (int x=0; x<nx; ++x) {
	vec2 p = getPosition(x,y);
	RealType dx = field.DX(p);
	RealType dy = field.DY(p);
	at(x,y) = vec2(dx, dy);
      }
  }
}
