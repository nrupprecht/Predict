#include "Field.hpp"

namespace Predictive {
  Field::Field() : FieldBase() {};

  Field::Field(const Bounds& b) : FieldBase(b) {};

  Field::Field(const Bounds& b, int n) : FieldBase(b, n) {};

  vec2 Field::grad(vec2 pos) {
    double X = (pos.x-bounds.left)*idx, Y = (pos.y-bounds.bottom)*idy;
    double bx = (int)X, by = (int)Y;
    double x = X-bx, y = Y-by; // in [0,1]
    vec2 tl = grad(bx, by+1), tr = grad(bx+1, by+1);
    vec2 bl = grad(bx, by), br = grad(bx+1, by);
    vec2 p_E = x*(br-bl) + bl; // Bottom
    vec2 p_F= x*(tr-tl) + tl;  // Top
    return y*(p_F-p_E) + p_E;
  }

  vec2 Field::grad(int x, int y) {
    return vec2(DX(x,y), DY(x,y));
  }

  void Field::laplacian(const Field& field) {
    // Set this field to be the laplacian of [field]
    for (int x=0; x<nx; ++x)
      for (int y=0; y<ny; ++y)
	at(x,y) = field.Lap(x,y);
  }
}
