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
    Field(Bounds);

    // Set this field to be the laplacian of another field
    void laplacian(Field&);
  };

}

#endif // __FIELD_HPP__
