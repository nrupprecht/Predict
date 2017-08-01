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

    // Set this field to be the laplacian of another field
    void laplacian(const Field&);
  };

}

#endif // __FIELD_HPP__
