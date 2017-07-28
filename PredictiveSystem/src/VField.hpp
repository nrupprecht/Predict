#ifndef __VFIELD_HPP__
#define __VFIELD_HPP__

#include "../include/vec2d.hpp"
#include "FieldBase.hpp"

namespace Predictive {

  // Forward declaration
  class Field;

  /*
   * @class VField
   * Vector field class
   */
  class VField : public FieldBase<vec2> {
  public:
    // Constructors
    VField();
    VField(const Bounds&);
    
    // Sets this field to be the gradient field of the supplied field
    void gradient(const Field&);
  };

}
#endif // __VFIELD_HPP__
