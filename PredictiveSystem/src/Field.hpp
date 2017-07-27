#ifndef __FIELD_HPP__
#define __FIELD_HPP__

#include "FieldBase.hpp"

namespace Predictive {
  
  class Field : public FieldBase<RealType> {
  public:
    Field();
    ~Field(); 
  };

}

#endif // __FIELD_HPP__
