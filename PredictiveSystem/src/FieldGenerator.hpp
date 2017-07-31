#ifndef __FIELD_GENERATOR_HPP__
#define __FIELD_GENERATOR_HPP__

#include "Field.hpp"

namespace Predictive {

  
  class FieldGenerator {
  public:
    // Field creation
    void createSmoothNoise(Field&, RealType=1.1);
    void createTwoPeaks(Field&);
    void createUniform(Field&, RealType);
  private:
    inline void addFrequency(int, RealType, Field&);
    inline RealType cosineInterpolate(RealType, RealType, RealType);
    inline RealType interpolate(RealType, RealType, RealType, RealType, RealType, RealType);
  };

}
#endif // __FIELD_GENERATOR_HPP__
