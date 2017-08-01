#ifndef __FIELD_GENERATOR_HPP__
#define __FIELD_GENERATOR_HPP__

#include "Field.hpp"

namespace Predictive {

  
  class FieldGenerator {
  public:
    // Field creation
    void createSmoothNoise(Field&, RealType=1.1);
    void createPeak(Field&, RealType=0.5, RealType=0.5, RealType=0.5, RealType=1.);
    void createTwoPeaks(Field&);
    void createUniform(Field&, RealType);
    void createSine(Field&, RealType=1., int=2);
  private:
    inline void addFrequency(int, RealType, Field&);
    inline RealType cosineInterpolate(RealType, RealType, RealType);
    inline RealType interpolate(RealType, RealType, RealType, RealType, RealType, RealType);
  };

}
#endif // __FIELD_GENERATOR_HPP__
