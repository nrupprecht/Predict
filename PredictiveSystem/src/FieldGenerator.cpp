#include "FieldGenerator.hpp"

namespace Predictive {

  // **** TODO: MAKE THIS BETTER ****
  void FieldGenerator::createSmoothNoise(Field& field, RealType mult) {
    int grains=4, maxGrain=16;
    // Reset field
    field.set(0.);
    
    int min = 4; // Minimum grain size
    int G = maxGrain;
    double amp = 1000;
    for (int i=0; i<grains && G>=min; ++i) {
      addFrequency(G, amp, field);
      amp *= mult;
      G *= 0.5;
    }
  }
  
  void FieldGenerator::createTwoPeaks(Field& field) {
    RealType m0 = 50, m1 = 25;
    RealType c0 = sqr(12), c1 = sqr(20);
    auto twoPeaks = [&] (vec2 pos) {
      RealType x = pos.x, y = pos.y;
      return (RealType)(m0*exp(-c0*sqr(x-0.5)-c0*sqr(y-0.5)) + m1*exp(-c1*sqr(x-0.25)-c1*sqr(y-0.5)));
    };
    field.set(twoPeaks);
  }
  
  void FieldGenerator::createUniform(Field& field, RealType value) {
    field.set(value);
  }

  inline void FieldGenerator::addFrequency(int grainSize, RealType amp, Field& field) {
    int dX = field.getNX(), dY = field.getNY();
    int W = dX/grainSize, H = dY/grainSize;
    RealType *D = new RealType[W*H];
    RealType *array = new RealType[dX*dY];

    // Translate where the grid points are
    int transX = static_cast<int>(drand48()*dX);
    int transY = static_cast<int>(drand48()*dY);
    // Set random points, we will smoothly interpolate between these
    for(int x=0; x<W; x++)
      for (int y=0; y<H; y++)
	D[x+y*H] = amp*drand48();
    
    // Interpolate
    for(int x=0; x<dX; x++)
      for (int y=0; y<dY; y++) {
	int X = (x+transX)%dX, Y = (y+transY)%dY;
	if (x%grainSize==0 && y%grainSize==0)
	  array[X+Y*dX] = D[(x/grainSize)+(y/grainSize)*H];
	else { // Wrapped b.c.
	  int aX = x/grainSize, bX = (aX+1)%W;
	  int aY = y/grainSize, bY = (aY+1)%H;
	  RealType TL = D[aX+aY*H], TR = D[bX+aY*H];
	  RealType BL = D[aX+bY*H], BR = D[bX+bY*H];
	  RealType dy = (RealType)y/grainSize - y/grainSize;
	  RealType dx = (RealType)x/grainSize - x/grainSize;
	  array[X+Y*dX] = interpolate(TL, TR, BL, BR, dx, dy);
	}
      }
    for (int x=0; x<dX; x++)
      for (int y=0; y<dY; y++)
	field.at(x,y) += array[y*dX+x];
    delete [] D;
    delete [] array;
  }

  inline RealType FieldGenerator::cosineInterpolate(RealType a, RealType b, RealType x) {
    float ft = x*PI;
    float f = (1 - cos(ft)) * 0.5;
    return a * (1 - f) + b * f;
  }

  inline double FieldGenerator::interpolate(RealType TL, RealType TR, RealType BL, RealType BR, RealType dx, RealType dy) {
    RealType top = cosineInterpolate(TL, TR, dx), bottom = cosineInterpolate(BL, BR, dx);
    return cosineInterpolate(top, bottom, dy);
  }


}
