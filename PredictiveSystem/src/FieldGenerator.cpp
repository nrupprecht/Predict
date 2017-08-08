#include "FieldGenerator.hpp"

namespace Predictive {

  /* Arguments: 
   *   field       - The field to set
   *   frequencies - The number of frequencies of noise to add, default 4
   *   mult        - How much to multiply the amplitude of the noise by, default 1.1
   *   minF        - Smallest frequency, default 4
   */
  void FieldGenerator::createSmoothNoise(Field& field, int frequencies, RealType mult, int minF) {
    // Reset field
    field.set(0.);
    // Set starting constants
    RealType maxWave = 1./static_cast<RealType>(minF);
    RealType amp = 1000, wavelength = maxWave;
    // Superimpose different frequencies of noise
    for (int i=0; i<frequencies; ++i) {
      // Add a frequency
      addFrequency(wavelength, amp, field);
      // Adjust noise parameters
      amp *= mult;
      wavelength *= 0.5;
    }
  }

  void FieldGenerator::createPeak(Field& field, RealType cx, RealType cy, RealType sigma, RealType vol) {
    RealType c0 = 0.5/sqr(sigma);
    RealType norm = 1./(sigma*sqrt(2*PI));
    auto peak = [&] (vec2 pos) {
      RealType x = pos.x, y = pos.y;
      return (RealType)(norm*exp(-c0*(sqr(x-cx)+sqr(y-cy))));
    };
    field.set(peak);
  }
  
  void FieldGenerator::createTwoPeaks(Field& field, RealType mult) {
    //RealType m0 = 50, m1 = 25;
    RealType m0 = 1, m1 = 0.5;
    RealType c0 = sqr(12), c1 = sqr(20);
    auto twoPeaks = [&] (vec2 pos) {
      RealType x = pos.x, y = pos.y;
      return mult*(RealType)(m0*exp(-c0*sqr(x-0.5)-c0*sqr(y-0.5)) + m1*exp(-c1*sqr(x-0.25)-c1*sqr(y-0.5)));
    };
    field.set(twoPeaks);
  }
  
  void FieldGenerator::createUniform(Field& field, RealType value) {
    field.set(value);
  }

  void FieldGenerator::createSine(Field& field, RealType amp, int k) {
    Bounds bounds = field.getBounds();
    RealType width = bounds.right-bounds.left, height = bounds.top-bounds.bottom;
    auto sine = [&] (vec2 pos) {
      return amp*sin(PI*k*pos.x/width)*sin(PI*k*pos.y/height);
    };
    field.set(sine);
  }

  inline void FieldGenerator::addFrequency(RealType length, RealType amp, Field& field) {
    // Get values
    int nx = field.getNX(), ny = field.getNY();
    Bounds b = field.getBounds();
    int px = b.width()/length, py = b.height()/length;
    RealType dx = b.width()/nx, dy = b.height()/ny; // Field spacing
    // Allocate array for the major points
    RealType *points = new RealType[px*py];
    // Randomly translate where the origin is
    int transX = static_cast<int>(drand48()*px);
    int transY = static_cast<int>(drand48()*py);
    // Lambda for accessing/setting [points]
    auto at = [&] (int x, int y) -> RealType& {
      x %= px; y %= py;
      return points[px*y + x];
    };
    // Set value of major points, we will interpolate between them
    for (int y=0; y<py; ++y)    
      for (int x=0; x<px; ++x)
	at(x,y) = amp*drand48();
    // Add values
    for (int y=0; y<ny; ++y)
      for (int x=0; x<nx; ++x) {
	// Which major point is left and below 
	int X = static_cast<int>(x*dx/length);
	int Y = static_cast<int>(y*dy/length);
	// Distance (fraction of) between the left / below major point and the field location
	RealType DX = x*dx/length - X;
	RealType DY = y*dy/length - Y;
	// Translate points
	X += transX; Y += transY;
	// Get the four values, for interpolatoin
	RealType TL = at(X, Y+1), TR = at(X+1, Y+1);
	RealType BL = at(X, Y),   BR = at(X+1, Y);
	// Add the (interpolated) value to the field
	field.at(x,y) += interpolate(BL, BR, TL, TR, DX, DY);
      }
	
    delete [] points;
    return;
  }

  inline RealType FieldGenerator::cosineInterpolate(RealType l, RealType r, RealType x) {
    float f = 0.5*(1 - cos(PI*x));
    return l * (1 - f) + r * f;
  }

  inline RealType FieldGenerator::interpolate(RealType TL, RealType TR, RealType BL, RealType BR, RealType dx, RealType dy) {
    RealType top = cosineInterpolate(TL, TR, dx), bottom = cosineInterpolate(BL, BR, dx);
    return cosineInterpolate(top, bottom, dy);
  }


}
