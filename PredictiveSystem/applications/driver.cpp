// Nathaniel Rupprecht
// July 27, 2018
// -- Portability and improvements to the original Predictive system simulation --

#include "../include/ArgParse.hpp"
#include "../src/System.hpp"

using namespace Predictive;

int main(int argc, char** argv) {

  Bounds b(0,1,0,1);
  Field f;
  f = Field(b);
  return 0;

  // Parameters
  RealType time        = 1.; // Simulation time
  int nPred            = 500;     // Number of predictive agents
  int nGrad            = 0;     // Number of gradient agents
  int sIters           = 1;
  RealType epsilon     = 0.005;
  RealType tau         = 0.05;  // Predictivity
  RealType velocity    = 0.1; 
  RealType temperature = 0;

  // Parse arguments
  ArgParse parser(argc, argv);
  parser.get("time", time);
  parser.get("nPred", nPred);
  parser.get("nGrad", nGrad);
  parser.get("sIters", sIters);
  parser.get("epsilon", epsilon);
  parser.get("tau", tau);
  parser.get("velocity", velocity);
  parser.get("temperature", temperature);

  // Set parameters
  DataRecord data;
  System predictive(data);
  predictive.setNPred(nPred);
  predictive.setNGrad(nGrad);
  predictive.setSIters(sIters);
  predictive.setEpsilon(epsilon);
  predictive.setTau(tau);
  predictive.setVelocity(velocity);
  predictive.setTemperature(temperature);
  
  // Run system
  predictive.run(time);

  // Write data
  data.write("RunData");

  // End
  return 0;
}
