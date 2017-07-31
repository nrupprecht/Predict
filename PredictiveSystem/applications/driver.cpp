// Nathaniel Rupprecht
// July 27, 2018
// -- Portability and improvements to the original Predictive system simulation --

#include "../include/ArgParse.hpp"
#include "../src/System.hpp"

#include <ctime>
#include <cstdlib>

using namespace Predictive;

int main(int argc, char** argv) {

  // Parameters
  RealType time        = 1.; // Simulation time
  int nPred            = 500;     // Number of predictive agents
  int nGrad            = 0;     // Number of gradient agents
  int sIters           = 1;
  RealType epsilon     = 0.005;
  RealType tau         = 0.05;  // Predictivity
  RealType velocity    = 0.1; 
  RealType temperature = 0;

  int nPredPaths       = 10;
  int nGradPaths       = 10;

  // Seed random
  srand48( std::time( NULL ) );
  seedNormalDistribution();


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

  parser.get("nPredPaths", nPredPaths);
  parser.get("nPredPaths", nPredPaths);
  // Make sure we didn't enter any illegal tokens (ones not listed above) on the command line
  try {
    parser.check();
  }
  catch (ArgParse::UncheckedToken illegal) {
    cout << "Illegal option: [" << illegal.token << "]. Exiting.\n";
    exit(1);
  }


  // Set parameters
  DataRecord data;
  data.setNPPaths(nPredPaths);
  data.setNGPaths(nPredPaths);
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
