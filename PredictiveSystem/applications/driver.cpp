// Nathaniel Rupprecht
// July 27, 2018
// -- Portability and improvements to the original Predictive system simulation --

#include "../include/ArgParse.hpp"
#include "../src/System.hpp"

using namespace Predictive;

int main(int argc, char** argv) {

  // Parameters
  RealType time; // Simulation time
  RealType tau;  // Predictivity
  int nPred;     // Number of predictive agents
  int nGrad;     // Number of gradient agents

  // Parse arguments
  ArgParse parser(argc, argv);
  parser.get("time", time);
  parser.get("tau", tau);
  parser.get("nPred", nPred);
  parser.get("nGrad", nGrad);

  // Set parameters
  System predictive;

  // Run system

  // Write data

  // End
  return 0;
}
