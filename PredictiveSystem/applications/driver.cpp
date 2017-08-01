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
  RealType time        = 1.;    // Simulated time
  int nPred            = 500;   // Number of predictive agents
  int nGrad            = 0;     // Number of gradient agents
  int sIters           = 1;     // Solution iterations
  RealType epsilon     = 0.001; // Time step
  RealType tau         = 0.05;  // Predictivity
  RealType velocity    = 0.1;   // Agent velocity
  RealType temperature = 0;     // Temperature applies brownian perturbation to agents
  RealType diffusion   = 0.1;  // Resource diffusion

  int fieldPoints      = -1;
  int nPredPaths       = nPred;
  int nGradPaths       = nGrad;

  string writeDirectory = "RunData"; // The directory we will create (or overwrite) to write data to

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
  parser.get("diffusion", diffusion);

  parser.get("fieldPoints", fieldPoints);
  parser.get("nPredPaths", nPredPaths);
  parser.get("nPredPaths", nPredPaths);
  
  parser.get("writeDirectory", writeDirectory);
  // Make sure we didn't enter any illegal tokens (ones not listed above) on the command line
  try {
    parser.check();
  }
  catch (ArgParse::UncheckedToken illegal) {
    cout << "Illegal option: [" << illegal.token << "]. Exiting.\n";
    exit(1);
  }


  // Set parameters
  DataRecord data(argc, argv);
  data.setNPPaths(nPredPaths);
  data.setNGPaths(nPredPaths);
  data.setWriteDirectory(writeDirectory);
  System predictive(data);
  if (fieldPoints>0) predictive.setFieldPoints(fieldPoints);
  predictive.setNPred(nPred);
  predictive.setNGrad(nGrad);
  predictive.setSIters(sIters);
  predictive.setEpsilon(epsilon);
  predictive.setTau(tau);
  predictive.setVelocity(velocity);
  predictive.setTemperature(temperature);
  predictive.setDiffusion(diffusion);

  
  // Run system
  predictive.run(time);

  // Write data
  data.writeSummary(&predictive);
  data.write();

  // End
  return 0;
}
