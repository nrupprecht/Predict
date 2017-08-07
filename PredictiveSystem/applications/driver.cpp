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
  int nPred            = 5000;  // Number of predictive agents
  int nGrad            = 0;     // Number of gradient agents
  int sIters           = -1;    // Solution iterations
  RealType epsilon     = 0.001; // Time step
  RealType tau         = 0.05;  // Predictivity
  RealType velocity    = -1.;   // Agent velocity
  RealType temperature = -1.;   // Temperature applies brownian perturbation to agents
  RealType diffusion   = -1.;   // Resource diffusion
  RealType consumption = -1.;   // Agents' resource consumption
  bool noiseResource   = false; // Use a smooth noise resource

  RealType idelay      = -1;
  int fieldPoints      = -1;
  int nPredPaths       = nPred;
  int nGradPaths       = nGrad;

  bool recPositions    = false;
  bool recResource     = false;
  bool rec             = false;

  string writeDirectory = "RunData"; // The directory we will create (or overwrite) to write data to

  // Seed random
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  srand48( seed );
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
  parser.get("consumption", consumption);
  parser.get("noiseResource", noiseResource);

  parser.get("idelay", idelay);
  parser.get("fieldPoints", fieldPoints);
  parser.get("nPredPaths", nPredPaths);
  parser.get("nGradPaths", nGradPaths);
  
  parser.get("recPositions", recPositions);
  parser.get("recResource", recResource);
  parser.get("rec", rec);

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
  data.setRecPositions(recPositions);
  data.setRecResource(recResource);
  if (rec) {
    data.setRecPositions(true);
    data.setRecResource(true);
  }
  data.setWriteDirectory(writeDirectory);
  System predictive(data);
  if (noiseResource)  predictive.setResource(NOISE);
  if (idelay>=0)      predictive.setIDelay(idelay);
  if (fieldPoints>0)  predictive.setFieldPoints(fieldPoints);
  if (nPred>=0)       predictive.setNPred(nPred);
  if (nGrad>=0)       predictive.setNGrad(nGrad);
  if (sIters>0)       predictive.setSIters(sIters);
  if (epsilon>0)      predictive.setEpsilon(epsilon);
  if (tau>=0)         predictive.setTau(tau);
  if (velocity>=0)    predictive.setVelocity(velocity);
  if (temperature>=0) predictive.setTemperature(temperature);
  if (diffusion>=0)   predictive.setDiffusion(diffusion);
  if (consumption>=0) predictive.setConsumption(consumption);
  
  // Run system
  predictive.run(time);
  
  // Write data
  data.writeSummary(&predictive);
  data.write(&predictive);

  // End
  return 0;
}
