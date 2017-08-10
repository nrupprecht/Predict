// Nathaniel Rupprecht
// August 2, 2018

#include "../include/ArgParse.hpp"
#include "../src/System.hpp"

#include <ctime>
#include <cstdlib>

using namespace Predictive;

int main(int argc, char** argv) {
  // Parameters
  RealType time = 1.;  // How much time to simulate
  RealType tau = 0.05; // What predictivity pred agents will have
  RealType velocity = 1; // What velocity the agents will have
  RealType min = 0.05; // The minimum consumption
  RealType max = 5.;   // The maximum consumption
  int label = 0;       // Label which program instance this is
  int expected = 1;    // How many files will be in this batch
  RealType fraction = 0.2; // Fraction of the agents that are predictive
  int total = 5000;    // Total number of agents we will use
  int sIters = 10;     // Solution iterations to use
  int divisions = 20;  // Number of divisions
  string writeDirectory = "CFvsConsumption"; // Base name of the directory to write data to
  bool print = false;  // Whether to print any output
  
  // Seed random
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  srand48( seed );
  seedNormalDistribution();

  // Parse arguments
  ArgParse parser(argc, argv);
  parser.get("time", time);
  parser.get("tau", tau);
  parser.get("velocity", velocity);
  parser.get("min", min);
  parser.get("max", max);
  parser.get("label", label);
  parser.get("expected", expected);
  parser.get("fraction", fraction);
  parser.get("total", total);
  parser.get("sIters", sIters);
  parser.get("divisions", divisions);
  parser.get("writeDirectory", writeDirectory);
  parser.get("print", print);
  // Make sure we didn't enter any illegal tokens (ones not listed above) on the command line
  try {
    parser.check();
  }
  catch (ArgParse::UncheckedToken illegal) {
    cout << "Illegal option: [" << illegal.token << "]. Exiting.\n";
    exit(1);
  }

  // Random label
  int LMax = 100000;
  if (label==-1) label = static_cast<int>(drand48()*LMax);
  
  // Create objects, set properties
  DataRecord data(argc, argv);
  System predictive(data);
  // Set the number of predictive and gradient agents
  int nPred = static_cast<int>(fraction*total), nGrad = total-nPred;
  predictive.setNPred(nPred);
  predictive.setNGrad(nGrad);
  // Set other parameters
  predictive.setSIters(sIters);
  predictive.setTau(tau);
  predictive.setVelocity(velocity);
  
  // Data arrays
  typedef pair<RealType, RealType> cpair;
  vector<cpair> pCF, gCF, pDiff, gDiff, pDiffFactor, gDiffFactor;
  // Do runs
  RealType slope = (max-min)/static_cast<RealType>(divisions);
  // Print opening message
  if (print) cout << "Starting runs ...\n";
  auto start = high_resolution_clock::now();
  for (int i=0; i<divisions; ++i) {
    // Get number of agents
    RealType consumption = min+i*slope;
    if (print) cout << "Running with Consumption=" << consumption << " (" << i+1 << " of " << divisions << ") ... ";
    auto start_time = high_resolution_clock::now();
    // Clear out old data
    predictive.clear(); // This will also clear data
    predictive.setConsumption(consumption);
    // Do a run
    predictive.run(time);
    auto end_time = high_resolution_clock::now();
    if (print) cout << "Done (" << time_span(end_time, start_time) << ").\n";
    // Gather data
    pCF.push_back(cpair (consumption, data.getAvePCF()));
    gCF.push_back(cpair (consumption, data.getAveGCF()));
    pDiff.push_back(cpair (consumption, data.getPDiff()));
    gDiff.push_back(cpair (consumption, data.getGDiff()));
    pDiffFactor.push_back(cpair (consumption, data.getPDiffFactor()));
    gDiffFactor.push_back(cpair(consumption, data.getGDiffFactor()));
  }
  auto end = high_resolution_clock::now();
  // Print closing message
  if (print) cout << "Done. Total time: " << time_span(end, start) << "\n";

  // Write data
  string wd = writeDirectory + "_num" + toStr(total) + "_frac" + toStr(fraction) + "_vel" + toStr(velocity);
  mkdir(wd.c_str(), 0777); // Will not occur if file exists already - so we're safe
  if (label==1) printToCSV(wd+"/number.csv", vector<int>(1, expected));
  printToCSV(wd+"/PCF"+toStr(label)+".csv", pCF);
  printToCSV(wd+"/GCF"+toStr(label)+".csv", gCF);
  printToCSV(wd+"/PDiff"+toStr(label)+".csv", pDiff);
  printToCSV(wd+"/GDiff"+toStr(label)+".csv", gDiff);
  printToCSV(wd+"/PDiffFactor"+toStr(label)+".csv", pDiffFactor);
  printToCSV(wd+"/GDiffFactor"+toStr(label)+".csv", gDiffFactor);

  // Write summary
  data.setWriteDirectory(wd);
  data.writeSummary(&predictive);
  
  return 0;
}
