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
  int label = 0;       // Label which program instance this is
  int total = 5000;    // Total number of agents we will use
  int sIters = 10;     // Solution iterations to use
  int divisions = 20;  // Number of divisions
  string writeDirectory = "RecData"; // Base name of the directory to write data to
  bool print = false;  // Whether to print any output
  
  // Seed random
  srand48( std::time( NULL ) );
  seedNormalDistribution();

  // Parse arguments
  ArgParse parser(argc, argv);
  parser.get("time", time);
  parser.get("tau", tau);
  parser.get("label", label);
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

  // Create objects, set properties
  DataRecord data(argc, argv);
  System predictive(data);
  predictive.setSIters(sIters);
  // Data arrays
  typedef pair<int, RealType> cpair;
  vector<cpair> pCF, gCF;
  // Do runs
  RealType minPortion = 0.01, maxPortion = 0.5;
  RealType slope = (maxPortion-minPortion)/static_cast<RealType>(divisions);
  // Print opening message
  if (print) cout << "Starting runs ...\n";
  auto start = high_resolution_clock::now();
  for (int i=0; i<divisions; ++i) {
    // Get number of agents
    int nPred = (minPortion + i*slope)*total;
    int nGrad = total - nPred;
    if (print) cout << "Running with nPred=" << nPred << " (" << i+1 << " of " << divisions << ") ... ";
    auto start_time = high_resolution_clock::now();
    // Clear out old data
    predictive.clear(); // This will also clear data
    predictive.setNPred(nPred);
    predictive.setNGrad(nGrad);
    // Do a run
    predictive.run(time);
    auto end_time = high_resolution_clock::now();
    if (print) cout << "Done (" << time_span(end_time, start_time) << ").\n";
    // Gather data
    pCF.push_back(cpair (nPred, data.getAvePCF()));
    gCF.push_back(cpair (nPred, data.getAveGCF()));
  }
  auto end = high_resolution_clock::now();
  // Print closing message
  if (print) cout << "Done. Total time: " << time_span(end, start) << "\n";

  // Write data
  string wd = writeDirectory + "_tau" + toStr(tau);
  mkdir(wd.c_str(), 0777); // Will not occur if file exists already - so we're safe
  printToCSV(wd+"/PCF"+toStr(label)+".csv", pCF);
  printToCSV(wd+"/GCF"+toStr(label)+".csv", gCF);
  
  // Write a summary 
  data.setWriteDirectory(wd);
  data.writeSummary(&predictive, toStr(label));

  return 0;
}
