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
  RealType temperature = 0; // Temperature
  RealType minP = 0.01;// Minimum portion predictive
  RealType maxP = 0.5; // Maximum portion predictive
  int label = 1;       // Label which program instance this is
  int expected = 1;    // How many files will be in this batch
  int total = 5000;    // Total number of agents we will use
  int sIters = 10;     // Solution iterations to use
  int divisions = 20;  // Number of divisions
  string writeDirectory = "DataVsNP"; // Base name of the directory to write data to
  bool print = false;  // Whether to print any output
  bool noiseResource = false; // Use a smooth noise resource
  string tag = "";     // A tag

  // Seed random
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  srand48( seed );
  seedNormalDistribution();

  // Parse arguments
  ArgParse parser(argc, argv);
  parser.get("time", time);
  parser.get("tau", tau);
  parser.get("velocity", velocity);
  parser.get("temperature", temperature);
  parser.get("min", minP);
  parser.get("max", maxP);
  parser.get("label", label);
  parser.get("expected", expected);
  parser.get("total", total);
  parser.get("sIters", sIters);
  parser.get("divisions", divisions);
  parser.get("writeDirectory", writeDirectory);
  parser.get("print", print);
  parser.get("noiseResource", noiseResource);
  parser.get("tag", tag);
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

  // Add a tag to the save directory name
  if (tag!="") writeDirectory += ("_"+tag);
  
  // Create objects, set properties
  DataRecord data(argc, argv);
  System predictive(data);
  if (noiseResource) predictive.setResource(NOISE);
  predictive.setSIters(sIters);
  predictive.setTau(tau);
  predictive.setVelocity(velocity);
  predictive.setTemperature(temperature);
  data.setNPathSamples(5000); // Collect path samples
  // Data arrays
  typedef pair<int, RealType> cpair;
  vector<cpair> pCF, gCF, pDiff, gDiff, pDiffFactor, gDiffFactor, L2pathDiff, fieldDiff;
  // Do runs
  RealType slope = (maxP-minP)/static_cast<RealType>(divisions);
  // Print opening message
  if (print) cout << "Starting runs ...\n";
  auto start = high_resolution_clock::now();
  for (int i=0; i<divisions; ++i) {
    // Get number of agents
    int nPred = (minP + i*slope)*total;
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
    pDiff.push_back(cpair (nPred, data.getPDiff()));
    gDiff.push_back(cpair (nPred, data.getGDiff()));
    pDiffFactor.push_back(cpair (nPred, data.getPDiffFactor()));
    gDiffFactor.push_back(cpair (nPred, data.getGDiffFactor()));
    L2pathDiff.push_back(cpair (nPred, data.getAveL2()));
    fieldDiff.push_back(cpair (nPred, data.getAveFieldDiff(&predictive)) );
  }
  auto end = high_resolution_clock::now();
  // Print closing message
  if (print) cout << "Done. Total time: " << time_span(end, start) << "\n";

  // Write data
  string wd = writeDirectory + "_tau" + toStr(tau) + "_num" + toStr(total) + "_vel" + toStr(velocity) + "_T" + toStr(temperature);
  mkdir(wd.c_str(), 0777); // Will not occur if file exists already - so we're safe
  if (label==1) printToCSV(wd+"/number.csv", vector<int>(1, expected));
  printToCSV(wd+"/PCF"+toStr(label)+".csv", pCF);
  printToCSV(wd+"/GCF"+toStr(label)+".csv", gCF);
  printToCSV(wd+"/PDiff"+toStr(label)+".csv", pDiff);
  printToCSV(wd+"/GDiff"+toStr(label)+".csv", gDiff);
  printToCSV(wd+"/PDiffFactor"+toStr(label)+".csv", pDiffFactor);
  printToCSV(wd+"/GDiffFactor"+toStr(label)+".csv", gDiffFactor);
  printToCSV(wd+"/L2PathDiff"+toStr(label)+".csv", L2pathDiff);
  printToCSV(wd+"/FieldDiff"+toStr(label)+".csv", fieldDiff);

  // Write summary
  data.setWriteDirectory(wd);
  data.writeSummary(&predictive);
  
  return 0;
}
