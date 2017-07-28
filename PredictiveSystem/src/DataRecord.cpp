#include "DataRecord.hpp"
#include "System.hpp"

namespace Predictive {
  DataRecord::DataRecord() : npPaths(10), ngPaths(10), recTimer(0), recDelay(1./15.) {};

  void DataRecord::initialize(System *system) {
    // Predictive agent tracking
    int np = min(npPaths, system->nPred);
    pPaths = vector<vector<vector<vec2> > >(system->sIters);
    for (auto &si : pPaths) {
      si = vector<vector<vec2> >(system->t_max);
      for (auto &ti : si) ti = vector<vec2>(np);
    }
    // Gradient agent tracking
    int ng = min(ngPaths, system->nGrad);
    gPaths = vector<vector<vector<vec2> > >(system->sIters);
    for (auto &si : gPaths) {
      si = vector<vector<vec2> >(system->t_max);
      for (auto&ti : si) ti = vector<vec2>(ng);
    }
  }

  void DataRecord::record(System *system) {
    if (recTimer>=recDelay) {
      int s_iter = system->s_iter;
      int t_iter = system->t_iter;
      int np = min(npPaths, system->nPred);
      int ng = min(ngPaths, system->nGrad);
      for (int i=0; i<np; ++i) pPaths.at(s_iter).at(t_iter).at(i) = system->pAgents.at(i);
      // Reset timer
      recTimer = 0;
    }
    recTimer += system->epsilon;
  }

  void DataRecord::write(string writeDirectory) {
    mkdir(writeDirectory.c_str(), 0777);
    // Print position data
    if (!pPaths.empty())
      printToDirectory(writeDirectory+"/Pos", "pos", pPaths.at(0));
  }
  
}
