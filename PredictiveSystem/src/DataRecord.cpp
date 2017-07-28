#include "DataRecord.hpp"
#include "System.hpp"

namespace Predictive {
  DataRecord::DataRecord() : npPaths(10), ngPaths(10), recTimer(0), recDelay(1./15.) {};

  void DataRecord::initialize(System *system) {
    // Predictive agent tracking
    int np = min(npPaths, system->nPred);
    pPaths = vector<vector<vector<vec2> > >(system->sIters);
    // Gradient agent tracking
    int ng = min(ngPaths, system->nGrad);
    gPaths = vector<vector<vector<vec2> > >(system->sIters);
  }

  void DataRecord::record(System *system) {
    if (recTimer>=recDelay) {
      int s_iter = system->s_iter;
      int np = min(npPaths, system->nPred);
      int ng = min(ngPaths, system->nGrad);
      
      int t_iter = pPaths.at(s_iter).size();
      // Push a vector for this time step
      pPaths.at(s_iter).push_back(vector<vec2>(np));
      for (int i=0; i<np; ++i) {
	vec2 pos = system->pAgents.at(i);
	pPaths.at(s_iter).at(t_iter).at(i) = pos;
      }
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
