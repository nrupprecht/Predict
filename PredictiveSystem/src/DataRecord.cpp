#include "DataRecord.hpp"
#include "System.hpp"

namespace Predictive {
  DataRecord::DataRecord(int argc, char** argv) : npPaths(10), ngPaths(10), recTimer(0), recDelay(1./15.) {
    for (int i=1; i<argc; ++i)
      command.push_back(argv[i]);
  };

  void DataRecord::initialize(System *system) {
    // Predictive agent tracking
    int np = min(npPaths, system->nPred);
    pPaths = vector<vector<vector<vec2> > >(system->sIters);
    // Gradient agent tracking
    int ng = min(ngPaths, system->nGrad);
    gPaths = vector<vector<vector<vec2> > >(system->sIters);
    // Create the directory
    mkdir(writeDirectory.c_str(), 0777);
  }

  void DataRecord::record(System *system) {
    if (recTimer>=recDelay) {
      int s_iter = system->s_iter;
      int np = min(npPaths, system->nPred);
      int ng = min(ngPaths, system->nGrad);
      
      int t_iter = max(pPaths.at(s_iter).size(), gPaths.at(s_iter).size());
      // Push a vector for this time step
      if (np>0) {
	pPaths.at(s_iter).push_back(vector<vec2>(np));
	for (int i=0; i<np; ++i) {
	  vec2 pos = system->pAgents.at(i);
	  pPaths.at(s_iter).at(t_iter).at(i) = pos;
	}
      }
      if (ng>0) {
	gPaths.at(s_iter).push_back(vector<vec2>(ng));
	for (int i=0; i<ng; ++i) {
          vec2 pos = system->gAgents.at(i);
          gPaths.at(s_iter).at(t_iter).at(i) = pos;
	}
      }
      // Reset timer
      recTimer = 0;
    }
    recTimer += system->epsilon;
  }

  void DataRecord::writeSummary(System* system) {
    std::ofstream fout(writeDirectory+"/run_summary.txt");
    if (fout.fail()) {
      // Write error message
      std::cerr << "Failed to open file [" << writeDirectory << "/run_summary.txt]." << endl;
      return;
    }
    // Print Header
    fout << "**********          SUMMARY          **********\n";
    fout << "********* Predictive System Simulator *********\n";
    fout << "********** 2017, Nathaniel Rupprecht **********\n";
    fout << "***********************************************\n\n";
    // Print command
    if (!command.empty()) {
      fout << "Command:\n  ";
      for (auto& c : command) fout << c << " ";
      fout << "\n\n";
    }

    fout << "Simulation and space:\n";
    fout << "  - Number of Predictors:      " << system->getNPred() << "\n";
    fout << "  - Number of Gradients:       " << system->getNGrad() << "\n";
    fout << "  - Predictivity:              " << system->getTau() << "\n";
    fout << "  - Agent velocity:            " << system->getVelocity() << "\n";
    fout << "  - Time step:                 " << system->getEpsilon() << "\n";
    fout << "  - Diffusion:                 " << system->getDiffusion() << "\n";
    fout << "  - Field points:              " << system->getFieldPoints() << "\n";
    fout << "  - Stability (<1 is good):    " << system->getDiffusion()*system->getEpsilon()*sqr(system->getFieldPoints())*2 << endl;

  }

  void DataRecord::write() {
    // Print position data
    if (!pPaths.empty())
      printToDirectory(writeDirectory+"/PosP", "pos", pPaths.at(0));
    if (!gPaths.empty())
      printToDirectory(writeDirectory+"/PosG", "pos", gPaths.at(0));
  }
  
}
