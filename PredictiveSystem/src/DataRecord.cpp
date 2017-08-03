#include "DataRecord.hpp"
#include "System.hpp"

namespace Predictive {
  DataRecord::DataRecord(int argc, char** argv) : npPaths(10), ngPaths(10), recTimer(0), recDelay(1./15.), recPositions(true), recResource(true), totalResource(0) {
    for (int i=1; i<argc; ++i)
      command.push_back(argv[i]);
    // Set the timers to have the current time
    start_time = high_resolution_clock::now();
    end_time = high_resolution_clock::now();
  };

  void DataRecord::initialize(System *system) {
    // Predictive agent tracking
    if (recPositions) {
      int np = min(npPaths, system->nPred);
      pPaths = vector<vector<vector<vec2> > >(system->sIters);
      // Gradient agent tracking
      int ng = min(ngPaths, system->nGrad);
      gPaths = vector<vector<vector<vec2> > >(system->sIters);
    }
    // Create the directory
    mkdir(writeDirectory.c_str(), 0777);
    // Record the total amount of resources
    totalResource = system->resourceRec[0].total();
  }

  void DataRecord::record(System *system) {
    // Find the current time
    RealType time = system->time;
    // Record data
    if (recTimer>=recDelay || time==0) {
      if (recPositions) {
	// Get the solution iteration number and the number of agents to record
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
      }
      
      // Record resource
      if (recResource) resourceRecord.push_back(system->resource);

      // Update consumption records
      if (system->nPred>0) Last(pConsumptionRec).push_back(vec2(time, system->pConsumption));
      if (system->nGrad>0) Last(gConsumptionRec).push_back(vec2(time, system->gConsumption));

      // Reset timer
      recTimer = 0;
    }
    recTimer += system->epsilon;
  }

  void DataRecord::startOfIteration(System* system) {
    // Check for a valid system
    if (system==nullptr) return;
    // Push new record vectors
    if (system->nPred>0) pConsumptionRec.push_back(vector<vec2>());
    if (system->nGrad>0) gConsumptionRec.push_back(vector<vec2>());
  }

  void DataRecord::endOfIteration(System* system) {
    // Check for a valid system
    if (system==nullptr) return;
    // Record the total consumption during the run
    pCvS.push_back(system->pConsumption);
    gCvS.push_back(system->gConsumption);
    // Record consumption factors
    int np = system->nPred, ng = system->nGrad;
    int nt = np + ng;
    if (np>0) pCF.push_back(system->pConsumption*nt/(totalResource*np));
    else pCF.push_back(0);
    if (ng>0) gCF.push_back(system->gConsumption*nt/(totalResource*ng));
    else gCF.push_back(0);
  }

  void DataRecord::start() {
    start_time = high_resolution_clock::now();
  }

  void DataRecord::end() {
    end_time = high_resolution_clock::now();
  }

  RealType DataRecord::getElapsedTime() {
    return time_span(end_time, start_time);
  }

  void DataRecord::clear() {
    npPaths = ngPaths = 0;
    pPaths.clear();
    gPaths.clear();
    resourceRecord.clear();
    pConsumptionRec.clear();
    gConsumptionRec.clear();
    pCvS.clear();
    gCvS.clear();
    pCF.clear();
    gCF.clear();
    totalResource = 0;
  }

  void DataRecord::writeSummary(System* system, string label) {
    // Check that system is non-null
    if (system==nullptr) return;
    // Open a file
    std::ofstream fout;
    if (label=="") fout.open(writeDirectory+"/run_summary.txt");
    else fout.open(writeDirectory+"_"+label+"/run_summary.txt");
    if (fout.fail()) {
      // Write error message
      std::cerr << "Failed to open file [" << writeDirectory << "/run_summary.txt]." << endl;
      return;
    }
    // Set some booleans
    bool hasP = (system->nPred>0), hasG = (system->nGrad>0);
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
    fout << "  - Simulated Time:            " << system->runTime << "\n";
    fout << "  - Real Time:                 " << getElapsedTime() << " (" << printAsTime(getElapsedTime()) << ")\n";
    fout << "  - Time per solution iter.:   " << getElapsedTime() / system->sIters << "\n";
    fout << "  - Diffusion:                 " << system->diffusion << "\n";
    fout << "\n";
    fout << "Agents:\n";
    fout << "  - Number of Predictors:      " << system->nPred << "\n";
    fout << "  - Number of Gradients:       " << system->nGrad << "\n";
    fout << "  - Predictivity (tau):        " << system->tau << "\n";
    fout << "  - Agent velocity:            " << system->velocity << "\n";
    fout << "  - Agent consumption rate:    " << system->consumption << "\n";
    fout << "  - Solution iterations:       " << system->sIters << "\n";
    fout << "\n";
    fout << "Integration and Discretization:\n";
    fout << "  - Field points:              " << system->fieldPoints << "\n";
    fout << "  - Stability (<1 is good):    " << system->diffusion*system->epsilon*sqr(system->fieldPoints)*2 << endl;
    fout << "  - Time step:                 " << system->epsilon << "\n";
    fout << "  - Time integration delay:    " << system->idelay << "\n";
    fout << "  - Time integration points:   " << (system->iPoints-1) << "\n";
    fout << "  - Average points per int.:   " << (system->iPoints-1) * system->tau << "\n";
    fout << "\n";
    fout << "Consumption:\n";
    // if (hasP) fout << "  - Ave. pred. consumption:    " << Average(pConsumptionRec).y << "\n";
    // if (hasG) fout << "  - Ave. grad. consumption:    " << Average(gConsumptionRec).y << "\n";
    // if (hasP) fout << "  - Max pred. consumption:     " << Max(pConsumptionRec) << "\n";
    // if (hasG) fout << "  - Max grad. consumption:     " << Max(gConsumptionRec) << "\n";
    if (hasP) fout << "  - Ave pred. CF:              " << Average(pCF) << "\n";
    if (hasG) fout << "  - Ave grad. CF:              " << Average(gCF) << "\n";
    if (hasP) fout << "  - Max pred. CF:              " << Max(pCF) << "\n";
    if (hasG) fout << "  - Max grad. CF:              " << Max(gCF) << "\n";
    // fout << "  - Ave resource consumed:     " << (Average(pConsumptionRec).y+Average(gConsumptionRec).y)/totalResource << "\n";
  }

  void DataRecord::write(System* system) {
    // Print position data
    if (!pPaths.empty())
      printToDirectory(writeDirectory+"/PosP", "pos", pPaths.at(0));
    if (!gPaths.empty())
      printToDirectory(writeDirectory+"/PosG", "pos", gPaths.at(0));
    // Print resource data
    if (!resourceRecord.empty()) {
      // Make a directory for the resource data
      mkdir((writeDirectory+"/Resource").c_str(), 0777);
      // Print the resource data
      int i=0;
      for (const auto& res : resourceRecord) {
	printToCSV(writeDirectory+"/Resource/res"+toStr(i)+".csv", res);
	++i;
      }
    }

    // Print consumption vs time (during a single solution iteration) data
    if (!pConsumptionRec.empty()) {
      mkdir((writeDirectory+"/pConsumption").c_str(), 0777);
      int i=0;
      for (const auto &pc : pConsumptionRec) {
	printToCSV(writeDirectory+"/pConsumption/pCon"+toStr(i)+".csv", pc);
	++i;
      }
    }
    if (!gConsumptionRec.empty()) {
      mkdir((writeDirectory+"/gConsumption").c_str(), 0777);
      int i = 0;
      for (const auto &gc : gConsumptionRec) {
	printToCSV(writeDirectory+"/gConsumption/gCon"+toStr(i)+".csv", gc);
	++i;
      }
    }

    // Print consumption vs solution iteration data
    printToCSV(writeDirectory+"/pCvS.csv", pCF);
    printToCSV(writeDirectory+"/gCvS.csv", gCF);
    // Print field difference data
    if (system && !system->fieldDiff.empty()) {
      // Make a directory for the field data
      mkdir((writeDirectory+"/FieldDiff").c_str(), 0777);
      // Print the field difference data
      int i=0;
      for (const auto& df : system->fieldDiff) {
	printToCSV(writeDirectory+"/FieldDiff/fd"+toStr(i)+".csv", df);
	++i;
      }
    }
  }

  RealType DataRecord::getAvePCF() {
    return Average(pCF);
  }

  RealType DataRecord::getAveGCF() {
    return Average(gCF);
  }
  
}
