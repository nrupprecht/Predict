#ifndef __DATA_RECORD_HPP__
#define __DATA_RECORD_HPP__

#include "../include/vec2d.hpp"

namespace Predictive {
  
  // Forward declarations
  class System;
  class Field;

  
  class DataRecord {
  public:
    // Constructor - pass in argc, argv
    DataRecord(int, char**);

    // Initialize recording
    void initialize(System*);

    // Record data
    void record(System*);
    void startOfIteration(System*);
    void endOfIteration(System*);

    // Timing
    void start();
    void end();
    RealType getElapsedTime();

    // Mutators
    void setNPPaths(int n) { npPaths = n; }
    void setNGPaths(int n) { ngPaths = n; }
    void setWriteDirectory(string d) { writeDirectory = d; }
    void setRecPositions(bool r) { recPositions = r; }
    void setRecResource(bool r) { recResource = r; }
    void setNPathSamples(int n) { nPathSamples = n; }
    void clear();

    // Write data
    void writeSummary(System*, string="");
    void write(System* = nullptr);

    // Get data
    RealType getAvePCF();
    RealType getAveGCF();
    RealType getPDiff();
    RealType getGDiff();
    RealType getAveL2();
    
  private:
    // Data storage
    int npPaths, ngPaths; // Number of predictive and gradient agent paths to record
    vector<vector<vector<vec2> > > pPaths; // [ s_iter ]  [ t_iter ] [ agent # ] (vec2)
    vector<vector<vector<vec2> > > gPaths; // [ s_iter ]  [ t_iter ] [ agent # ] (vec2)
    vector<Field> resourceRecord;          // What the resource looked like
    vector<vector<vec2> > pConsumptionRec, gConsumptionRec; // Amount consumed vs time 
    vector<RealType> pCvS, gCvS;           // Consumption vs solution iteration
    vector<RealType> pCF, gCF;             // Pred and Grad consumption factors
    RealType totalResource;                // The total amount of resource
    // For L2 path difference - this data can be extracted from pPaths, but we keep them separate so we can set the number of animated objects and L2 paths independently
    int nPathSamples;                      // Number of paths to record, for L2 path difference
    vector<vector<vector<vec2> > > pathSamples;     // [ s_iter ] [ t_iter ] [ agent # ] (vec2)

    // Timers
    RealType recTimer;
    RealType recDelay;
    high_resolution_clock::time_point start_time, end_time;

    // The command line supplied command
    vector<string> command;

    // The directory to record data in
    string writeDirectory;
  
    // Options
    bool recPositions;
    bool recResource;
  };

}
#endif // __DATA_RECORD_HPP__
