#ifndef __DATA_RECORD_HPP__
#define __DATA_RECORD_HPP__

#include "../include/vec2d.hpp"

namespace Predictive {
  
  class System;

  class DataRecord {
  public:
    // Constructor - pass in argc, argv
    DataRecord(int, char**);

    // Initialize recording
    void initialize(System*);

    // Record data
    void record(System*);
    void writeSummary(System*);

    // Mutators
    void setNPPaths(int n) { npPaths = n; }
    void setNGPaths(int n) { ngPaths = n; }
    void setWriteDirectory(string d) { writeDirectory = d; }

    // Write data
    void write();
    
  private:
    int npPaths, ngPaths; // Number of predictive and gradient agent paths to record
    vector<vector<vector<vec2> > > pPaths; // [ s_iter ]  [ t_iter ] [ agent # ] (vec2)
    vector<vector<vector<vec2> > > gPaths; // [ s_iter ]  [ t_iter ] [ agent # ] (vec2)

    // Timers
    RealType recTimer;
    RealType recDelay;

    // The command line supplied command
    vector<string> command;

    // The directory to record data in
    string writeDirectory;
  };

}
#endif // __DATA_RECORD_HPP__
