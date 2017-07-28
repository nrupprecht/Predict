#ifndef __SYSTEM_HPP__
#define __SYSTEM_HPP__

#include "Field.hpp"
#include "VField.hpp"
#include "DataRecord.hpp"

namespace Predictive {
  
  class System {
  public:
    // Constructors
    System();
    System(DataRecord&);
    // Destructor
    ~System();
    
    // Run the simulation
    void run(RealType);

    // Accessors

    // Mutators
    void setNPred(int n)        { nPred = max(0,n); }
    void setNGrad(int n)        { nGrad = max(0,n); }
    void setSIters(int s)       { sIters = max(0,s); }
    void setEpsilon(RealType e) { epsilon = fabs(e); }
    void setTau(RealType t)     { tau = fabs(t); }
    void setVelocity(RealType v) { velocity = fabs(v); }
    void setTemperature(RealType t) { temperature = fabs(t); }
    
  private:
    // Private helper functions
    inline void initialize(RealType);
    inline void singleIteration();
    inline void updateAgents();
    inline void consume();
    inline void diffusion();
    inline void computeTrajectory();

    // Constants (external parameters)
    int nPred, nGrad;
    int sIters;        // Number of solution iterations to use
    Bounds bounds;     // Simulation bounds
    RealType tau;      // Predictivity
    RealType epsilon;  // Time step
    RealType velocity; // Agent velocity
    RealType temperature; // A stochastic perturbation strength for the agents

    // Internal parameters
    RealType runTime;  // How much time to simulate
    RealType time;     // Current time
    int s_iter;        // The current solution iteration
    int t_iter, t_max; // The current temporal iteration, ending temporal iteration
    RealType itimer;   // How long since the last integration
    RealType idelay;   // Integration delay
    // The resource at different time slices. The first (0-th) time slice is the initial resource - we never change this. The t_iter-th resource is the current resource. Resources after this are the *predicted* resource fields.
    Field *resource;
    // Field to calculate diffusion
    Field diffField;
    VField trajectory; // The predictive agent trajectory field
    VField gradient;   // The gradient of the resource
    
    // Data
    aligned_array<vec2> pAgents, gAgents;   // Predictive and gradient agent positions
    aligned_array<vec2> ipAgents, igAgents; // Initial predictive and gradient agent positions
    aligned_array<RealType> pAgentConsumption, gAgentConsumption; // Consumption records
    RealType weight(RealType, RealType);    // Weighing function - function of space and time

    DataRecord *data;
    friend class DataRecord;
  };

}
#endif // __SYSTEM_HPP__
