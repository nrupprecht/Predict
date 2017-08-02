#ifndef __SYSTEM_HPP__
#define __SYSTEM_HPP__

#include "Field.hpp"
#include "VField.hpp"
#include "DataRecord.hpp"
#include "FieldGenerator.hpp"

namespace Predictive {
  
  // Default constants
  const int default_field_points = 50;
  const RealType default_velocity = 1.;
  const RealType default_temperature = 0;
  const RealType default_diffusion = 0.01;
  const RealType default_consumption = 1.;
  
  /*
   * @class System
   * The class that runs predictive simulations
   * 
   */
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
    int getNPred() { return nPred; }
    int getNGrad() { return nGrad; }
    int getSIters() { return sIters; }
    RealType getTau() { return tau; }
    RealType getEpsilon() { return epsilon; }
    RealType getVelocity() { return velocity; }
    RealType getTemperature() { return temperature; }
    RealType getDiffusion() { return diffusion; }
    RealType getConsumption() { return consumption; }
    int getFieldPoints() { return fieldPoints; }

    // Mutators
    void setNPred(int n)        { nPred = max(0,n); }
    void setNGrad(int n)        { nGrad = max(0,n); }
    void setSIters(int s)       { sIters = max(0,s); }
    void setEpsilon(RealType e);
    void setTau(RealType t)     { tau = fabs(t); }
    void setVelocity(RealType v) { velocity = fabs(v); }
    void setTemperature(RealType t);
    void setDiffusion(RealType d) { diffusion = d; }
    void setConsumption(RealType c) { consumption = c; }
    void setFieldPoints(int n)  { fieldPoints = n; }
    
  private:
    // Private helper functions
    inline void initialize(RealType);
    inline void singleIteration();
    inline void updateAgents();
    inline void consume();
    inline void resourceDiffusion();
    inline void computeTrajectory();
    inline RealType weight(RealType, RealType, RealType);

    // Constants (external parameters)
    int nPred, nGrad;
    int sIters;        // Number of solution iterations to use
    Bounds bounds;     // Simulation bounds
    RealType tau;      // Predictivity
    RealType epsilon;  // Time step
    RealType velocity; // Agent velocity
    RealType temperature; // A stochastic perturbation strength for the agents
    RealType diffusion;   // Resource diffusion
    RealType consumption; // Agents' resource consumptions
    
    // Internal parameters
    RealType runTime;  // How much time to simulate
    RealType time;     // Current time
    int s_iter;        // The current solution iteration
    int t_iter, t_max; // The current temporal iteration, ending temporal iteration
    RealType itimer;   // How long since the last integration
    RealType idelay;   // Integration delay
    int iIter;         // What integration step we are on
    int iPoints;       // How many integration slices we will have - this is calculated before each run
    int fieldPoints;   // How many points we use in the fields
    // The resource at different time slices. The first (0-th) time slice is the initial resource - we never change this. The t_iter-th resource is the current resource. Resources after this are the *predicted* resource fields.
    Field *resourceRec;
    aligned_array<RealType> timeStamps; // The times that the resource record refer to
    Field resource, resbb; // Resource and resource buffer
    // Field to calculate diffusion
    Field diffField;
    VField trajectory; // The predictive agent trajectory field
    VField gradient;   // The gradient of the resource

    // For temperature
    RealType radius, viscosity, Dt, factor;
    
    // Data
    aligned_array<vec2> pAgents, gAgents;   // Predictive and gradient agent positions
    aligned_array<vec2> ipAgents, igAgents; // Initial predictive and gradient agent positions
    aligned_array<RealType> pAgentConsumption, gAgentConsumption; // Consumption records
    RealType weight(RealType, RealType);    // Weighing function - function of space and time

    DataRecord *data;
    friend class DataRecord;
    RealType pConsumption, gConsumption; // Amount consumed by predictive and gradient agents
  };

}
#endif // __SYSTEM_HPP__
