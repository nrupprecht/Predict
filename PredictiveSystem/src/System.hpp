#ifndef __SYSTEM_HPP__
#define __SYSTEM_HPP__

#include "Field.hpp"
#include "VField.hpp"

namespace Predictive {
  
  class System {
  public:
    System();
    ~System();
    
    void run(RealType);
    
  private:
    // Private helper functions
    inline void updateAgents();
    inline void consume();
    inline void diffusion();
    inline void computeTrajectory();

    // Constants
    int nPred, nGrad;
    int sIters;        // Number of solution iterations to use
    int s_iter;        // The current solution iteration
    
    RealType tau;      // Predictivity
    RealType epsilon;  // Time step
    RealType time;     // How much time to simulate
    int t_iter;        // The current temporal iteration
    RealType itimer;   // How long since the last integration
    RealType idelay;   // Integration delay
    RealType velocity; // Agent velocity
    RealType temperature; // A stochastic perturbation strength for the agents
    // The resource at different time slices. The first (0-th) time slice is the initial resource - we never change this. The t_iter-th resource is the current resource. Resources after this are the *predicted* resource fields.
    Field *resource;   
    VField trajectory; // The predictive agent trajectory field
    VField gradient;   // The gradient of the resource
    
    // Data
    aligned_array<vec2> pAgents, gAgents; // Predictive and gradient agent positions
    aligned_array<RealType> pAgentConsumption, gAgentConsumption; // Consumption records
    
    RealType weight(RealType, RealType); // Weighing function - function of space and time
    
  };

}
#endif // __SYSTEM_HPP__
