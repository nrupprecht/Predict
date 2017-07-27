#include "System.hpp"

namespace Predictive {

  System::System() : nPred(5000), nGrad(0), sIters(25), s_iter(0), tau(0.05), epsilon(0.001), time(1.), t_iter(0), idelay(0.005), temperature(0) {};

  System::~System() {};
  
  void System::run(RealType runTime) {
    // Reset time
    time = 0;
    // Run simulation
    while (time<runTime) {
      // Update agents
      updateAgents();
      // Consume resource
      consume();
      // Update resource (diffusion)
      diffusion();
      // Possible update trajectory field
      if (itimer>=idelay) {
	computeTrajectory();
	itimer = 0.;
      }

      // Update times
      itimer += epsilon;
      time   += epsilon;
      ++t_iter;
    }
    
  }

  inline void System::updateAgents() {
    // Update predictive agents
    for (auto &p : pAgents) {
      vec2 v = trajectory.get(p);
      normalize(v);
      p += velocity*v;
    }
    // Update gradient agents
    for (auto &g : gAgents) {
      vec2 v = gradient.get(g);
      normalize(v);
      g += velocity*v;
    }
  }

  inline void System::consume() {

    // Update resource at the next temporal iteration based on the current field values - this means that everyone eats "at the same time"
    for (auto p : pAgents) {
      
    }
    // Gradient agents eat (note, as above, they are not actually eating "after" the predictive agents.
    for (auto g : gAgents) {
      
    }
  }

  inline void System::diffusion() {

  }

  inline void System::computeTrajectory() {
    
  }

}
