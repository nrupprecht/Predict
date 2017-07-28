#include "System.hpp"

namespace Predictive {

  System::System() : nPred(5000), nGrad(0), sIters(25), s_iter(0), tau(0.05), epsilon(0.001), time(1.), t_iter(0), idelay(0.005), temperature(0), data(nullptr), bounds(Bounds(0,1,0,1)) {};

  System::System(DataRecord& dat) : nPred(5000), nGrad(0), sIters(25), s_iter(0), tau(0.05), epsilon(0.001), time(1.), t_iter(0), idelay(0.005), temperature(0), bounds(Bounds(0,1,0,1)) {
    data = &dat;
  }

  System::~System() {};

  void System::run(RealType rt) {
    // Set up
    initialize(rt);
    // Do the simulation
    for (int s_iter=0; s_iter<sIters; ++s_iter) {
      singleIteration();
    }
  }
  
  inline void System::initialize(RealType rt) {
    // Set times
    runTime = rt;
    t_max = runTime/epsilon;
    // Initialize fields
    resource = new Field[t_max]; // Resource fields
    diffField  = Field(bounds);  // Difference field
    trajectory = VField(bounds); // Trajectory field
    gradient   = VField(bounds); // Gradient field
    // Initialize resource fields
    for (int i=0; i<t_max; ++i) resource[i] = Field(bounds);
    // Initialize agents with random positions
    pAgents.resize(nPred);
    ipAgents.resize(nPred);
    for (auto &p : ipAgents) p = vec2(bounds.width()*drand48(), bounds.height()*drand48());
    gAgents.resize(nGrad);
    igAgents.resize(nGrad);
    for (auto &g : igAgents) g = vec2(bounds.width()*drand48(), bounds.height()*drand48());
    // Initialize data record
    if (data) data->initialize(this);
  }

  void System::singleIteration() {
    // Reset variables
    time = 0;
    itimer = 2*idelay; // Make itimer > idelay
    t_iter = 0;
    // Reset agents
    for (int i=0; i<nPred; ++i) pAgents.at(i) = ipAgents.at(i);
    for (int i=0; i<nGrad; ++i) gAgents.at(i) = igAgents.at(i);
    // Initial record
    if (data) data->record(this);
    // Run simulation
    while (time<runTime) {
      // Update agents
      updateAgents();
      // Consume resource
      // consume();
      // Update resource (diffusion)
      // diffusion();
      // Possible update trajectory field
      if (itimer>=idelay) {
	// computeTrajectory();
	itimer = 0.;
      }
      // Record data
      if (data)	data->record(this);
      // Update times
      itimer += epsilon;
      time   += epsilon;
      ++t_iter;
    }
    
  }

  inline void System::updateAgents() {
    // Temperature causes brownian motion
    // --> DT = Kb T / (6 Pi eta R) -> Translational diffusion coefficient
    // --> DR = Kb T / (8 Pi eta R^3) -> Rotational diffusion coefficient

    // Update predictive agents
    for (auto &p : pAgents) {
      // vec2 v = trajectory.get(p);
      //normalize(v);
      // p += velocity*v;
      if (temperature>0) {
	vec2 perturb = sqrt(temperature*epsilon)*randNormal()*randV();
	p += perturb;
      }
      bounds.wrap(p);
    }
    // Update gradient agents
    for (auto &g : gAgents) {
      vec2 v = gradient.get(g);
      normalize(v);
      g += velocity*v;
      bounds.wrap(g);
    }
  }

  inline void System::consume() {

    // Update resource at the next temporal iteration based on the current field values - this means that everyone eats "at the same time"
    
    // Predictive agents eat: bin agents so we have to access field data less
    
    for (auto p : pAgents) {
      
    }
    // Gradient agents eat (note, as above, they are not actually eating "after" the predictive agents.
    for (auto g : gAgents) {
      
    }
  }

  inline void System::diffusion() {
    /*
    diffField.laplacian(resource[t_iter]);
    // Do diffusion
    resource[t_iter].minusEq(diffField, epsilon);
    */
  }

  inline void System::computeTrajectory() {
    /*
    if (nPred>0) {
      // Compute the trajectory field for predictive agents
    }
    if (nGrad>0) {
      // Compute the gradient of the current resource, for the gradient agents' use
      gradient.gradient(resource[t_iter]);
    }
    */
  }

}

