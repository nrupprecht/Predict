#include "System.hpp"

namespace Predictive {

  System::System() : nPred(5000), nGrad(0), sIters(25), s_iter(0), tau(0.05), epsilon(0.001), time(1.), t_iter(0), idelay(0.005), iIter(0), iPoints(0), fieldPoints(default_field_points), temperature(0), diffusion(1.), consumption(1.), data(nullptr), bounds(Bounds(0,1,0,1)) {
    radius = 0.05;
    viscosity = 1.308e-3; // Viscosity of water
    Dt = temperature/(6*viscosity*PI*radius);
    factor = sqrt(2*Dt*epsilon);
  };
  
  System::System(DataRecord& dat) : nPred(5000), nGrad(0), sIters(25), s_iter(0), tau(0.05), epsilon(0.001), time(1.), t_iter(0), idelay(0.005), iIter(0), iPoints(0), fieldPoints(default_field_points), temperature(0), diffusion(1.), consumption(1.), bounds(Bounds(0,1,0,1)) {
    radius = 0.05;
    viscosity = 1.308e-3; // Viscosity of water
    Dt = temperature/(6*viscosity*PI*radius);
    factor = sqrt(2*Dt*epsilon);
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

  void System::setEpsilon(RealType ep) {
    epsilon = ep;
    factor = sqrt(2*Dt*epsilon);
  }

  void System::setTemperature(RealType t) {
    temperature = t;
    Dt = temperature/(6*viscosity*PI*radius);
    factor = sqrt(2*Dt*epsilon);
  }
  
  inline void System::initialize(RealType rt) {
    // Set times
    runTime = rt;
    t_max = runTime/epsilon;
    time = 0;
    // Compute iPoints, correct idelay
    iPoints = static_cast<int>(runTime / idelay);
    idelay = runTime/iPoints;
    ++iPoints;
    iIter = 0;
    // Check for stability. If unstable, reduce timestep
    if (diffusion*epsilon*sqr(fieldPoints) >= 0.25*0.5) epsilon = 0.25 * 0.5/(diffusion*sqr(fieldPoints));
    // Initialize fields
    resourceRec = new Field[iPoints]; // Recorded resource fields
    resource   = Field(bounds, fieldPoints);  // Resource field
    diffField  = Field(bounds, fieldPoints);  // Difference field
    trajectory = VField(bounds, fieldPoints); // Trajectory field
    gradient   = VField(bounds, fieldPoints); // Gradient field
    // Initialize resource fields
    for (int i=0; i<iPoints; ++i) resourceRec[i] = Field(bounds, fieldPoints);
    // Set the initial resource field
    FieldGenerator fieldGenerator;
    fieldGenerator.createTwoPeaks(resourceRec[0]);
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
    // Reset resource
    resource = resourceRec[0];
    // Set trajectory fields
    computeTrajectory();
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
      consume();
      // Update resource (diffusion)
      resourceDiffusion();
      // Possible update trajectory field
      if (itimer>=idelay) {
	++iIter; // Increment beforehand so resourceRec[0] is never changed
	computeTrajectory();
	// Check, in case of rounding errors
	if (iIter<iPoints) resourceRec[iIter] = resource; // Save the current resource
	itimer = 0.;
      }
      // Record data
      if (data)	data->record(this);
      // Update times
      itimer += epsilon;
      time   += epsilon;
      ++t_iter;
    }

    printToCSV("Field.csv",resourceRec[0]);
    printToCSV("FieldEnd.csv", resource);
  }

  inline void System::updateAgents() {
    // Temperature causes brownian motion
    // --> DT = Kb T / (6 Pi eta R) -> Translational diffusion coefficient
    // --> DR = Kb T / (8 Pi eta R^3) -> Rotational diffusion coefficient

    // Update predictive agents
    for (auto &p : pAgents) {
      vec2 v = trajectory.get(p);
      normalize(v);
      p += epsilon*velocity*v;
      if (temperature>0) p += epsilon*factor*randNormal()*randV();
      bounds.wrap(p);
    }
    // Update gradient agents
    for (auto &g : gAgents) {
      vec2 v = gradient.get(g);
      normalize(v);
      g += epsilon*velocity*v;
      if (temperature>0) g += epsilon*factor*randNormal()*randV();
      bounds.wrap(g);
    }
  }

  inline void System::consume() {
    // Update resource at the next temporal iteration based on the current field values - this means that everyone eats "at the same time"
    // Initialize resource to the resource of the current time step, then eat from it and diffuse the resulting resource
    resbb = resource;
    // Predictive agents eat: bin agents so we have to access field data less
    
    for (auto p : pAgents) {
      
    }
    // Gradient agents eat (note, as above, they are not actually eating "after" the predictive agents.
    for (auto g : gAgents) {
      resource.get(g) -= epsilon*consumption*resbb.get(g);
    }
  }

  inline void System::resourceDiffusion() {
    // Calculate the laplacian of the resource
    diffField.laplacian(resource);
    // Do diffusion
    resource.plusEq(diffField, diffusion*epsilon);
  }

  inline void System::computeTrajectory() {
    if (nPred>0) {
      // Compute the trajectory field for predictive agents
    }
    if (nGrad>0) {
      // Compute the gradient of the current resource, for the gradient agents' use
      gradient.gradient(resource);
    }
  }

}

