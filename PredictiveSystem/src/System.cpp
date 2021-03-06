#include "System.hpp"

namespace Predictive {

  System::System() : nPred(5000), nGrad(0), sIters(25), s_iter(0), tau(default_tau), epsilon(default_epsilon), velocity(default_velocity), time(1.), t_iter(0), idelay(0.005), iIter(0), iPoints(0), fieldPoints(default_field_points), resourceRec(nullptr), resType(TWOPEAKS), temperature(default_temperature), diffusion(default_diffusion), consumption(default_consumption), data(nullptr), bounds(Bounds(0,1,0,1)), pConsumption(0), gConsumption(0) {
    radius = 0.05;
    viscosity = 1.308e-3; // Viscosity of water
    Dt = temperature/(6*viscosity*PI*radius);
    factor = sqrt(2*Dt*epsilon);
    // Set weight function
    wght = invRSqr;
  };
  
  System::System(DataRecord& dat) : nPred(5000), nGrad(0), sIters(25), s_iter(0), tau(default_tau), epsilon(default_epsilon), velocity(default_velocity), time(1.), t_iter(0), idelay(0.005), iIter(0), iPoints(0), fieldPoints(default_field_points), resourceRec(nullptr), resType(TWOPEAKS), temperature(default_temperature), diffusion(default_diffusion), consumption(default_consumption), bounds(Bounds(0,1,0,1)), pConsumption(0), gConsumption(0) {
    radius = 0.05;
    viscosity = 1.308e-3; // Viscosity of water
    Dt = temperature/(6*viscosity*PI*radius);
    factor = sqrt(2*Dt*epsilon);
    data = &dat;
    // Set weight function
    wght = invRSqr;
  }

  System::~System() {};

  void System::run(RealType rt) {    
    // Set up
    initialize(rt);
    // Do the simulation
    if (data) data->start();
    for (s_iter=0; s_iter<sIters; ++s_iter) {
      if (data) data->startOfIteration(this);
      singleIteration();
      if (data) data->endOfIteration(this);
    }
    if (data) data->end();
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

  void System::clear() {
    pConsumption = 0;
    gConsumption = 0;
    if (data) data->clear();
  }
  
  inline void System::initialize(RealType rt) {
    // Set times
    runTime = rt;
    t_max = runTime/epsilon;
    time = 0;
    // Check for stability. If unstable, reduce timestep
    if (diffusion*epsilon*sqr(fieldPoints) >= 0.25*0.5) epsilon = 0.25 * 0.5/(diffusion*sqr(fieldPoints));
    // Compute iPoints, correct idelay
    if (idelay>0) {
      iPoints = static_cast<int>(runTime / idelay);
      iPoints = min(iPoints, static_cast<int>(runTime/epsilon));
    }
    else iPoints = runTime/epsilon;
    // Correct idelay
    idelay = runTime/iPoints;
    ++iPoints;

    // Initialize fields
    resourceRec = new Field[iPoints]; // Recorded resource fields
    resource    = Field(bounds, fieldPoints);  // Resource field
    diffField   = Field(bounds, fieldPoints);  // Difference field
    trajectory  = VField(bounds, fieldPoints); // Trajectory field
    gradient    = VField(bounds, fieldPoints); // Gradient field
    // Set the initial resource field
    FieldGenerator fieldGenerator;
    resourceRec[0] = Field(bounds, fieldPoints);
    switch (resType) {
    default:
    case TWOPEAKS:
      fieldGenerator.createTwoPeaks(resourceRec[0], 10000);
      break;
    case NOISE:
      fieldGenerator.createSmoothNoise(resourceRec[0]);
      break;
    }
    // Allocate time stamps
    timeStamps.resize(iPoints);
    // Initialize resource fields - do simple diffusion
    iIter = 0;
    resource = resourceRec[iIter];
    timeStamps.at(0) = 0;
    while (time<runTime) {
      // Update times
      itimer += epsilon;
      time += epsilon;
      // Diffuse
      diffField.laplacian(resource);
      resource.plusEq(diffField, diffusion*epsilon);
      // Possible record field data
      if (fabs(time-(iIter+1)*idelay)<0.5*epsilon) {
	++iIter;
	resourceRec[iIter] = resource;
	timeStamps.at(iIter) = time;
	itimer = 0;
      }
    }
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
    t_iter = 0;
    itimer = 2*idelay; // Make itimer > idelay
    iIter = 0;
    // Reset resource
    resource = resourceRec[0];
    pConsumption = 0; 
    gConsumption = 0;
    // Set trajectory fields
    computeTrajectory();
    // Reset agents
    for (int i=0; i<nPred; ++i) pAgents.at(i) = ipAgents.at(i);
    for (int i=0; i<nGrad; ++i) gAgents.at(i) = igAgents.at(i);
    // Initial record
    if (data) data->record(this);
    fieldDiff.push_back(vector<vec2>()); // New entry
    // Run simulation
    while (time<runTime) {
      // Update agents
      updateAgents();
      // Consume resource
      consume();
      // Update resource (diffusion)
      resourceDiffusion();
      // Possible update trajectory field
      if (fabs(time-(iIter+1)*idelay)<0.5*epsilon) { // If this is the closest iteration to when we should integrate
	++iIter; // Increment beforehand so resourceRec[0] is never changed
	computeTrajectory();
	// Could also use inner product
	Last(fieldDiff).push_back(vec2(time, compairProduct(resourceRec[iIter], resource)));
	resourceRec[iIter] = resource; // Save the current resource
	timeStamps.at(iIter) = time;   // Save what time the resource corresponds to
	itimer = 0.; //-- We don't actually need this anymore
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
    RealType area = resource.getDX()*resource.getDY();
    RealType scaledConsumption = consumption/(nPred+nGrad);
    int nx = resource.getNX(), ny = resource.getNY();

    // Binned eating method
    if (nPred>0) {
      vector<int> bins(nx*ny, 0);
      for (auto p : pAgents) ++bins.at( resource.getBin(p) );
      for (int y=0; y<nx; ++y)
	for (int x=0; x<nx; ++x) {
	  // Calculate the volume of resources to eat
	  RealType rho    = bins.at(nx*y+x) / area; // Agent density
	  RealType factor = scaledConsumption*rho*resbb.at(x,y); // c * rho * phi
	  RealType amount = epsilon * factor;
	  // Subtract the volume
	  resource.at(x,y) -= amount;
	  pConsumption += amount * area;
	}
    }
    
    // Gradient agents eat (note, as above, they are not actually eating "after" the predictive agents.
    
    // Binned eating method
    if (nGrad>0) {
      vector<int> bins(nx*ny, 0);
      for (auto g : gAgents) ++bins.at( resource.getBin(g) );
      for (int y=0; y<nx;++y)
	for (int x=0; x<nx; ++x) {
	  // Calculate the volume of resources to eat
	  RealType rho    = bins.at(nx*y+x) / area;              // Agent density
	  RealType factor = scaledConsumption*rho*resbb.at(x,y); // c * rho * phi
	  RealType amount = epsilon * factor;
	  // Subtract the volume
	  resource.at(x,y) -= amount;
	  gConsumption += amount * area;
	}
    }
  }

  inline void System::resourceDiffusion() {
    // Make sure resource is non negative after eating.
    clamp(resource);
    // Calculate the laplacian of the resource
    diffField.laplacian(resource);
    // Do diffusion
    resource.plusEq(diffField, diffusion*epsilon);
  }

  inline void System::computeTrajectory() {
    if (0<nPred) {
      // For the first solution iteration, act like gradient agents
      if (s_iter==0) trajectory.gradient(resource);
      else predictiveTrajectory();
    }
    if (nGrad>0) {
      // Compute the gradient of the current resource, for the gradient agents' use
      gradient.gradient(resource);
    }
  }

  // Compute the trajectory field for predictive agents  
  inline void System::predictiveTrajectory() {
    // Find first slice to look at
    int iMin = 0, iMax = iPoints-1;
    for (int i=0; i<iPoints; ++i) {
      if (time <= timeStamps.at(i)      && iMin==0) iMin = i;
      if (time + tau < timeStamps.at(i) && iMax==iPoints-1) iMax = i;
    }
    if (time>timeStamps.last()) iMin = iPoints-1;
    // Tau = 0 results in iMax = iMin + 1, we want iMin==iMax
    if (tau==0) iMin = iMax = 0;

    // Look locally for current time step
    for (int y=0; y<fieldPoints; ++y) {
      for (int x=0; x<fieldPoints; ++x) {
	// Look at points in space that we could get to going at our velocity
	RealType dxs = sqr(resource.getDX()); // We have set dx==dy, so there isn't a problem here
	vec2 value = Zero;
	// Calculate for the four points around you (up, down, left, right)
	RealType weight = wght(dxs, 0);
	value += weight*resource.at(x+1,y)*vec2(1,0);
	value += weight*resource.at(x-1,y)*vec2(-1,0);
	value += weight*resource.at(x,y+1)*vec2(0,1);
	value += weight*resource.at(x,y-1)*vec2(0,-1);
	trajectory.at(x,y) = value; // This is the same as reseting trajectory, then starting to add
      }
    }
    
    // Calculate trajectory based on future resource projection
    RealType diff = resource.getDX(); // We have set dx==dy so there isn't a problem here
    if (iMin!=iMax) {
      for (int y=0; y<fieldPoints; ++y) {
	for (int x=0; x<fieldPoints; ++x) {
	  // Look ahead the appropriate number of iterations
	  for (int iter=iMin; iter<iMax; ++iter) {
	    // Get dt
	    RealType dt = timeStamps.at(iter) - time;
	    // Look at points in space that we could get to going at our velocity
	    RealType radius = velocity*dt;
	    // Don't double count points
	    int cut = min(fieldPoints/2, static_cast<int>(ceil(radius/diff)));
	    cut = max(1, cut);
	    vec2 value = Zero;
	    for (int px=-cut; px<=cut; ++px)
	      for (int py=-cut; py<=cut; ++py) {
		int ds = sqr(px)+sqr(py);
		if (ds<=cut && ds!=0) {
		  RealType dxs = ds*sqr(diff);
		  RealType res = resourceRec[iter].at(x+px, y+py);
		  vec2 norm(px, py);
		  normalize(norm);
		  value += wght(dxs, dt)*res*norm;
		}
	      }
	    trajectory.at(x,y) += value;
	  }
	}
      }
    }
    // Done calculating trajectory for predictive agents
  }

}

