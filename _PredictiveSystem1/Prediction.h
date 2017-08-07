#ifndef _PREDICTION_H
#define _PREDICTION_H

#include "Utility.h"
#include "Field.h"

// What the initial resource distribution should look like
enum ResType { UNIFORM, TWOPEAKS, NOISE, PARABOLA };
// What type of weighting we do 
enum WeightType { INVRSQR, INVR, PROPRSQR, PROPR, CONST };

// Resource type printing function
inline string print(ResType r) {
  switch (r) {
  case UNIFORM: return "UNIFORM";
  case TWOPEAKS: return "TWOPEAKS";
  case PARABOLA: return "PARABOLA";
  case NOISE: return "NOISE";
  default: return "UNRECOGNIZED/UNIMPLEMENTED";
  }
}

inline ResType makeResType(string str) {
  if (str=="TWOPEAKS") return TWOPEAKS;
  else if (str=="NOISE") return NOISE;
  else if (str=="PARABOLA") return PARABOLA;
  else return UNIFORM;
}

// Weight type printing function
inline string print(WeightType w) {
  switch(w) {
  case INVRSQR: return "1/R^2";
  case INVR: return "1/R";
  case PROPRSQR: return "R^2";
  case PROPR: return "R";
  case CONST: return "1";
  default: return "UNRECOGNIZED/UNIMPLEMENTED";
  }
}

inline WeightType makeWeightType(string str) {
  if (str=="INVRSQR") return INVRSQR;
  else if (str=="INVR") return INVR;
  else if (str=="PROPRSQR") return PROPRSQR;
  else if (str=="PROPR") return PROPR;
  else return CONST;
}

class Prediction {
 public:
  Prediction(int, int, double, int, int=0);
  ~Prediction();
  
  void run(double);

  /// Accessors
  double getEpsilon() { return epsilon; }
  double getPredictivity() { return predictivity; }
  double getRunTime() { return runTime; }
  double getEatRate() { return eatRate; }
  double getVelocity() { return velocity; }
  double getTotalResources() { return totalResources; }
  double getIntegralT() { return integralT; }
  double getTemperature() { return temperature; }
  double getResourceDiffusion() { return resourceDiffusion; }
  double getCurrentResources(int i);
  bool getPerturb() { return perturb; }
  int getStartSampling() { return startSampling; }
  int getSIters() { return sIters; }
  int getPNumber() { return pNumber; }
  int getGNumber() { return gNumber; }
  vector<double> getPEatRec() { return pEatRec; }
  vector<double> getGEatRec() { return gEatRec; }
  vector<double> getIndividualEatRec(int i);
  vector<double> getNormPEatRec();
  vector<double> getNormGEatRec();
  vector<vect<> > getInitPos(int num);
  vector<vector<vect<>>>& getPathRec(int i) { return pathRecord.at(i); }
  vector<vector<double>> getPathRecX(int i);
  vector<vector<double>> getPathRecY(int i);
  vector<double> getPathRecX(int i, int tstep);
  vector<double> getPathRecY(int i, int tstep);
  vector<double> getAvePathDifference();
  vector<double> getAvePathCmp();
  vect<> getPInitialPos(int i) { return pAgents[i]; }
  vect<> getGInitialPos(int i) { return gAgents[i]; }
  ResType getResType() { return resType; }
  WeightType getWeightType() { return weightType; }

  /// Mutators
  void setFPS(int f) { recFPS = f; }
  void setSIters(int s) { sIters = max(s,1); }
  void setStartSampling(int s) { startSampling = s; }
  void setGradientInit(bool g) { gradientInit = g; }
  void setDiffusion(double d) { resourceDiffusion = d; }
  void setEatRate(double e) { eatRate = e; }
  void setPredictivity(double);
  void setRecAgents(bool r) { recAgents = r; }
  void setEpsilon(double);
  void setVelocity(double v) { velocity = v; }
  void setPerturb(bool p) { perturb = p; }
  void setTemperature(double t) { temperature = t; }
  void setResType(ResType r) { resType = r; }
  void setWeightType(WeightType w) { weightType = w; }
  void setWrap(bool);
  void setNFTSamples(int s) { nftSamples = s; }
  void setNSamples(int s) { nSamples = s; }
  void srand() { srand48(time(0)); }

  /// Display functions
  string printAgents();
  string printAnimationCommand();
  string printResource();
  string printInitResource();
  string printResourceAnimation();
  string printTrajectory();
  string printTrajectoryAnimation();
  string printPathAnimation();
  string printGradient();
  string printGradientAnimation();

 private:
  /// Helper functions
  inline void initializeFields();
  inline void initializeDFT();
  inline void setResource(Field&);
  inline void initializeFuture();
  inline void findTrajectory();
  inline vect<> distanceT(int, int, int, int); // Toroidal distance vector (space)
  inline vect<> distanceS(int, int, int, int); // Non-toroidal distance
  inline vect<> displacement(vect<>, vect<>); // Vector displacement
  inline double weight(vect<>, double, int, int, int);   // Weight for integration in trajectory finding
  inline void resetAgents();
  inline void updateAgents();
  inline void updateResource();
  inline void record();
  inline void dftRecord();
  inline void createDFTRec();
  inline void KIB(vect<>&); // Keep In Bounds

  /// Perlin noise
  inline void generateSmoothNoise(int, int, double, Field&);
  inline void addFrequency(int, double, Field&);
  inline double cosineInterpolate(double, double, double);
  inline double interpolate(double, double, double, double, double, double);

  /// Member variables
  Field *resource;     // The resource prediction as a function of time)
  Field diffusion;     // The field used to compute diffusion
  double resourceDiffusion, replenish; // Diffusion constant and source term of the resource
  double eatRate;      // The eating rate of the agents
  bool gradientInit;   // Whether we initialize the resource as if it was eaten by gradient agents or not
  VField trajectory;   // The trajectory agents should take as a function of position
  VField gradField;    // The gradient field of the resource
  ResType resType;     // What type of resource we should initialize
  bool wrap;           // Whether the space is a torus or a square
  int dX, dY;          // Field discretization
  double left, right, bottom, top; // Physical bounds
  double deltaX, deltaY; // Physical space per x and y bin
  double epsilon;      // Temporal step
  int tIters, t_iter;  // How many temporal iterations there are and what is the current one
  int sIters, s_iter;  // How many solution iterations we should try and what is the current one
  double integralT;    // Only integrate over time points at this rate (difference between time points we integrate over) 
  int t_step;          // Only integrate over every [t_step]th time slice (to save time)
  double runTime;      // How long it took the simulation to actually run
  double predictivity; // How far agents can look into the future
  int p_iters;         // How many iterations the predictivity corresponds to
  double velocity;     // How fast agents move
  bool perturb;        // Whether we perturb based on temperature
  double temperature;  // Magnitude of random perturbations
  WeightType weightType; // How we weigh the resource

  /// Noisy resource generation
  int grains, maxGrain;
  double noiseFactor;

  /// Agents
  vect<> *pAgents, *pInitAgents; // Predictive agents current and initial positions
  vect<> *gAgents, *gInitAgents; // Gradient agents current and initial positions
  vector<double> pEaten, gEaten;
  int pNumber, gNumber;

  // Sectorized eating
  Field eaten;
  
  /// Recording
  bool recAgents;      // Whether we should record agent positions or not
  string pAgentPosStr; // Record the positions of the predictive agents the final time
  string gAgentPosStr; // Record the positions of the gradient agents the final time
  string trajectoryStr;// Record the trajectory solution
  string gradStr;      // Record the gradient
  int lastRecIt, recDelay; // Last iter we recorded at and how many iters b/w recordings
  int recIt;           // How many times we have recorded data
  int recFPS;          // FPS

  /// Eating record
  vector<double> pEatRec, gEatRec;
  double totalResources;

  /// Individual agent eat record
  vector<vector<double> > eatRecord; // [ Particle # ] [ sIter ]
  int nSamples; // How many agents should be tracked for individual eat records
  /// Path record (for DFT)
  vector<vector<vector<vect<> > > > pathRecord; // [ Particle # ] [ sIter ] [ tIter ] (position)
  int nftSamples;       // Number of paths we sample for the fourier transform
  int startSampling;    // What solution iteration to start sampling on
};

#endif
