#include "Prediction.h"

Prediction::Prediction(int dx, int dy, double ep, int pNumber, int gNumber) : dX(dx), dY(dy), epsilon(ep), pNumber(pNumber), gNumber(gNumber) {
  // Set bounds
  left = bottom = 0;
  top = right = 1;
  deltaX = 1./dx; //** This is actually correct since the bdds are wrapped
  deltaY = 1./dy; //** Same here.
  // Set constants
  resourceDiffusion = 0.01;
  replenish = 0;
  eatRate = 1;
  gradientInit = true;
  resType = TWOPEAKS;
  wrap = true;
  tIters = 0;
  t_iter = 0;
  sIters = 10;
  s_iter = 0;
  runTime = 0;
  predictivity = 0;
  integralT = 0.01;
  t_step = max((int)(integralT/epsilon),1);
  p_iters = 0;
  velocity = 1.;
  perturb = false;
  temperature = 5;
  weightType = INVRSQR;
  grains = 4;
  maxGrain = 16;
  noiseFactor = 1.1;
  lastRecIt = 0;
  recDelay = 1;
  recFPS = 60;
  recIt = 0;
  nftSamples = 0;
  startSampling = 15;
  // Initialize agents
  pAgents = pInitAgents = gAgents = gInitAgents = 0;
  if (pNumber>0) {
    pAgents = new vect<>[pNumber];
    pInitAgents = new vect<>[pNumber];
  }
  if (gNumber>0) {
    gAgents = new vect<>[gNumber];
    gInitAgents = new vect<>[gNumber];
  }
  // Uniformly distribute agents
  for (int i=0; i<pNumber; i++) {
    vect<> pos(left+drand48()*(right-left), bottom+drand48()*(top-bottom));
    pAgents[i] = pos;
    pInitAgents[i] = pos;
  }
  for (int i=0; i<gNumber; i++) {
    vect<> pos(left+drand48()*(right-left), bottom+drand48()*(top-bottom));
    gAgents[i] = pos;
    gInitAgents[i] = pos;
  }
}

Prediction::~Prediction() {
  if (pAgents) delete [] pAgents;
  if (pInitAgents) delete [] pInitAgents;
  if (gAgents) delete [] gAgents;
  if (gInitAgents) delete [] gInitAgents;
}

void Prediction::setEpsilon(double dt) {
  epsilon = dt;
  t_step = max((int)(integralT/dt),1);
}

void Prediction::setWrap(bool w) {
  wrap = w;
  diffusion.setWrap(w,w);
  trajectory.setWrap(w,w);
  gradField.setWrap(w,w);
}

string Prediction::printAgents() {
  if (!recAgents) return "";
  if (!pAgentPosStr.empty()) pAgentPosStr.pop_back();
  if (!gAgentPosStr.empty()) gAgentPosStr.pop_back();
  string str = "predPos={"+pAgentPosStr+"};\ngradPos={"+gAgentPosStr+"};\n";
  pAgentPosStr.push_back(',');
  gAgentPosStr.push_back(',');
  return str;
}

string Prediction::printAnimationCommand() {
  if (!recAgents) return "";
  stringstream stream;
  string str = "R=0.005;\n", str1;
  stream << "agentFrames=Table[Show[Graphics[Table[{Black,Circle[predPos[[j]][[i]],R]},{i,1,Length[predPos[[j]]]}]],Graphics[Table[{Red,Circle[gradPos[[j]][[i]],R]},{i,1,Length[gradPos[[j]]]}]],PlotRange->{{" << left << "," << right << "},{" << bottom << "," << top << "}}],{j,1,Length[predPos]}];";
  stream >> str1;
  str += (str1+"\nListAnimate[agentFrames]");
  return str;
}

string Prediction::printResource() {
  int times = epsilon*tIters*recFPS;
  int step = 1./(recFPS*epsilon);

  string str;
  for (int i=0; i<times; i++) {
    str += resource[i*step].print();
    if (i!=times-1) str += ',';
  }
  return "res={"+str+"};";
}

string Prediction::printInitResource() {
  return "initRes="+resource[0].print()+";";
}

string Prediction::printResourceAnimation() {
  return "resFrames=Table[MatrixPlot[res[[i]]],{i,1,Length[res]}];\nListAnimate[resFrames]";
}

string Prediction::printTrajectory() {
  if (!trajectoryStr.empty()) trajectoryStr.pop_back();
  else return "";
  string str = "traj={"+trajectoryStr+"};\n";
  trajectoryStr += ',';
  return str;
}

string Prediction::printTrajectoryAnimation() {
  if (trajectoryStr.empty()) return "";
  return "trajFrames=Table[ListVectorPlot[traj[[i]]],{i,1,Length[traj]}];\nListAnimate[trajFrames]";
}

string Prediction::printPathAnimation() {
  stringstream stream;
  string str;
  stream << "traj={";
  for (int i=0; i<nftSamples; i++) {
    stream << getPathRec(i);
    if (i!=nftSamples-1) stream << ",";
  }
  stream << "};";
  stream >> str;
  str += "dist[v_,w_]=Sqrt[(v-w).(v-w)];\n";
  str += "join[list_]:=Table[If[dist[list[[i]],list[[i-1]]]<0.5,Line[ {list[[i]], list[[i-1]]}],],{i,2,Length[list]}];";
  str += "\ncolors=Table[RGBColor[RandomReal[],RandomReal[],RandomReal[]],{i,1,Length[traj]}];\npaths = Table[Show[Table[Graphics[{{colors[[n]],join[traj[[n]][[t]]]},{colors[[n]],Disk[traj[[n]][[t]][[1]],0.01]}},PlotRange->{{0,1},{0,1}},ImageSize->Large],{n,1,Length[traj]}]], {t,1,Length[traj[[1]]]}];\n";
  str += "vid=ListAnimate[paths,1];";
  // To remove the slider, use " ListAnimate[paths,1]/.HoldPattern[AppearanceElements->_]->(AppearanceElements->None) "
  return str;
}

string Prediction::printGradient() {
  if (!gradStr.empty()) gradStr.pop_back();
  else return "";
  string str = "grad={"+gradStr+"};\n";
  gradStr += ',';
  return str;
}

string Prediction::printGradientAnimation() {
  if (gradStr.empty()) return "";
  return "gradFrames=Table[ListVectorPlot[grad[[i]]],{i,1,Length[grad]}];\nListAnimate[gradFrames]";
}

void Prediction::run(double time) {
  // Set up parameters
  tIters = time/epsilon;
  recDelay = 1./recFPS/epsilon;
  if (pNumber==0) sIters = 1; // Only need one iteration if we only have gradient agents
  nSamples = nSamples<0 ? 0 : nSamples; // Don't want to cause an error
  pEatRec = vector<double>(sIters+1, 0);
  gEatRec = vector<double>(sIters+1, 0);
  // Set up fields
  initializeFields();
  // Make sure there are actually agents to simulate
  if (gNumber==0 && pNumber==0) return;
  if (sIters<startSampling && 0<nftSamples) startSampling=0;
  if (nftSamples>pNumber) nftSamples = pNumber;
  if (nftSamples>0) initializeDFT();
  // Clear old records
  pEaten.clear(); gEaten.clear(); // Reset consumption records
  eatRecord.clear();
  // Create eatRecord vector
  eatRecord = vector<vector<double> >(nSamples);
  for (int i=0; i<nSamples; i++) eatRecord.at(i) = vector<double>(sIters);
  // Run simulation
  auto start_t = clock();
  if (gradientInit) initializeFuture();
  for (s_iter=0; s_iter<sIters; s_iter++) {
    resetAgents();
    lastRecIt = 0;
    recIt = 0;
    createDFTRec(); // Create a dft record for this iteration
    if (s_iter==sIters-1) record(); // Initial record
    else dftRecord();
    lastRecIt = 0;
    for (t_iter=1; t_iter<tIters; t_iter++) {
      updateResource(); // Copy the [t_iter-1]th resouce to the [t_iter] spot, then have the agents eat from it
      findTrajectory(); // Agents are at their [t_iter-1]th position, calculate their trajectory to bring them to their [t_iter]th position
      updateAgents();   // Move agents to their [t_iter]th position using the trajectory
      // Record
      if (s_iter==sIters-1 && t_iter-lastRecIt>recDelay) record();
      if (s_iter!=sIters-1 && t_iter-lastRecIt>recDelay) dftRecord();
    }
  }
  auto end_t = clock();
  runTime = (double)(end_t-start_t)/CLOCKS_PER_SEC;
}

double Prediction::getCurrentResources(int time_iter) {
  if (time_iter<0 || time_iter>tIters) return 0;
  double total = 0;
  for (int y=0; y<dY; y++)
    for (int x=0; x<dX; x++)
      total += resource[time_iter].at(x,y);
  return total;
}

vector<double> Prediction::getIndividualEatRec(int i) {
  if (eatRecord.size()<=i) return vector<double>();
  vector<double> nr;
  for (auto r : eatRecord.at(i)) nr.push_back(r*(pNumber+gNumber)/totalResources);
  return nr;
}

vector<double> Prediction::getNormPEatRec() {
  if (pNumber==0) return vector<double>(sIters, 0);
  vector<double> nr;
  for (auto r : pEatRec) nr.push_back(r*(pNumber+gNumber)/(totalResources*pNumber));
  return nr;
}

vector<double> Prediction::getNormGEatRec() {
  if (gNumber==0) return vector<double>(sIters, 0);
  vector<double> nr;
  for (auto r : gEatRec) nr.push_back(r*(pNumber+gNumber)/(totalResources*gNumber));
  return nr;
}

vector<vect<>> Prediction::getInitPos(int num) {
  vector<vect<>> lst;
  for (int i=0; i<num; i++) lst.push_back(pInitAgents[i]);
  return lst;
}

vector<vector<double>> Prediction::getPathRecX(int n) {
  vector<vector<double>> rec;
  for (int i=0; i<pathRecord.at(n).size(); i++) {
    rec.push_back(vector<double>());
    for (int j=0; j<pathRecord.at(n).at(i).size(); j++)
      rec.at(i).push_back(pathRecord.at(n).at(i).at(j).x);
  }
  return rec;
}

vector<vector<double>> Prediction::getPathRecY(int n) {
  vector<vector<double>> rec;
  for (int i=0;i<pathRecord.at(n).size(); i++) {
    rec.push_back(vector<double>());
    for(int j=0; j<pathRecord.at(n).at(i).size(); j++)
      rec.at(i).push_back(pathRecord.at(n).at(i).at(j).y);
  }
  return rec;
}

vector<double> Prediction::getPathRecX(int n, int tstep) {
  if (pathRecord.empty()) return vector<double>();
  vector<double> rec;
  for (int i=0;i<pathRecord.at(n).size(); i++)
    if (pathRecord.at(n).at(i).size()>tstep) 
      rec.push_back(pathRecord.at(n).at(i).at(tstep).x);
  return rec;
}

vector<double> Prediction::getPathRecY(int n, int tstep) {
  if (pathRecord.empty()) return vector<double>();
  vector<double> rec;
  for (int i=0;i<pathRecord.at(n).size(); i++)
    if (pathRecord.at(n).at(i).size()>tstep)
      rec.push_back(pathRecord.at(n).at(i).at(tstep).y);
  return rec;
}

vector<double> Prediction::getAvePathDifference() {
  if (pathRecord.empty()) return vector<double>();
  vector<double> pathDiff;
  double norm = 1./(pathRecord.size()*recIt);

  for (int s=1; s<sIters-startSampling; s++) { // Solution iteration
    double aveDiff = 0;
    for (int n=0; n<pathRecord.size(); ++n) { // Agents
      double diff = 0;
      // Time iteration, L2 difference b/w paths
      for (int t=0; t<recIt; ++t) diff += sqr(displacement(pathRecord.at(n).at(s).at(t),pathRecord.at(n).at(s-1).at(t)));
      aveDiff += diff;
    }
    pathDiff.push_back(sqrt(aveDiff*norm));
  }
  return pathDiff;
}

vector<double> Prediction::getAvePathCmp() {
  if (pathRecord.empty()) return vector<double>();
  vector<double> pathDiff;
  double norm = 1./(pathRecord.size()*recIt);
  int last=sIters-startSampling-1; // The last iteration
  for (int s=0; s<sIters-startSampling; s++) { // Solution iteration
    double aveDiff = 0;
    for (int n=0; n<pathRecord.size(); ++n) { // Agents
      double diff = 0;
      // Time iteration, L2 difference b/w paths
      for (int t=0; t<recIt; ++t) diff += sqr(displacement(pathRecord.at(n).at(last).at(t), pathRecord.at(n).at(s).at(t)));
      aveDiff += diff;
    }
    pathDiff.push_back(sqrt(aveDiff*norm));
  }
  return pathDiff;
}

void Prediction::setPredictivity(double p) {
  predictivity = p;
  p_iters = (int)(p/epsilon);
}

void Prediction::initializeFields() {
  resource = new Field[tIters];
  for (int i=0;i<tIters; i++) {
    resource[i].setDims(dX, dY);
    resource[i].setBounds(left, right, bottom, top);
    // Initialize resources
    setResource(resource[i]);
  }
  // Set diffusion field
  diffusion.setDims(dX, dY);
  diffusion.setBounds(left, right, bottom, top);
  diffusion.setWrap(wrap, wrap);
  // Set trajectory field
  trajectory.setDims(dX,dY);
  trajectory.setBounds(left, right, bottom, top);
  trajectory.setWrap(wrap, wrap);
  // Set gradient field
  gradField.setDims(dX, dY);
  gradField.setBounds(left, right, bottom, top);
  gradField.setWrap(wrap, wrap);
}

void Prediction::initializeDFT() {
  pathRecord = vector<vector<vector<vect<>>>>(nftSamples, vector<vector<vect<>>>());
}

void Prediction::setResource(Field& res) {
  const double c1 = 12, c2 = 20;
  const double m1 = 50, m2 = 25;
  totalResources = 0;
  if (resType==NOISE) {
    generateSmoothNoise(grains, maxGrain, noiseFactor, res);
    // Integrate to find total resources
    totalResources = getCurrentResources(0);
    return;
  }
  for (int y=0; y<dY; y++)
    for (int x=0; x<dX; x++) {
      double R = 0;
      switch (resType) {
      default:
      case TWOPEAKS: {
	double X = x, Y = y;
	R = m1*expf(-sqr(c1*(x-0.5*dX)/dX)-sqr(c1*(y-0.5*dY)/dY)) + m2*expf(-sqr(c2*(x-0.25*dX)/dX)-sqr(c2*(y-0.5*dY)/dY));
	break;
      }
      case UNIFORM: {
	R = 10;
	break;
      }
      case PARABOLA: {
	double X = x, Y = y;
	R = 2-sqr(X/dX-0.5)-sqr(Y/dY-0.5);
	break;
      }
      }
      res.at(x,y) = R;
      totalResources += R;
    }
}

void Prediction::initializeFuture() {
  double peat = 0, geat = 0;
  int total = pNumber+gNumber;
  
  vect<> *agents = new vect<>[total];
  // Set initial positions
  int i;
  for (i=0; i<gNumber; i++) agents[i] = gInitAgents[i];
  for (int j=0; j<pNumber; j++) agents[i+j] = pInitAgents[j];
  double eaten = 0;

  // Simulate
  for (int iter=1; iter<tIters; iter++) {
    // Find trajectory (just follow the gradient)
    grad(resource[iter-1], gradField);
    // Update agents
    for (int i=0; i<total; i++) {
      vect<> pos = agents[i];
      vect<> normal = gradField(pos);
      normal.normalize();
      pos += velocity*epsilon*normal;
      if (perturb) pos += epsilon*temperature*randV();
      // Update position
      KIB(pos);
      agents[i] = pos;
    }    
    // Update resource
    resource[iter] = resource[iter-1];
    delSqr(resource[iter], diffusion); // Diffusion
    for (int y=0; y<dY; y++)
      for (int x=0; x<dX; x++) {
	double res = resource[iter].at(x,y);
	res += epsilon*(resourceDiffusion*diffusion.at(x,y) + replenish);
	res = res>0 ? res : 0;
	resource[iter].at(x,y) = res;
      }
    for (int i=0; i<total; i++) {
      int x = (int)((agents[i].x-left)/(right-left)*dX);
      int y = (int)((agents[i].y-bottom)/(top-bottom)*dY);
      double res = resource[iter].at(x,y);
      double eat = eatRate*res*epsilon;
      eaten += eat;
      res -= eat;
      resource[iter].at(x,y) = res;
    }
  }
  pEatRec.at(0) = eaten*(pNumber/(pNumber+gNumber));
  gEatRec.at(0) = eaten*(gNumber/(pNumber+gNumber));
  delete [] agents;
}

// Look from the [t_iter-1]th iteration into the future to find your next trajectory to bring you to your [t_iter]th position
void Prediction::findTrajectory() {
  // Predictive trajectory
  if (pNumber>0) {
    int iterEnd = min(tIters, t_iter+p_iters+1); // Always can see local (hence the +1)
    for (int y=0; y<dY; y++)
      for (int x=0; x<dX; x++) {
	vect<> traj;
	// Integrate over points we can see in time and space
	for (int i=t_iter; i<iterEnd; i+=t_step) {
	  double time = epsilon*(i-t_iter);
	  // At least see locally
	  double cutoffSqr = sqr(velocity*time + max(deltaX,deltaY));
	  for(int y2=0; y2<dY; y2++)
	    for (int x2=0; x2<dX; x2++) {
	      if (x==x2 && y==y2) continue;
	      vect<> dist = wrap ? distanceT(x, y, x2, y2) : distanceS(x, y, x2, y2);
	      if (sqr(dist)<=cutoffSqr)
		traj += weight(dist, time, x2, y2, i)*normalize(dist);
	    }
	}
	trajectory.at(x,y) = traj;
      }
  }

  // Find gradient trajectory
  if (gNumber>0) 
    grad(resource[t_iter-1], gradField); // Calculate the gradiend
}

// Points from (x2,y2) to (x1,y1) on a torus
vect<> Prediction::distanceT(int x1, int y1, int x2, int y2) {
  int A, B, s;
  if (x1<x2) { A=x1; B=x2; s=1; }
  else { A=x2; B=x1; s=-1; }

  int d1 = B-A, d2 = dX-B+A;
  double dx = d1<d2 ? d1 : -d2;
  dx *= s*deltaX;

  if (y1<y2) { A=y1; B=y2; s=1; }
  else { A=y2; B=y1; s=-1; }

  d1 = B-A; d2 =dX-B+A;
  double dy = d1<d2 ? d1 : -d2;
  dy *= s*deltaY; 

  return vect<>(dx,dy);
}

// Points from (x2,y2) to (x1,y1) on a square
inline vect<> Prediction::distanceS(int x1, int y1, int x2, int y2) {
  return vect<>(x2-x1, y2-y1);
}

inline vect<> Prediction::displacement(vect<> A, vect<> B) {
  // Get the correct (minimal) displacement vector pointing from B to A
  double X = A.x-B.x;
  double Y = A.y-B.y;
  if (wrap) {
    double dx = (right-left)-fabs(X);
    if (dx<fabs(X)) X = X>0 ? -dx : dx;
  }
  if (wrap) {
    double dy =(top-bottom)-fabs(Y);
    if (dy<fabs(Y)) Y = Y>0 ? -dy : dy;
  }
  return vect<>(X,Y);
}

inline double Prediction::weight(vect<> dist, double time, int x, int y, int iter) {
  // Power: R^-2
  switch (weightType) {
  case INVRSQR: // Inverse R^2
    return resource[iter].at(x,y)/(sqr(time)+sqr(dist));
  case INVR:    // Inverse R
    return resource[iter].at(x,y)/sqrt(sqr(time)+sqr(dist));
  case PROPR:   // Proportional to R
    return resource[iter].at(x,y)*sqrt(sqr(time)+sqr(dist));
  case PROPRSQR:// Proportional to R^2
    return resource[iter].at(x,y)*(sqr(time)+sqr(dist));
  default:
  case CONST: // Same weight for everything
    return resource[iter].at(x,y);
  }
}

inline void Prediction::resetAgents() {
  for (int i=0; i<pNumber; i++) pAgents[i] = pInitAgents[i];
  for (int i=0; i<gNumber; i++) gAgents[i] = gInitAgents[i];
}

inline void Prediction::updateAgents() {
  // Update predictive agents
  for (int i=0; i<pNumber; i++) {
    vect<> pos = pAgents[i];
    vect<> norm = trajectory(pos);
    norm.normalize(); // We have to do this here b/c of interpolation
    pos += velocity*epsilon*norm;
    // Wrap position
    if (perturb) pos += epsilon*temperature*randV();
    KIB(pos);
    // Update position
    pAgents[i] = pos;
  }
  // Update gradient agents (by calculating the gradient of the [t_iter-1]th field
  for (int i=0; i<gNumber; i++) {
    vect<> pos = gAgents[i];
    vect<> normal = gradField(pos);
    normal.normalize();
    pos += velocity*epsilon*normal;
    if (perturb) pos += epsilon*temperature*randV();
    KIB(pos);
    // Update position
    gAgents[i] = pos;
  }
}

inline void Prediction::updateResource() {
  resource[t_iter] = resource[t_iter-1];
  
  // Diffusion
  delSqr(resource[t_iter], diffusion);
  for (int y=0; y<dY; y++)
    for (int x=0; x<dX; x++) {
      double res = resource[t_iter-1].at(x,y);
      res += epsilon*(resourceDiffusion*diffusion.at(x,y) + replenish);
      res = res>0 ? res : 0;
      resource[t_iter].at(x,y) = res;
    }
  // We now have resource[t_iter] = resource[t_iter-1] + diffusion
  eaten = resource[t_iter];

  //-- MAYBE FIND A MORE EFFICIENT WAY TO DO THIS.

  // Predictors eat
  for (int i=0; i<pNumber; i++) {
    int x = (int)((pAgents[i].x-left)/(right-left)*dX);
    int y = (int)((pAgents[i].y-bottom)/(top-bottom)*dY);
    double res = resource[t_iter].at(x,y);
    double eat = eatRate*epsilon*res;
    // Record individual agents' eating per sInter
    if (i<nSamples) eatRecord.at(i).at(s_iter) += eat;
    // Update resource
    //res -= eat;
    eaten.at(x,y) -= eat;

    pEatRec.at(s_iter+1) += eat;
    //resource[t_iter].at(x,y) = res;
  }
  // Gradient agents eat
  for (int i=0; i<gNumber; i++) {
    int x = (int)((gAgents[i].x-left)/(right-left)*dX);
    int y = (int)((gAgents[i].y-bottom)/(top-bottom)*dY);
    double res = resource[t_iter].at(x,y);
    double eat = eatRate*res*epsilon;
    //res -= eat;
    eaten.at(x,y) -= eat;

    gEatRec.at(s_iter+1) += eat;
    //resource[t_iter].at(x,y) = res;
  }

  resource[t_iter] = eaten;
}

inline void Prediction::record() {
  // Record the final trajectory solution and agent movement
  lastRecIt = t_iter; // Reset counter
  // Record trajectory
  trajectoryStr += (trajectory.print()+",");
  gradStr += (gradField.print()+",");
  // Record predictive agents
  if (recAgents) {
    stringstream stream;
    string str;
    stream << '{';
    for (int i=0; i<pNumber; i++) {
      stream << pAgents[i];
      if (i!=pNumber-1) stream << ',';
    }
    stream << '}';
    stream >> str;
    pAgentPosStr += (str+',');
    stream.clear();
    // Record gradient agents
    stream << '{';
    for (int i=0; i<gNumber; i++) {
      stream << gAgents[i];
      if (i!=gNumber-1) stream << ',';
    }
    stream << '}';
    stream >> str;
    gAgentPosStr += (str+',');
  }
  // Record paths for DFT
  if (startSampling<=s_iter)
    for (int i=0; i<nftSamples; i++)
      pathRecord.at(i).at(s_iter-startSampling).push_back(pAgents[i]);
  // Increment recIt
  recIt++;
}

inline void Prediction::dftRecord() {
  if (s_iter<startSampling) return;
  lastRecIt = t_iter;
  for (int i=0; i<nftSamples; i++) {
    int s = pathRecord.at(i).size();
    pathRecord.at(i).at(s-1).push_back(pAgents[i]);
  }
  // Increment recIt
  recIt++;
}

inline void Prediction::createDFTRec() {
  if (s_iter<startSampling) return;
  for (int i=0; i<nftSamples; i++)
    pathRecord.at(i).push_back(vector<vect<>>());
}

inline void Prediction::KIB(vect<> &pos) {
  if (wrap) { // Wrap position
    if (pos.x<left) pos.x+=(right-left);
    else if (right<pos.x) pos.x-=(right-left);
    if (pos.y<bottom) pos.y+=(top-bottom);
    else if (top<pos.y) pos.y-=(top-bottom);
  }
  else { // Reflect position
    if (pos.x<left) pos.x=2*left-pos.x;
    else if (right<pos.x) pos.x=2*right-pos.x;
    if (pos.y<bottom) pos.y=2*bottom-pos.y;
    else if (top<pos.y) pos.y=2*top-pos.y;
  }
}

inline void Prediction::generateSmoothNoise(int grains, int maxGrain, double mult, Field &res) {
  res = 0; // Set resource to 0
  int min = 4; // Minimum grain size
  int G = maxGrain;
  float amp = 1000;
  for (int i = 0; i < grains && G >= min; i++) {
    addFrequency(G, amp, res);
    amp *= mult;
    G *= 0.5;
  }
}

inline void Prediction::addFrequency(int grainSize, double amp, Field &res) {
  int W = dX/grainSize, H = dY/grainSize;
  float *D = new float[W*H];
  float *array = new float[dX*dY];

  // Translate where the grid points are
  int transX = static_cast<int>(drand48()*dX);
  int transY = static_cast<int>(drand48()*dY);

  for(int x=0; x<W; x++)
    for (int y=0; y<H; y++)
      D[x+y*H] = amp*drand48();

  for(int x=0; x<dX; x++)
    for (int y=0; y<dY; y++) {
      int X = (x+transX)%dX, Y = (y+transY)%dY;
      if (x%grainSize==0 && y%grainSize==0)
        array[X+Y*dX] = D[(x/grainSize)+(y/grainSize)*H];
      else { // Wrapped b.c.
        int aX = x/grainSize, bX = (aX+1)%W;
        int aY = y/grainSize, bY = (aY+1)%H;
        float TL = D[aX+aY*H], TR = D[bX+aY*H];
        float BL = D[aX+bY*H], BR = D[bX+bY*H];
        float dy = (float)y/grainSize - y/grainSize;
        float dx = (float)x/grainSize - x/grainSize;
        array[X+Y*dX] = interpolate(TL, TR, BL, BR, dx, dy);
      }
    }
  for (int x=0; x<dX; x++)
    for (int y=0; y<dY; y++)
      res.at(x,y) += array[y*dX+x];
  delete [] D;
  delete [] array;
}

inline double Prediction::cosineInterpolate(double a, double b, double x) {
  float ft = x*PI;
  float f = (1 - cos(ft)) * 0.5;
  return a * (1 - f) + b * f;
}

inline double Prediction::interpolate(double TL, double TR, double BL, double BR, double dx, double dy) {
  float top = cosineInterpolate(TL, TR, dx), bottom = cosineInterpolate(BL, BR, dx);
  return cosineInterpolate(top, bottom, dy);
}
