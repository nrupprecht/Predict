#include "Prediction.h"

int main(int argc, char** argv) {
  /// Parameters
  double epsilon = 0.001;
  double predictivity = 0.05;
  double temperature = 1.;
  double eatRate = 1;
  string label="";
  string wstring="";
  string rstring="";
  int sIters = 10;
  int nP = 5000;
  int nG = 0;
  int dim = 32;
  int fps = 40;
  int nftSamples = 25;
  int nSamples = 0;
  int startSampling = 0;
  int dispMark = -1;
  bool perturb = false;
  bool recAgents = false;
  bool wrap = true;
  bool gradInit = true;
  bool printRes = false;      // Print the resource as it changes
  bool printInitRes = false;  // Print the initial resource
  bool printTrajectoryField = false; // Prints the trajectory field vs time
  bool printPaths = false;    // Print paths for agents
  bool printFTData = false;   // Fourier transform data
  bool pathDiff = false;      // Print the average distance between paths of successive iterations
  bool pathCmp = false;       // Print the average distance between the first recorded path and the ith path
  bool printGrad = false;
  bool printInitPos = false;
  bool srnd = true;
  bool nc = false; // Don't print the consumption record
  WeightType weightType = INVRSQR;
  ResType resType = TWOPEAKS;

  //----------------------------------------
  // Parse command line arguments
  //----------------------------------------
  ArgParse parser(argc, argv);
  parser.get("epsilon", epsilon);
  parser.get("pred", predictivity);
  parser.get("temperature", temperature);
  parser.get("sIters", sIters);
  parser.get("eatRate", eatRate);
  parser.get("label", label);
  parser.get("weightType", wstring);
  parser.get("resType", rstring);
  parser.get("nP", nP);
  parser.get("nG", nG);
  parser.get("dim", dim);
  parser.get("fps", fps);
  parser.get("nftSamples", nftSamples);
  parser.get("nSamples", nSamples);
  parser.get("startSampling", startSampling);
  parser.get("dispMark", dispMark);
  parser.get("perturb", perturb);
  parser.get("recAgents", recAgents);
  parser.get("wrap", wrap);
  parser.get("gradInit", gradInit);
  parser.get("printRes", printRes);
  parser.get("printInitRes", printInitRes);
  parser.get("printTrajectoryField", printTrajectoryField);
  parser.get("printPaths", printPaths);
  parser.get("pathDiff", pathDiff);
  parser.get("pathCmp", pathCmp);
  parser.get("printGrad", printGrad);
  parser.get("printInitPos", printInitPos);
  parser.get("srand", srnd);
  parser.get("nc", nc);
  //----------------------------------------
  
  if (srnd) { // Seed random number generators
    srand48( time(0) );
    srand( time(0) );
  }

  // Set variables
  Prediction prediction(dim, dim, epsilon, nP, nG);
  prediction.setRecAgents(recAgents);
  prediction.setFPS(40);
  prediction.setWrap(wrap);
  prediction.setSIters(sIters);
  prediction.setEatRate(eatRate);
  prediction.setPredictivity(predictivity);
  prediction.setTemperature(temperature);
  prediction.setPerturb(perturb);
  prediction.setGradientInit(gradInit);
  prediction.setWeightType(weightType);
  prediction.setResType(resType);
  prediction.setNFTSamples(nftSamples);
  prediction.setNSamples(nSamples);
  prediction.setStartSampling(startSampling);
  if (wstring!="") prediction.setWeightType(makeWeightType(wstring));
  if (rstring!="") prediction.setResType(makeResType(rstring));

  //--- Print variables
  if (dispMark<=0) {
    cout << "----------------------- RUN SUMMARY -----------------------\n\n";
    cout << "Command: ";
    for (int i=0; i<argc; i++) cout << argv[i] << " ";
    cout << endl << endl; // Line break
    cout << "Epsilon: " << prediction.getEpsilon() << endl;
    cout << "Solution iters: " << prediction.getSIters() << ", Start sampling: " << prediction.getStartSampling() << "\n";
    cout << "Velocity: " << prediction.getVelocity() << "\n";
    cout << "Gradient Init: " << (gradInit ? "true" : "false") << "\n";
    cout << "Resource Type: " << print(prediction.getResType()) << ", Weight Type: " << print(prediction.getWeightType()) << "\n";
    if (nftSamples>0) cout << "Number of sampled agents: " << nftSamples << "\n";
    cout << "Eat Rate: " << prediction.getEatRate() << ", Resource Diffusion: " << prediction.getResourceDiffusion() << "\n";
    cout << "Dim: " << dim << ", Wrap: " << (wrap ? "true" : "false") << "\n";
    cout << "Temperature: " << prediction.getTemperature() << ", Perturb: " << (prediction.getPerturb() ? "true" : "false") << "\n";
    cout << endl;
    cout << "Predictivity: " << prediction.getPredictivity() << ", Integration time diff: " << prediction.getIntegralT() << "\n";
    cout << "P Agents: " << prediction.getPNumber() << ", G Agents: " << prediction.getGNumber() << '\n';
    cout << "...........................................................\n";
  }
  // Run
  prediction.srand();
  prediction.run(1.);
  // Print time summary
  if (dispMark<=0) {
    cout << "Run Time: " << prediction.getRunTime() << "s (" << printAsTime(prediction.getRunTime()) << ")\n";
    cout << "-----------------------------------------------------------\n\n";
  }
  
  //--- Agents
  if (recAgents) {
    cout << mmPreproc(prediction.printAgents()) << endl;
    cout << prediction.printAnimationCommand() << endl;
  }
  //--- Resource
  if (printRes) {
    cout << mmPreproc(prediction.printResource()) << endl;
    cout << prediction.printResourceAnimation() << endl;
  }
  if (printInitRes) {
    cout << mmPreproc(prediction.printInitResource()) << endl;
  }
  //--- Trajectory
  if (printTrajectoryField) {
    cout << mmPreproc(prediction.printTrajectory()) << endl;
    cout << prediction.printTrajectoryAnimation() << endl;
  }
  //--- Path difference
  if (pathDiff) {
    if (label=="-1") cout << "Print[\"Average Path Difference\"]\n";
    cout << "avePathDiff" << (label!="-1"?label:"") << "=" << prediction.getAvePathDifference() << ";\n";
    if (label=="-1") cout << "ListLinePlot[avePathDiff" << (label!="-1"?label:"") << ",ImageSize->Large,PlotStyle->Black,PlotRange->{0,1}]\n";
  }
  if (pathCmp) {
    if (label=="-1") cout << "Print[\"Path Cmp\"]\n";
    cout << "pathCmp" << (label!="-1"?label:"") << "=" << prediction.getAvePathCmp() << ";\n";
    if (label=="-1") cout << "ListLinePlot[pathCmp" << (label!="-1"?label:"") << ",ImageSize->Large,PlotStyle->Black,PlotRange->{0,1}]\n";
  }
  //--- Gradient
  if (printGrad) {
    cout << mmPreproc(prediction.printGradient()) << endl;
    cout << prediction.printGradientAnimation() << endl;
  }
  //--- nSamples
  if (nSamples>0) {
    cout << "indRec={";
    for (int i=0; i<nSamples; i++) {
      cout << prediction.getIndividualEatRec(i);
      if (i!=nSamples-1) cout << ",";
    }
    cout << "};\n";
  }
  //--- Trajectory samples
  if (printPaths) cout << prediction.printPathAnimation() << endl;
  //--- DFT samples
  if (printFTData) {
    int magicNumber = 17;
    cout << "dftX={";
    for (int i=0; i<nftSamples; i++) {
      auto vec = prediction.getPathRecX(i, magicNumber);
      cout << vec;
      if (i!=nftSamples-1) cout << ",";
    }
    cout << "};\n";
    cout << "dftY={";
    for (int i=0; i<nftSamples; i++) {
      auto vec = prediction.getPathRecY(i, magicNumber);
      cout << vec;
      if (i!=nftSamples-1) cout << ",";
    }
    cout << "};\n";
  }
  if (printInitPos) cout << "initPos=" << prediction.getInitPos(nftSamples) << ";";
  //--- Consumption
  if (!nc) {
    cout << "pEat" << (label!="-1"?label:"") << "=" << prediction.getNormPEatRec() << ";\n";
    cout << "gEat" << (label!="-1"?label:"") << "=" << prediction.getNormGEatRec() << ";\n";
  }
  if (dispMark<0) {
    cout << "top=Max[{Max[pEat],Max[gEat]}];\n";
    cout << "Print[\"Eating Records (Red: Predictors, Green: Gradient\"]\n";
    cout << "ListLinePlot[{pEat,gEat},PlotStyle->{Red,Green},PlotRange->{0,1.1*top},ImageSize->Large]\n";
  }
  return 0;
}
