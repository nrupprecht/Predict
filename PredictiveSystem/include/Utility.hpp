/*
 * Author: Nathaniel Rupprecht
 * Start Data: May 12, 2017
 *
 * -- NOTE: Modified for use with improved Predictive system simulator
 */

#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "Settings.hpp"

// Includes  
#include <iostream>
using std::cout;
using std::endl;

#include <ostream>
using std::ostream;

#include <fstream>
using std::ifstream;
using std::ofstream;

#include <sstream>
using std::stringstream;

#include <vector>
using std::vector;
using std::pair;

#include <list>
using std::list;
  
#include <tuple>
using std::tuple;

#include <map>

#include <set>

#include <algorithm>

#include <functional>

#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

#include <string>
using std::string;

#include <random>

#include <math.h>

#include <cstring> // For memset it mpi.h is not defined

namespace Predictive {

  // What type to use as our real type
  typedef double RealType;
  typedef pair<RealType, RealType> RPair;

  // What to use for Verlet list
  typedef vector<int>          VListSubType;
  typedef vector<VListSubType> VListType;

  // What to use for Wall list
  typedef vector<int>          WListSubType;
  typedef vector<WListSubType> WListType;

  // Constants
  const RealType PI = 3.14159265358979;

  // Squaring function
  template<typename T> inline T sqr(const T value) {
    return value*value;
  }

  // Min function
  template<typename T> inline T min(const T a, const T b) {
    return a<b ? a : b;
  }

  // Max function
  template<typename T> inline T max(const T a, const T b) {
    return a<b ? b : a;
  }

  // Clamp function
  inline RealType clamp(const RealType x) {
    return x<0 ? 0 : x;
  }

  inline RealType sign(const RealType x) {
    return x<0 ? -1. : 1.;
  }

  template<typename T> inline void swap(T& a, T& b) {
    T temp = a;
    a = b; b = temp;
  }

  template<typename T> inline T& Last(vector<T>& lst) {
    return lst.at(lst.size()-1);
  }

  template<typename T> inline T Average(const vector<T>& lst) {
    T ave = T();
    for (const auto &v : lst) ave += v;
    return (1./lst.size())*ave;
  }

  template<typename T> inline T Max(const vector<T>& lst) {
    if (lst.empty()) return T();
    T mx = *lst.begin();
    for(const auto &v : lst) 
      if (v>mx) mx = v;
    return mx;
  }

  /// Get the current time
  inline std::chrono::time_point<std::chrono::high_resolution_clock> current_time() {
    return high_resolution_clock::now();
  }

  // high_resolution_clock::time_point
  template<typename T> inline RealType time_span(T end, T start) {
    duration<RealType> span = duration_cast<duration<double>>(end-start);
    return span.count();
  }

  // Print as (hrs):(mins):(sec)
  inline string printAsTime(double seconds) {
    stringstream stream;
    string str;
    // Just print seconds as usual
    if (seconds<60) {
      stream << seconds;
      stream >> str;
      return str;
    }
    // Print full
    int hours = floor(seconds/3600.);
    seconds -= 3600*hours;
    int minutes = floor(seconds/60.);
    seconds -= 60*minutes;
    // Round seconds
    int sec = seconds;
    // Print hours
    if (hours==0);
    else if (hours<10) stream << "0" << hours << ":";
    else stream << hours << ":";
    // Print minutes
    if (minutes<10 && hours>0) stream << "0" << minutes << ":";
    else if (minutes<10) stream << minutes << ":";
    else stream << minutes << ":";
    // Print seconds
    if (seconds<10) stream << "0" << sec;
    else stream << sec;
    stream >> str;
    return str;
  }

  // PData records all the data you need to print particles:
  // { pos x, pos y, sigma, theta, interaction, 'color' }
  typedef std::tuple<RealType, RealType, RealType, RealType, RealType, RealType> PData;

  // Random number generators
  static std::mt19937 generator;
  static std::normal_distribution<RealType> normal_dist(0., 1.);
  // static std::poisson_distribution<int> poisson_dist();

  inline void seedNormalDistribution() {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::mt19937(seed);
  }

  inline double randNormal() {
    return normal_dist(generator);
  }
}

#endif // __UTILITY_HPP__
