#pragma once
#include "incl.hpp"

#define MATLIBPLOT
#ifdef MATLIBPLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp; 
#endif

// #define OPENCV_VIZ
#ifdef OPENCV_VIZ
#include <cv.h> 
#include <highgui.h> 
#endif 

class ppc{
public:
    
    state st;
    double dt;

    ppc(double x, double y, double theta, double v); 
    int implementPPC(const path& p, const double& targetSpeed, int currentIndex);
private:
    static double k; 
    static double Lfc; 
    static double Kp; 
    
    double L; 
    int oldNearestPointIndex; 
     

    double calDistance(const state& st, double x, double y) const; 
    double PIDControl(double target, double current) const;
    int calTargetIndex(const state& st, const path& p); 
    std::pair<double,int> purePursuitControl(const path&  p, 
                                             int pIndex);

};
