#pragma once
#include "common.hpp"

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
    
    States st;
    double dt;

    ppc(double x, double y, double theta, double v); 
    std::vector<double> implementPPC(const Path& p, const double& targetSpeed, int currentIndex);
    std::vector<double> implementPPCWoLoc(const Path& p, const double& targetSpeed, 
                                          int currentIndex, double measX, double measY);
private:
    static double k; 
    static double Lfc; 
    static double Kp; 
    
    double L; 
    int oldNearestPointIndex; 
     

    double calDistance(const States& st, double x, double y) const; 
    double PIDControl(double target, double current) const;
    int calTargetIndex(const States& st, const Path& p); 
    std::pair<double,int> purePursuitControl(const Path&  p, 
                                             int pIndex);

};
