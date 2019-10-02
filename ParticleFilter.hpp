#pragma once

#include "common.hpp"
// #include "model.h"


class ParticleFilter 
{
private:

    static int m_cov;

    int  step;
    int  iter;
    int  particlesArrSize;
    bool init;
    double limX; 
    double limY; 
    double totalWeight;
    std::vector<States>   particlesArr;   
    std::vector<double>  weightArr; 
    
public:

    ParticleFilter(const int& numOfParticles);
    void calAverage(double& x, double& y);
    void priorUpdate(const States& n, const std::vector<double>& param);
    void assignWeight(const States& n);
    void resample();
    // vector<float> implement();
    // int getStep() const; 
};
