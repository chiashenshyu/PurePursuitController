#pragma once

#include "incl.hpp"
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
    std::vector<state>   particlesArr;   
    std::vector<double>  weightArr; 
    
public:

    ParticleFilter(const int& numOfParticles);
    void calAverage(double& x, double& y);
    void priorUpdate(const state& n, const std::vector<double>& param);
    void assignWeight(const state& n);
    void resample();
    // vector<float> implement();
    // int getStep() const; 
};
