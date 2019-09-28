#pragma once 
#include <iostream> 
#include <cmath> 
#include <vector>
#include <limits>
#include <random>

inline double mod2pi(double theta){
    return theta - 2*M_PI*floor(theta/2/M_PI); 
}

struct path{
    std::vector<double> cx; 
    std::vector<double> cy;
};

struct state{
    double x; 
    double y; 
    double theta; 
    double v; 
    double rearX; 
    double rearY; 
    state(){}; 
    void NoiseState(const state& st, const double& nx, const double& ny){
        x = st.x + nx; 
        y = st.y + ny; 
    }
    void operator=(const state& st){
        x = st.x; 
        y = st.y; 
        theta = st.theta; 
        v = st.v; 
        rearX = st.rearX; 
        rearY = st.rearY; 
    }
    void update(const double a, const double delta, const double dt, const double L){
        x +=  v * cos(theta) * dt; 
        y +=  v * sin(theta) * dt; 
        theta = theta + v / L * tan(delta) * dt; 
        theta = mod2pi(theta); 
        v = v + a * dt; 
        rearX = x - (L/2) * cos(theta); 
        rearY = y - (L/2) * sin(theta); 
    }
    void setNoise(const double noiseX, const double noiseY){
        x += noiseX; 
        y += noiseY; 
    }
};


