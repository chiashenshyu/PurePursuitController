#include "common.hpp" 
#include "ParticleFilter.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp; 


int main(){
    default_random_engine gen(time(0)); 
    normal_distribution<double> distribution(0,1); 
    int particleSize; 
    cout << "Input size of particles: " << endl;
    cin >> particleSize; 
    
    States car(0, 0, 0, 4); 
    States carDR = car; 
    ParticleFilter pf(particleSize);
    vector<double> params = {0.0, M_PI/30, 0.1, 2};
    vector<double> x, y, xDR, yDR, pfX, pfY; 
    x.push_back(car.x); 
    y.push_back(car.y); 
    xDR = x; 
    yDR = y; 
    pfX = x; 
    pfY = y; 

    // Landmark
    vector<vector<double>> landmark = {{ 20,  40},
                                       { 30,  10},
                                       {-10,  40},
                                       {  0, -15},
                                       {  0,   0},
                                       {-20, -15},
                                       {-40, -25}};

    bool init = false; 
    double time = 0, TIME = 25; 
    while(time < TIME){
        time += 0.1;
        if(!init){ // Initialized Particle Filter 
            init = true; 
            pf.priorUpdate(car, params); 
        }
        double mx, my; 
        car.update(params[0], params[1], params[2], params[3]);
        carDR.update(params[0] + distribution(gen)/5, params[1] + distribution(gen)*M_PI/36, params[2], params[3]);
        pf.priorUpdate(car, params);

        // pf.assignWeight(car);
        pf.observation(car, landmark);
        pf.assignWeightLandmark();
        
        pf.resample(); 
        pf.calAverage(mx, my); 

        x.push_back(car.x); 
        y.push_back(car.y);
        xDR.push_back(carDR.x); 
        yDR.push_back(carDR.y);
        pfX.push_back(mx); 
        pfY.push_back(my); 

        plt::clf(); 
        for(int i = 0; i < landmark.size(); i++){
            plotPoint(landmark[i][0], landmark[i][1], "c*");
            plotLine(landmark[i][0], landmark[i][1], car.x, car.y, "c-");
        }
        plt::named_plot("PF Localization", pfX, pfY, "b-"); 
        plt::named_plot("Traj", x, y, "r-"); 
        plt::named_plot("Dead Reckoning", xDR, yDR, "k-"); 
        plt::xlim(-50, 50); 
        plt::ylim(-35, 50);
        plt::legend();
        plt::title("Particle Filter Localization");
        plt::pause(0.001); 
    }
    plt::show();

    return 0;
}

