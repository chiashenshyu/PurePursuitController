#include "ppc.hpp"
#include "ParticleFilter.hpp"

bool init = false; 

int main(){
    ParticleFilter pf(1500); 

    path p; 
    for(double i = 0; i <= 50; i += 0.1){
        p.cx.push_back(i); 
        p.cy.push_back(cos(i / 5.0) * i / 2.0); 
    }

    double targetSpeed = 10.0/3.6; 
    double T = 100.0; 

    ppc car(0, -3, 0, 0); 

    int lastIndex = p.cx.size()-1, currentIndex = 0; 
    double time = 0.0; 
    std::vector<double> x = {car.st.x};
    std::vector<double> y = {car.st.y}; 
    std::vector<double> theta = {car.st.theta}; 
    std::vector<double> v = {car.st.v}; 
    std::vector<double> t = {time}; 

    std::vector<double> pfX, pfY;

    while(T >= time && lastIndex > currentIndex){
        if(!init){
            init = true; 
            std::vector<double> nullV;
            pf.priorUpdate(car.st, nullV); 
        }

        std::vector<double> ret = car.implementPPC(p, targetSpeed, currentIndex); 
        currentIndex = ret[0];
        time += car.dt; 
        
        std::vector<double> param(ret.begin()+1, ret.begin()+5);
        pf.priorUpdate(car.st, param); 
        pf.assignWeight(car.st);
        pf.resample();
        double xAvg, yAvg; 
        pf.calAverage(xAvg, yAvg); 
        pfX.push_back(xAvg); 
        pfY.push_back(yAvg); 

        x.push_back(car.st.x); 
        y.push_back(car.st.y); 
        v.push_back(car.st.theta); 
        t.push_back(time); 
        theta.push_back(car.st.theta); 

        #ifdef MATLIBPLOT
        plt::clf(); 
        // plt::figure_size(1200, 780); 
        plt::xlim(-10, 60); 
        plt::ylim(-25, 25); 
        plt::named_plot("path", p.cx, p.cy, "r-");
        plt::named_plot("Traj", x,  y,  "b*");
        plt::named_plot("pfTraj", pfX, pfY, "co");
        plt::title("PurePursuitControl"); 
        plt::legend(); 
        plt::pause(0.001);
        #endif

        std::cout << xAvg << " " << yAvg << std::endl;
    }
    plt::show();
    std::cout << "finish" << std::endl;
    return 0;
}