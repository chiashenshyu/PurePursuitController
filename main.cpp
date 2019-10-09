#include "ppc.hpp"
#include "ParticleFilter.hpp"

bool init = false; 

int main(){
    ParticleFilter pf(1500); 

    Path p; 
    for(double i = 0; i <= 50; i += 0.1){
        p.cx.push_back(i); 
        p.cy.push_back(cos(i / 5.0) * i / 2.0); 
    }

    double targetSpeed = 10.0/3.6; 
    double T = 100.0; 

    ppc car(0, -3, 0, 0); 
    ppc carWoLoc(0, -3, 0, 0); 

    int lastIndex = p.cx.size()-1, currentIndex = 0;
    int lastIndexWoLoc = lastIndex, currentIndexWoLoc = 0; 
    double mTime = 0.0; 
    std::vector<double> x = {car.st.x}, xWoLoc = x;
    std::vector<double> y = {car.st.y}, yWoLoc = y; 
    std::vector<double> theta = {car.st.theta}, thetaWoLoc = theta; 
    std::vector<double> v = {car.st.v}, vWoLoc = v; 
    std::vector<double> t = {mTime}; 

    while(T >= mTime && lastIndex > currentIndex && lastIndexWoLoc > currentIndexWoLoc){
        if(!init){
            init = true; 
            std::vector<double> nullV;
            pf.priorUpdate(car.st, nullV); 
        }

        std::vector<double> ret = car.implementPPC(p, targetSpeed, currentIndex); 
        
        std::default_random_engine generator(time(0));
        std::normal_distribution<double> distributionX(0, 0.5), distributionY(0, 0.5);
        double xMeas = carWoLoc.st.x + distributionX(generator);; /**/
        double yMeas = carWoLoc.st.y + distributionY(generator);; /**/
        std::vector<double> retWoLoc = carWoLoc.implementPPCWoLoc(p, targetSpeed, currentIndexWoLoc, xMeas, yMeas); 

        currentIndex = ret[0];
        currentIndexWoLoc = retWoLoc[0]; 
        mTime += car.dt; 
        
        std::vector<double> param(ret.begin()+1, ret.begin()+5);
        double mx, my; 
        pf.priorUpdate(car.st, param); 
        pf.calAverage(mx, my); 
        pf.assignWeight(car.st);
        pf.resample();
        
        double xAvg, yAvg; 
        pf.calAverage(xAvg, yAvg);
        car.st.x = xAvg; 
        car.st.y = yAvg; 

        x.push_back(car.st.x); 
        y.push_back(car.st.y); 
        v.push_back(car.st.theta); 
        theta.push_back(car.st.theta);
        
        xWoLoc.push_back(carWoLoc.st.x); 
        yWoLoc.push_back(carWoLoc.st.y); 
        vWoLoc.push_back(carWoLoc.st.theta); 
        thetaWoLoc.push_back(carWoLoc.st.theta);
        
        t.push_back(mTime);        

        #ifdef MATLIBPLOT
        plt::clf(); 
        // plt::figure_size(1200, 780); 
        // plt::xlim(-10, 60); 
        // plt::ylim(-25, 25); 
        plt::named_plot("path", p.cx, p.cy, "r-");
        plt::named_plot("Traj_wPF", x,  y,  "b*");
        plt::named_plot("Traj_woPF", xWoLoc, yWoLoc, "c*");

        // plt::named_plot("pfTraj", pfX, pfY, "co");
        plt::title("PurePursuitControl"); 
        plt::legend(); 
        plt::pause(0.001);
        #endif

    }
    // plt::show();
    std::cout << "finish" << std::endl;
    return 0;
}