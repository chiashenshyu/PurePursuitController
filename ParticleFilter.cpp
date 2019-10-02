#include "ParticleFilter.hpp"

int ParticleFilter::m_cov = 100;

ParticleFilter::ParticleFilter(const int& numOfParticles)
:particlesArrSize(numOfParticles)
{
    init = false; 
    iter = 1;
    particlesArr.resize(particlesArrSize);
    weightArr.resize(particlesArrSize);
}

void ParticleFilter::calAverage(double& x, double& y){
    double x_tot = 0, y_tot = 0; 
    for(auto p : particlesArr){
        x_tot += p.x;
        y_tot += p.y;
    }
    x = x_tot / particlesArr.size();
    y = y_tot / particlesArr.size(); 
}

void ParticleFilter::priorUpdate(const States& n, const std::vector<double>& param){
    std::default_random_engine generator(time(0));
    std::normal_distribution<double> distributionX(0, 10), distributionY(0, 10); // (center, std)
    if(!init || param.empty()){
        init = true;
        for(int i = 0; i < particlesArrSize; i++){
            States a = n;
            a.NoiseState(n, distributionX(generator), distributionY(generator));
            particlesArr[i] = a; 
        }
    }else{
        iter = 0; 
        for(auto& p : particlesArr){
            p.update(param[0], param[1], param[2], param[3]);
        }
    }
}

void ParticleFilter::assignWeight(const States& car)
{
    totalWeight = 0; 
    double x1 = car.x, y1 = car.y, x2, y2, delta; 
    for(int i = 0; i < particlesArrSize; i++){
        x2 = particlesArr[i].x;
        y2 = particlesArr[i].y;
        delta = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
        weightArr[i] = 1 / sqrt(2 * M_PI * m_cov) * exp(-(pow(delta, 2) / (2 * m_cov))); 
        totalWeight += weightArr[i];
    }
}

void ParticleFilter::resample()
{
    std::default_random_engine generator(time(0)); 
    std::uniform_real_distribution<double> distribution(0.0, totalWeight);
    std::normal_distribution<double> d(0.0,1.0);
    std::vector<States> tmp;
    for(int i = 0; i < particlesArrSize; i++){
        double threshold = distribution(generator);
        double accu = 0; 
        int iter = 0; 
        while(accu < threshold){
            accu += weightArr[iter++];
        }
        States _p = (iter != 0)? particlesArr[--iter] : particlesArr[iter];
        _p.setNoise(d(generator), d(generator));
        tmp.push_back(_p);
    }
    particlesArr = tmp; 
}

// vector<double> ParticleFilter::implement(Model& car)
// {
//     if(!init){
//         priorUpdate(car, car.acc, car.steeringAngle);
//     }
//     //pf.priorUpdate(ff91, particles);
//     priorUpdate(car, car.acc, car.steeringAngle);
//     assignWeight(car);
//     resample();

//     double x, y; 
//     calAverage(x, y);
//     double err = sqrt(pow(x - car.getPos()[0], 2) + pow(y - car.getPos()[1], 2));

//     vector<double> estimation = {x, y, (double)err};
    
//     return estimation;
// }

// int ParticleFilter::getStep() const
// {
//     return m_step;
// }
