#include "incl.h"
#include "ParticleFilter.h"
#include "Visualizer.h"
//using namespace Eigen;

int main(){
    // Matrix3f a = Matrix3f::Random(); 
    // std::cout << std::atan(1) * 180/M_PI << std::endl;

    /*
    float m_x;
    float m_y;
    float m_v;
    float m_beta;
    float m_headingAngle;

    float dt; 
    */

    //Initialize a car
    Model ff91(1);
    ff91.acc = 0, ff91.steeringAngle = M_PI/60; // Input parameters 
    ff91.setIC(300,300,5,0,0); 

    // Initialize particle filter    
    int _size, _step, iter = 0;
    cout << "Particles size: ";
    cin >> _size;
    cout << "Step: ";
    cin >> _step;
    cout << endl;

    _step = (_step == 0)? INT_MAX : _step;

    // Image
    bool imag = true;
    // Particle Filter
    ParticleFilter pf(_size, _step);
    Visualizer visualizer; 
    
    vector<float> estimation;
    // Point vehPos, estPos;

    while(imag && iter++ < pf.getStep()){

        estimation = pf.implement(ff91);
        
        // imag = visualizer.show(ff91, estimation);
        imag = visualizer.showSnake(ff91);
        /*
        Point vehPos, estPos;
        vehPos.x = ff91.getPos()[0];
        vehPos.y = ff91.getPos()[1];
        estPos.x = estimation[0];
        estPos.y = estimation[1];
        
        cv::circle(img,
                   cv::Point2f(estimation[0],estimation[1]),
                   1,
                   cv::Scalar(0,0,255),
                   -1,
                   CV_AA);
        cv::circle(img,
                   cv::Point2f(ff91.getPos()[0], ff91.getPos()[1]),
                   1,
                   cv::Scalar(255,0,0),
                   -1,
                   CV_AA);
        // cv::imshow("Traj", img);
        // cv::waitKey(50);
        */
    }
    waitKey(0);
}