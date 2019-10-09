#include "ppc.hpp"


#ifdef OPENCV_VIZ

int cols = 1000, rows = 1000; 
double factor = 10.0; 
cv::Mat map = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3); 

void tfXy2Pixel(double& x, double& y, const int& width, const int& height){
    x *= factor; 
    y *= factor; 
    x += width/2;
    y = height/2 - y;
}

void drawTrajectory(std::vector<double>& cx, std::vector<double>& cy){
    int n = cx.size(); 
    for(int i = 0; i < n; i++){
        double x = cx[i], y = cy[i]; 
        tfXy2Pixel(x, y, cols, rows); 
        cv::circle(map,
                   cv::Point2d(x,y),
                   3, 
                   cv::Scalar(255,0,0),
                   -1,
                   CV_AA);  
    }
}

void drawPoint(double x, double y){
    tfXy2Pixel(x, y, cols, rows); 
    cv::circle(map,
               cv::Point2d(x,y),
               2, 
               cv::Scalar(255,255,255),
               -1,
               CV_AA);  
}
#endif 

double ppc::k   = 0.1; 
double ppc::Lfc = 2.0; 
double ppc::Kp  = 1.0; 

ppc::ppc(double x, double y, double theta, double v){
    oldNearestPointIndex = -1; 
    L = 2.9;
    dt = 0.1;
    st.x = x; 
    st.y = y; 
    st.theta = theta; 
    st.v = v;
    st.rearX = x - (L/2)*cos(theta);
    st.rearY = y - (L/2)*sin(theta); 
}

double ppc::calDistance(const States& st, const double x, const double y) const{
    double dx = st.rearX - x; 
    double dy = st.rearY - y;
    return sqrt(pow(dx,2)+pow(dy,2)); 
}

// void ppc::update(double a, double delta){
//     st.x = st.x + st.v * cos(st.theta) * dt; 
//     st.y = st.y + st.v * sin(st.theta) * dt; 
//     st.theta = st.theta + st.v / L * tan(delta) * dt; 
//     st.theta = mod2pi(st.theta); 
//     st.v = st.v + a * dt; 
//     st.rearX = st.x - (L/2) * cos(st.theta); 
//     st.rearY = st.y - (L/2) * sin(st.theta); 
// }

double ppc::PIDControl(double target, double current) const{
    double a = Kp * (target - current); 
    return a; 
}

int ppc::calTargetIndex(const States& st, const Path& p){
    int res; 

    if(oldNearestPointIndex == -1){
        int n = p.cx.size(), index = -1;
        double minDist = std::numeric_limits<double>::max();  
        std::vector<double> dx(n), dy(n), d(n); 
        for(int i = 0; i < n; i++){
            dx[i] = st.x - p.cx[i]; 
            dy[i] = st.y - p.cy[i]; 
            d[i]  = sqrt(pow(dx[i],2) + pow(dy[i], 2));  
            if(d[i] < minDist){
                minDist = d[i]; 
                index = i; 
            }
        }
        res = index; 
        oldNearestPointIndex = res; 
    }else{
        res = oldNearestPointIndex; 
        double distanceThisIndex = calDistance(st, p.cx[res], p.cy[res]); 
        while(true){
            res = (res< p.cx.size()-1)? res + 1 : res; 
            double distanceNextIndex = calDistance(st, p.cx[res], p.cy[res]); 
            if(distanceThisIndex < distanceNextIndex) break; 
            distanceThisIndex = distanceNextIndex; 
        }
        oldNearestPointIndex = res; 
    }

    L = 0.0; 

    double Lf = k * st.v + Lfc; 

    // search lookahead target point index
    while(Lf > L && (res) < p.cx.size()){ // check 
        double dx = p.cx[res] - st.rearX; 
        double dy = p.cy[res] - st.rearY; 
        L = sqrt(pow(dx,2) + pow(dy,2)); 
        res++; 
    }
    return res; 
}

std::pair<double,int> ppc::purePursuitControl(const Path&  p,
                                              int pIndex){
    int index = calTargetIndex(st, p);
    double tx, ty; 

    if(pIndex >= index) index = pIndex;
    if(index < p.cx.size() - 1){
        tx = p.cx[index]; 
        ty = p.cy[index]; 
    }else{
        tx = p.cx.back(); 
        ty = p.cy.back(); 
        index = p.cx.size()-1; 
    }

    double alpha = atan2(ty - st.rearY, tx - st.rearX) - st.theta; 
    alpha = mod2pi(alpha); 
    double Lf = k * st.v + Lfc; 
    
    double delta = atan2(2.0 * L * sin(alpha)/ Lf, 1.0); 
    delta = mod2pi(delta); 
    std::pair<double,int> res = std::make_pair(delta, index);

    return res; 
}

std::vector<double> ppc::implementPPC(const Path& p, const double& targetSpeed, int currentIndex){
    int targetIndex = (oldNearestPointIndex < 0)? calTargetIndex(st, p) : currentIndex; 
    double ai = PIDControl(targetSpeed, st.v); 
    std::pair<double,int> pr = purePursuitControl(p, targetIndex);
    targetIndex = pr.second; 
    st.update(ai, pr.first, dt, L); 
    std::vector<double> ret = {static_cast<double>(pr.second), ai, pr.first, dt, L};
    return ret;
}

std::vector<double> ppc::implementPPCWoLoc(const Path& p, const double& targetSpeed, 
                                           int currentIndex, double measX, double measY){
    double x = st.x, y = st.y; 
    st.x = measX; st.y = measY; 
    int targetIndex = (oldNearestPointIndex < 0)? calTargetIndex(st, p) : currentIndex; 
    double ai = PIDControl(targetSpeed, st.v); 
    std::pair<double,int> pr = purePursuitControl(p, targetIndex);
    targetIndex = pr.second; 
    st.update(ai, pr.first, dt, L); 
    std::vector<double> ret = {static_cast<double>(pr.second), ai, pr.first, dt, L};
    return ret;
}

// int main(){

//     std::vector<double> cx, cy; 
//     for(double i = 0; i <= 50; i += 0.1){
//         cx.push_back(i); 
//         cy.push_back(cos(cx.back() / 5.0) * cx.back() / 2.0); 
//     }
//     Path p; 
//     p.cx = cx;
//     p.cy = cy;
    
//     // #ifdef OPENCV_VIZ
//     // drawTrajectory(p.cx, p.cy);
//     // #endif

//     double targetSpeed = 10.0/3.6; 
//     double T = 100.0; 

//     ppc car(0, -3, 0, 0); 

//     int lastIndex = cx.size()-1, currentIndex = 0; 
//     double time = 0.0; 
//     std::vector<double> x = {car.st.x};
//     std::vector<double> y = {car.st.y};
//     std::vector<double> theta = {car.st.theta}; 
//     std::vector<double> v = {car.st.v};
//     std::vector<double> t = {0.0};

//     while(T >= time && lastIndex > currentIndex){
//         std::pair<double, int> pr = car.implementPPC(p, targetSpeed, currentIndex);
//         currentIndex = pr.second; 
//         time += car.dt;
//         x.push_back(car.st.x);
//         y.push_back(car.st.y);
//         v.push_back(car.st.v); 
//         theta.push_back(car.st.theta);
//         t.push_back(time);
//         // std::cout << st.x << " " << st.y << " " << st.theta << std::endl;

//         #ifdef OPENCV_VIZ
//         drawPoint(st.x, st.y);
//         cv::imshow("ppc", map); 
//         cv::waitKey(50); 
//         #endif
//         #ifdef MATLIBPLOT
//         plt::clf(); 
//         // plt::figure_size(1200, 780); 
//         plt::xlim(-10, 60); 
//         plt::ylim(-25, 25); 
//         plt::named_plot("path", cx, cy, "r-");
//         plt::named_plot("Traj", x,  y,  "b*");
//         plt::title("PurePursuitControl"); 
//         plt::legend(); 
//         plt::pause(0.001);
//         #endif
//     }

//     // #ifdef OPENCV_VIZ
//     // cv::imshow("ppc", map);
//     // cv::waitKey(1000000); 
//     // #endif 
//     std::cout << "finish" << std::endl;
//     return 0;
// }

/*
* compile command:
*   g++ -std=c++11 -o a.o ppc.cpp -I/usr/include/python2.7 -lpython2.7
*/