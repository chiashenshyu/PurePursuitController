#include <iostream> 
#include <cmath> 
#include <vector>
#include <limits>


// #define VIZ
#ifdef VIZ
#include <cv.h> 
#include <highgui.h> 

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

double k   = 0.1; 
double Lfc = 2.0; 
double Kp  = 1.0; 
double dt  = 0.1; 
double L   = 2.9; 

int oldNearestPointIndex = -1; 

struct state{
    double x; 
    double y; 
    double theta; 
    double v; 
    double rearX; 
    double rearY; 
    state(){}; 
    state(double _x = 0, double _y = 0, double _theta = 0, double _v = 0)
    :x(_x), y(_y), theta(_theta), v(_v)
    {
        rearX = x - (L/2)*cos(theta); 
        rearY = y - (L/2)*sin(theta); 
    }
}; 

double mod2pi(double theta){
    return theta - 2*M_PI*floor(theta/2/M_PI); 
}

double calDistance(state st, double x, double y){
    double dx = st.rearX - x; 
    double dy = st.rearY - y;
    return sqrt(pow(dx,2)+pow(dy,2)); 
}

void update(state& st, double a, double delta){
    st.x = st.x + st.v * cos(st.theta) * dt; 
    st.y = st.y + st.v * sin(st.theta) * dt; 
    std::cout << L << std::endl;
    st.theta = st.theta + st.v / L * tan(delta) * dt; 
    st.theta = mod2pi(st.theta); 
    st.v = st.v + a * dt; 
    st.rearX = st.x - (L/2) * cos(st.theta); 
    st.rearY = st.y - (L/2) * sin(st.theta); 
}

double PIDControl(double target, double current){
    double a = Kp * (target - current); 
    return a; 
}

int calTargetIndex(state st, std::vector<double> cx, std::vector<double> cy){
    int res; 

    if(oldNearestPointIndex == -1){
        int n = cx.size(), index = -1;
        double minDist = std::numeric_limits<double>::max();  
        std::vector<double> dx(n), dy(n), d(n); 
        for(int i = 0; i < n; i++){
            dx[i] = st.x - cx[i]; 
            dy[i] = st.y - cy[i]; 
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
        double distanceThisIndex = calDistance(st, cx[res], cy[res]); 
        while(true){
            res = (res< cx.size()-1)? res + 1 : res; 
            double distanceNextIndex = calDistance(st, cx[res], cy[res]); 
            if(distanceThisIndex < distanceNextIndex) break; 
            distanceThisIndex = distanceNextIndex; 
        }
        oldNearestPointIndex = res; 
    }

    L = 0.0; 

    double Lf = k * st.v + Lfc; 

    // search lookahead target point index
    while(Lf > L && (res) < cx.size()){ // check 
        double dx = cx[res] - st.rearX; 
        double dy = cy[res] - st.rearY; 
        L = sqrt(pow(dx,2) + pow(dy,2)); 
        res++; 
    }
    return res; 
}

std::pair<double,int> purePursuitControl(state st,
                                       std::vector<double> cx,
                                       std::vector<double> cy, 
                                       int pIndex){
    int index = calTargetIndex(st, cx, cy);
    double tx, ty; 

    if(pIndex >= index) index = pIndex;
    if(index < cx.size() - 1){
        tx = cx[index]; 
        ty = cy[index]; 
    }else{
        tx = cx.back(); 
        ty = cy.back(); 
        index = cx.size()-1; 
    }

    double alpha = atan2(ty - st.rearY, tx - st.rearX) - st.theta; 
    alpha = mod2pi(alpha); 
    double Lf = k * st.v + Lfc; 
    
    double delta = atan2(2.0 * L * sin(alpha)/ Lf, 1.0); 
    delta = mod2pi(delta); 
    std::pair<double,int> res = std::make_pair(delta, index);

    return res; 
}


int main(){

    std::vector<double> cx, cy; 
    for(double i = 0; i <= 50; i += 0.1){
        cx.push_back(i); 
        cy.push_back(cos(cx.back() / 5.0) * cx.back() / 2.0); 
    }
    
    #ifdef VIZ
    drawTrajectory(cx, cy);
    #endif

    double targetSpeed = 10.0/3.6; 
    double T = 100.0; 

    state st(0, -3, 0, 0); 
    int lastIndex = cx.size()-1; 
    double time = 0.0; 
    std::vector<double> x = {st.x};
    std::vector<double> y = {st.y};
    std::vector<double> theta = {st.theta}; 
    std::vector<double> v = {st.v};
    std::vector<double> t = {0.0};

    int targetIndex = calTargetIndex(st, cx, cy); 

    while(T >= time && lastIndex > targetIndex){
        double ai = PIDControl(targetSpeed, st.v); 
        std::pair<double, int> p = purePursuitControl(st, cx, cy, targetIndex); 
        targetIndex = p.second; 
        update(st, ai, p.first); 
        time += dt; 

        x.push_back(st.x);
        y.push_back(st.y);
        v.push_back(st.v); 
        theta.push_back(st.theta);
        t.push_back(time);
        std::cout << st.x << " " << st.y << " " << st.theta << std::endl;

        #ifdef VIZ
        drawPoint(st.x, st.y);
        cv::imshow("ppc", map); 
        cv::waitKey(50); 
        #endif
    }

    #ifdef VIZ
    cv::imshow("ppc", map);
    cv::waitKey(1000000); 
    #endif 
    
    
    std::cout << "finish" << std::endl;
    return 0;
}
