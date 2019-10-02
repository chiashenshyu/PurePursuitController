#pragma once 
#include <iostream> 
#include <cmath> 
#include <vector>
#include <limits>
#include <random>
#include <eigen3/Eigen/Dense>

using namespace std; 

inline double mod2pi(double theta){
    return theta - 2*M_PI*floor(theta/2/M_PI); 
}

struct Path{
    vector<double> cx; 
    vector<double> cy; 
}; 

struct Point{
  double x;
  double y;
  Point(const double x_ = 0,const double y_ = 0){
    x = x_;
    y = y_;
  }

  void print() const{
    cout << "X, Y: " << x << ", " << y << endl;
  }
 
  Point operator-(const Point& A) const{
    return Point(x-A.x, y-A.y);
  }
 
  Point operator+(const Point& A) const{
    return Point(x+A.x, y+A.y);
  }
 
  Point operator*(const double a) const{
    return Point(x*a, y*a);
  }
  bool operator==(const Point& A) const{
   return (x == A.x && y == A.y);
  }
   
  void operator=(const Point& A){
   x = A.x;
   y = A.y;
  }
};// Inculde z maybe

typedef struct Planner_params{
    Point origin;
    Point goal;
    Eigen::MatrixXd obstacle;
    double iterations;
    int width;
    int height;
    int goalProx;
}planner_params;

typedef struct states{
    double x; 
    double y;
    double theta;
    double v;
    double theta_dot;
    double rearX; 
    double rearY; 
    bool operator==(const states& A) const{
        return (x == A.x && y == A.y);
    }
    bool operator!=(const states& A) const{
        return (x != A.x || y != A.y);
    }
    void operator=(const states& A){
        x = A.x;
        y = A.y;
        theta = A.theta;
        v = A.v;
        theta_dot = A.theta_dot;
        rearX = A.rearX; 
        rearY = A.rearY;
    }
    void SetCoord(Point& A){ // camelcase all func names
        x = A.x;
        y = A.y;
        theta = M_PI;
        v = 0;
        theta_dot = 0;
    }
    void NoiseState(const states& st, const double& nx, const double& ny){
        x = st.x + nx; 
        y = st.y + ny; 
    }
    void setNoise(const double noiseX, const double noiseY){
        x += noiseX;
        y += noiseY;
    }
    void RandomState(const double& Random){
        theta =  2*M_PI*Random;
        v = 0;
        theta_dot = 0;
    }
    double Cost(const states& q2){
        return (sqrt(pow((x-q2.x),2)+pow((y-q2.y),2)));
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
}States; // decide later on typdef bussiness 

typedef struct node{
    States state;
    double input;
    double cost;
    Point  parent; // look into whether this is needed with kd tree 

    Point GetCoord(){
        Point A(this->state.x,this->state.y);
        return A;
    }
    bool operator==(const node& A) const{
        return (state == A.state);
    }
    bool operator!=(const node& A) const{
        return (state != A.state);
        }
    void operator=(const node& A){
        state  = A.state;
        input  = A.input;
        cost   = A.cost;
        parent = A.parent;
    }
    void SetCoord(Point& A){
        this->state.SetCoord(A); // check if i can remove this->
    }
}Node;

inline void tfXy2Pixel(double& x, double& y, const int& width, const int& height){
    x += width/2; 
    y  = height/2 - y; 
} //why is inline needed on this 

inline double calDistNode(const Node& n1, const Node& n2){
    return sqrt(pow(n1.state.x-n2.state.x,2) + pow(n1.state.y-n2.state.y,2));
}

inline int Orientation(Point p,Point q,Point r){

    float val = (q.y - p.y) * (r.x - q.x) - 
                (q.x - p.x) * (r.y - q.y); 

    if (val == 0) return 0;  // colinear 

    return (val > 0)? 1: 2;
}

inline bool OnSegment(Point p, Point q, Point r){ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
        return true; 
    return false; 
} 
  
inline bool CollisionCheck(Node qa, Node qb, Eigen::MatrixXd& obstacle){
    int o1,o2,o3,o4;
    bool safe = true;
    for (int i=0;i<obstacle.rows();i++){
        Point p1(obstacle(i,0),obstacle(i,1));
        Point q1(obstacle(i,2),obstacle(i,3));

        o1 = Orientation(qb.GetCoord(),qa.GetCoord(),p1);
        o2 = Orientation(qb.GetCoord(),qa.GetCoord(),q1);
        o3 = Orientation(p1,q1,qb.GetCoord());     
        o4 = Orientation(p1,q1,qa.GetCoord());

        if (o1 != o2 && o3 != o4)
            return !safe ;
        if (o1 == 0 && OnSegment(qb.GetCoord(),p1,qa.GetCoord()))
            return !safe ;

        if (o2 == 0 && OnSegment(qb.GetCoord(),q1,qa.GetCoord()))
            return !safe ;

        if (o3 == 0 && OnSegment(p1,qb.GetCoord(),q1))
            return !safe ;

        if (o4 == 0 && OnSegment(p1,qa.GetCoord(),q1))
            return !safe ;
    }
    return safe;
} 

inline bool CollisionCheckPoint(Point q, Point p, Eigen::MatrixXd& obstacle){
    int o1,o2,o3,o4;
    bool safe = true;
    for (int i=0;i<obstacle.rows();i++){
        Point p1(obstacle(i,0),obstacle(i,1));
        Point q1(obstacle(i,2),obstacle(i,3));

        o1 = Orientation(p, q, p1);
        o2 = Orientation(p, q, q1);
        o3 = Orientation(p1, q1, p);     
        o4 = Orientation(p1, q1, q);

        if (o1 != o2 && o3 != o4)
            return !safe ;
        if (o1 == 0 && OnSegment(p, p1, q))
            return !safe ;

        if (o2 == 0 && OnSegment(p, q1, q))
            return !safe ;

        if (o3 == 0 && OnSegment(p1, p, q1))
            return !safe ;

        if (o4 == 0 && OnSegment(p1, q, q1))
            return !safe ;
    }
    return safe;
} 
