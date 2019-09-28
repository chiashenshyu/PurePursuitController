#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include <iostream> 
#include <cmath> 
#include <random>
#include <vector>
#include <eigen3/Eigen/Core>
#include <ctime>

#include <cv.h>
#include <highgui.h> 

struct Point
{
    Point(cv::Point2f p)
        : x(p.x),
          y(p.y)
    {}
    
    Point()
        : x(0),
          y(0)
    {}

    float x; 
    float y; 
};

struct LineSegment
{
    LineSegment(Point _p1, Point _p2)
        : p1(_p1),
          p2(_p2)
    {}

    Point p1;
    Point p2;
};

inline Point rotateP(Point c, Point p, float angle)
{
    Point _p;

    _p.x = cos(angle) * (p.x - c.x) - sin(angle) * (p.y - c.y) + c.x;
    _p.y = sin(angle) * (p.x - c.x) + cos(angle) * (p.y - c.y) + c.y;
    
    return _p;
}

inline float calDist(Point p1, Point p2)
{
    return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2));
}

inline bool checkRecIntersect(cv::Point2f* vr, cv::Point2f* vc)
{
    Point p1(vc[2]);
    Point p2(vc[3]);
    Point center;
    for(int i = 0; i < 4; i++){
        center.x += vr[i].x;
        center.y += vr[i].y;
    }
    center.x /= 4;
    center.y /= 4;
    float dist;
    for(int i = 2; i < 4; i++){
        dist = calDist(center, Point(vc[i]));
        std::cout << dist << std::endl;
        if(dist < 7.08) return true;
    }
    std::cout << std::endl;
    return false;
}

#endif