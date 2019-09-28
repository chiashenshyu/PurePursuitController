/*
Kinematic Bicycle Model 
*/
#ifndef __MODEL_H__
#define __MODEL_H__

#include "incl.h"
using namespace std;

const float FRONT_LEN = 2.5;
const float REAR_LEN = 2.5; 

static const float CAR_SIZE_MAX_X  = 3.86;
static const float CAR_SIZE_MIN_X  = -1.05;
static const float CAR_SIZE_MAX_Y  = 0.95;
static const float CAR_SIZE_MIN_Y  = -0.95;
static const float CAR_SIZE_WIDTH  = 1.9;
static const float CAR_SIZE_LENGTH = 5.0;

class Model{

private:

    float m_x;
    float m_y;
    float m_v;
    float m_beta;
    float m_headingAngle;
    float dt; 

public:
    Model(); 
    Model(float deltatime);

    float acc;
    float steeringAngle;

    void move(const float acc, const float steeringAngle);
    void setIC(const float x, const float y, const float v, const float headingAngle, const float beta);
    void setIcNoise(const float noise);
    vector<float> getPos() const;
    float getVel() const; 
    float getAngle() const;
};

#endif 