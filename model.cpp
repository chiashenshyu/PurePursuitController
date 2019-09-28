#include "model.h"
using namespace std; 

/*
float x;
float y;
float v;
float acc; 
float beta;
float headingAngle;
float steeringAngle; 
*/

Model::Model()
: dt(0.1)
{
}

Model::Model(float deltatime)
: dt(deltatime)
{
}

void Model::setIC(const float x, const float y, const float v, const float headingAngle, const float beta)
{
    m_x = x;
    m_y = y; 
    m_v = v; 
    m_headingAngle = headingAngle;
    m_beta = beta; 
}

void Model::setIcNoise(const float noise)
{
    m_x += noise;
    m_y += noise;
    m_v; // += noise;
    m_headingAngle; // += noise;
    m_beta; // += noise;
}

void Model::move(const float acc, const float steeringAngle)
{
    m_x += m_v * cos(m_beta + m_headingAngle) * dt;
    m_y += m_v * sin(m_beta + m_headingAngle) * dt;
    m_headingAngle += m_v / REAR_LEN * sin(m_beta) * dt;
    m_v += acc * dt; 
    m_beta = atan(.5 * tan(steeringAngle));    
}

vector<float> Model::getPos() const
{
    vector<float> pos(2);
    pos[0] = m_x;
    pos[1] = m_y;
    return pos;
}

float Model::getVel() const 
{
    return m_v;
}

float Model::getAngle() const
{
    return m_beta + m_headingAngle; 
}