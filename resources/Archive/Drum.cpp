#include <stdlib.h>
#include <math.h>

#include "Drum.h"

Drum::Drum(int i_index, double i_radius, double i_w)
{
    m_index = i_index;
    m_radius = i_radius;
    m_wx = i_w;
    m_wy = 0.;
    m_wz = 0.;
    m_theta = 0.;
}

Drum::~Drum()
{

}

int Drum::index()
{
    return m_index;
}

double Drum::radius()
{
    return m_radius;
}

double Drum::theta()
{
    return m_theta;
}

double Drum::wx()
{
    return m_wx;
}

double Drum::wy()
{
    return m_wy;
}

double Drum::wz()
{
    return m_wz;
}

void Drum::update_position(double i_dt)
{
    m_theta += m_wx*i_dt;
}
