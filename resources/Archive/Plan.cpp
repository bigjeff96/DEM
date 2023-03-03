#include <stdlib.h>
#include <math.h>

#include "Plan.h"

Plan::Plan(int i_index, double i_x, double i_y, double i_z, double i_nx, double i_ny, double i_nz, double i_amplx, double i_amply, double i_amplz, double i_freqx, double i_freqy, double i_freqz, double i_wx)
{
    m_index = i_index;
    double norm=sqrt(i_nx*i_nx+i_ny*i_ny+i_nz*i_nz);
    m_x0 = i_x;
    m_y0 = i_y;
    m_z0 = i_z;
    m_x = m_x0;
    m_y = m_y0;
    m_z = m_z0;
    m_nx = i_nx/norm;
    m_ny = i_ny/norm;
    m_nz = i_nz/norm;
    m_amplx = i_amplx;
    m_amply = i_amply;
    m_amplz = i_amplz;
    m_omegax = 2.*M_PI*i_freqx;
    m_omegay = 2.*M_PI*i_freqy;
    m_omegaz = 2.*M_PI*i_freqz;
    m_vx = m_amplx*m_omegax;
    m_vy = m_amply*m_omegay;
    m_vz = m_amplz*m_omegaz;
    m_wx = i_wx;
    m_wy = 0.;
    m_wz = 0.;
    m_time = 0.;
}

Plan::~Plan()
{
    
}

void Plan::update_position_and_velocity(double i_dt)
{
    m_time += i_dt;
    
    m_x = m_x0+m_amplx*sin(m_omegax*m_time);
    m_y = m_y0+m_amply*sin(m_omegay*m_time);
    m_z = m_z0+m_amplz*sin(m_omegaz*m_time);
    
    m_vx = m_amplx*m_omegax*cos(m_omegax*m_time);
    m_vy = m_amply*m_omegay*cos(m_omegay*m_time);
    m_vz = m_amplz*m_omegay*cos(m_omegaz*m_time);
}

int Plan::index()
{
    return m_index;
}

double Plan::x()
{
    return m_x;
}

double Plan::y()
{
    return m_y;
}

double Plan::z()
{
    return m_z;
}

double Plan::vx()
{
    return m_vx;
}

double Plan::vy()
{
    return m_vy;
}

double Plan::vz()
{
    return m_vz;
}

double Plan::wx()
{
    return m_wx;
}

double Plan::wy()
{
    return m_wy;
}

double Plan::wz()
{
    return m_wz;
}

double Plan::nx()
{
    return m_nx;
}

double Plan::ny()
{
    return m_ny;
}

double Plan::nz()
{
    return m_nz;
}
