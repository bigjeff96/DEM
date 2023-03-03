#include <stdlib.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "Sphere.h"

Sphere::Sphere(int i_index, double i_radius, double i_mass, double i_magnetic_moment, double i_x, double i_y, double i_z, double i_vx, double i_vy, double i_vz)
{
    m_index = i_index;
    m_next = NULL;
    m_cell_ptr = NULL;
    m_radius = i_radius;
    m_mass = i_mass;
    m_inertia = 0.4*m_mass*i_radius*i_radius;
    m_magnetic_moment = i_magnetic_moment;

    m_x = i_x;
    m_y = i_y;
    m_z = i_z;
    m_vx = i_vx;
    m_vy = i_vy;
    m_vz = i_vz;
    
    m_q0 = 1.;
    m_q1 = 0.;
    m_q2 = 0.;
    m_q3 = 0.;
    m_wx = 0.;
    m_wy = 0.;
    m_wz = 0.;
    
    m_Fx = 0.;
    m_Fy = 0.;
    m_Fz = 0.;
    m_Mx = 0.;
    m_My = 0.;
    m_Mz = 0.;
    
    /*e1*/
    //m_ex = 1.-2.*m_q2*m_q2-2.*m_q3*m_q3;
    //m_ey = 2.*m_q1*m_q2 + 2.*m_q3*m_q0;
    //m_ez = 2.*m_q1*m_q3 - 2.*m_q2*m_q0;
    
    /*e3*/
    //m_ex = 2.*m_q1*m_q3 + 2.*m_q2*m_q0;
    //m_ey = 2.*m_q2*m_q3 - 2.*m_q1*m_q0;
    //m_ez = 1.-2.*m_q1*m_q1-2.*m_q2*m_q2;
}

Sphere::~Sphere()
{

}

void Sphere::reset_force()
{
    m_Fx = 0.;
    m_Fy = 0.;
    m_Fz = 0.;
    m_Mx = 0.;
    m_My = 0.;
    m_Mz = 0.;
}

void Sphere::add_force(double i_Fx, double i_Fy, double i_Fz)
{
    m_Fx += i_Fx;
    m_Fy += i_Fy;
    m_Fz += i_Fz;
}

void Sphere::add_torque(double i_Mx, double i_My, double i_Mz)
{
    m_Mx += i_Mx;
    m_My += i_My;
    m_Mz += i_Mz;
}

void Sphere::add_gravity_force(double i_gx, double i_gy, double i_gz)
{
    m_Fx += m_mass*i_gx;
    m_Fy += m_mass*i_gy;
    m_Fz += m_mass*i_gz;
}

int Sphere::index()
{
    return m_index;
}

double Sphere::radius()
{
    return m_radius;
}

double Sphere::mass()
{
    return m_mass;
}

double Sphere::x()
{
    return m_x;
}

double Sphere::y()
{
    return m_y;
}

double Sphere::z()
{
    return m_z;
}

double Sphere::vx()
{
    return m_vx;
}

double Sphere::vy()
{
    return m_vy;
}

double Sphere::vz()
{
    return m_vz;
}

double Sphere::wx()
{
    return m_wx;
}

double Sphere::wy()
{
    return m_wy;
}

double Sphere::wz()
{
    return m_wz;
}

double Sphere::q0()
{
    return m_q0;
}

double Sphere::q1()
{
    return m_q1;
}

double Sphere::q2()
{
    return m_q2;
}

double Sphere::q3()
{
    return m_q3;
}

double Sphere::mx()
{
    return  m_magnetic_moment*(1.-2.*m_q2*m_q2-2.*m_q3*m_q3);
}

double Sphere::my()
{
    return  m_magnetic_moment*(2.*m_q1*m_q2 + 2.*m_q3*m_q0);
}

double Sphere::mz()
{
    return  m_magnetic_moment*(2.*m_q1*m_q3 - 2.*m_q2*m_q0);
}

void Sphere::set_next(Sphere* i_next)
{
    m_next = i_next;
}

Sphere* Sphere::next()
{
    return m_next;
}

void Sphere::set_cell_ptr(Cell* i_cell_ptr)
{
    m_cell_ptr = i_cell_ptr;
}

Cell* Sphere::cell_ptr()
{
    return m_cell_ptr;
}

void Sphere::update_position(double i_dt)
{    
    //TRANSLATION
    
    m_x += m_vx*i_dt;
    m_y += m_vy*i_dt;
    m_z += m_vz*i_dt;

    //ROTATION

    double w = sqrt(m_wx*m_wx + m_wy*m_wy + m_wz*m_wz);
    double r_w = 1./w;
    
    if(w > 0.)
    {
        //linking rotation angle and axis to quaternions
        double theta = 0.5*w*i_dt;
        double p0 = cos(theta);
        double p1 = (m_wx*r_w)*sin(theta);
        double p2 = (m_wy*r_w)*sin(theta);
        double p3 = (m_wz*r_w)*sin(theta);
        
        //Hamiltonian product p*q
        double s_q0 = p0*m_q0 - p1*m_q1 - p2*m_q2 - p3*m_q3;
        double s_q1 = p0*m_q1 + p1*m_q0 + p2*m_q3 - p3*m_q2;
        double s_q2 = p0*m_q2 + p2*m_q0 - p1*m_q3 + p3*m_q1;
        double s_q3 = p0*m_q3 + p3*m_q0 + p1*m_q2 - p2*m_q1;
        
        //Verifying normalization
        double normeq = sqrt(s_q0*s_q0+s_q1*s_q1+s_q2*s_q2+s_q3*s_q3);
        double r_normeq = 1./normeq;
        m_q0 = s_q0*r_normeq;
        m_q1 = s_q1*r_normeq;
        m_q2 = s_q2*r_normeq;
        m_q3 = s_q3*r_normeq;
    }
    /*e1*/
    //m_ex = 1.-2.*m_q2*m_q2-2.*m_q3*m_q3;
    //m_ey = 2.*m_q1*m_q2 + 2.*m_q3*m_q0;
    //m_ez = 2.*m_q1*m_q3 - 2.*m_q2*m_q0;
        
    /*e3*/
    //m_ex = 2.*m_q1*m_q3 + 2.*m_q2*m_q0;
    //m_ey = 2.*m_q2*m_q3 - 2.*m_q1*m_q0;
    //m_ez = 1.-2.*m_q1*m_q1-2.*m_q2*m_q2;
}

void Sphere::update_velocity(double i_dt)
{
    //TRANSLATION
    
    double r_mass_dt = 1./m_mass*i_dt;

    //integration
    m_vx += m_Fx*r_mass_dt;
    m_vy += m_Fy*r_mass_dt;
    m_vz += m_Fz*r_mass_dt;

    //ROTATION

    double r_inertia_dt = 1./m_inertia*i_dt;
    
    //integration
    m_wx += m_Mx*r_inertia_dt;
    m_wy += m_My*r_inertia_dt;
    m_wz += m_Mz*r_inertia_dt;
}

void Sphere::print(int i_num)
{
  FILE *ft;
  char fileName[1024];
  sprintf(fileName,"grain%d.txt",i_num);
  ft=fopen(fileName,"a+");
    fprintf(ft,"%d\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\n",m_index,m_x,m_y,m_z,m_vx,m_vy,m_vz,m_q0,m_q1,m_q2,m_q3,m_wx,m_wy,m_wz,m_radius);
  fclose(ft);
  fflush(ft);
}
