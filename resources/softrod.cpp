#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <vector>
#include "sphere.h"
#include "softrod.h"

using namespace std;

softrod::softrod(int i_index, int i_n, double i_sphereRadius, double i_x, double i_y, double i_z)
{
    m_index = i_index;
    m_n = i_n;
    m_sphereRadius = i_sphereRadius;
    m_x = i_x;
    m_y = i_y;
    m_z = i_z;
}

softrod::~softrod()
{

}

int softrod::index()
{
    return m_index;
}

int softrod::n()
{
    return m_n;
}


double softrod::x()
{
    return m_x;
}

double softrod::y()
{
    return m_y;
}

double softrod::z()
{
    return m_z;
}

double softrod::sphereRadius()
{
    return m_sphereRadius;
}

void softrod::addSphere(sphere* i_p)
{
    m_grain.push_back(i_p);
}

void softrod::computeInternalForce()
{
    for(int i = 0 ; i<(int)(m_grain.size())-1 ; i++)
    {
        sphere* a = m_grain[i];
        sphere* b = m_grain[i+1];
        
        //TENSILE STRESS
        double nx = a->x()+a->radius()*a->ex()-(b->x()-b->radius()*b->ex());
        double ny = a->y()+a->radius()*a->ey()-(b->y()-b->radius()*b->ey());
        double nz = a->z()+a->radius()*a->ez()-(b->z()-b->radius()*b->ez());
        double rij = sqrt(nx*nx+ny*ny+nz*nz);
        
        double vx,vy,vz,vn;
        double F,Fx,Fy,Fz;
        double Mx,My,Mz;
        
        if(rij!=0)
        {
            nx /= rij;
            ny /= rij;
            nz /= rij;
            
            vx = a->vx() + a->radius()*(a->wy()*a->ez()-a->wz()*a->ey()) - b->vx() + b->radius()*(b->wy()*b->ez()-b->wz()*b->ey());
            vy = a->vy() + a->radius()*(a->wz()*a->ex()-a->wx()*a->ez()) - b->vy() + b->radius()*(b->wz()*b->ex()-b->wx()*b->ez());
            vz = a->vz() + a->radius()*(a->wx()*a->ey()-a->wy()*a->ex()) - b->vz() + b->radius()*(b->wx()*b->ey()-b->wy()*b->ex());
            vn = vx*nx+vy*ny+vz*nz;
            
            F = -100.*rij-0.01*vn;
            Fx = F*nx;
            Fy = F*ny;
            Fz = F*nz;
            
            //force
            a->addForce(Fx,Fy,Fz);
            b->addForce(-Fx,-Fy,-Fz);
            
            //momentum
            Mx = a->radius()*(a->ey()*Fz-a->ez()*Fy);
            My = a->radius()*(a->ez()*Fx-a->ex()*Fz);
            Mz = a->radius()*(a->ex()*Fy-a->ey()*Fx);
            a->addMomentum(Mx,My,Mz);
            Mx = b->radius()*(b->ey()*Fz-b->ez()*Fy);
            My = b->radius()*(b->ez()*Fx-b->ex()*Fz);
            Mz = b->radius()*(b->ex()*Fy-b->ey()*Fx);
            b->addMomentum(Mx,My,Mz);
        }
        
        //SHEAR STRESS
        
        //turns a toward b
        double r0 = b->q0()*a->q0() + b->q1()*a->q1() + b->q2()*a->q2() + b->q3()*a->q3();
        double r1 = -b->q0()*a->q1() + b->q1()*a->q0() - b->q2()*a->q3() + b->q3()*a->q2();
        double r2 = -b->q0()*a->q2() + b->q2()*a->q0() + b->q1()*a->q3() - b->q3()*a->q1();
        double r3 = -b->q0()*a->q3() + b->q3()*a->q0() - b->q1()*a->q2() + b->q2()*a->q1();
        double norm = sqrt(r1*r1+r2*r2+r3*r3);
        
        if(norm>0)
        {
            double theta = 2.*atan2(norm,r0);
            double ux = r1/norm;
            double uy = r2/norm;
            double uz = r3/norm;
            
            Mx = 0.0001*theta*ux;
            My = 0.0001*theta*uy;
            Mz = 0.0001*theta*uz;
            a->addMomentum(Mx,My,Mz);
            b->addMomentum(-Mx,-My,-Mz);
        }
    }
}
