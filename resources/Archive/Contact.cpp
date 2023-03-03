#include <stdlib.h>
#include <math.h>

#include "Contact.h"
#include "Sphere.h"
#include "Cell.h"
#include "Plan.h"
#include "Drum.h"


Contact::Contact()
{
    m_a = NULL;
    m_b = NULL;
    m_p = NULL;
    m_d = NULL;
   
    m_nx = 1.;
    m_ny = 0.;
    m_nz = 0.;
    m_tx = 0.;
    m_ty = 1.;
    m_tz = 0.;

    m_vn = 0.;
    m_vt = 0.;
    
    m_vrx = 0.;
    m_vry = 0.;
    m_vrz = 0.;

    m_deltan = 0.;
    m_deltat = 0.;
    m_elx = 0.;
    m_ely = 0.;
    m_elz = 0.;
    m_mu = m_mu_static;
}

Contact::~Contact()
{

}

void Contact::compute_force()
{
    double Fn, Ft, signFt, Ftx, Fty, Ftz, Mx, My, Mz, etan, etat, effectiveMass, a_radius, b_radius;
    double vr, r_vr, Fr, mu_rolling = 0.005;
    double xs, rs, wsx, wsy, wsz, ws, r_ws, Ms, mu_spinning = 0.005;
    
    if(m_b!=NULL)
    {
        effectiveMass = m_a->mass()*m_b->mass()/(m_a->mass()+m_b->mass());
        etan = m_etan_cst*sqrt(effectiveMass);
        etat = 0.2*etan;
        a_radius = m_a->radius();
        b_radius = m_b->radius();

        //normal repulsion
        Fn = -m_kn*m_deltan-etan*m_vn;
        if(Fn > 0.)
        {
            m_a->add_force(Fn*m_nx,Fn*m_ny,Fn*m_nz);
            m_b->add_force(-Fn*m_nx,-Fn*m_ny,-Fn*m_nz);
        }
        else
        {
            Fn = 0.;
        }

        //tangential friction
        Ft = -m_kt*m_deltat-etat*m_vt;
        signFt = (Ft < 0.)? -1. : 1.;
        if(fabs(Ft) >= Fn*m_mu){
            //sliding friction
            m_mu = m_mu_dynamic;
            Ftx = signFt*Fn*m_mu*m_tx;
            Fty = signFt*Fn*m_mu*m_ty;
            Ftz = signFt*Fn*m_mu*m_tz;
            m_a->add_force(Ftx,Fty,Ftz);
            m_b->add_force(-Ftx,-Fty,-Ftz);

            m_deltat = -signFt*(Fn*m_mu+etat*m_vt)/m_kt;
            m_elx = m_deltat*m_tx;
            m_ely = m_deltat*m_ty;
            m_elz = m_deltat*m_tz;
        }
        else
        {
            //static friction
            m_mu = m_mu_static;
            Ftx = Ft*m_tx;
            Fty = Ft*m_ty;
            Ftz = Ft*m_tz;
            m_a->add_force(Ftx,Fty,Ftz);
            m_b->add_force(-Ftx,-Fty,-Ftz);
        }
        
        //torque
        Mx = m_nz*Fty-m_ny*Ftz;
        My = m_nx*Ftz-m_nz*Ftx;
        Mz = m_ny*Ftx-m_nx*Fty;
        m_a->add_torque(a_radius*Mx, a_radius*My, a_radius*Mz);
        m_b->add_torque(b_radius*Mx, b_radius*My, b_radius*Mz);
        
        //rolling friction
        vr = sqrt(m_vrx*m_vrx + m_vry*m_vry + m_vrz*m_vrz);
        if(vr > 0.)
        {
            r_vr = 1./vr;
            m_vrx *= r_vr;
            m_vry *= r_vr;
            m_vrz *= r_vr;
            
            Fr = mu_rolling*Fn;
            Mx = Fr*(m_ny*m_vrz-m_nz*m_vry);
            My = Fr*(m_nz*m_vrx-m_nx*m_vrz);
            Mz = Fr*(m_nx*m_vry-m_ny*m_vrx);
            m_a->add_torque(a_radius*Mx, a_radius*My, a_radius*Mz);
            m_b->add_torque(b_radius*Mx, b_radius*My, b_radius*Mz);
        }
        
        //spinning friction
        xs = 0.5*(a_radius*a_radius-b_radius*b_radius)/(a_radius+b_radius+m_deltan)+0.5*(a_radius+b_radius+m_deltan);
        rs = sqrt(a_radius*a_radius - xs*xs);
        wsx = m_a->wx() - m_b->wx();
        wsy = m_a->wy() - m_b->wy();
        wsz = m_a->wz() - m_b->wz();
        ws = sqrt(wsx*wsx + wsy*wsy + wsz*wsz);
        if(ws > 0.)
        {
            r_ws = 1./ws;
            wsx *= r_ws;
            wsy *= r_ws;
            wsz *= r_ws;
        }
        Ms = -mu_spinning*Fn*rs*(wsx*m_nx + wsy*m_ny + wsz*m_nz);
        m_a->add_torque(Ms*m_nx, Ms*m_ny, Ms*m_nz);
        m_b->add_torque(-Ms*m_nx, -Ms*m_ny, -Ms*m_nz);
    }
    else
    {
        effectiveMass = m_a->mass();
        etan = m_etan_cst*sqrt(effectiveMass);
        etat = 0.2*etan;
        a_radius = m_a->radius();

        //normal repulsion
        Fn = -m_kn*m_deltan-etan*m_vn;
        if(Fn > 0.)
        {
            m_a->add_force(Fn*m_nx,Fn*m_ny,Fn*m_nz);
        }
        else
        {
            Fn = 0.;
        }
        
        //tangential friction
        Ft = -m_kt*m_deltat-etat*m_vt;
        signFt = (Ft < 0.)? -1. : 1.;
        if(fabs(Ft) > Fn*m_mu){
            //sliding friction
            m_mu = m_mu_dynamic;
            Ftx = signFt*Fn*m_mu*m_tx;
            Fty = signFt*Fn*m_mu*m_ty;
            Ftz = signFt*Fn*m_mu*m_tz;
            m_a->add_force(Ftx,Fty,Ftz);

            m_deltat = -signFt*Fn*m_mu/m_kt;
            m_elx = m_deltat*m_tx;
            m_ely = m_deltat*m_ty;
            m_elz = m_deltat*m_tz;
        }
        else
        {
            //static friction
            m_mu = m_mu_static;
            Ftx = Ft*m_tx;
            Fty = Ft*m_ty;
            Ftz = Ft*m_tz;
            m_a->add_force(Ftx,Fty,Ftz);
        }
  
        //torque
        Mx = m_nz*Fty-m_ny*Ftz;
        My = m_nx*Ftz-m_nz*Ftx;
        Mz = m_ny*Ftx-m_nx*Fty;
        m_a->add_torque(a_radius*Mx, a_radius*My, a_radius*Mz);
        
        //rolling friction
        vr = sqrt(m_vrx*m_vrx + m_vry*m_vry + m_vrz*m_vrz);
        if(vr > 0.)
        {
            r_vr = 1./vr;
            m_vrx *= r_vr;
            m_vry *= r_vr;
            m_vrz *= r_vr;
            
            Fr = mu_rolling*Fn;
            Mx = Fr*(m_ny*m_vrz-m_nz*m_vry);
            My = Fr*(m_nz*m_vrx-m_nx*m_vrz);
            Mz = Fr*(m_nx*m_vry-m_ny*m_vrx);
            m_a->add_torque(a_radius*Mx, a_radius*My, a_radius*Mz);
        }
        
        //spinning friction
        xs = a_radius + m_deltan;
        rs = sqrt(a_radius*a_radius-xs*xs);
        wsx = m_a->wx();
        wsy = m_a->wy();
        wsz = m_a->wz();
        ws = sqrt(wsx*wsx + wsy*wsy + wsz*wsz);
        if(ws > 0.)
        {
            r_ws = 1./ws;
            wsx *= r_ws;
            wsy *= r_ws;
            wsz *= r_ws;
        }
        Ms = -mu_spinning*Fn*rs*(wsx*m_nx + wsy*m_ny + wsz*m_nz);
        m_a->add_torque(Ms*m_nx, Ms*m_ny, Ms*m_nz);
    }
}

void Contact::update(int i_tag, double i_time, Sphere* i_a, Sphere* i_b, double i_deltan, double i_nx, double i_ny, double i_nz)
{
    m_tag = i_tag;
    m_update_time = i_time;
    m_a = i_a;
    m_b = i_b;
    m_p = NULL;
    m_d = NULL;
    m_deltan = i_deltan;
    m_nx = i_nx;
    m_ny = i_ny;
    m_nz = i_nz;

    m_vrx = m_a->radius()*(m_ny*m_a->wz()-m_nz*m_a->wy()) + m_b->radius()*(m_ny*m_b->wz()-m_nz*m_b->wy());
    m_vry = m_a->radius()*(m_nz*m_a->wx()-m_nx*m_a->wz()) + m_b->radius()*(m_nz*m_b->wx()-m_nx*m_b->wz());
    m_vrz = m_a->radius()*(m_nx*m_a->wy()-m_ny*m_a->wx()) + m_b->radius()*(m_nx*m_b->wy()-m_ny*m_b->wx());
    
    double vx = m_a->vx() - m_b->vx() + m_vrx;
    double vy = m_a->vy() - m_b->vy() + m_vry;
    double vz = m_a->vz() - m_b->vz() + m_vrz;

    m_vn = vx*m_nx+vy*m_ny+vz*m_nz;

    double vnx = m_vn*m_nx;
    double vny = m_vn*m_ny;
    double vnz = m_vn*m_nz;
    double vtx = vx-vnx;
    double vty = vy-vny;
    double vtz = vz-vnz;
    
    m_vt = sqrt(vtx*vtx + vty*vty + vtz*vtz);

    if(m_vt > 0.)
    {
        m_tx = vtx/m_vt;
        m_ty = vty/m_vt;
        m_tz = vtz/m_vt;
    }
    else
    {
        m_tx = 0.;
        m_ty = 0.;
        m_tz = 0.;
    }
    
    //projection du vieux m_el(m_elx,m_ely,m_elz) sur la nouvelle direction tangente
    double old_elongation_projection = m_elx*m_tx+m_ely*m_ty+m_elz*m_tz;

    m_deltat = m_vt*m_dt + old_elongation_projection;
    m_elx = m_deltat*m_tx;
    m_ely = m_deltat*m_ty;
    m_elz = m_deltat*m_tz;
}

void Contact::update(int i_tag, double i_time, Sphere* i_a, Drum* i_d, double i_deltan, double i_nx, double i_ny, double i_nz)
{
    m_tag = i_tag;
    m_update_time = i_time;
    m_a = i_a;
    m_b = NULL;
    m_d = i_d;
    m_p = NULL;
    m_deltan = i_deltan;
    m_nx = i_nx;
    m_ny = i_ny;
    m_nz = i_nz;
    
    m_vrx = m_a->radius()*(m_ny*m_a->wz()-m_nz*m_a->wy());
    m_vry = m_a->radius()*(m_nz*m_a->wx()-m_nx*m_a->wz()) - m_d->radius()*(m_nz*m_d->wx());
    m_vrz = m_a->radius()*(m_nx*m_a->wy()-m_ny*m_a->wx()) - m_d->radius()*(-m_ny*m_d->wx());

    double vx = m_a->vx() + m_vrx;
    double vy = m_a->vy() + m_vry;
    double vz = m_a->vz() + m_vrz;

    m_vn = vx*m_nx+vy*m_ny+vz*m_nz;

    double vnx = m_vn*m_nx;
    double vny = m_vn*m_ny;
    double vnz = m_vn*m_nz;
    double vtx = vx-vnx;
    double vty = vy-vny;
    double vtz = vz-vnz;
    
    m_vt = sqrt(vtx*vtx + vty*vty + vtz*vtz);

    if(m_vt > 0.)
    {
        m_tx = vtx/m_vt;
        m_ty = vty/m_vt;
        m_tz = vtz/m_vt;
    }
    else
    {
        m_tx = 0.;
        m_ty = 0.;
        m_tz = 0.;
    }

    //projection du vieux m_el(m_elx,m_ely,m_elz) sur la nouvelle direction tangente
    double old_elongation_projection = m_elx*m_tx+m_ely*m_ty+m_elz*m_tz;

    m_deltat = m_vt*m_dt + old_elongation_projection;
    m_elx = m_deltat*m_tx;
    m_ely = m_deltat*m_ty;
    m_elz = m_deltat*m_tz;
}

void Contact::update(int i_tag, double i_time, Sphere* i_a, Plan* i_p, double i_deltan, double i_nx, double i_ny, double i_nz)
{
    m_tag = i_tag;
    m_update_time = i_time;
    m_a = i_a;
    m_b = NULL;
    m_d = NULL;
    m_p = i_p;
    m_deltan = i_deltan;
    m_nx = i_nx;
    m_ny = i_ny;
    m_nz = i_nz;

    m_vrx = m_a->radius()*(m_ny*m_a->wz()-m_nz*m_a->wy());
    m_vry = m_a->radius()*(m_nz*m_a->wx()-m_nx*m_a->wz()) + m_p->wx()*m_a->z();
    m_vrz = m_a->radius()*(m_nx*m_a->wy()-m_ny*m_a->wx()) - m_p->wx()*m_a->y();
    
    double vx = m_a->vx() - m_p->vx() + m_vrx;
    double vy = m_a->vy() - m_p->vy() + m_vry;
    double vz = m_a->vz() - m_p->vz() + m_vrz;
    
    m_vn = vx*m_nx+vy*m_ny+vz*m_nz;
    
    double vnx = m_vn*m_nx;
    double vny = m_vn*m_ny;
    double vnz = m_vn*m_nz;
    double vtx = vx-vnx;
    double vty = vy-vny;
    double vtz = vz-vnz;
    
    m_vt = sqrt(vtx*vtx + vty*vty + vtz*vtz);

    if(m_vt > 0.)
    {
        m_tx = vtx/m_vt;
        m_ty = vty/m_vt;
        m_tz = vtz/m_vt;
    }
    else
    {
        m_tx = 0.;
        m_ty = 0.;
        m_tz = 0.;
    }

    //projection du vieux m_el(m_elx,m_ely,m_elz) sur la nouvelle direction tangente
    double old_elongation_projection = m_elx*m_tx+m_ely*m_ty+m_elz*m_tz;

    m_deltat = m_vt*m_dt + old_elongation_projection;
    m_elx = m_deltat*m_tx;
    m_ely = m_deltat*m_ty;
    m_elz = m_deltat*m_tz;
}

bool Contact::inactive(double i_time)
{
    return (m_update_time < i_time);
}

//Static member declaration
double Contact::m_etan_cst;
double Contact::m_mu_static;
double Contact::m_mu_dynamic;
double Contact::m_kn;
double Contact::m_kt;
double Contact::m_dt;

void Contact::SetParameters(double i_e, double i_mu_static, double i_mu_dynamic, double i_kn, double i_kt, double i_dt)
{
    m_mu_static = i_mu_static;
    m_mu_dynamic = i_mu_dynamic;
    m_kn = i_kn;
    m_kt = i_kt;
    m_etan_cst = -2.*log(i_e)*sqrt(i_kn)/sqrt(log(i_e)*log(i_e)+M_PI*M_PI);
    m_dt = i_dt;
}
