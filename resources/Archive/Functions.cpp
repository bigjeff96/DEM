#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <unordered_map>

#include "Sphere.h"
#include "Plan.h"
#include "Contact.h"
#include "Functions.h"
#include "Grid.h"
#include "Cell.h"
#include "Drum.h"

const double MU_0 = 0.0000004*M_PI;


// --------- READ / WRITE ----------


bool overlap(Sphere& i_a, double i_x, double i_y, double i_z, double i_radius)
{
    double nx = i_a.x()-i_x;
    double ny = i_a.y()-i_y;
    double nz = i_a.z()-i_z;

    return sqrt(nx*nx+ny*ny+nz*nz)<i_a.radius()+i_radius;
}

double rnd()
{
    return ((double)(rand())/RAND_MAX);
}

void read_data(int i_numberOfCommands, char** i_commands, int& i_fps, double& i_time_step, double& i_start_capture, double& i_total_time, double& i_gx, double& i_gy, double& i_gz, std::vector<Plan>& i_boite,std::vector<Drum>& i_tambour,std::vector<Sphere>& i_grains,Grid& i_cell_mesh, std::unordered_map<int, Contact>& i_contacts, std::vector<Sphere_Function>& i_sphere_functions)
{
    //upload
    bool boxUploaded = false;
    bool drumUploaded = false;
    char c[128];
    char container_type[128];
    
    int numberOfGrains = 1;
    double minRadius = 0.01;
    double maxRadius = 1.;
    double density = 1.;
    double remanence = 1.;
    double lx = 1.;
    double ly = 1.;
    double lz = 1.;
    double amplx = 0.;
    double amply = 0.;
    double amplz = 0.;
    double freqx = 0.;
    double freqy = 0.;
    double freqz = 0.;
    double drumRadius = 1.;
    double drumWidth = 1.;
    double drumW = 0.;
    double cell_size = 1.;
    double e = 1.;
    double mu_static = 0.;
    double mu_dynamic = 0.;
    double kn = 1.;
    double kt = 1.;
    
    //initialize container
    FILE *ft;
    for(int i=1;i<i_numberOfCommands;i++)
    {
        if(strcmp(i_commands[i],"-i")==0)
        {
            ft=fopen(i_commands[i+1],"r");
            fscanf(ft,"%s",c);
            fscanf(ft,"%s",container_type);
            if(strcmp(container_type,"box")==0)
            {
                fscanf(ft,"%s\t%s\t%s\t%s\t%s\t%s",c,c,c,c,c,c);
                fscanf(ft,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf",&e,&mu_static,&mu_dynamic,&kn,&kt,&i_time_step);
                fscanf(ft,"%s\t%s\t%s",c,c,c);
                fscanf(ft,"%d\t%lf\t%lf",&i_fps,&i_start_capture,&i_total_time);
                fscanf(ft,"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s",c,c,c,c,c,c,c,c,c);
                fscanf(ft,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf",&lx,&ly,&lz,&amplx,&amply,&amplz,&freqx,&freqy,&freqz);
                fscanf(ft,"%s\t%s\t%s\t%s\t%s",c,c,c,c,c);
                fscanf(ft,"%d\t%lf\t%lf\t%lf\t%lf",&numberOfGrains,&minRadius,&maxRadius,&density,&remanence);
                fscanf(ft,"%s\t%s\t%s",c,c,c);
                fscanf(ft,"%lf\t%lf\t%lf",&i_gx,&i_gy,&i_gz);
                fscanf(ft,"%s",c);
                fscanf(ft,"%lf",&cell_size);
                fclose(ft);
                fflush(ft);
                boxUploaded=true;
                
                //initialize boite
                i_boite.reserve(6);
                i_boite.emplace_back(2,-lx/2.,0.,0.,1.,0.,0.,amplx,amply,amplz,freqx,freqy,freqz,0.);
                i_boite.emplace_back(3,lx/2.,0.,0.,-1.,0.,0.,amplx,amply,amplz,freqx,freqy,freqz,0.);
                i_boite.emplace_back(4,0.,-ly/2.,0.,0.,1.,0.,amplx,amply,amplz,freqx,freqy,freqz,0.);
                i_boite.emplace_back(5,0.,ly/2.,0.,0.,-1.,0.,amplx,amply,amplz,freqx,freqy,freqz,0.);
                i_boite.emplace_back(6,0.,0.,-lz/2.,0.,0.,1.,amplx,amply,amplz,freqx,freqy,freqz,0.);
                i_boite.emplace_back(7,0.,0.,lz/2.,0.,0.,-1.,amplx,amply,amplz,freqx,freqy,freqz,0.);
            }
            if(strcmp(container_type,"drum")==0)
            {
                fscanf(ft,"%s\t%s\t%s\t%s\t%s\t%s",c,c,c,c,c,c);
                fscanf(ft,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf",&e,&mu_static,&mu_dynamic,&kn,&kt,&i_time_step);
                fscanf(ft,"%s\t%s\t%s",c,c,c);
                fscanf(ft,"%d\t%lf\t%lf",&i_fps,&i_start_capture,&i_total_time);
                fscanf(ft,"%s\t%s\t%s",c,c,c);
                fscanf(ft,"%lf\t%lf\t%lf",&drumRadius,&drumWidth,&drumW);
                fscanf(ft,"%s\t%s\t%s\t%s\t%s",c,c,c,c,c);
                fscanf(ft,"%d\t%lf\t%lf\t%lf\t%lf",&numberOfGrains,&minRadius,&maxRadius,&density,&remanence);
                fscanf(ft,"%s\t%s\t%s",c,c,c);
                fscanf(ft,"%lf\t%lf\t%lf",&i_gx,&i_gy,&i_gz);
                fscanf(ft,"%s",c);
                fscanf(ft,"%lf",&cell_size);
                fclose(ft);
                fflush(ft);
                drumUploaded=true;
                
                //initialize drum
                i_tambour.reserve(1);
                i_tambour.emplace_back(8,drumRadius,drumW);
                lx = drumWidth;
                ly = 2.*drumRadius;
                lz = 2.*drumRadius;
                
                //initialize side walls
                i_boite.reserve(2);
                i_boite.emplace_back(2,-lx/2.,0.,0.,1.,0.,0.,amplx,amply,amplz,freqx,freqy,freqz,drumW);
                i_boite.emplace_back(3,lx/2.,0.,0.,-1.,0.,0.,amplx,amply,amplz,freqx,freqy,freqz,drumW);
            }
        }
    }
    if(!boxUploaded && !drumUploaded)
    {
        exit(0);
    }
    
    //initialize linked cells
    i_cell_mesh.initialize_linked_cells(lx,ly,lz,amplx,amply,amplz,cell_size);
    
    //select module
    for(int i=1;i<i_numberOfCommands;i++)
    {
        if(strcmp(i_commands[i],"-dipole")==0 || strcmp(i_commands[i],"-d")==0)
        {
            i_sphere_functions.push_back(sphere_dipole_function);
        }
    }
    
    //initialize grains
    i_grains.reserve(numberOfGrains);
    
    double radius, volume, mass, magnetic_moment;
    double x,y,z;
    double vx,vy,vz;
    
    int ind = 0;
    int numberOfOverlaps;
    while(ind < numberOfGrains)
    {
        numberOfOverlaps = 0;
        
        radius = minRadius+rnd()*(maxRadius-minRadius);
        volume = 4./3.*M_PI*radius*radius*radius;
        mass = volume*density;
        magnetic_moment = remanence*volume/(MU_0);

        x = -lx/2.+radius+rnd()*(lx-2.*radius);
        y = -ly/2.+radius+rnd()*(ly-2.*radius);
        z = -lz/2.+radius+rnd()*(lz-2.*radius);
        
        vx = 0.;
        vy = 0.;
        vz = 0.;
        
        if(boxUploaded || y*y+z*z < (drumRadius-radius)*(drumRadius-radius))
        {
            for(Sphere& sph : i_grains)
            {
                if(overlap(sph,x,y,z,radius))
                {
                    numberOfOverlaps++;
                }
            }
            if(numberOfOverlaps==0)
            {
                i_grains.emplace_back(ind,radius,mass,magnetic_moment,x,y,z,vx,vy,vz);
                ind++;
            }
        }
    }
    
    //recording the initial state
    for(Sphere& sph : i_grains)
    {
        sph.print(0);
    }
    
    //init hash map of contacts
    i_contacts.reserve(numberOfGrains*10);
    
    Contact::SetParameters(e, mu_static, mu_dynamic, kn, kt, i_time_step);
}

void write_data(int i_fps, double i_t, double i_start_capture, double i_time_step, std::vector<Sphere>& i_grain)
{
    double rec_time = i_t-i_start_capture;
    int file_number = (int)((rec_time+i_time_step)*i_fps);
    
    if((rec_time >= 0.) && (file_number > (int)(rec_time*i_fps)))
    {
        for(Sphere& sph : i_grain)
        {
            sph.print(file_number);
        }
    }
}


// --------- CONTACT DETECTION ----------


void with_sphere(double i_time, std::unordered_map<int, Contact>& i_contacts, Sphere& i_a, Sphere* i_b, std::vector<Sphere_Function>& i_sphere_functions)
{
    double nx = i_a.x()-i_b->x();
    double ny = i_a.y()-i_b->y();
    double nz = i_a.z()-i_b->z();
    double rij = sqrt(nx*nx+ny*ny+nz*nz);
    double r_rij = 1./rij;
    nx *= r_rij;
    ny *= r_rij;
    nz *= r_rij;

    for(Sphere_Function& func : i_sphere_functions)
    {
        func(i_a, i_b, rij, nx, ny, nz);
    }
    
    double deltan = rij-(i_a.radius()+i_b->radius());
    if(deltan<0.)
    {
        int a_index = i_a.index();
        int b_index = i_b->index();
        int tag = (a_index < b_index)? 10000*a_index+b_index : 10000*b_index+a_index;
        i_contacts[tag].update(tag, i_time, &i_a, i_b, deltan, nx, ny, nz);
    }
}

void with_plan(double i_time, std::unordered_map<int, Contact>& i_contacts, Sphere& i_a, Plan& i_p)
{
    double nx = i_p.nx();
    double ny = i_p.ny();
    double nz = i_p.nz();
    double rx = i_a.x()-i_p.x();
    double ry = i_a.y()-i_p.y();
    double rz = i_a.z()-i_p.z();
    
    //contact
    double deltan = rx*nx+ry*ny+rz*nz-i_a.radius();
    if(deltan<0.)
    {
        int tag = -(10000*i_a.index()+i_p.index());
        i_contacts[tag].update(tag, i_time, &i_a, &i_p, deltan, nx, ny, nz);
    }
}

void with_drum(double i_time, std::unordered_map<int, Contact>& i_contacts, Sphere& i_a, Drum& i_d)
{
    double nx = 0.;
    double ny = -i_a.y();
    double nz = -i_a.z();
    double rij = sqrt(ny*ny+nz*nz);
    ny /= rij;
    nz /= rij;
    
    //contact
    double deltan = i_d.radius()-rij-i_a.radius();
    if(deltan<0.)
    {
        int tag = -(10000*i_a.index()+i_d.index());
        i_contacts[tag].update(tag, i_time, &i_a, &i_d, deltan, nx, ny, nz);
    }
}

void detect_contacts(double i_time, std::unordered_map<int, Contact>& i_contacts,Sphere& i_sph,std::vector<Plan>& i_boite,std::vector<Drum>& i_tambour, std::vector<Sphere_Function>& i_sphere_functions)
{
    Cell* cl=i_sph.cell_ptr();
    Sphere* b=cl->head_node();
    
    //contacts with grains in the same cell
    while(b!=NULL)
    {
        if(i_sph.index()<b->index())
        {
            with_sphere(i_time, i_contacts, i_sph, b, i_sphere_functions);
        }
        b=b->next();
    }
    
    //contacts with grains in neighboring cells
    for(Cell* ncl : cl->neighbor_ptrs())
    {
        b=ncl->head_node();
        while(b!=NULL)
        {
            with_sphere(i_time, i_contacts, i_sph, b, i_sphere_functions);
            b=b->next();
        }
    }
    
    //contacts with the box
    for(Plan& pl : i_boite)
    {
        with_plan(i_time, i_contacts, i_sph, pl);
    }
    
    //contacts with the drum
    for(Drum& dr : i_tambour)
    {
        with_drum(i_time, i_contacts, i_sph, dr);
    }
}


//--------- SHORT RANGE INTERACTION FUNCTIONS ----------


void sphere_dipole_function(Sphere& i_a, Sphere* i_b, double i_r, double i_nx, double i_ny, double i_nz)
{
    double r_r = 1./i_r;
    double C_torque = 0.0000001*r_r*r_r*r_r;
    double C_force = 3.*C_torque*r_r;
    
    double a_mx = i_a.mx();
    double a_my = i_a.my();
    double a_mz = i_a.mz();
    
    double b_mx = i_b->mx();
    double b_my = i_b->my();
    double b_mz = i_b->mz();
    
    double a_m_dot_n = a_mx*i_nx + a_my*i_ny + a_mz*i_nz;
    double b_m_dot_n = b_mx*i_nx + b_my*i_ny + b_mz*i_nz;
    double a_m_dot_b_m = a_mx*b_mx + a_my*b_my + a_mz*b_mz;
    
    double C_normal = a_m_dot_b_m - 5.*a_m_dot_n*b_m_dot_n;
    double Fx = C_force*(a_mx*b_m_dot_n + b_mx*a_m_dot_n + i_nx*C_normal);
    double Fy = C_force*(a_my*b_m_dot_n + b_my*a_m_dot_n + i_ny*C_normal);
    double Fz = C_force*(a_mz*b_m_dot_n + b_mz*a_m_dot_n + i_nz*C_normal);
    
    i_a.add_force(Fx, Fy, Fz);
    i_b->add_force(-Fx, -Fy, -Fz);
    
    double b_Bx = C_torque*(3.*i_nx*(b_m_dot_n)-b_mx);
    double b_By = C_torque*(3.*i_ny*(b_m_dot_n)-b_my);
    double b_Bz = C_torque*(3.*i_nz*(b_m_dot_n)-b_mz);
    
    double a_Mx = a_my*b_Bz - a_mz*b_By;
    double a_My = a_mz*b_Bx - a_mx*b_Bz;
    double a_Mz = a_mx*b_By - a_my*b_Bx;
    
    i_a.add_torque(a_Mx, a_My, a_Mz);
    
    double a_Bx = C_torque*(3.*i_nx*(a_m_dot_n)-a_mx);
    double a_By = C_torque*(3.*i_ny*(a_m_dot_n)-a_my);
    double a_Bz = C_torque*(3.*i_nz*(a_m_dot_n)-a_mz);
    
    double b_Mx = b_my*a_Bz - b_mz*a_By;
    double b_My = b_mz*a_Bx - b_mx*a_Bz;
    double b_Mz = b_mx*a_By - b_my*a_Bx;
    
    i_b->add_torque(b_Mx, b_My, b_Mz);
}


//--------- FORCE COMPUTATION ----------


void compute_contact_forces(double i_time, std::unordered_map<int, Contact>& i_contacts)
{
    for (auto i = i_contacts.begin(), last = i_contacts.end(); i != last; )
    {
        if((i->second).inactive(i_time))
        {
            //erase inactive contacts, returns iterator to next element
            i = i_contacts.erase(i);
        }
        else
        {
            //compute force for active contacts
            (i->second).compute_force();
            
            //iterate
            ++i;
        }
    }
}
