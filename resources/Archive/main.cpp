#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <unordered_map>

#include "Cell.h"
#include "Sphere.h"
#include "Functions.h"
#include "Contact.h"
#include "Plan.h"
#include "Drum.h"
#include "Grid.h"

int main(int argc,char **argv){
    
    srand((unsigned int)time(NULL));
        
    //declaration
    int fps;
    double time_step, start_capture, total_time, gx, gy, gz;
    std::vector<Plan> boite;
    std::vector<Drum> tambour;
    std::vector<Sphere> grains;
    std::unordered_map<int, Contact> contacts;
    std::vector<Sphere_Function> sphere_functions;
    Grid cell_mesh;
    
    //initialization
    read_data(argc, argv, fps, time_step, start_capture, total_time, gx, gy, gz, boite, tambour, grains, cell_mesh, contacts, sphere_functions);
    double half_time_step = 0.5*time_step;
    
    //computation
    for(double t = 0. ; t <= total_time ; t += time_step)
    {
        cell_mesh.reset();
        
        for(Sphere& sph : grains)
        {
            sph.update_position(half_time_step);
            cell_mesh.add_sphere(sph);
            sph.reset_force();
            sph.add_gravity_force(gx, gy, gz);
        }
        
        for(Plan& pl : boite)
        {
            pl.update_position_and_velocity(half_time_step);
        }
        
        for(Sphere& sph : grains)
        {
            detect_contacts(t, contacts, sph, boite, tambour, sphere_functions);
        }
        compute_contact_forces(t, contacts);
        
        for(Sphere& sph : grains)
        {
            sph.update_velocity(time_step);
            sph.update_position(half_time_step);
        }
        
        for(Plan& pl : boite)
        {
            pl.update_position_and_velocity(half_time_step);
        }
        
        //record data
        write_data(fps, t, start_capture, time_step, grains);
    }
    
    return 0;
}
