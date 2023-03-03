#include <unordered_map>

class Sphere;
class Plan;
class Contact;
class Grid;
class Drum;
typedef void(*Sphere_Function)(Sphere&, Sphere*, double, double, double, double);


// --------- READ / WRITE ----------


bool overlap(Sphere&,double,double,double,double);
double rnd();
void read_data(int,char**,int&,double&,double&,double&,double&,double&,double&,std::vector<Plan>&,std::vector<Drum>&,std::vector<Sphere>&,Grid&,std::unordered_map<int, Contact>&,std::vector<Sphere_Function>&);
void write_data(int,double,double,double,std::vector<Sphere>&);


// --------- CONTACT DETECTION ----------


void with_sphere(double, std::unordered_map<int, Contact>&, Sphere&, Sphere*,std::vector<Sphere_Function>&);
void with_plan(double, std::unordered_map<int, Contact>&, Sphere&, Plan&);
void with_drum(double, std::unordered_map<int, Contact>&, Sphere&, Drum&);
void detect_contacts(double, std::unordered_map<int, Contact>&, Sphere&, std::vector<Plan>&, std::vector<Drum>&,std::vector<Sphere_Function>&);


//--------- SHORT RANGE FUNCTIONS ----------


void sphere_dipole_function(Sphere&, Sphere*, double, double, double, double);


//--------- FORCE COMPUTATION ----------


void compute_contact_forces(double, std::unordered_map<int, Contact>&);
