class Cell;

class Sphere
{
    
private:
    
    int m_index;
    Cell* m_cell_ptr;
    Sphere* m_next;
    
    double m_radius, m_mass, m_inertia, m_magnetic_moment;
    double m_x, m_y, m_z;
    double m_vx, m_vy, m_vz;
    double m_q0, m_q1, m_q2, m_q3;
    double m_wx, m_wy, m_wz;
    double m_Fx, m_Fy, m_Fz;
    double m_Mx, m_My, m_Mz;
        
public:
    
    Sphere(int,double,double,double,double,double,double,double,double,double);
    ~Sphere();
    
    void update_velocity(double);
    void update_position(double);
    void reset_force();
    void add_force(double,double,double);
    void add_torque(double,double,double);
    void add_gravity_force(double,double,double);

    int index();
    double radius();
    double mass();
    double x();
    double y();
    double z();
    double vx();
    double vy();
    double vz();
    double wx();
    double wy();
    double wz();
    double q0();
    double q1();
    double q2();
    double q3();
    double mx();
    double my();
    double mz();
    
    void set_next(Sphere*);
    Sphere* next();
    void set_cell_ptr(Cell*);
    Cell* cell_ptr();
    void print(int);
};
