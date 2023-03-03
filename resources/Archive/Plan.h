class Plan
{
private:
    int m_index;
    double m_x0, m_y0, m_z0;
    double m_x, m_y, m_z;
    double m_vx, m_vy, m_vz;
    double m_wx, m_wy, m_wz;
    double m_nx, m_ny, m_nz;
    double m_amplx, m_amply, m_amplz;
    double m_omegax, m_omegay, m_omegaz;
    double m_time;
            
public:
    Plan(int,double,double,double,double,double,double,double,double,double,double,double,double,double);
    ~Plan();
    void update_position_and_velocity(double);
    int index();
    double x();
    double y();
    double z();
    double vx();
    double vy();
    double vz();
    double wx();
    double wy();
    double wz();
    double nx();
    double ny();
    double nz();
};
