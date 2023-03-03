class Sphere;
class Plan;
class Cell;
class Drum;

class Contact
{

private:

    Sphere* m_a;
    Sphere* m_b;
    Plan* m_p;
    Drum* m_d;
    int m_tag;
    double m_update_time;
    double m_deltan, m_deltat;
    double m_nx, m_ny, m_nz;
    double m_tx, m_ty, m_tz;
    double m_vn, m_vt;
    double m_elx, m_ely, m_elz;
    double m_mu;
    double m_vrx, m_vry, m_vrz;
    
    static double m_mu_static;
    static double m_mu_dynamic;
    static double m_kn;
    static double m_kt;
    static double m_dt;
    static double m_etan_cst;

public:

    Contact();
    ~Contact();

    void compute_force();
    void update(int, double, Sphere*, Sphere*, double, double, double, double);
    void update(int, double, Sphere*, Drum*, double, double, double, double);
    void update(int, double, Sphere*, Plan*, double, double, double, double);
    bool inactive(double);
    static void SetParameters(double, double, double, double, double, double);
};
