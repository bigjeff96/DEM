class Drum
{

private:

    int m_index;
    double m_radius;
    double m_theta;
    double m_wx;
    double m_wy;
    double m_wz;

public:

    Drum(int, double, double);
    ~Drum();
    void update_position(double);
    int index();
    double radius();
    double theta();
    double wx();
    double wy();
    double wz();
};
