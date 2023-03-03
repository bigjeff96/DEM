class Sphere;
class Contact;
class Cell;

class Grid
{
    
private:
    
    int m_nbx, m_nby, m_nbz;
    double m_lx, m_ly, m_lz;
    double m_amplx, m_amply, m_amplz;
    double m_cell_size, m_dx, m_dy, m_dz;
    std::vector<Cell> m_cellule;

public:
    
    Grid();
    ~Grid();
    void initialize_linked_cells(double, double, double, double, double, double, double);
    void add_sphere(Sphere&);
    void reset();
};
