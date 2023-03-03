#include<vector>

class Sphere;

class Cell
{
    
private:
    
    int m_index;
    double m_x, m_y, m_z;
    Sphere* m_sphere_head_node;
    std::vector<Cell*> m_neighbor_ptrs;
    
public:
    
    Cell(int,double,double,double);
    ~Cell();
    Sphere* head_node();
    void set_head_node(Sphere*);
    void add_neighbor(Cell*);
    double x();
    double y();
    double z();
    std::vector<Cell*>& neighbor_ptrs();
};


