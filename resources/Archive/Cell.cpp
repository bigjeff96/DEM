#include <stdlib.h>

#include "Cell.h"

Cell::Cell(int i_index, double i_x, double i_y, double i_z)
{
    m_index = i_index;
    m_x = i_x;
    m_y = i_y;
    m_z = i_z;
    m_sphere_head_node = NULL;
}


Cell::~Cell()
{

}

Sphere* Cell::head_node()
{
    return m_sphere_head_node;
}

void Cell::add_neighbor(Cell* i_neihgbor)
{
    m_neighbor_ptrs.push_back(i_neihgbor);
}

void Cell::set_head_node(Sphere* i_sphere_head_node)
{
    m_sphere_head_node = i_sphere_head_node;
}

double Cell::x()
{
    return m_x;
}

double Cell::y()
{
    return m_y;
}

double Cell::z()
{
    return m_z;
}

std::vector<Cell*>& Cell::neighbor_ptrs()
{
    return m_neighbor_ptrs;
}
