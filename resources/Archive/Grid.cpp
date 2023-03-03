#include <stdlib.h>
#include <math.h>
#include <vector>

#include "Sphere.h"
#include "Contact.h"
#include "Cell.h"
#include "Grid.h"

Grid::Grid()
{
    
}

Grid::~Grid()
{
    
}

void Grid::initialize_linked_cells(double i_lx, double i_ly, double i_lz, double i_amplx, double i_amply, double i_amplz, double i_cell_size)
{
    //initialization of cell grid
    m_lx = i_lx;
    m_ly = i_ly;
    m_lz = i_lz;
    m_amplx = i_amplx;
    m_amply = i_amply;
    m_amplz = i_amplz;
    
    m_cell_size = i_cell_size;
    m_nbx = (m_lx+2.*m_amplx+m_cell_size)/m_cell_size;
    m_nby = (m_ly+2.*m_amply+m_cell_size)/m_cell_size;
    m_nbz = (m_lz+2.*m_amplz+m_cell_size)/m_cell_size;
    int number_of_cells = m_nbx*m_nby*m_nbz;
    m_cellule.reserve(number_of_cells);
    
    m_dx = (m_lx+2.*m_amplx+m_cell_size)/m_nbx;
    m_dy = (m_ly+2.*m_amply+m_cell_size)/m_nby;
    m_dz = (m_lz+2.*m_amplz+m_cell_size)/m_nbz;
    
    int ix,iy,iz;
    int jx,jy,jz;
    double x,y,z;

    for(int i=0 ; i<number_of_cells ; i++)
    {
        iz = i/(m_nbx*m_nby);
        iy = (i%(m_nbx*m_nby))/m_nbx;
        ix = (i%(m_nbx*m_nby))%m_nbx;
        x = -0.5*m_lx-m_amplx-0.5*m_cell_size+(0.5+ix)*m_dx;
        y = -0.5*m_ly-m_amply-0.5*m_cell_size+(0.5+iy)*m_dy;
        z = -0.5*m_lz-m_amplz-0.5*m_cell_size+(0.5+iz)*m_dz;
        m_cellule.emplace_back(i,x,y,z);
        
        for(int j=0 ; j<i ; j++)
        {
            jz = j/(m_nbx*m_nby);
            jy = (j%(m_nbx*m_nby))/m_nbx;
            jx = (j%(m_nbx*m_nby))%m_nbx;
            
            if(abs(ix-jx)<2 && abs(iy-jy)<2 && abs(iz-jz)<2)
            {
                m_cellule[i].add_neighbor(&m_cellule[j]);
            }
        }
    }
}

void Grid::add_sphere(Sphere& i_sph)
{
    int cell_index = (int)((i_sph.x()+m_lx/2.+m_amplx+m_cell_size/2.)/m_dx)
    +(int)((i_sph.y()+m_ly/2.+m_amply+m_cell_size/2.)/m_dy)*m_nbx
    +(int)((i_sph.z()+m_lz/2.+m_amplz+m_cell_size/2.)/m_dz)*m_nbx*m_nby;
    
    Cell* cl=&m_cellule[cell_index];
    i_sph.set_cell_ptr(cl);
    i_sph.set_next(cl->head_node());
    cl->set_head_node(&i_sph);
}

void Grid::reset()
{
    for(Cell& cll : m_cellule)
    {
        cll.set_head_node(NULL);
    }
}
