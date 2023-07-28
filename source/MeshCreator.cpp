#include "MeshCreator.h"

namespace reconstruction

{

MeshCreator::MeshCreator(/* args */)
{
    max_x_ = 0;
    max_y_ = 0;
    min_x_ = MAXFLOAT;
    min_y_ = MAXFLOAT;
}

MeshCreator::~MeshCreator()
{
}

/**
 * @brief Create the mesh from the calculated point cloud and the normals.
 * 
 * @param cgal_cloud 
 * @param spacing 
 */
void MeshCreator::createMesh(Point_set cgal_cloud, double spacing)
{
    getBoundaries(cgal_cloud);
    std::cout<<"Started Poisson"<<std::endl;
    CGAL::poisson_surface_reconstruction_delaunay
        (cgal_cloud.begin(), cgal_cloud.end(),
        cgal_cloud.point_map(), cgal_cloud.normal_map(),
        output_mesh_, spacing);
    CGAL::IO::write_PLY("out_poisson.ply", output_mesh_);
    std::cout<<"Finished"<<std::endl;
}


void MeshCreator::getBoundaries(Point_set cgal_cloud)
{
    for(auto it = cgal_cloud.points().begin(); it != cgal_cloud.points().end(); it++)
    {
        auto point = *it;
        if(max_x_ < point.x())
        {
            max_x_ = point.x();
        }
        if(max_y_ < point.y())
        {
            max_y_ = point.y();
        }
        if(min_x_ > point.x())
        {
            min_x_ = point.x();
        }
        if(min_y_ > point.y())
        {
            min_y_ = point.y();
        }
    }
    std::cout<<"Max = ("<<max_x_<<", "<<max_y_<<")"<<std::endl;
    std::cout<<"Min = ("<<min_x_<<", "<<min_y_<<")"<<std::endl;
}

};