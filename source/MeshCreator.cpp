#include "MeshCreator.h"

namespace reconstruction

{

MeshCreator::MeshCreator(/* args */)
{
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
    std::cout<<"Started Poisson"<<std::endl;
    CGAL::poisson_surface_reconstruction_delaunay
        (cgal_cloud.begin(), cgal_cloud.end(),
        cgal_cloud.point_map(), cgal_cloud.normal_map(),
        output_mesh_, spacing);
    CGAL::IO::write_PLY("out_poisson.ply", output_mesh_);
    std::cout<<"Finished"<<std::endl;
}

};