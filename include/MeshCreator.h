/**
 * @file MeshCreator.h
 * @brief This class creates the mesh from the point cloud and normals.
 * @version 0.1
 * @date 2023-06-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
//TODO: Whole
#ifndef MESH_CREATOR_H
#define MESH_CREATOR_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <cstdlib>
#include <vector>
#include <fstream>
namespace reconstruction{
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Sphere_3 Sphere_3;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;

class MeshCreator
{
private:
    CGAL::Surface_mesh<Point_3> output_mesh_;

public:
    MeshCreator(/* args */);
    ~MeshCreator();
    void createMesh(Point_set cgal_cloud, double spacing);
};

};

#endif //MESH_CREATOR_H