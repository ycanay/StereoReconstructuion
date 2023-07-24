/**
 * @file NormalCalculator.h
 * @brief This class needs to calculate normals of the points in pointcloud by principal component analysis.
 * @version 0.1
 * @date 2023-06-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
//TODO: Whole
#ifndef NORMAL_CALCULATOR_H
#define NORMAL_CALCULATOR_H

#include <cstdlib>
#include <vector>
#include <fstream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/scanline_orient_normals.h>
#include <CGAL/IO/write_ply_points.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Sphere_3 Sphere_3;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;

namespace reconstruction
{
class NormalCalculator
{
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> pcl_ne_;
    pcl::PointCloud<pcl::Normal>::Ptr pcl_normals_;
    Point_set cgal_cloud_;
    double spacing_;
public:
    NormalCalculator();
    ~NormalCalculator();
    void pclCalculateNormals(pcl::PointCloud<pcl::PointXYZRGB> cloud);
    void pclVisualise(pcl::PointCloud<pcl::PointXYZRGB> cloud);

    void cgalLoadCloud();
    void cgalCalculateNormals();
    void cgalPreprocessCloud();
    void cgalSaveProcessedCloud();
    Point_set cgalGetCloud();
    double cgalGetSpacing();
};

};

#endif //NORMAL_CALCULATOR_H