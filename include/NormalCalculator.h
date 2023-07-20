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

#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

namespace reconstruction
{
class NormalCalculator
{
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
public:
    NormalCalculator();
    ~NormalCalculator();
    void setCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud);
    void calculateNormals(pcl::PointCloud<pcl::PointXYZRGB> cloud);
    void visualise(pcl::PointCloud<pcl::PointXYZRGB> cloud);
};

};

#endif //NORMAL_CALCULATOR_H