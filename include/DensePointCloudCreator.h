/**
 * @file DensePointCloudCreator.h
 * @brief This component needs to create dense point cloud from images and depth map.
 * @version 0.1
 * @date 2023-06-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//TODO: Whole
#ifndef DENSEPOINTCLOUDCREATOR_H
#define DENSEPOINTCLOUDCREATOR_H

#include <opencv2/core.hpp>
#include "Types.hpp"
namespace reconstruction{

class DensePointCloudCreator
{
private:
    /* data */
public:
    DensePointCloudCreator(/* args */);
    ~DensePointCloudCreator();
    void createPointCloud(cv::Mat depthMap, ImagePair images);
};

};


#endif //DENSEPOINTCLOUDCREATOR_H