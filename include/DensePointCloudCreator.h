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
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
namespace reconstruction{

enum MatcherType
{
    BM,
    SGBM
};

class DensePointCloudCreator
{
private:
    cv::Ptr<cv::StereoMatcher> left_matcher_;
    cv::Ptr<cv::StereoMatcher> right_matcher_;
    cv::Ptr<cv::StereoSGBM> sbgm_matcher_;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> filter_;
    cv::Mat left_disparity_;
    cv::Mat right_disparity_;
    cv::Mat filtered_disparity_;
    cv::Mat reProjectedPoints_;
    MatcherType type_;

    void saveDisparityMap();
public:
    DensePointCloudCreator(MatcherType type);
    DensePointCloudCreator();
    ~DensePointCloudCreator();
    void createPointCloud(ImagePair images, cv::Mat transform);
};

};


#endif //DENSEPOINTCLOUDCREATOR_H