/**
 * @file ExtrinsicsCalculator.h
 * @brief That Function calculates the extrinsics between two camreas.
 * @version 0.1
 * @date 2023-06-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef EXTRINSICCALCULATOR_H
#define EXTRINSICCALCULATOR_H

#include "Eigen/Dense"

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/core/eigen.hpp>

#include <Types.hpp>
#include <Image.hpp>

namespace reconstruction
{

/**
 * TODO:
 * Check homography. Add the cv::sfm module.
 * 
 */


class ExtrinsicsCalculator
{
private:
    cv::Mat homography_;
    Eigen::Matrix3d homography_eigen_;
    std::vector<PointPair> normalized_matching_points_;
public:
    ExtrinsicsCalculator();
    ~ExtrinsicsCalculator();
    void eightPointMatching(std::vector<PointPair> matching_points_);
    Eigen::Matrix3d getHomography();
//    void multiplyIntrinsics(std::vector<PointPair> matching_points_);

};
};

#endif //EXTRINSICCALCULATOR_H