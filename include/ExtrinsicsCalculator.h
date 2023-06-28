/**
 * @file ExtrinsicsCalculator.h
 * @brief That Function calculates the essential and the fundamental matrices.
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
#include <opencv2/sfm/fundamental.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

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
    cv::Mat fundamental_;
    Eigen::Matrix3d fundamental_eigen_;
public:
    ExtrinsicsCalculator();
    ~ExtrinsicsCalculator();
    void eightPointMatching(std::vector<PointPair> matching_points);
    void eightPointMatchingCV(std::vector<PointPair> matching_points);
    Eigen::Matrix3d getHomography();
    void drawEpipolarLines(std::vector<PointPair> matching_points, ImagePair images);

//    void multiplyIntrinsics(std::vector<PointPair> matching_points_);

};
};

#endif //EXTRINSICCALCULATOR_H