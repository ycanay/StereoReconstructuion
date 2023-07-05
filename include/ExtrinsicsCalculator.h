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
#include "Eigen/Jacobi"

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
    cv::Mat essential_;
    Eigen::Matrix3d essential_eigen_;
    cv::Mat transformation_;
    Eigen::Matrix4d transformation_eigen_;
    ImagePair lined_images_;
    cv::Ptr<cv::StereoBM> macther_;
    cv::Mat H1_;
    cv::Mat H2_;
    cv::Mat left_rect_;
    cv::Mat right_rect_;

    void eightPointMatching(std::vector<PointPair> matching_points);
    void eightPointMatchingCV(std::vector<PointPair> matching_points);
    Eigen::Matrix3d getHomography();
    void drawEpipolarLines(std::vector<PointPair> matching_points, ImagePair images);
    void calculateEssentialMatrix(ImagePair images);
    void calculateTransofmration(std::vector<PointPair> matching_points);
    void drawRectifiedImages();
    void calculateRectifiedImages();

public:
    ExtrinsicsCalculator();
    ~ExtrinsicsCalculator();
    void process(std::vector<PointPair> matching_points, ImagePair images);
    Eigen::Matrix3d getEssentialMatrix();
    cv::Mat getEssentialMat();
    Eigen::Matrix3d getFundamentalMatrix();
    cv::Mat getFundamentalMat();

};
};

#endif //EXTRINSICCALCULATOR_H