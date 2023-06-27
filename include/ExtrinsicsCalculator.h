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


#include <Image.hpp>

namespace reconstruction
{

struct KeyPoints
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

struct Matchings
{
    KeyPoints left_keypoints;
    KeyPoints right_keypoints;
    std::vector<cv::DMatch> good_matches;
};

struct PointPair
{
    cv::Point2f left_point;
    cv::Point2f right_point;
};

enum DetectorType{
    SIFT,
    ORB,
    SURF
};

class ExtrinsicsCalculator
{
private:
    cv::Ptr<cv::Feature2D> detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    int k_min_hessian_;
    float k_ratio_threshold_;
    Matchings good_matchings_;
    std::vector<PointPair> matching_points_;
    cv::Mat homography_;
    Eigen::Matrix3d homography_eigen_;
    std::vector<PointPair> normalized_matching_points_;
public:
    ExtrinsicsCalculator();
    ExtrinsicsCalculator(DetectorType detector_type, int min_hessian, float ratio_threshold);
    ~ExtrinsicsCalculator();
    void calculateMatches(Image left_image, Image right_image);
    void drawMatches(Image left_image, Image right_image);
    void eightPointMatching();
    void calculateMatchingPointsCoordinates();
    Matchings getGoodMatchings();
    std::vector<PointPair> getMatchingPointCoordinates();
    Eigen::Matrix3d getHomography();
    void multiplyIntrinsics(Image left_image, Image right_image);

};
};

#endif //EXTRINSICCALCULATOR_H