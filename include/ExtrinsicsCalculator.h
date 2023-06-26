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

#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

#include "Eigen/Dense"

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
public:
    ExtrinsicsCalculator();
    ExtrinsicsCalculator(DetectorType detector_type, int min_hessian, float ratio_threshold);
    ~ExtrinsicsCalculator();
    Matchings getMatches(Image left_image, Image right_image);
    void drawMatches(Matchings matchings, Image left_image, Image right_image);
};
};

#endif //EXTRINSICCALCULATOR_H