/**
 * @file Types.hpp
 * @brief This file is where all of the custom types defined.
 * @version 0.1
 * @date 2023-06-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef TYPES_H
#define TYPES_H

#include "Image.hpp"

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

struct ImagePair
{
    Image left_image;
    Image right_image;
    float baseline_;
    float doffs;
};

};//reconstruction

#endif //TYPES_H