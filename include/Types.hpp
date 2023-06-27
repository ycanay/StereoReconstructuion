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
};

};//reconstruction

#endif //TYPES_H