/**
 * @file ExtrinsicsCalculator.h
 * @brief That Function calculates the essential and fundamental matrices and rectifying transforms.
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
    cv::Mat R_;
    cv::Mat T_;
    cv::Mat left_camera_rect_rot_;
    cv::Mat right_camera_rect_rot_;
    cv::Mat left_projection_matrix_;
    cv::Mat right_projection_matrix_;
    cv::Mat disparity_map_matrix_;
    cv::Rect ROI_left_;
    cv::Rect ROI_right_;
    cv::Mat mappings_left_[2];
    cv::Mat mappings_right_[2];
    cv::Mat rectified_image_left_;
    cv::Mat rectified_image_right_;

    void eightPointMatching(std::vector<PointPair> matching_points);
    void eightPointMatchingCV(std::vector<PointPair> matching_points);
    Eigen::Matrix3d getHomography();
    void drawEpipolarLines(std::vector<PointPair> matching_points, ImagePair images);
    void calculateEssentialMatrix(ImagePair images);
    void calculateTransofmration(std::vector<PointPair> matching_points);
    void drawRectifiedImages();
    void calculateRectifiedImages();
    void calculateDisparity();
    Eigen::Vector3d calculateLeftEpipole();
    Eigen::Vector3d calculateRightEpipole();
    void findCorrectTranslations(std::vector<PointPair> matching_points, ImagePair images);
    void rectifyImage(ImagePair images);
    void saveImages(ImagePair images);


public:
    ExtrinsicsCalculator();
    ~ExtrinsicsCalculator();
    void process(std::vector<PointPair> matching_points, ImagePair images);
    Eigen::Matrix3d getEssentialMatrix();
    cv::Mat getEssentialMat();
    Eigen::Matrix3d getFundamentalMatrix();
    cv::Mat getFundamentalMat();
    ImagePair getRectifiedImages();
    cv::Mat getDisparityMatrix();
    cv::Rect getROILeft();
    cv::Rect getROIRight();
};
};

#endif //EXTRINSICCALCULATOR_H