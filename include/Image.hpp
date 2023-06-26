/**
 * @file Image.hpp
 * @brief Image class that we use for the project.
 * @version 0.1
 * @date 2023-06-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace reconstruction{

class Image
{
private:
    cv::Mat image_data_;
    Eigen::Matrix3d camera_intrinsics_;
public:
    Image()
    {
        image_data_ = cv::Mat();
        camera_intrinsics_ = Eigen::Matrix3d();
    }

    Image(cv::Mat data, Eigen::Matrix3d camera_intrinsics)
    {
        data.copyTo(image_data_);
        camera_intrinsics = camera_intrinsics;
    }

    ~Image()
    {
        
    }

    cv::Mat getData(){
        return image_data_;
    }

    Eigen::Matrix3d getCameraIntrinsics(){
        return camera_intrinsics_;
    }
};

};
#endif //IMAGE_HPP