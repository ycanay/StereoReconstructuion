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