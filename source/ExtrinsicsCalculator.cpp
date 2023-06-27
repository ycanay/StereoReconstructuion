#include <ExtrinsicsCalculator.h>

namespace reconstruction
{
/**
 * @brief Construct a new Extrinsics Calculator:: Extrinsics Calculator object
 * 
 */
ExtrinsicsCalculator::ExtrinsicsCalculator()
{
}

/**
 * @brief Destroy the Extrinsics Calculator:: Extrinsics Calculator object
 * 
 */
ExtrinsicsCalculator::~ExtrinsicsCalculator()
{
}

/**
 * @brief Calculate essential matrix
 * 
 * @param matching_points Points to match
 */
void ExtrinsicsCalculator::eightPointMatching(std::vector<PointPair> matching_points)
{
    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    for(auto pair:matching_points)
    {
        left_points.push_back(pair.left_point);
        right_points.push_back(pair.right_point);
    }
    homography_ = cv::findHomography(left_points,right_points);
    cv::cv2eigen(homography_, homography_eigen_);
}

Eigen::Matrix3d ExtrinsicsCalculator::getHomography()
{
    return homography_eigen_;
}

/*
void ExtrinsicsCalculator::multiplyIntrinsics(std::vector<PointPair> matching_points)
{
    Eigen::Matrix3d left_intrinsic = left_image.getCameraIntrinsics();
    Eigen::Matrix3d right_intrinsic = right_image.getCameraIntrinsics();
    
    for(PointPair pair:matching_points)
    {
        Eigen::Vector3d left_point = Eigen::Vector3d(pair.left_point.x, pair.left_point.y, 1);
        Eigen::Vector3d right_point = Eigen::Vector3d(pair.right_point.x, pair.right_point.y, 1);
        left_point = left_intrinsic * left_point;
        right_point = right_intrinsic * right_point;
        PointPair new_pair;
        new_pair.left_point = cv::Point2f(left_point[0],left_point[1]);
        new_pair.right_point = cv::Point2f(right_point[0],right_point[1]);
        normalized_matching_points.push_back(new_pair);
    }
}
*/
};