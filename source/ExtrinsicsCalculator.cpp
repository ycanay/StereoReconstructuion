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


void ExtrinsicsCalculator::eightPointMatchingCV(std::vector<PointPair> matching_points)
{
    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    for(auto pair:matching_points)
    {
        left_points.push_back(pair.left_point);
        right_points.push_back(pair.right_point);
    }
    fundamental_ = cv::findFundamentalMat(left_points,right_points, cv::FM_8POINT);
    cv::cv2eigen(fundamental_, fundamental_eigen_);
}


void ExtrinsicsCalculator::drawEpipolarLines(std::vector<PointPair> matching_points, ImagePair images)
{
    cv::Mat left_image = images.left_image.getData();
    cv::Mat right_image = images.right_image.getData();
    for(auto pair:matching_points)
    {
        Eigen::Vector3d left_point(pair.left_point.x, pair.left_point.y, 1);
        Eigen::Vector3d epipolar_line = fundamental_eigen_ * left_point;
        cv::Point p1(0, epipolar_line[2] / (-epipolar_line[1]));
        cv::Point p2(right_image.cols, (epipolar_line[2] + right_image.cols*epipolar_line[0]) / (-epipolar_line[1]));
        cv::line(right_image, p1, p2, cv::Scalar(0, 0, 255), 3);
        cv::circle(right_image, pair.left_point, 5, cv::Scalar(0,0,0),5);

        Eigen::Vector3d right_point(pair.right_point.x, pair.right_point.y, 1);
        epipolar_line = fundamental_eigen_.transpose() * right_point;
        p1 = cv::Point(0, epipolar_line[2] / (-epipolar_line[1]));
        p2 = cv::Point(left_image.cols, (epipolar_line[2] + left_image.cols*epipolar_line[0]) / (-epipolar_line[1]));
        cv::line(left_image, p1, p2, cv::Scalar(255, 0, 0), 3);
        cv::circle(left_image, pair.right_point, 5, cv::Scalar(0,0,0),5);

    }
    cv::Mat output;
    cv::hconcat(left_image, right_image, output);
    cv::imwrite("output.jpg", output);
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