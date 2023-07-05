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
    cv::Mat left_image,right_image; 
    images.left_image.getData().copyTo(left_image);
    images.right_image.getData().copyTo(right_image);
    for(auto pair:matching_points)
    {
        Eigen::Vector3d left_point(pair.left_point.x, pair.left_point.y, 1);
        Eigen::Vector3d epipolar_line = fundamental_eigen_ * left_point;
        cv::Point p1(0, epipolar_line[2] / (-epipolar_line[1]));
        cv::Point p2(right_image.cols, (epipolar_line[2] + right_image.cols*epipolar_line[0]) / (-epipolar_line[1]));
        cv::line(right_image, p1, p2, cv::Scalar(0, 255, 0), 2);
        cv::circle(right_image, pair.left_point, 5, cv::Scalar(0,0,0),5);

        Eigen::Vector3d right_point(pair.right_point.x, pair.right_point.y, 1);
        epipolar_line = fundamental_eigen_.transpose() * right_point;
        p1 = cv::Point(0, epipolar_line[2] / (-epipolar_line[1]));
        p2 = cv::Point(left_image.cols, (epipolar_line[2] + left_image.cols*epipolar_line[0]) / (-epipolar_line[1]));
        cv::line(left_image, p1, p2, cv::Scalar(0, 255, 0), 2);
        cv::circle(left_image, pair.right_point, 5, cv::Scalar(0,0,0),5);

    }
    lined_images_.left_image = Image(left_image, images.left_image.getCameraIntrinsics());
    lined_images_.right_image = Image(right_image, images.right_image.getCameraIntrinsics());
    cv::Mat output;
    cv::hconcat(left_image, right_image, output);
    cv::imwrite("output.jpg", output);
}



Eigen::Matrix3d ExtrinsicsCalculator::getHomography()
{
    return homography_eigen_;
}

void ExtrinsicsCalculator::calculateEssentialMatrix(ImagePair images)
{
    Eigen::Matrix3d K_1 = images.left_image.getCameraIntrinsics();
    Eigen::Matrix3d K_2 = images.right_image.getCameraIntrinsics();
    essential_eigen_ = K_2.transpose() * fundamental_eigen_ * K_1;
    cv::eigen2cv(essential_eigen_,essential_);
}

Eigen::Matrix3d ExtrinsicsCalculator::getEssentialMatrix()
{
    return essential_eigen_;
}

cv::Mat ExtrinsicsCalculator::getEssentialMat()
{
    return essential_;
}

Eigen::Matrix3d ExtrinsicsCalculator::getFundamentalMatrix()
{
    return fundamental_eigen_;
}

cv::Mat ExtrinsicsCalculator::getFundamentalMat()
{
    return fundamental_;
}

void ExtrinsicsCalculator::calculateTransofmration(std::vector<PointPair> matching_points)
{
    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    for(auto pair:matching_points)
    {
        left_points.push_back(pair.left_point);
        right_points.push_back(pair.right_point);
    }
    cv::stereoRectifyUncalibrated(left_points, right_points, fundamental_, cv::Size(lined_images_.left_image.getData().cols, lined_images_.left_image.getData().rows), H1_, H2_);

}

void ExtrinsicsCalculator::drawRectifiedImages()
{
    cv::imwrite("left_rect.jpeg", left_rect_);
    cv::imwrite("right_rect.jpeg", right_rect_);
    cv::Mat out;
    cv::hconcat(left_rect_, right_rect_, out);
    cv::imwrite("rectitied_concat.jpg", out);
}

void ExtrinsicsCalculator::calculateRectifiedImages()
{
    cv::warpPerspective(lined_images_.left_image.getData(), left_rect_, H1_, cv::Size(lined_images_.left_image.getData().cols, lined_images_.left_image.getData().rows));
    cv::warpPerspective(lined_images_.right_image.getData(), right_rect_, H2_, cv::Size(lined_images_.right_image.getData().cols, lined_images_.right_image.getData().rows));
}

void ExtrinsicsCalculator::process(std::vector<PointPair> matching_points, ImagePair images)
{
    eightPointMatchingCV(matching_points);
    drawEpipolarLines(matching_points, images);
    calculateEssentialMatrix(images);
    calculateTransofmration(matching_points);
    calculateRectifiedImages();
    drawRectifiedImages();
}

};