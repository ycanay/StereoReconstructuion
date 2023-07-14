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
    fundamental_ = cv::findFundamentalMat(left_points, right_points, cv::FM_8POINT);
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
    lined_images_.baseline_ = images.baseline_;
    lined_images_.doffs = images.doffs;
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
    findCorrectTranslations(matching_points, images);
    rectifyImage(images);
    saveImages(images);

}

Eigen::Vector3d ExtrinsicsCalculator::calculateLeftEpipole()
{
    Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(fundamental_eigen_);
    Eigen::Matrix3d V = svd.matrixV().transpose();
    Eigen::Vector3d e = V.row(2);
    e = e/ e[2];
    return e;
}

Eigen::Vector3d ExtrinsicsCalculator::calculateRightEpipole()
{
    Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd(fundamental_eigen_.transpose());
    Eigen::Matrix3d V = svd.matrixV().transpose();
    Eigen::Vector3d e = V.row(2);
    e = e/ e[2];
    return e;
}


void ExtrinsicsCalculator::findCorrectTranslations(std::vector<PointPair> matching_points, ImagePair images){
    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    cv::Mat intrinsics;
    cv::eigen2cv(images.left_image.getCameraIntrinsics(),intrinsics);
    for(auto pair:matching_points)
    {
        left_points.push_back(pair.left_point);
        right_points.push_back(pair.right_point);
    }
    cv::recoverPose(essential_, left_points, right_points, intrinsics, R_, T_);
    T_*= images.baseline_;
}

void ExtrinsicsCalculator::rectifyImage(ImagePair images)
{
    cv::Mat distortion;
    cv::Mat intr_left, intr_right;
    cv::eigen2cv(images.left_image.getCameraIntrinsics(),intr_left);
    cv::eigen2cv(images.right_image.getCameraIntrinsics(),intr_right);
    cv::Size img_size(images.left_image.getData().cols,images.left_image.getData().rows);
    cv::stereoRectify(intr_left, distortion, intr_right, distortion, img_size, 
                      R_, T_, left_camera_rect_rot_, right_camera_rect_rot_, 
                      left_projection_matrix_, right_projection_matrix_, 
                      disparity_map_matrix_, cv::CALIB_ZERO_DISPARITY,
                      -1, img_size, &ROI_left_, &ROI_right_);

    cv::initUndistortRectifyMap(intr_left, distortion, left_camera_rect_rot_, left_projection_matrix_, img_size, CV_16SC2, mappings_left_[0], mappings_left_[1]);
    cv::initUndistortRectifyMap(intr_right, distortion, right_camera_rect_rot_, right_projection_matrix_, img_size, CV_16SC2, mappings_right_[0], mappings_right_[1]);

    remap(images.left_image.getData(), rectified_image_left_, mappings_left_[0], mappings_left_[1], cv::INTER_LINEAR);
    remap(images.right_image.getData(), rectified_image_right_, mappings_right_[0], mappings_right_[1], cv::INTER_LINEAR);

}

void ExtrinsicsCalculator::saveImages(ImagePair images)
{
    cv::imwrite("rectified_image_left_uncropped.jpg", rectified_image_left_);
    cv::imwrite("rectified_image_right_uncropped.jpg", rectified_image_right_);
    cv::Mat out;
    cv::hconcat(rectified_image_left_, rectified_image_right_, out);
    for(int i=1; i < floor(out.rows/25); i++)
    {
        float y_point = (out.rows/25) * i;
        cv::Point p1(0, y_point);
        cv::Point p2(out.cols, y_point);
        cv::line(out, p1, p2, cv::Scalar(0, 255, 0), 2);

    }

    cv::imwrite("rectified_together.jpeg", out);
}

ImagePair ExtrinsicsCalculator::getRectifiedImages()
{
    ImagePair return_pair;
    Image left(rectified_image_left_, lined_images_.left_image.getCameraIntrinsics());
    Image right(rectified_image_right_, lined_images_.right_image.getCameraIntrinsics());
    return_pair.left_image = left;
    return_pair.right_image = right;
    return_pair.baseline_ = lined_images_.baseline_;
    return_pair.doffs = lined_images_.doffs;
    return return_pair;
}

cv::Mat ExtrinsicsCalculator::getDisparityMatrix()
{
    return disparity_map_matrix_;
}

cv::Rect ExtrinsicsCalculator::getROILeft()
{
    return ROI_left_;
}

cv::Rect ExtrinsicsCalculator::getROIRight()
{
    return ROI_right_;
}


};