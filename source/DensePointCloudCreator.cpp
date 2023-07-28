#include "DensePointCloudCreator.h"

namespace reconstruction
{

DensePointCloudCreator::DensePointCloudCreator()
{
    type_ = SGBM;
    sbgm_matcher_ = cv::StereoSGBM::create(0,256,5, 8*5*5*3, 32*5*5*3, 1, 63, 10, 200,32, cv::StereoSGBM::MODE_HH);
}

DensePointCloudCreator::DensePointCloudCreator(MatcherType type)
{
    type_ = type;
    if (type == BM)
    {
        left_matcher_ = cv::StereoBM::create(255, 9);
        left_matcher_->setMinDisparity(0);
        left_matcher_->setSpeckleWindowSize(200);
        left_matcher_->setSpeckleRange(32);
        left_matcher_->setDisp12MaxDiff(1);
        filter_ = cv::ximgproc::createDisparityWLSFilter(left_matcher_);
        filter_->setLambda(8000.0);
        filter_->setSigmaColor(1.5);
        right_matcher_ = cv::ximgproc::createRightMatcher(left_matcher_);
    }
    else
    {
        left_matcher_ = cv::StereoSGBM::create(0,256,5, 8*5*5*3, 32*5*5*3, 1, 63, 10, 200,32, cv::StereoSGBM::MODE_HH);
        filter_ = cv::ximgproc::createDisparityWLSFilter(left_matcher_);
        filter_->setLambda(8000.0);
        filter_->setSigmaColor(1.5);

    }
}

DensePointCloudCreator::~DensePointCloudCreator()
{
}

void DensePointCloudCreator::createPointCloud(ImagePair images, cv::Mat transform)
{
    if (type_ == BM)
    {
        cv::Mat left_grey_, right_grey_;
        cv::cvtColor(images.left_image.getData(), left_grey_, cv::COLOR_BGR2GRAY);
        cv::cvtColor(images.right_image.getData(), right_grey_, cv::COLOR_BGR2GRAY);
        left_matcher_->compute(left_grey_, right_grey_, left_disparity_);
        right_matcher_->compute(right_grey_, left_grey_, right_disparity_);
        filter_->filter(left_disparity_, left_grey_, filtered_disparity_, right_disparity_ );
        saveDisparityMap();
    }
    else
    {
        right_matcher_ = cv::ximgproc::createRightMatcher(left_matcher_);
        left_matcher_->compute(images.left_image.getData(), images.right_image.getData(), left_disparity_);
        right_matcher_->compute(images.right_image.getData(), images.left_image.getData(), right_disparity_);
        filter_->filter(left_disparity_, images.left_image.getData(), filtered_disparity_, right_disparity_ , cv::Rect(), images.right_image.getData());
        saveDisparityMap();
    }
    calculatePointCloud(images);
    saveCloud();
}

void DensePointCloudCreator::saveDisparityMap()
{
    if(type_ == BM)
    {
        cv::Mat image_to_save, left, right;
        cv::ximgproc::getDisparityVis(filtered_disparity_, image_to_save);
        cv::ximgproc::getDisparityVis(left_disparity_, left);
        cv::ximgproc::getDisparityVis(right_disparity_, right);
        cv::imwrite("disparity_map.jpeg", image_to_save);
        cv::imwrite("left_disparity.jpeg", left);
        cv::imwrite("right_disparity.jpeg", right);
    }
    else
    {
        /*
        cv::Mat left;
        cv::normalize(left_disparity_, left, 255, 0, cv::NORM_MINMAX);
        cv::imwrite("left_disparity.jpeg", left);*/
        cv::Mat confidence = filter_->getConfidenceMap();
        cv::Rect roi(0,0,filtered_disparity_.size().width*0.8, filtered_disparity_.size().height*0.8);
        cv::Mat image_to_save, left, right, conf;
        cv::ximgproc::getDisparityVis(filtered_disparity_, image_to_save);
        cv::ximgproc::getDisparityVis(left_disparity_, left);
        cv::ximgproc::getDisparityVis(right_disparity_, right, -1);
        cv::imwrite("disparity_map.jpeg", image_to_save);
        cv::imwrite("left_disparity.jpeg", left);
        cv::imwrite("right_disparity.jpeg", right);
        cv::imwrite("confidence.jpeg", confidence);

    }
}

void DensePointCloudCreator::calculatePointCloud(ImagePair images)
{
    Eigen::Matrix3d reverse_intr = images.left_image.getCameraIntrinsics().inverse();
    for(int i = 0; i < filtered_disparity_.size().width; i+=16)
    {
        for (int j = 0; j < filtered_disparity_.size().height; j+=16)
        {
            short pixVal = filtered_disparity_.at<short>(j, i);
            float disparity = pixVal / 16.0f;
            if(disparity > 255.0 || disparity < 0 || isnan(disparity) || isinf(disparity))
            {
                continue;
            }
            float depth = (images.baseline_ * images.left_image.getCameraIntrinsics().coeff(0,0)) / ( abs(disparity)+ images.doffs);
            Eigen::Vector3d point = reverse_intr * Eigen::Vector3d(i, j, 1);
            point *= depth / 5000;
            cv::Vec3b color = images.left_image.getData().at<cv::Vec3b>(cv::Point(i, j));
            pcl::PointXYZRGB rgb_point;
            rgb_point.x = point[0];
            rgb_point.y = point[1];
            rgb_point.z = point[2];

            rgb_point.b = color[0];
            rgb_point.g = color[1];
            rgb_point.r = color[2];

            cloud.push_back(rgb_point);
        }
    }
}

void DensePointCloudCreator::saveCloud()
{
    pcl::io::savePLYFile("Point_clouds.ply", cloud, false);
}


pcl::PointCloud<pcl::PointXYZRGB> DensePointCloudCreator::getCloud()
{
    return cloud;
}
}