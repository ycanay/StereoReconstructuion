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
        left_matcher_ = cv::StereoBM::create(160, 9);
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
        cv::reprojectImageTo3D(filtered_disparity_, reProjectedPoints_, transform, true);
    }
    else
    {
//        left_matcher_->setNumDisparities((images.left_image.getData().size().width/8) + 15);
        right_matcher_ = cv::ximgproc::createRightMatcher(left_matcher_);
        left_matcher_->compute(images.left_image.getData(), images.right_image.getData(), left_disparity_);
        right_matcher_->compute(images.right_image.getData(), images.left_image.getData(), right_disparity_);
        filter_->filter(left_disparity_, images.left_image.getData(), filtered_disparity_, right_disparity_ , cv::Rect(), images.right_image.getData());
        saveDisparityMap();
        cv::reprojectImageTo3D(left_disparity_, reProjectedPoints_, transform, true);
    }
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
        image_to_save = filtered_disparity_(roi);
        cv::ximgproc::getDisparityVis(image_to_save, image_to_save);
        cv::ximgproc::getDisparityVis(left_disparity_, left);
        cv::ximgproc::getDisparityVis(right_disparity_, right, -1);
        cv::imwrite("disparity_map.jpeg", image_to_save);
        cv::imwrite("left_disparity.jpeg", left);
        cv::imwrite("right_disparity.jpeg", right);
        cv::imwrite("confidence.jpeg", confidence);

    }
}

}