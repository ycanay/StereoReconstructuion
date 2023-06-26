#include <ExtrinsicsCalculator.h>

namespace reconstruction
{
/**
 * @brief Construct a new Extrinsics Calculator:: Extrinsics Calculator object
 * 
 */
ExtrinsicsCalculator::ExtrinsicsCalculator()
{
    detector_ = cv::SIFT::create();
    k_min_hessian_ = 400;
    k_ratio_threshold_ = 0.1f;
    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
}

/**
 * @brief Destroy the Extrinsics Calculator:: Extrinsics Calculator object
 * 
 */
ExtrinsicsCalculator::~ExtrinsicsCalculator()
{
}

/**
 * @brief Construct a new Extrinsics Calculator:: Extrinsics Calculator object
 * 
 * @param detector_type Type of the detector
 * @param min_hessian 
 * @param ratio_threshold 
 */
ExtrinsicsCalculator::ExtrinsicsCalculator(DetectorType detector_type, int min_hessian, float ratio_threshold)
{
    switch (detector_type)
    {
    case SIFT:
        detector_ = cv::SIFT::create();
        break;
    case ORB:
        detector_ = cv::ORB::create();
        break;
    case SURF:
        detector_ = cv::xfeatures2d::SURF::create();
        break;
    default:
        detector_ = cv::SIFT::create();
        break;
    }
    k_min_hessian_ = min_hessian;
    k_ratio_threshold_ = ratio_threshold;
    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
}

/**
 * @brief Calculate all of the matches and return them
 * 
 * @param left_image 
 * @param right_image 
 * @return Matchings  All of the good matchings.
 */
Matchings ExtrinsicsCalculator::getMatches(Image left_image, Image right_image)
{
    KeyPoints keypoints_left, keypoints_right;
    detector_->detectAndCompute( left_image.getData(), cv::noArray(), keypoints_left.keypoints, keypoints_left.descriptors );
    detector_->detectAndCompute( right_image.getData(), cv::noArray(), keypoints_right.keypoints, keypoints_right.descriptors );

    std::vector< std::vector<cv::DMatch> > knn_matches;
    matcher_->knnMatch( keypoints_left.descriptors, keypoints_right.descriptors, knn_matches, 2 );
    Matchings matches;
    matches.left_keypoints = keypoints_left;
    matches.right_keypoints = keypoints_right;
    for (size_t ratio = 1; ratio < 20; ratio++)
    {
        matches.good_matches.clear();
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ((float)ratio*0.05) * knn_matches[i][1].distance)
            {
                matches.good_matches.push_back(knn_matches[i][0]);
            }
        }
        if (matches.good_matches.size() > 16)
            break;
    }
    return matches;
}

/**
 * @brief Draw the matches and save them
 * 
 * @param matchings Matchings gathered before
 * @param left_image 
 * @param right_image 
 */
void ExtrinsicsCalculator::drawMatches(Matchings matchings, Image left_image, Image right_image)
{
    cv::Mat img_matches;
    cv::drawMatches( left_image.getData(), matchings.left_keypoints.keypoints, right_image.getData(), matchings.right_keypoints.keypoints, matchings.good_matches, img_matches, cv::Scalar::all(-1),
    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    cv::imwrite("test.jpg", img_matches);
}

};