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
 * @return * void 
 */
void ExtrinsicsCalculator::calculateMatches(Image left_image, Image right_image)
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
    size_t size_left = keypoints_left.keypoints.size();
    size_t size_right = keypoints_right.keypoints.size();
    good_matchings_ = matches;
    std::sort(good_matchings_.good_matches.begin(), good_matchings_.good_matches.end(),
            [](cv::DMatch const &a, cv::DMatch const &b) {
                return a.distance<b.distance;
            });
}

/**
 * @brief Draw the matches and save them
 * 
 * @param left_image 
 * @param right_image 
 */
void ExtrinsicsCalculator::drawMatches(Image left_image, Image right_image)
{
    cv::Mat img_matches;
    cv::drawMatches( left_image.getData(), good_matchings_.left_keypoints.keypoints, right_image.getData(), good_matchings_.right_keypoints.keypoints, good_matchings_.good_matches, img_matches, cv::Scalar::all(-1),
    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    cv::imwrite("test.jpg", img_matches);
}

/**
 * @brief Calculate x,y coordinates of the matchings.
 * 
 */
void ExtrinsicsCalculator::calculateMatchingPointsCoordinates()
{
    for(auto match:good_matchings_.good_matches){
        PointPair new_pair;
        new_pair.left_point = cv::Point2f(good_matchings_.left_keypoints.keypoints[match.queryIdx].pt);
        new_pair.right_point = cv::Point2f(good_matchings_.right_keypoints.keypoints[match.trainIdx].pt);
        matching_points_.push_back(new_pair);
    }
}

/**
 * @brief Get matchings labeled as good.
 * 
 * @return Matchings 
 */
Matchings ExtrinsicsCalculator::getGoodMatchings()
{
    return this->good_matchings_;
}

/**
 * @brief Get coordinates of the good matchings
 * 
 * @return std::vector<PointPair> List of coordinates of matching points
 */
std::vector<PointPair> ExtrinsicsCalculator::getMatchingPointCoordinates()
{
    return this->matching_points_;
}

void ExtrinsicsCalculator::eightPointMatching()
{
    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    for(auto pair:matching_points_)
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

void ExtrinsicsCalculator::multiplyIntrinsics(Image left_image, Image right_image)
{
    Eigen::Matrix3d left_intrinsic = left_image.getCameraIntrinsics();
    Eigen::Matrix3d right_intrinsic = right_image.getCameraIntrinsics();
    
    for(PointPair pair:matching_points_)
    {
        Eigen::Vector3d left_point = Eigen::Vector3d(pair.left_point.x, pair.left_point.y, 1);
        Eigen::Vector3d right_point = Eigen::Vector3d(pair.right_point.x, pair.right_point.y, 1);
        left_point = left_intrinsic * left_point;
        right_point = right_intrinsic * right_point;
        PointPair new_pair;
        new_pair.left_point = cv::Point2f(left_point[0],left_point[1]);
        new_pair.right_point = cv::Point2f(right_point[0],right_point[1]);
        normalized_matching_points_.push_back(new_pair);
    }
}
};