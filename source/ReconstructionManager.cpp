#include <ReconstructionManager.h>

namespace reconstruction
{
/**
 * @brief Construct a new Reconstruction Manager:: Reconstruction Manager object
 * 
 */
ReconstructionManager::ReconstructionManager()
{
    detector_ = cv::SIFT::create();
    k_min_hessian_ = 400;
    k_min_number_of_matches_ = 100;
    matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    point_cloud_creator_ = DensePointCloudCreator(SGBM);
}

/**
 * @brief Destroy the Reconstruction Manager:: Reconstruction Manager object
 * 
 */
ReconstructionManager::~ReconstructionManager()
{
}

/**
 * @brief Set type of the detector either SIFT, ORB or SURF
 * 
 * @param detector_type 
 */
void ReconstructionManager::setDetectorType(DetectorType detector_type)
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
}


/**
 * @brief Detect keypoints and match them until we have enough of them
 * 
 */
void ReconstructionManager::detectKeypointsAndeMatch()
{
    KeyPoints keypoints_left, keypoints_right;
    detector_->detectAndCompute( images_.left_image.getData(), cv::noArray(), keypoints_left.keypoints, keypoints_left.descriptors );
    detector_->detectAndCompute( images_.right_image.getData(), cv::noArray(), keypoints_right.keypoints, keypoints_right.descriptors );

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
        if (matches.good_matches.size() > k_min_number_of_matches_)
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
void ReconstructionManager::drawMatches()
{
    cv::Mat img_matches;
    cv::drawMatches( images_.left_image.getData(), good_matchings_.left_keypoints.keypoints, images_.right_image.getData(), good_matchings_.right_keypoints.keypoints, good_matchings_.good_matches, img_matches, cv::Scalar::all(-1),
    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    cv::imwrite("test.jpg", img_matches);
}

/**
 * @brief Calculate x,y coordinates of the matchings.
 * 
 */
void ReconstructionManager::calculateMatchingPointsCoordinates()
{
    for(auto match:good_matchings_.good_matches){
        PointPair new_pair;
        new_pair.left_point = cv::Point2f(good_matchings_.left_keypoints.keypoints[match.queryIdx].pt);
        new_pair.right_point = cv::Point2f(good_matchings_.right_keypoints.keypoints[match.trainIdx].pt);
        matching_points_.push_back(new_pair);
    }
}

Matchings ReconstructionManager::getGoodMatchings()
{
    return this->good_matchings_;
}

/**
 * @brief Get coordinates of the good matchings
 * 
 * @return std::vector<PointPair> List of coordinates of matching points
 */
std::vector<PointPair> ReconstructionManager::getMatchingPointCoordinates()
{
    return this->matching_points_;
}


/**
 * @brief Main Function of the whole processor
 * 
 */
void ReconstructionManager::process()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    images_ = dataset_loader_.getImages();
    detectKeypointsAndeMatch();
    drawMatches();
    calculateMatchingPointsCoordinates();
    now = std::chrono::steady_clock::now();
    std::cout << "Keypoint extraction and matching took = " << std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() << "[ms]" << std::endl;
    extrinsic_calculator_.process(matching_points_, images_);
    std::cout << "Extrinsics Calculation took = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count() << "[ms]" << std::endl;
    now = std::chrono::steady_clock::now();
    point_cloud_creator_.createPointCloud(extrinsic_calculator_.getRectifiedImages(), extrinsic_calculator_.getDisparityMatrix());
    std::cout << "Point Cloud Calculation took = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count() << "[ms]" << std::endl;
    now = std::chrono::steady_clock::now();
    normal_calculator_.cgalLoadCloud();
    normal_calculator_.cgalPreprocessCloud();
    normal_calculator_.cgalSaveProcessedCloud();
    normal_calculator_.cgalCalculateNormals();
    std::cout << "Normal Calculation took = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count() << "[ms]" << std::endl;
    now = std::chrono::steady_clock::now();
    mesh_creator_.createMesh(normal_calculator_.cgalGetCloud(), normal_calculator_.cgalGetSpacing());
    std::cout << "Mesh Creation took = " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - now).count() << "[ms]" << std::endl;
    now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Whole Process = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
}


};
