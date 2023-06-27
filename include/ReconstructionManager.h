/**
 * @file ReconstructionManager.h
 * @brief Main class that contains whole pipeline.
 * @version 0.1
 * @date 2023-06-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef RECONSTRUCTIONMANAGER_H
#define RECONSTRUCTIONMANAGER_H

#include <DatasetLoader.h>
#include <ExtrinsicsCalculator.h>

namespace reconstruction{

class ReconstructionManager
{
private:
    //Components
    DatasetLoader dataset_loader_;
    cv::Ptr<cv::Feature2D> detector_;
    ExtrinsicsCalculator extrinsic_calculator_;

    //Variables
    ImagePair images_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    Matchings good_matchings_;
    std::vector<PointPair> matching_points_;

    //Constants
    int k_min_hessian_;

public:
    ReconstructionManager();
    ~ReconstructionManager();
    
    void process();

    void calculateMatches();
    void drawMatches();
    void calculateMatchingPointsCoordinates();
    Matchings getGoodMatchings();
    std::vector<PointPair> getMatchingPointCoordinates();
    void setDetectorType(DetectorType detector_type);
};

};
#endif //RECONSTRUCTIONMANAGER_H