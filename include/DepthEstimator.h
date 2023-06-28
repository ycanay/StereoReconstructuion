/**
 * @file DepthEstimator.h
 * @brief This class needs to create depthmap from the essential and fundamental matrices.
 * @version 0.1
 * @date 2023-06-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
//TODO: Whole
#ifndef DEPTH_ESTIMATOR_H
#define DEPTH_ESTIMATOR_H

#include "opencv2/core.hpp"
#include "Types.hpp"

namespace reconstruction{

class DepthEstimator
{
private:
    /* data */
public:
    DepthEstimator(/* args */);
    ~DepthEstimator();
    cv::Mat getDepthMap(ImagePair images); // Feel free to change arugments.
};

};

#endif //DEPTH_ESTIMATOR_H