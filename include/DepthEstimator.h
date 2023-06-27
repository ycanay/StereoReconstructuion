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