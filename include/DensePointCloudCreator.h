#ifndef DENSEPOINTCLOUDCREATOR_H
#define DENSEPOINTCLOUDCREATOR_H

#include <opencv2/core.hpp>
#include "Types.hpp"
namespace reconstruction{

class DensePointCloudCreator
{
private:
    /* data */
public:
    DensePointCloudCreator(/* args */);
    ~DensePointCloudCreator();
    void createPointCloud(cv::Mat depthMap, ImagePair images);
};

};


#endif //DENSEPOINTCLOUDCREATOR_H