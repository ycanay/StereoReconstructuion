#include "NormalCalculator.h"

namespace reconstruction
{

NormalCalculator::NormalCalculator(/* args */)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_ = cloudPTR;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    normals_ = normals;
}

NormalCalculator::~NormalCalculator()
{
}


void NormalCalculator::calculateNormals(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    cloud_ = cloud.makeShared();
    ne_.setInputCloud(cloud_);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne_.setSearchMethod(tree);
    ne_.setKSearch(8);
    ne_.compute(*normals_);
    std::cout<<normals_->size()<<std::endl;
}

void NormalCalculator::visualise(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_, normals_);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}



};