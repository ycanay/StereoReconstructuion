#include "NormalCalculator.h"

namespace reconstruction
{
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Sphere_3 Sphere_3;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;


/**
 * @brief Constructor
 * 
 */
NormalCalculator::NormalCalculator(/* args */)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cloud_ = cloudPTR;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl_normals_ = normals;
}


/**
 * @brief Destroy the Normal Calculator:: Normal Calculator objectD
 * 
 */
NormalCalculator::~NormalCalculator()
{
}


/**
 * @brief Calculate the normals using pcl functions
 * 
 * @param cloud 
 */
void NormalCalculator::pclCalculateNormals(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    pcl_cloud_ = cloud.makeShared();
    pcl_ne_.setInputCloud(pcl_cloud_);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl_ne_.setSearchMethod(tree);
    pcl_ne_.setKSearch(8);
    pcl_ne_.compute(*pcl_normals_);
    std::cout<<pcl_normals_->size()<<std::endl;
}


/**
 * @brief Visualise the normal with pcl
 * 
 * @param cloud 
 */
void NormalCalculator::pclVisualise(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pcl_cloud_, pcl_normals_);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}


/**
 * @brief Load the cloud from the file saved on the Dense Point Creator using CGAL functions
 * 
 */
void NormalCalculator::cgalLoadCloud()
{
    std::string fname = "Point_clouds.ply";
    std::ifstream stream (fname, std::ios_base::binary);
    if (!stream)
    {
        std::cerr << "Error: cannot read file " << fname << std::endl;
        return;
    }
    stream >> cgal_cloud_;

    std::cout << "Read " << cgal_cloud_.size () << " point(s)" << std::endl;
    if (cgal_cloud_.empty())
        return;
}


/**
 * @brief Preprocess Cloud to reduce number of points and remove outlier points uisng CGAL functions
 * 
 */
void NormalCalculator::cgalPreprocessCloud()
{
    typename Point_set::iterator rout_it = CGAL::remove_outliers<CGAL::Sequential_tag>
        (cgal_cloud_,
        24, // Number of neighbors considered for evaluation
        cgal_cloud_.parameters().threshold_percent (10.0)); // Percentage of points to remove
    cgal_cloud_.remove(rout_it, cgal_cloud_.end());
    std::cout << cgal_cloud_.number_of_removed_points()
                << " point(s) are outliers." << std::endl;
    cgal_cloud_.collect_garbage();


    spacing_ = CGAL::compute_average_spacing<CGAL::Sequential_tag> (cgal_cloud_, 6);
    CGAL::jet_smooth_point_set<CGAL::Sequential_tag> (cgal_cloud_, 24);
}


/**
 * @brief Save Processesd Cloud
 * 
 */
void NormalCalculator::cgalSaveProcessedCloud()
{
    std::cout<<cgal_cloud_.size()<< " point(s) left after normals." << std::endl;
    CGAL::IO::write_PLY("processed.ply", cgal_cloud_);
}


/**
 * @brief Calculate normals using CGAL functions
 * 
 */
void NormalCalculator::cgalCalculateNormals()
{
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>
        (cgal_cloud_, 24); // Use 12 neighbors
    // Orientation of normals, returns iterator to first unoriented point
    cgalOrientNormals();
}


/**
 * @brief 
 * 
 * @return Point_set 
 */
Point_set NormalCalculator::cgalGetCloud()
{
    return cgal_cloud_;
}


/**
 * @brief 
 * 
 * @return double 
 */
double NormalCalculator::cgalGetSpacing()
{
    return spacing_;
}



void NormalCalculator::cgalOrientNormals()
{
    for(auto it = cgal_cloud_.normals().begin(); it != cgal_cloud_.normals().end(); it++)
    {
      auto normal = *it;
      if (normal.z() < 0)
      {
        *it *= -1;
      }
    }
}

};