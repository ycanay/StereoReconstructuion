#include <DatasetLoader.h>
#include <iostream>
namespace reconstruction
{

/**
 * @brief Construct a new Dataset Loader:: Dataset Loader object
 * 
 */
DatasetLoader::DatasetLoader()
{
    dataset_path_ = "data/Piano-perfect";
}

/**
 * @brief Destroy the Dataset Loader:: Dataset Loader object
 * 
 */
DatasetLoader::~DatasetLoader()
{
}

/**
 * @brief Get images from the datasetloader
 * 
 * @return ImagePair Left and righ images
 */
ImagePair DatasetLoader::getImages(){
    cv::Mat image_left = cv::imread("../" + dataset_path_ + "/im0.png", cv::IMREAD_COLOR);
    cv::Mat image_right = cv::imread("../" + dataset_path_ + "/im1.png", cv::IMREAD_COLOR);
    file_.open("../" + dataset_path_+"/calib.txt");
    int i = 0;
    Eigen::Matrix3d left_intrinsics;
    Eigen::Matrix3d right_intrinsics;
    for(std::string line; std::getline( file_, line );i++){
        line = line.substr(6,line.size()-7);
        std::vector<std::string> rows = split(line, ';');
        for (int j = 0; j < rows.size(); j++)
        {
            std::string row = rows[j];
            if(row.c_str()[0] == ' '){
                row = row.substr(1,row.size());
            }
            std::vector<std::string> vals = split(row, ' ');
            if(i == 0)
            {
                left_intrinsics.row(j) << std::stod(vals[0]), std::stod(vals[1]), std::stod(vals[2]);
            }
            else
            {
                right_intrinsics.row(j) << std::stod(vals[0]), std::stod(vals[1]), std::stod(vals[2]);
            }
        }
        if (i==1)
            break;
    }
    ImagePair images;
    images.left_image = Image(image_left, left_intrinsics);
    images.right_image = Image(image_right, right_intrinsics);
    return images;
}

/**
 * @brief Split the string
 * 
 * @param s string to be splited
 * @param delim delimiter
 * @return std::vector<std::string> list of strings.
 */
std::vector<std::string> DatasetLoader::split(const std::string &s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> elems;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}
}