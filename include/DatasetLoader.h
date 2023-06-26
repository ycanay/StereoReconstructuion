/**
 * @file DatasetLoader.h
 * @brief Loader class that reads images from folder.
 * @version 0.1
 * @date 2023-06-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef DATASETLOADER_H
#define DATASETLOADER_H

#include "opencv2/core.hpp"
#include <opencv2/imgcodecs.hpp>

#include <vector>
#include <fstream>

#include <Image.hpp>


namespace reconstruction{

class DatasetLoader
{
private:
    std::string dataset_path_;
    Image image_left_;
    Image image_right_;
    std::ifstream file_;
    std::vector<std::string> split(const std::string &s, char delim);
public:
    DatasetLoader();
    ~DatasetLoader();
    std::pair<Image,Image> getImages();
};
};

#endif //DATASETLOADER_H