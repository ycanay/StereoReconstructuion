#include <DatasetLoader.h>
#include <iostream>

int main()
{
    reconstruction::DatasetLoader TestClass1;
    std::pair<reconstruction::Image,reconstruction::Image> images = TestClass1.getImages();
    std::cout<<"Finished Test"<<std::endl;
    return 0;
}