#include <DatasetLoader.h>
#include <ExtrinsicsCalculator.h>
#include <iostream>


int main()
{
    reconstruction::DatasetLoader TestClass1;
    reconstruction::ExtrinsicsCalculator Calculator;
    std::pair<reconstruction::Image,reconstruction::Image> images = TestClass1.getImages();
    Calculator.drawMatches(Calculator.getMatches(images.first, images.second), images.first, images.second);
    std::cout<<"Finished Test"<<std::endl;
    return 0;
}

