#include <DatasetLoader.h>
#include <ExtrinsicsCalculator.h>
#include <iostream>


int main()
{
    reconstruction::DatasetLoader TestClass1;
    reconstruction::ExtrinsicsCalculator Calculator;
    std::pair<reconstruction::Image,reconstruction::Image> images = TestClass1.getImages();
    Calculator.calculateMatches(images.first, images.second);
    Calculator.drawMatches(images.first, images.second);
    Calculator.calculateMatchingPointsCoordinates();
    std::vector<reconstruction::PointPair> pairs = Calculator.getMatchingPointCoordinates();
    Calculator.multiplyIntrinsics(images.first, images.second);
    Calculator.eightPointMatching();
    auto homo = Calculator.getHomography();
    std::cout<<"Finished Test"<<std::endl;
    return 0;
}

