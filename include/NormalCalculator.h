/**
 * @file NormalCalculator.h
 * @brief This class needs to calculate normals of the points in pointcloud by principal component analysis.
 * @version 0.1
 * @date 2023-06-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */
//TODO: Whole
#ifndef NORMAL_CALCULATOR_H
#define NORMAL_CALCULATOR_H

namespace reconstruction
{
class NormalCalculator
{
private:
    /* data */
public:
    NormalCalculator(/* args */);
    ~NormalCalculator();
    void calculateNormals();
};

};

#endif //NORMAL_CALCULATOR_H