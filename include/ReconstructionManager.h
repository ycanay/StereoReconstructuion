/**
 * @file ReconstructionManager.h
 * @brief Main class that contains whole pipeline.
 * @version 0.1
 * @date 2023-06-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef RECONSTRUCTIONMANAGER_H
#define RECONSTRUCTIONMANAGER_H

#include <DatasetLoader.h>
#include <ExtrinsicsCalculator.h>

namespace reconstruction{

class ReconstructionManager
{
private:
    DatasetLoader dataset_loader_;
    ExtrinsicsCalculator extrinsic_calculator_;
public:
    ReconstructionManager();
    ~ReconstructionManager();
    void process();
};

};
#endif //RECONSTRUCTIONMANAGER_H