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
};

};