
#include "sensor_fix.h"

namespace wolf {

SensorFix::SensorFix() :
        SensorBase(SEN_ABSOLUTE_POSE, "FIX", nullptr, nullptr, nullptr, 0)
{
    //
}

SensorFix::~SensorFix()
{
}

// Define the factory method
SensorBase* SensorFix::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_p,
                              const IntrinsicsBase* _intrinsics)
{
    SensorBase* sen = new SensorFix();
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf

// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
namespace
{
const bool registered_gps = SensorFactory::get().registerCreator("FIX", SensorFix::create);
}
} // namespace wolf
