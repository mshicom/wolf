#include "sensor_base.h"

SensorBase::SensorBase(const Eigen::VectorXs & _sp) : 
    sensor_pose_(_sp)
{
    //
}

SensorBase::~SensorBase()
{
    //
}

const Eigen::VectorXs * SensorBase::getSensorPose() const
{   
    return & sensor_pose_;
}
