#include "sensor_base.h"

SensorBase::SensorBase(const SensorType & _tp, const Eigen::VectorXs & _spv) :
    type_(_tp), 
	sensor_pose_vehicle_(_spv)
{
    //
}

SensorBase::~SensorBase()
{
    //
}

const SensorType SensorBase::getSensorType() const
{
    return type_;
}

const Eigen::VectorXs * SensorBase::getSensorPose() const
{   
    return & sensor_pose_vehicle_;
}
