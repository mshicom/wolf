#include "sensor_base.h"

SensorBase::SensorBase(const SensorType & _tp, const Eigen::VectorXs & _spv) :
	sensor_pose_vehicle_(_sp),
	type_(_tp)
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
    return & sensor_pose_vehicle;
}
