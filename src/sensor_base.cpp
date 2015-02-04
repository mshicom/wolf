#include "sensor_base.h"

SensorBase::SensorBase(const SensorType & _tp, const Eigen::VectorXs & _sp) :
	sensor_pose_(_sp),
	type_(_tp)
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

const SensorType SensorBase::getSensorType() const
{
	return type_;
}
