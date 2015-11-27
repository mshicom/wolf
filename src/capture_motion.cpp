#include "capture_motion.h"

CaptureMotion::CaptureMotion(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr) :
    CaptureBase(_init_ts, _sensor_ptr),
    final_time_stamp_(_final_ts)
{
    //
}

CaptureMotion::CaptureMotion(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase(_init_ts, _sensor_ptr, _data),
    final_time_stamp_(_final_ts)
{
	//
}

CaptureMotion::CaptureMotion(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase(_init_ts, _sensor_ptr, _data, _data_covariance),
    final_time_stamp_(_final_ts)
{
	//
}
CaptureMotion::~CaptureMotion()
{
	//
}

TimeStamp CaptureMotion::getInitTimeStamp() const
{
    return this->getTimeStamp();
}

TimeStamp CaptureMotion::getFinalTimeStamp() const
{
    return final_time_stamp_;
}

void CaptureMotion::setInitTimeStamp(const TimeStamp & _ts)
{
    this->setTimeStamp(_ts);
}

void CaptureMotion::setFinalTimeStamp(const TimeStamp & _ts)
{
    final_time_stamp_ = _ts;
}