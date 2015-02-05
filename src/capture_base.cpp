#include "capture_base.h"

CaptureBase::CaptureBase(double _ts, const SensorBasePtr& _sensor_ptr) :
    NodeLinked(MID, "CAPTURE"),
    time_stamp_(_ts),
    sensor_ptr_(_sensor_ptr)
{
    //
}

CaptureBase::CaptureBase(double _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data) :
	NodeLinked(MID, "CAPTURE"),
	time_stamp_(_ts),
	sensor_ptr_(_sensor_ptr),
	data_(_data)
{
	//
}

CaptureBase::CaptureBase(double _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	NodeLinked(MID, "CAPTURE"),
	time_stamp_(_ts),
	sensor_ptr_(_sensor_ptr),
	data_(_data),
	data_covariance_(_data_covariance)
{
	//
}

CaptureBase::~CaptureBase()
{
	//std::cout << "Destroying capture...\n";
}

inline void CaptureBase::linkToFrame(const FrameBaseShPtr& _frm_ptr)
{
    linkToUpperNode(_frm_ptr.get());
}

// TODO: Why the linker throws an error when this function is inline...
void CaptureBase::addFeature(FeatureBaseShPtr & _ft_ptr)
{
    addDownNode(_ft_ptr);
}

inline const FrameBasePtr CaptureBase::getFramePtr() const
{
    return upperNodePtr();
}

inline const FeatureBaseList & CaptureBase::getFeatureList() const
{
    return downNodeList();
}

inline WolfScalar CaptureBase::getTimeStamp() const
{
    return time_stamp_.get();
}

inline SensorBasePtr CaptureBase::getSensorPtr() const
{
    return sensor_ptr_;
}

inline SensorType CaptureBase::getSensorType() const
{
	return sensor_ptr_->getSensorType();
}

inline void CaptureBase::setTimeStamp(const WolfScalar & _ts)
{
    time_stamp_ = _ts;
}

inline void CaptureBase::setTimeStampToNow()
{
    time_stamp_.setToNow();
}

void CaptureBase::setData(unsigned int _size, const WolfScalar *_data)
{
    data_.resize(_size);
    for (unsigned int ii=0; ii<_size; ii++) data_(ii) = *(&_data[ii]);
}

void CaptureBase::setData(const Eigen::VectorXs& _data)
{
    data_=_data;
}

void CaptureBase::setDataCovariance(const Eigen::MatrixXs& _data_cov)
{
    data_covariance_ = _data_cov;
}

inline void CaptureBase::processCapture()
{
    std::cout << "... processing capture" << std::endl;
}

void CaptureBase::printSelf(unsigned int _ntabs, std::ostream & _ost) const
{
    NodeLinked::printSelf(_ntabs, _ost);
    //printTabs(_ntabs);
    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
    //printNTabs(_ntabs);
    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
}



