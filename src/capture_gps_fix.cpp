#include "capture_gps_fix.h"

CaptureGPSFix::CaptureGPSFix(double _ts, const SensorBasePtr& _sensor_ptr) :
    CaptureBase(_ts, _sensor_ptr)
{
    //
}

CaptureGPSFix::CaptureGPSFix(double _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase(_ts, _sensor_ptr, _data)
{
	//
}

CaptureGPSFix::CaptureGPSFix(double _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase(_ts, _sensor_ptr, _data, _data_covariance)
{
	//
}

CaptureGPSFix::~CaptureGPSFix()
{
	//std::cout << "Destroying GPS fix capture...\n";
}

inline void CaptureGPSFix::processCapture()
{
    std::cout << "... processing GPS fix capture" << std::endl;
    FeatureBaseShPtr new_feature(new FeatureGPSFix(CaptureBaseShPtr(this),this->data_));
    addFeature(new_feature);
}

//void CaptureGPSFix::printSelf(unsigned int _ntabs, std::ostream & _ost) const
//{
//    NodeLinked::printSelf(_ntabs, _ost);
//    //printTabs(_ntabs);
//    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
//    //printNTabs(_ntabs);
//    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
//}



