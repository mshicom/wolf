#include "capture_odom_2D.h"

CaptureOdom2D::CaptureOdom2D(double _ts, const SensorBasePtr& _sensor_ptr) :
    CaptureBase(_ts, _sensor_ptr)
{
    //
}

CaptureOdom2D::CaptureOdom2D(double _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data) :
	CaptureBase(_ts, _sensor_ptr, _data)
{
	//
}

CaptureOdom2D::CaptureOdom2D(double _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase(_ts, _sensor_ptr, _data, _data_covariance)
{
	//
}

CaptureOdom2D::~CaptureOdom2D()
{
	//std::cout << "Destroying GPS fix capture...\n";
}

inline void CaptureOdom2D::processCapture()
{
    std::cout << "... processing GPS fix capture" << std::endl;
    FeatureBaseShPtr new_feature(new FeatureOdom2D(CaptureBaseShPtr(this),this->data_));
    addFeature(new_feature);
}

//void CaptureOdom2D::printSelf(unsigned int _ntabs, std::ostream & _ost) const
//{
//    NodeLinked::printSelf(_ntabs, _ost);
//    //printTabs(_ntabs);
//    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
//    //printNTabs(_ntabs);
//    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
//}



