#include "capture_laser_2D.h"

CaptureLaser2D::CaptureLaser2D(const TimeStamp & _ts, const SensorLaser2DPtr & _sensor_ptr, const Eigen::VectorXs& _ranges):
    CaptureBase(_ts, _sensor_ptr, _ranges)
{
    // 
}

CaptureLaser2D::~CaptureLaser2D()
{
    //
}

void CaptureLaser2D::processCapture()
{
    extractCorners();
}

void CaptureLaser2D::extractCorners()
{
    std::cout << "Extracting corners ... " << std::endl;
    //TODO by Andreu: create class FeatureCorner2D + main to test this method.
    //TODO by Juan AC
    // Laser ranges are at data_
    // Scan size is data_.size()
    // Corners should be created as FeatureCorner2D. Corner 3 params to be stored at FeatureBase::measurement_ 
    // After creation, they have to be pushed back to down_node_list_ by means of the method Capture::addFeature(const FeatureShPtr& _f_ptr)  
}
