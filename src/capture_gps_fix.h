
#ifndef CAPTURE_GPS_FIX_H_
#define CAPTURE_GPS_FIX_H_

//std includes
//

//Wolf includes
#include "capture_base.h"
#include "feature_gps_fix.h"

//class CaptureGPSFix
class CaptureGPSFix : public CaptureBase
{
    public:
		CaptureGPSFix(double _ts, const SensorBasePtr& _sensor_ptr);

		CaptureGPSFix(double _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data);

		CaptureGPSFix(double _ts, const SensorBasePtr& _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);
        
        virtual ~CaptureGPSFix();
        
        virtual void processCapture();

        //virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;
};
#endif