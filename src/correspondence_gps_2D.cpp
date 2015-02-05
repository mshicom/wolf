#include "correspondence_gps_2D.h"

CorrespondenceGPS2D::CorrespondenceGPS2D(const FeatureBaseShPtr& _ftr_ptr, WolfScalar* _statePtr) :
	CorrespondenceSparse<2,2>(_ftr_ptr,CORR_GPS_FIX_2D, _statePtr)
{
}

CorrespondenceGPS2D::CorrespondenceGPS2D(const FeatureBaseShPtr& _ftr_ptr, const StateBaseShPtr& _statePtr) :
	CorrespondenceSparse<2,2>(_ftr_ptr,CORR_GPS_FIX_2D, _statePtr->getPtr())
{
}

CorrespondenceGPS2D::~CorrespondenceGPS2D()
{
}

template <typename T>
bool CorrespondenceGPS2D::operator()(const T* const _x, T* _residuals) const
{
	_residuals[0] = ((T(*measurement_ptr_)(0))   - _x[0]) / T((*measurement_covariance_ptr_)(0,0));
	_residuals[1] = ((T(*measurement_ptr_)(1)) - _x[1]) / T((*measurement_covariance_ptr_)(1,1));
	return true;
}
