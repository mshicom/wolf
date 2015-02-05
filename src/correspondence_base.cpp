#include "correspondence_base.h"

CorrespondenceBase::CorrespondenceBase(const FeatureBaseShPtr& _ftr_ptr, CorrespondenceType _tp) :
    NodeLinked(BOTTOM, "CORRESPONDENCE", _ftr_ptr.get()),
    type_(_tp),
	measurement_ptr_(_ftr_ptr->getMeasurementPtr()),
	measurement_covariance_ptr_(_ftr_ptr->getMeasurementCovariancePtr())
{
	//
}

CorrespondenceBase::~CorrespondenceBase()
{
	//
}

inline CorrespondenceType CorrespondenceBase::getType() const
{
    return type_;
}

const Eigen::VectorXs * CorrespondenceBase::getMeasurementPtr()
{
	return upperNodePtr()->getMeasurementPtr();
}

FeatureBasePtr CorrespondenceBase::getFeaturePtr() const
{
	return upperNodePtr();
}

CaptureBasePtr CorrespondenceBase::getCapturePtr() const
{
	return upperNodePtr()->upperNodePtr();
}
