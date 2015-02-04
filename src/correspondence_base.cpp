#include "correspondence_base.h"

CorrespondenceBase::CorrespondenceBase(const FeatureBaseShPtr& _ftr_ptr, CorrespondenceType _tp) :
    NodeLinked(BOTTOM, "CORRESPONDENCE", _ftr_ptr.get()),
    type_(_tp)
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
