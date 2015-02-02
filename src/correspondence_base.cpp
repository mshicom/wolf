#include "correspondence_base.h"

CorrespondenceBase::CorrespondenceBase(CorrespondenceType _tp) : 
    NodeLinked(BOTTOM, "CORRESPONDENCE"),
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
