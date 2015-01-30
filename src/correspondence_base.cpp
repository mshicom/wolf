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

inline const std::vector<WolfScalar*> * CorrespondenceBase::getStateBlockPtrVector()
{
    return & state_block_ptr_vector_;
}
