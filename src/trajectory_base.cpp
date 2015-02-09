#include "trajectory_base.h"

TrajectoryBase::TrajectoryBase() : 
    NodeLinked(TOP, "TRAJECTORY")
{
    //
}

TrajectoryBase::~TrajectoryBase()
{
    //
}

FrameBaseList* TrajectoryBase::getFrameListPtr()
{
    return getDownNodeListPtr();
}

// const inline FrameBaseList* TrajectoryBase::frameList() const
// {
//     return downNodeListPtr();
// }
