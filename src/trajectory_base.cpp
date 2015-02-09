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

inline FrameBaseList & TrajectoryBase::frameList() const
{
    return downNodeList();
}

const inline FrameBaseList & TrajectoryBase::frameList() const
{
    return downNodeList();
}
