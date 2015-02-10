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

void TrajectoryBase::addFrame(FrameBaseShPtr& _frame_ptr)
{
	std::cout << "adding frame\n";
	addDownNode(_frame_ptr);
}

FrameBaseList* TrajectoryBase::getFrameListPtr()
{
    return getDownNodeListPtr();
}

// const inline FrameBaseList* TrajectoryBase::frameList() const
// {
//     return downNodeListPtr();
// }
