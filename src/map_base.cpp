#include "map_base.h"
#include "wolf_problem.h"
#include "landmark_base.h"

MapBase::MapBase() :
    NodeLinked(MID, "MAP")
{
    //std::cout << "MapBase::MapBase(): " << __LINE__ << std::endl;
}

MapBase::~MapBase()
{
	//std::cout << "deleting MapBase " << nodeId() << std::endl;
}

void MapBase::addLandmark(LandmarkBase* _landmark_ptr)
{
    addDownNode(_landmark_ptr);

    if (getTop()!= nullptr)
    {
        if (_landmark_ptr->getPPtr() != nullptr)
            getTop()->addStateBlockPtr(_landmark_ptr->getPPtr());
        if (_landmark_ptr->getOPtr() != nullptr)
            getTop()->addStateBlockPtr(_landmark_ptr->getOPtr());
    }
}

void MapBase::removeLandmark(LandmarkBase* _landmark_ptr)
{
    removeDownNode(_landmark_ptr->nodeId());
}

void MapBase::removeLandmark(const LandmarkBaseIter& _landmark_iter)
{
    removeDownNode(_landmark_iter);
}
