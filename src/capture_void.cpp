#include "capture_void.h"

namespace wolf {

CaptureVoid::CaptureVoid(const TimeStamp& _ts, SensorBase* _sensor_ptr) :
    CaptureBase("VOID", _ts, _sensor_ptr)
{
    //
}

CaptureVoid::~CaptureVoid()
{
	//std::cout << "deleting CaptureVoid " << nodeId() << std::endl;
}


} // namespace wolf
