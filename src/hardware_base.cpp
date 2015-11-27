#include "hardware_base.h"

HardwareBase::HardwareBase() :
    NodeLinked(MID, "HARDWARE")
{
    //std::cout << "HardwareBase::HardwareBase(): " << __LINE__ << std::endl;
}

HardwareBase::~HardwareBase()
{
	//std::cout << "deleting HardwareBase " << nodeId() << std::endl;
}

void HardwareBase::addSensor(SensorBase* _sensor_ptr)
{
    //std::cout << "adding sensor..." << std::endl;
	addDownNode(_sensor_ptr);
    //std::cout << "added!" << std::endl;
}

void HardwareBase::removeSensor(const SensorBaseIter& _sensor_iter)
{
	removeDownNode(_sensor_iter);
}

void HardwareBase::removeSensor(SensorBase* _sensor_ptr)
{
	removeDownNode(_sensor_ptr->nodeId());
}

SensorBaseList* HardwareBase::getSensorListPtr()
{
    return getDownNodeListPtr();
}