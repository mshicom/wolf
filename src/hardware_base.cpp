#include "hardware_base.h"


namespace wolf {

HardwareBase::HardwareBase() :
    NodeLinked(MID, "HARDWARE", "BASE")
{
    //std::cout << "HardwareBase::HardwareBase(): " << __LINE__ << std::endl;
}

HardwareBase::~HardwareBase()
{
	//std::cout << "deleting HardwareBase " << nodeId() << std::endl;
}

SensorBase* HardwareBase::addSensor(SensorBase* _sensor_ptr)
{
    //std::cout << "adding sensor..." << std::endl;
	addDownNode(_sensor_ptr);
    //std::cout << "added!" << std::endl;

    _sensor_ptr->registerNewStateBlocks();

    return _sensor_ptr;

}

void HardwareBase::removeSensor(SensorBase* _sensor_ptr)
{
    removeDownNode(_sensor_ptr->nodeId());
}

SensorBase* HardwareBase::findSensor(const std::string _sensor_name)
{
    auto sen_it = std::find_if(getSensorListPtr()->begin(),
                               getSensorListPtr()->end(), [&](SensorBase* sb)
                               {
                                   return sb->getName() == _sensor_name;
                               }); // lambda function for the find_if
    if (sen_it == getSensorListPtr()->end())
        return nullptr;

    return (*sen_it);
}

} // namespace wolf
