#include "processor_base.h"

namespace wolf {

unsigned int ProcessorBase::processor_id_count_ = 0;

ProcessorBase::ProcessorBase(const std::string& _type, const std::string& _name) :
        NodeLinked(MID, "PROCESSOR", _type, _name),
        processor_id_(++processor_id_count_)
{
}

ProcessorBase::~ProcessorBase()
{
    //
}

bool ProcessorBase::permittedKeyFrame()
{
    return getProblem()->permitKeyFrame(this);
}

} // namespace wolf
